// License: GNU AGPLv3 or higher

#include "FiberTailRelease.hpp"
#include "FiberStrand.hpp"
// FiberEmitter.hpp for FiberEmitParams + zone_feed_mm_min: the activation-block
// builder below has to mirror emit_strand() move for move, so it reads the same
// zone-feed function the emitter uses rather than a copy of it.
#include "FiberEmitter.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace Slic3r {
namespace Fiber {

namespace {

const double k_pi = 3.14159265358979323846;

double rad2deg(double r) { return r * 180.0 / k_pi; }

// Wrap an angle difference into (-pi, pi] so a 359 degree turn reads as -1.
double wrap_pi(double a)
{
    while (a > k_pi)
        a -= 2.0 * k_pi;
    while (a <= -k_pi)
        a += 2.0 * k_pi;
    return a;
}

double seg_len(const FiberPoint& a, const FiberPoint& b)
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Perpendicular distance of `p` from the infinite line through `a`->`b`.
double perp_deviation(const FiberPoint& a, const FiberPoint& b, const FiberPoint& p)
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    if (len2 <= 0.0)
        return seg_len(a, p); // degenerate chord: distance to the point itself
    const double cross = dx * (p.y - a.y) - dy * (p.x - a.x);
    return std::fabs(cross) / std::sqrt(len2);
}

void put(std::string& s, const std::string& line)
{
    s += line;
    s += '\n';
}

std::string fmt3(const char* fmt, double v)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), fmt, v);
    return std::string(buf);
}

} // namespace

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

const char* fs_code_name(FsCode code)
{
    switch (code) {
    case FsCode::WaitOutsideStation:         return "FS_WAIT_OUTSIDE_STATION";
    case FsCode::CoolBeforeRelease:          return "FS_COOL_BEFORE_RELEASE";
    case FsCode::CoolBeforeClean:            return "FS_COOL_BEFORE_CLEAN";
    case FsCode::PreheatClobbered:           return "FS_PREHEAT_CLOBBERED";
    case FsCode::ReleaseNotExecuted:         return "FS_RELEASE_NOT_EXECUTED";
    case FsCode::ReleaseExtrusion:           return "FS_RELEASE_EXTRUSION";
    case FsCode::ReleaseUnsupported:         return "FS_RELEASE_UNSUPPORTED";
    case FsCode::TailDistanceMismatch:       return "FS_TAIL_DISTANCE_MISMATCH";
    case FsCode::ERecoveryLocation:          return "FS_E_RECOVERY_LOCATION";
    case FsCode::TailSharpTurn:              return "FS_TAIL_SHARP_TURN";
    case FsCode::MacroContractUnverified:    return "FS_MACRO_CONTRACT_UNVERIFIED";
    case FsCode::NonblockingHeatUnsupported: return "FS_NONBLOCKING_HEAT_UNSUPPORTED";
    case FsCode::ThermalManagementDisabled:  return "THERMAL_MANAGEMENT_DISABLED";
    case FsCode::PurgeReleaseOutOfBounds:    return "FS_PURGE_RELEASE_OUT_OF_BOUNDS";
    }
    return "FS_UNKNOWN";
}

bool fs_code_is_warning(FsCode code)
{
    // The spec classifies exactly two of the codes as WARN; everything else is
    // an ERROR. Kept as an explicit whitelist so adding a code cannot silently
    // inherit the wrong severity.
    return code == FsCode::TailSharpTurn || code == FsCode::MacroContractUnverified;
}

// ---------------------------------------------------------------------------
// Tail / cut / release distances
// ---------------------------------------------------------------------------

bool validate_tail_release_params(const TailReleaseParams& p, std::string* error)
{
    auto fail = [error](const char* msg) {
        if (error)
            *error = msg;
        return false;
    };
    if (!std::isfinite(p.nominal_tail_mm) || !(p.nominal_tail_mm > 0.0))
        return fail("fs_tail_length must be finite and > 0");
    if (!std::isfinite(p.margin_mm) || p.margin_mm < 0.0 || p.margin_mm > 3.0)
        return fail("fs_fiber_tail_margin_mm must be finite and within 0..3 mm");
    if (!std::isfinite(p.release_mm) || p.release_mm < 0.0 || p.release_mm > 10.0)
        return fail("fs_fiber_release_length_mm must be finite and within 0..10 mm");
    if (!std::isfinite(p.release_speed_mm_s) || p.release_speed_mm_s < 1.0 || p.release_speed_mm_s > 20.0)
        return fail("fs_fiber_release_speed_mm_s must be finite and within 1..20 mm/s");
    if (!std::isfinite(p.anchor_mm) || p.anchor_mm < 2.0 || p.anchor_mm > 20.0)
        return fail("fs_fiber_release_anchor_mm must be finite and within 2..20 mm");
    // A missing composite width is a configuration error, not a licence to
    // inherit a plastic width: buffering by the wrong half-width would pass a
    // release that is actually unsupported.
    if (!std::isfinite(p.bead_width_mm) || !(p.bead_width_mm > 0.0))
        return fail("composite bead width must be finite and > 0 (a missing fibre width is a configuration error)");
    return true;
}

bool plan_tail_release_distances(double planned_length_mm,
                                 const TailReleaseParams& p,
                                 TailReleaseDistances& out,
                                 std::string* error)
{
    out = TailReleaseDistances{};
    if (!std::isfinite(planned_length_mm) || !(planned_length_mm > 0.0)) {
        if (error)
            *error = "FS_RELEASE_UNSUPPORTED: planned strand length must be finite and > 0";
        return false;
    }
    const double post = p.nominal_tail_mm + p.margin_mm;
    // S <= T + M would leave a zero or negative body. Reject before export; the
    // spec forbids silently shortening the tail or the margin to rescue it.
    if (!(planned_length_mm > post)) {
        if (error)
            *error = "FS_RELEASE_UNSUPPORTED: strand length " + fmt3("%.3f", planned_length_mm)
                   + " mm does not exceed tail + margin " + fmt3("%.3f", post)
                   + " mm (no body would remain before the cut)";
        return false;
    }
    out.planned_length_mm   = planned_length_mm;
    out.cut_distance_mm     = planned_length_mm - post;
    out.post_cut_deposit_mm = post;
    out.release_mm          = p.release_mm;
    out.post_cut_xy_mm      = post + p.release_mm;
    out.release_f           = std::isfinite(p.release_speed_mm_s) ? p.release_speed_mm_s * 60.0 : 0.0;
    return true;
}

// ---------------------------------------------------------------------------
// Straight runs
// ---------------------------------------------------------------------------

namespace {

// Greedy maximal run starting at segment `first`. Returns the last segment index
// that still qualifies, and fills spread/deviation. `nseg` bounds the scan;
// indices are taken modulo `npts` so a closed path can wrap.
size_t extend_run(const std::vector<FiberPoint>& pts,
                  size_t nseg,
                  size_t first,
                  double max_spread_deg,
                  double max_dev_mm,
                  double& spread_out,
                  double& dev_out)
{
    const size_t npts = pts.size();
    double spread = 0.0;
    double dev = 0.0;
    size_t last = first;
    while (last + 1 < nseg) {
        const size_t next = last + 1;
        const FiberPoint& a = pts[(first) % npts];
        const FiberPoint& b = pts[(next + 1) % npts];
        // Heading spread: add the turn between segment `last` and `next`.
        const double h0 = std::atan2(pts[(last + 1) % npts].y - pts[last % npts].y,
                                     pts[(last + 1) % npts].x - pts[last % npts].x);
        const double h1 = std::atan2(pts[(next + 1) % npts].y - pts[next % npts].y,
                                     pts[(next + 1) % npts].x - pts[next % npts].x);
        const double trial_spread = spread + std::fabs(wrap_pi(h1 - h0));
        if (rad2deg(trial_spread) > max_spread_deg)
            break;
        // Perpendicular deviation of every intermediate vertex from the new chord.
        double trial_dev = 0.0;
        for (size_t k = first + 1; k <= next; ++k) {
            const FiberPoint& p = pts[k % npts];
            const double d = perp_deviation(a, b, p);
            if (d > trial_dev)
                trial_dev = d;
        }
        if (trial_dev > max_dev_mm)
            break;
        spread = trial_spread;
        dev = trial_dev;
        last = next;
    }
    spread_out = spread;
    dev_out = dev;
    return last;
}

} // namespace

std::vector<StraightRun> find_straight_runs(const std::vector<FiberPoint>& pts,
                                            bool closed,
                                            double max_spread_deg,
                                            double max_dev_mm)
{
    std::vector<StraightRun> runs;
    if (pts.size() < 2)
        return runs;

    // Normalise a closed path to exactly one vertex per corner. A path the
    // planner already closed by repeating its first vertex would otherwise
    // contribute a zero-length wrap edge, which reads as a degenerate segment
    // and corrupts the cumulative distances; a path closed only topologically
    // gets its closure vertex added. Either way `v` carries exactly `nseg` real
    // edges, which is what the scan below assumes.
    std::vector<FiberPoint> v = pts;
    if (closed) {
        if (seg_len(v.front(), v.back()) <= 1e-9)
            v.pop_back();
        else
            v.push_back(v.front());
        if (v.size() < 3)
            return runs;
    }
    const size_t npts = v.size();
    const size_t nseg = npts - 1;
    if (nseg == 0)
        return runs;

    // For a closed path scan the doubled vertex list so a run that wraps the
    // closure point is found; for an open path the plain single pass.
    std::vector<double> cum(2 * npts, 0.0);
    const size_t scan_pts = closed ? 2 * npts : npts;
    for (size_t i = 0; i + 1 < scan_pts; ++i)
        cum[i + 1] = cum[i] + seg_len(v[i % npts], v[(i + 1) % npts]);
    const double loop_len = closed ? cum[npts - 1] : 0.0;

    // One pass over each distinct start segment. A closed scan may wrap, so the
    // start index runs through the doubled range but never past one full lap.
    const size_t scan_segs = closed ? 2 * nseg : nseg;
    size_t first = 0;
    while (first < scan_segs) {
        double spread = 0.0;
        double dev = 0.0;
        size_t last = extend_run(v, scan_segs, first, max_spread_deg, max_dev_mm, spread, dev);
        StraightRun r;
        r.first_seg = first % nseg;
        r.last_seg = last % nseg;
        r.length_mm = cum[last + 1] - cum[first];
        r.heading_spread_deg = rad2deg(spread);
        r.max_deviation_mm = dev;
        r.start_distance_mm = closed ? std::fmod(cum[first], loop_len) : cum[first];
        // Cap a wrapping run at one lap so it can never claim the loop twice.
        if (closed && r.length_mm > loop_len)
            r.length_mm = loop_len;
        runs.push_back(r);
        if (last + 1 <= first)
            break;
        first = last + 1;
        if (closed && first >= nseg)
            break; // every distinct start segment has had its one pass
    }

    if (closed) {
        // A wrapping run can strictly contain a non-wrapping one found in the
        // same pass; keep only the maximal set, preserving path order.
        std::vector<bool> drop(runs.size(), false);
        for (size_t i = 0; i < runs.size(); ++i) {
            if (drop[i])
                continue;
            for (size_t j = 0; j < runs.size(); ++j) {
                if (i == j || drop[j])
                    continue;
                if (runs[j].length_mm >= runs[i].length_mm
                    && runs[j].start_distance_mm <= runs[i].start_distance_mm
                    && runs[j].start_distance_mm + runs[j].length_mm
                           >= runs[i].start_distance_mm + runs[i].length_mm
                    && runs[j].length_mm > runs[i].length_mm)
                    drop[i] = true;
            }
        }
        std::vector<StraightRun> kept;
        for (size_t i = 0; i < runs.size(); ++i)
            if (!drop[i])
                kept.push_back(runs[i]);
        runs = kept;
    }
    return runs;
}

bool select_release_seam(const std::vector<FiberPoint>& pts,
                         bool closed,
                         const TailReleaseParams& p,
                         ReleaseSeam& out,
                         std::string* error)
{
    out = ReleaseSeam{};
    if (!closed) {
        if (error)
            *error = "FS_RELEASE_UNSUPPORTED: seam rotation applies only to a closed strand";
        return false;
    }
    const auto runs = find_straight_runs(pts, true);
    const double need = p.anchor_mm + p.release_mm + 0.02;
    const StraightRun* best = nullptr;
    for (const StraightRun& r : runs) {
        if (r.length_mm < need) {
            out.rejected.push_back({FsCode::ReleaseUnsupported,
                                    "run at " + fmt3("%.3f", r.start_distance_mm) + " mm is "
                                    + fmt3("%.3f", r.length_mm) + " mm, short of required "
                                    + fmt3("%.3f", need) + " mm"});
            continue;
        }
        if (!best
            || r.length_mm > best->length_mm
            || (r.length_mm == best->length_mm && r.first_seg < best->first_seg)
            || (r.length_mm == best->length_mm && r.first_seg == best->first_seg
                && r.start_distance_mm < best->start_distance_mm))
            best = &r;
    }
    if (!best) {
        out.rejected.push_back({FsCode::ReleaseUnsupported, "no qualifying straight run"});
        if (error)
            *error = "FS_RELEASE_UNSUPPORTED: no straight run of " + fmt3("%.3f", need)
                   + " mm (anchor + release + 0.02) is available for the forward release";
        return false;
    }
    out.valid = true;
    out.run_first_seg = best->first_seg;
    out.run_last_seg = best->last_seg;
    out.run_length_mm = best->length_mm;
    // The seam sits R + 0.01 mm before the run's forward end, so the anchor
    // deposition and the whole release lie inside the same straight run.
    out.seam_distance_mm = best->start_distance_mm + best->length_mm - (p.release_mm + 0.01);
    out.rotation_mm = out.seam_distance_mm;
    out.release_start_distance_mm = out.seam_distance_mm;
    return true;
}

std::vector<FiberPoint> rotate_closed_path(const std::vector<FiberPoint>& pts, double seam_distance_mm)
{
    std::vector<FiberPoint> out;
    if (pts.size() < 2)
        return out;

    // Normalise to an explicitly closed vertex list.
    std::vector<FiberPoint> v = pts;
    const bool already_closed = seg_len(pts.front(), pts.back()) <= 1e-9;
    if (!already_closed)
        v.push_back(pts.front());

    const size_t n = v.size();
    std::vector<double> cum(n, 0.0);
    for (size_t i = 0; i + 1 < n; ++i)
        cum[i + 1] = cum[i] + seg_len(v[i], v[i + 1]);
    const double total = cum[n - 1];
    if (!(total > 0.0))
        return out;

    double d = std::fmod(seam_distance_mm, total);
    if (d < 0.0)
        d += total;

    // Locate the segment containing d.
    size_t i = 0;
    while (i + 2 < n && cum[i + 1] <= d)
        ++i;
    const double seg = cum[i + 1] - cum[i];
    const double t = seg > 0.0 ? (d - cum[i]) / seg : 0.0;
    FiberPoint seam;
    seam.x = v[i].x + (v[i + 1].x - v[i].x) * t;
    seam.y = v[i].y + (v[i + 1].y - v[i].y) * t;

    out.push_back(seam);
    for (size_t k = i + 1; k < n; ++k)
        out.push_back(v[k]);            // forward to the closure vertex (== v[0])
    for (size_t k = 1; k <= i; ++k)
        out.push_back(v[k]);            // wrap: continue from v[1] up to v[i]
    out.push_back(seam);                // close on the seam itself
    return out;
}

// ---------------------------------------------------------------------------
// Support
// ---------------------------------------------------------------------------

std::vector<FiberPoint> release_footprint(const std::vector<FiberPoint>& path, double half_width_mm)
{
    std::vector<FiberPoint> out;
    if (path.size() < 2 || !(half_width_mm > 0.0))
        return out;
    // Offset each end of every segment by the half width along the segment
    // normal; the footprint is the swept band, reported as its two rails.
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const double dx = path[i + 1].x - path[i].x;
        const double dy = path[i + 1].y - path[i].y;
        const double len = std::sqrt(dx * dx + dy * dy);
        if (len <= 0.0)
            continue;
        const double nx = -dy / len * half_width_mm;
        const double ny = dx / len * half_width_mm;
        FiberPoint a{path[i].x + nx, path[i].y + ny};
        FiberPoint b{path[i + 1].x + nx, path[i + 1].y + ny};
        FiberPoint c{path[i + 1].x - nx, path[i + 1].y - ny};
        FiberPoint d{path[i].x - nx, path[i].y - ny};
        out.push_back(a);
        out.push_back(b);
        out.push_back(c);
        out.push_back(d);
    }
    return out;
}

bool plan_fiber_release(const std::vector<FiberPoint>& pts,
                        bool closed,
                        double z,
                        const TailReleaseParams& p,
                        const ReleaseSupportModel* support,
                        ReleasePlan& out,
                        std::string* error)
{
    out = ReleasePlan{};
    out.closed = closed;
    out.z = z;
    out.requested_mm = p.release_mm;
    out.f_mm_min = p.release_speed_mm_s * 60.0;

    std::string perr;
    if (!validate_tail_release_params(p, &perr)) {
        out.findings.push_back({FsCode::ReleaseUnsupported, perr});
        if (error)
            *error = perr;
        return false;
    }

    // R == 0 is the feature being disabled, not a failure: variant A must stay
    // independently exportable with the baseline seam.
    if (p.release_mm <= 0.0) {
        out.valid = true;
        out.support = SupportVerdict::NotEvaluated;
        return true;
    }
    if (pts.size() < 2) {
        out.findings.push_back({FsCode::ReleaseUnsupported, "strand path has fewer than two points"});
        if (error)
            *error = "FS_RELEASE_UNSUPPORTED: strand path has fewer than two points";
        return false;
    }

    std::vector<FiberPoint> path;
    if (closed) {
        // The path arrives already rotated to the seam, so the release is the
        // first R mm of it, forward, over material deposited moments earlier.
        double need = p.release_mm;
        path.push_back(pts.front());
        for (size_t i = 0; i + 1 < pts.size() && need > 1e-9; ++i) {
            const double len = seg_len(pts[i], pts[i + 1]);
            if (len <= 1e-9)
                continue;
            if (len <= need + 1e-9) {
                path.push_back(pts[i + 1]);
                need -= len;
            } else {
                const double t = need / len;
                FiberPoint q{pts[i].x + (pts[i + 1].x - pts[i].x) * t,
                             pts[i].y + (pts[i + 1].y - pts[i].y) * t};
                path.push_back(q);
                need = 0.0;
            }
        }
        if (need > 1e-6) {
            out.findings.push_back({FsCode::ReleaseUnsupported,
                                    "path carries only " + fmt3("%.3f", p.release_mm - need)
                                    + " mm of the requested " + fmt3("%.3f", p.release_mm) + " mm"});
            if (error)
                *error = "FS_RELEASE_UNSUPPORTED: closed path is shorter than the release";
            return false;
        }
        out.support = SupportVerdict::Supported; // retrace over its own deposition
    } else {
        // Open strand: the endpoint is preserved, the last anchor_mm must be
        // straight, and the release continues along the final forward tangent.
        const auto runs = find_straight_runs(pts, false);
        double total = 0.0;
        for (size_t i = 0; i + 1 < pts.size(); ++i)
            total += seg_len(pts[i], pts[i + 1]);
        bool anchored = false;
        for (const StraightRun& r : runs) {
            if (r.start_distance_mm + r.length_mm >= total - 1e-6 && r.length_mm >= p.anchor_mm) {
                anchored = true;
                break;
            }
        }
        if (!anchored) {
            out.findings.push_back({FsCode::ReleaseUnsupported,
                                    "final " + fmt3("%.3f", p.anchor_mm) + " mm is not a qualifying straight run"});
            if (error)
                *error = "FS_RELEASE_UNSUPPORTED: open strand lacks a straight anchor before its endpoint";
            return false;
        }
        const FiberPoint& a = pts[pts.size() - 2];
        const FiberPoint& b = pts.back();
        const double len = seg_len(a, b);
        if (len <= 0.0) {
            if (error)
                *error = "FS_RELEASE_UNSUPPORTED: degenerate final segment";
            return false;
        }
        const double ux = (b.x - a.x) / len;
        const double uy = (b.y - a.y) / len;
        FiberPoint end{b.x + ux * p.release_mm, b.y + uy * p.release_mm};
        path.push_back(b);
        path.push_back(end);

        if (!support) {
            // An open-air extension is exactly what the spec forbids guessing
            // about: no footprint geometry means the export must fail.
            out.support = SupportVerdict::NotEvaluated;
            out.findings.push_back({FsCode::ReleaseUnsupported,
                                    "no deposited-footprint geometry available to verify an open-end release"});
            if (error)
                *error = "FS_RELEASE_UNSUPPORTED: open-end release support cannot be evaluated";
            return false;
        }
        ReleaseSupportQuery q;
        q.path = path;
        // Half the ACTUAL composite bead width from the fibre configuration.
        // Never an inherited "T1 WIDTH" comment: that is the plastic nozzle.
        q.half_width_mm = p.bead_width_mm / 2.0;
        q.z = z;
        q.tolerance_mm = 0.02;
        out.support = support->query(q);
        if (out.support != SupportVerdict::Supported) {
            out.findings.push_back({FsCode::ReleaseUnsupported,
                                    out.support == SupportVerdict::Unsupported
                                        ? "release footprint leaves deposited material of this layer"
                                        : "release support not evaluated"});
            if (error)
                *error = "FS_RELEASE_UNSUPPORTED: open-end release footprint is not supported";
            return false;
        }
    }

    double planned = 0.0;
    for (size_t i = 0; i + 1 < path.size(); ++i)
        planned += seg_len(path[i], path[i + 1]);

    out.valid = true;
    out.path = path;
    out.start = path.front();
    out.end = path.back();
    out.planned_mm = planned;
    return true;
}

bool plan_closed_strand_release(const FiberStrand& strand,
                                const TailReleaseParams& p,
                                ReleasePlan& out,
                                std::string* error)
{
    out = ReleasePlan{};
    out.closed = true;
    out.z = strand.z;
    out.requested_mm = p.release_mm;
    out.f_mm_min = p.release_speed_mm_s * 60.0;

    std::string perr;
    if (!validate_tail_release_params(p, &perr)) {
        out.findings.push_back({FsCode::ReleaseUnsupported, perr});
        if (error)
            *error = perr;
        return false;
    }
    // R == 0 is the feature disabled, not a failure.
    if (p.release_mm <= 0.0) {
        out.valid = true;
        out.support = SupportVerdict::NotEvaluated;
        return true;
    }
    if (!strand.finalized || strand.pts.size() < 3) {
        out.findings.push_back({FsCode::ReleaseUnsupported,
                                "strand is not finalized or carries fewer than three points"});
        if (error)
            *error = "FS_RELEASE_UNSUPPORTED: strand cannot carry a release";
        return false;
    }

    // The planner hands a closed strand back with its first vertex repeated, so
    // find_straight_runs() sees a genuinely closed ring. Only the run that opens
    // at the strand's start can carry this release: the dry move has to retrace
    // material this same strand deposited, and that is the opening run.
    const auto runs = find_straight_runs(strand.pts, true);
    const double need = p.anchor_mm + p.release_mm + 0.02;
    bool qualified = false;
    for (const StraightRun& r : runs) {
        if (r.first_seg != 0)
            break; // the opening run is the first one found, by construction
        if (r.length_mm >= need)
            qualified = true;
        else
            out.findings.push_back({FsCode::ReleaseUnsupported,
                                    "opening run is " + fmt3("%.3f", r.length_mm) +
                                    " mm, short of required " + fmt3("%.3f", need) + " mm"});
        break;
    }
    if (!qualified) {
        if (out.findings.empty())
            out.findings.push_back({FsCode::ReleaseUnsupported,
                                    "strand does not open on a straight run"});
        if (error)
            *error = "FS_RELEASE_UNSUPPORTED: strand does not open on a straight run of "
                   + fmt3("%.3f", need) + " mm";
        return false;
    }

    // Take the first R mm forward, splitting the containing segment if needed.
    double remaining = p.release_mm;
    out.path.push_back(strand.pts.front());
    for (size_t i = 0; i + 1 < strand.pts.size() && remaining > 1e-9; ++i) {
        const double len = seg_len(strand.pts[i], strand.pts[i + 1]);
        if (len <= 1e-9)
            continue;
        if (len <= remaining + 1e-9) {
            out.path.push_back(strand.pts[i + 1]);
            remaining -= len;
        } else {
            const double t = remaining / len;
            FiberPoint q{strand.pts[i].x + (strand.pts[i + 1].x - strand.pts[i].x) * t,
                         strand.pts[i].y + (strand.pts[i + 1].y - strand.pts[i].y) * t};
            out.path.push_back(q);
            remaining = 0.0;
        }
    }
    if (remaining > 1e-6) {
        out.findings.push_back({FsCode::ReleaseUnsupported,
                                "path shorter than the requested release"});
        if (error)
            *error = "FS_RELEASE_UNSUPPORTED: strand path shorter than the release";
        return false;
    }

    double planned = 0.0;
    for (size_t i = 0; i + 1 < out.path.size(); ++i)
        planned += seg_len(out.path[i], out.path[i + 1]);
    out.valid = true;
    out.start = out.path.front();
    out.end = out.path.back();
    out.planned_mm = planned;
    // Supported by construction: the dry move lies on material this same strand
    // deposited at its opening. This is the closed-strand case the spec allows
    // without a footprint oracle; an OPEN strand still requires one and is refused
    // by plan_fiber_release() rather than assumed supported.
    out.support = SupportVerdict::Supported;
    return true;
}

std::string emit_fiber_release(const ReleasePlan& plan)
{
    std::string s;
    if (!plan.valid || plan.path.size() < 2)
        return s;
    put(s, "; FS_RELEASE_BEGIN length_mm=" + fmt3("%.3f", plan.planned_mm)
           + " speed_mm_s=" + fmt3("%.3f", plan.f_mm_min / 60.0));
    char buf[96];
    for (size_t i = 0; i + 1 < plan.path.size(); ++i) {
        // XY and F only. No U, no V, no E, no Z: the release is dry travel by
        // definition, and the analyzer errors on any material word in here.
        std::snprintf(buf, sizeof(buf), "G1 X%.3f Y%.3f F%.0f",
                      FiberRun::round3(plan.path[i + 1].x), FiberRun::round3(plan.path[i + 1].y),
                      plan.f_mm_min);
        put(s, buf);
    }
    put(s, "; FS_RELEASE_END physical_clearance=unmeasured");
    return s;
}

// ---------------------------------------------------------------------------
// Preheat scheduler
// ---------------------------------------------------------------------------

double motion_block_nominal_seconds(const MotionBlock& b)
{
    // Temperature waits and opaque macros contribute ZERO by contract. The
    // clock models commanded motion only; saying so is part of the contract.
    switch (b.kind) {
    case MotionBlock::Kind::TempWait:
    case MotionBlock::Kind::Macro:
        return 0.0;
    case MotionBlock::Kind::Dwell:
        return (std::isfinite(b.dwell_s) && b.dwell_s > 0.0) ? b.dwell_s : 0.0;
    case MotionBlock::Kind::Linear:
    case MotionBlock::Kind::Deposit:
        if (!std::isfinite(b.path_mm) || b.path_mm <= 0.0 || !std::isfinite(b.feed_mm_min) || b.feed_mm_min <= 0.0)
            return 0.0;
        return 60.0 * b.path_mm / b.feed_mm_min;
    case MotionBlock::Kind::Stationary:
        if (!std::isfinite(b.max_material_mm) || b.max_material_mm <= 0.0 || !std::isfinite(b.feed_mm_min) || b.feed_mm_min <= 0.0)
            return 0.0;
        return 60.0 * b.max_material_mm / b.feed_mm_min;
    }
    return 0.0;
}

double nominal_activation_seconds(const std::vector<MotionBlock>& blocks)
{
    double t = 0.0;
    for (const MotionBlock& b : blocks)
        t += motion_block_nominal_seconds(b);
    return t;
}

PreheatPlan plan_tool_preheat(const std::vector<MotionBlock>& outgoing_activation,
                              int incoming_tool,
                              int incoming_target_c,
                              double lead_s)
{
    PreheatPlan plan;
    plan.tool = incoming_tool;
    plan.target_c = incoming_target_c;
    plan.requested_lead_s = std::isfinite(lead_s) && lead_s > 0.0 ? lead_s : 0.0;

    if (incoming_target_c <= 0 || incoming_tool < 0) {
        plan.findings.push_back({FsCode::ThermalManagementDisabled,
                                 "no working target for the incoming head: thermal management is disabled for it"});
        return plan; // valid == false: emit no heat command
    }

    // Cumulative nominal time at the START of each block.
    std::vector<double> start_at(outgoing_activation.size(), 0.0);
    double t = 0.0;
    for (size_t i = 0; i < outgoing_activation.size(); ++i) {
        start_at[i] = t;
        t += motion_block_nominal_seconds(outgoing_activation[i]);
    }
    const double total = t;
    plan.endpoint_s = total;

    if (outgoing_activation.empty()) {
        // Nothing modelled in the outgoing activation (a purge shorter than the
        // lead is the spec's own case): the preheat goes right after the entry
        // setup, which is index 0, and the achieved lead is zero.
        plan.insert_index = 0;
        plan.nominal_lead_s = 0.0;
        plan.clamped_to_activation = true;
        plan.valid = true;
        plan.inserted = true;
        return plan;
    }

    double target_time = total - plan.requested_lead_s;
    if (target_time < 0.0) {
        target_time = 0.0;
        plan.clamped_to_activation = true;
    }

    // Insert before the first block that starts at or after the target time. A
    // target time at or beyond the last block's start has no later block to sit
    // before, so the insert lands at the END of the activation (index == size),
    // which is where a zero lead belongs: the target is still commanded, just
    // with no predictive head start.
    size_t idx = outgoing_activation.size();
    for (size_t i = 0; i < outgoing_activation.size(); ++i) {
        if (start_at[i] >= target_time - 1e-9) {
            idx = i;
            break;
        }
    }
    plan.insert_index = idx;
    plan.nominal_lead_s = idx < outgoing_activation.size() ? total - start_at[idx] : 0.0;
    plan.valid = true;
    plan.inserted = true;
    return plan;
}

std::string emit_tool_preheat(const PreheatPlan& plan)
{
    if (!plan.valid || !plan.inserted || plan.target_c <= 0)
        return std::string();
    char buf[128];
    std::snprintf(buf, sizeof(buf), "M104 S%d T%d ; FS_PREHEAT next_tool=%d lead_s=%.0f",
                  plan.target_c, plan.tool, plan.tool, plan.requested_lead_s);
    std::string s(buf);
    s += '\n';
    return s;
}

// ---------------------------------------------------------------------------
// E withdrawal ledger
// ---------------------------------------------------------------------------

void EWithdrawalLedger::withdraw(double mm)
{
    if (!std::isfinite(mm) || mm <= 0.0)
        return;
    m_pending = FiberRun::round3(m_pending + mm);
}

bool EWithdrawalLedger::covers(double travel_retract_mm) const
{
    return m_pending + 1e-9 >= travel_retract_mm;
}

double EWithdrawalLedger::recover()
{
    const double v = m_pending;
    m_pending = 0.0;
    return v;
}

// ---------------------------------------------------------------------------
// Evidence
// ---------------------------------------------------------------------------

namespace {

std::string json_escape(const std::string& in)
{
    std::string out;
    out.reserve(in.size() + 8);
    for (char c : in) {
        switch (c) {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n";  break;
        case '\r': out += "\\r";  break;
        case '\t': out += "\\t";  break;
        default:   out += c;      break;
        }
    }
    return out;
}

void kv_str(std::string& j, const char* key, const std::string& v, bool last)
{
    j += "    \"";
    j += key;
    j += "\": \"";
    j += json_escape(v);
    j += "\"";
    if (!last)
        j += ",";
    j += '\n';
}

void kv_num(std::string& j, const char* key, double v, bool last)
{
    j += "    \"";
    j += key;
    j += "\": " + fmt3("%.3f", v);
    if (!last)
        j += ",";
    j += '\n';
}

void kv_int(std::string& j, const char* key, long long v, bool last)
{
    j += "    \"";
    j += key;
    j += "\": " + std::to_string(v);
    if (!last)
        j += ",";
    j += '\n';
}

} // namespace

std::string evidence_to_json(const TailReleaseEvidence& e)
{
    std::string j;
    j += "{\n";
    kv_str(j, "object_id", e.object_id, false);
    kv_int(j, "copy_id", static_cast<long long>(e.copy_id), false);
    kv_int(j, "layer_index", static_cast<long long>(e.layer_index), false);
    kv_num(j, "physical_z_mm", e.physical_z, false);
    kv_str(j, "window_id", e.window_id, false);
    kv_str(j, "original_seam", e.original_seam, false);
    kv_str(j, "selected_seam", e.selected_seam, false);
    kv_num(j, "rotation_mm", e.rotation_mm, false);
    kv_num(j, "nominal_tail_mm", e.nominal_tail_mm, false);
    kv_num(j, "margin_mm", e.margin_mm, false);
    kv_num(j, "requested_release_mm", e.requested_release_mm, false);
    kv_num(j, "actual_release_mm", e.actual_release_mm, false);
    kv_num(j, "post_cut_deposit_mm", e.post_cut_deposit_mm, false);
    kv_num(j, "cut_x_mm", e.cut_xyz.x, false);
    kv_num(j, "cut_y_mm", e.cut_xyz.y, false);
    kv_num(j, "deposit_end_x_mm", e.deposit_end_xyz.x, false);
    kv_num(j, "deposit_end_y_mm", e.deposit_end_xyz.y, false);
    kv_num(j, "release_end_x_mm", e.release_end_xyz.x, false);
    kv_num(j, "release_end_y_mm", e.release_end_xyz.y, false);
    kv_num(j, "release_speed_mm_s", e.release_speed_mm_s, false);
    kv_num(j, "anchor_mm", e.anchor_mm, false);
    kv_num(j, "bead_width_mm", e.bead_width_mm, false);
    kv_str(j, "support_result", e.support_result, false);
    kv_num(j, "stationary_v_mm", e.stationary_v_mm, false);
    kv_num(j, "deposited_v_mm", e.deposited_v_mm, false);
    kv_int(j, "budget_L", static_cast<long long>(e.budget_L), false);
    kv_int(j, "post_cut_turns_gt_60", static_cast<long long>(e.post_cut_turns_gt_60), false);
    kv_num(j, "max_post_cut_turn_deg", e.max_post_cut_turn_deg, false);
    // The two honesty fields the spec pins. A planned release is never described
    // as measured clearance, and the macro contract stays unverified until a
    // real macro source revision is available to check against.
    kv_str(j, "physical_tail_clearance", e.physical_tail_clearance, false);
    kv_str(j, "macro_contract", e.macro_contract, true);
    j += "}";
    return j;
}

std::string evidence_manifest_to_json(const std::vector<TailReleaseEvidence>& records,
                                      const std::string& variant_name)
{
    std::string j;
    j += "{\n";
    kv_str(j, "variant", variant_name, false);
    kv_str(j, "clock_definition",
           "nominal: 60*path/modal_F for translating blocks, 60*max(|E|,|U|,|V|)/modal_F for "
           "stationary blocks, dwell duration for explicit dwells", false);
    kv_str(j, "clock_zero_contributors", "temperature waits and opaque macro calls contribute zero", false);
    kv_str(j, "clock_not_modelled",
           "acceleration, heater behaviour and hidden macro duration are not modelled", false);
    kv_str(j, "static_vs_physical",
           "this file is static export verification only; physical print results are UNTESTED until "
           "the operator supplies them", false);
    kv_int(j, "record_count", static_cast<long long>(records.size()), false);
    j += "  \"records\": [\n";
    for (size_t i = 0; i < records.size(); ++i) {
        j += "    " + evidence_to_json(records[i]);
        j += (i + 1 < records.size()) ? ",\n" : "\n";
    }
    j += "  ]\n";
    j += "}\n";
    return j;
}

// ---------------------------------------------------------------------------
// Exporter glue
// ---------------------------------------------------------------------------

bool apply_tail_margin(const FiberStrand& strand, double margin_mm,
                       FiberStrand& out, std::string* error)
{
    // No margin means no shift: hand back the strand as planned so an untouched
    // profile keeps byte-identical output.
    if (margin_mm <= 0.0) {
        out = strand;
        return true;
    }
    if (!std::isfinite(margin_mm)) {
        if (error)
            *error = "apply_tail_margin: margin must be finite";
        return false;
    }

    FiberStrand shifted = strand;
    shifted.tail_length_mm = strand.tail_length_mm - margin_mm;
    if (!(shifted.tail_length_mm > 0.0)) {
        if (error)
            *error = "apply_tail_margin: margin " + fmt3("%.3f", margin_mm) +
                     " mm exceeds the calibrated tail " + fmt3("%.3f", strand.tail_length_mm) +
                     " mm (S <= T + M leaves no body)";
        return false;
    }
    // Re-finalize the copy. finalize() clears the move lists on failure, so the
    // result is only published when it succeeds.
    std::string ferr;
    if (!shifted.finalize(&ferr)) {
        if (error)
            *error = "apply_tail_margin: shifted strand cannot finalize (" + ferr + ")";
        return false;
    }
    out = std::move(shifted);
    return true;
}

std::vector<MotionBlock> strand_activation_blocks(const FiberStrand& strand,
                                                  const FiberEmitParams& params,
                                                  const ReleasePlan* release,
                                                  const FiberPoint* from_xy)
{
    std::vector<MotionBlock> b;
    auto macro = [&b]() { MotionBlock m; m.kind = MotionBlock::Kind::Macro; b.push_back(m); };

    if (!strand.finalized)
        return b;

    // Comments and the window-open line: zero by contract.
    if (params.emit_layer_marker)
        macro();
    if (params.verbose_comments)
        macro();
    macro(); // M1001 L<budget>

    const double restart = FiberRun::round3(params.restart_feed_mm);
    const double z_hi    = strand.z + std::max(params.restart_z_mm, params.lift_z_mm);

    // Approach: Z to z_hi, then XY to the strand start. The emitter does not know
    // where the previous window closed, so the XY path length is genuinely
    // unknown here; it is recorded as Linear with path 0 (zero seconds) rather
    // than invented, and a caller that knows the incoming position can splice a
    // real block in front of this list.
    {
        MotionBlock z_up;
        z_up.kind = MotionBlock::Kind::Linear;
        z_up.path_mm = std::max(0.0, z_hi - strand.z);
        z_up.feed_mm_min = params.lift_f;
        b.push_back(z_up);
    }
    {
        MotionBlock xy;
        xy.kind = MotionBlock::Kind::Linear;
        // The emitter does not know where the previous window closed, so the
        // approach length is only knowable when the caller supplies the arrival
        // position. Without it the block is zero: under-estimated, never invented.
        xy.path_mm = from_xy ? std::hypot(strand.pts.front().x - from_xy->x,
                                          strand.pts.front().y - from_xy->y)
                             : 0.0;
        xy.feed_mm_min = params.lift_f;
        b.push_back(xy);
    }
    {
        MotionBlock u;
        u.kind = MotionBlock::Kind::Stationary;
        u.max_material_mm = restart;
        u.feed_mm_min = params.restart_feed_f;
        b.push_back(u);
    }
    {
        MotionBlock z_dn;
        z_dn.kind = MotionBlock::Kind::Linear;
        z_dn.path_mm = std::max(0.0, z_hi - strand.z);
        z_dn.feed_mm_min = params.lift_f;
        b.push_back(z_dn);
    }
    // Stationary V prime, split exactly as the emitter splits it: the recover of
    // the previous retract, then the extra anchor prime.
    {
        const double recover = FiberRun::round3(params.retract_v_mm);
        const double extra   = FiberRun::round3(std::max(0.0, params.prime_v_mm - params.retract_v_mm));
        MotionBlock v;
        v.kind = MotionBlock::Kind::Stationary;
        v.feed_mm_min = params.prime_f;
        v.max_material_mm = recover + extra;
        if (v.max_material_mm <= 0.0)
            v.max_material_mm = FiberRun::round3(params.prime_v_mm);
        b.push_back(v);
    }

    // Deposition, body then tail, on one continuous distance measure so the
    // three-zone ramp sees what the emitter sees.
    const double total = strand.total_path;
    double dist = 0.0;
    FiberPoint prev = strand.pts.front();
    for (size_t i = 0; i < strand.body_u.size(); ++i) {
        const FiberPoint& pe = strand.body_pts[i];
        MotionBlock d;
        d.kind = MotionBlock::Kind::Deposit;
        d.path_mm = std::hypot(pe.x - prev.x, pe.y - prev.y);
        d.feed_mm_min = zone_feed_mm_min(params, dist, total, strand.feed_mm_min);
        b.push_back(d);
        dist += d.path_mm;
        prev = pe;
    }

    macro(); // ; Start to cut
    macro(); // M2800
    macro(); // M400
    macro(); // ;CUT DISTANCE

    for (size_t i = 0; i < strand.tail_v.size(); ++i) {
        const FiberPoint& pe = strand.tail_pts[i];
        MotionBlock d;
        d.kind = MotionBlock::Kind::Deposit;
        d.path_mm = std::hypot(pe.x - prev.x, pe.y - prev.y);
        d.feed_mm_min = zone_feed_mm_min(params, dist, total, strand.feed_mm_min);
        b.push_back(d);
        dist += d.path_mm;
        prev = pe;
    }

    macro(); // ; Cutting completed.

    {
        MotionBlock v;
        v.kind = MotionBlock::Kind::Stationary;
        v.max_material_mm = FiberRun::round3(params.retract_v_mm);
        v.feed_mm_min = params.retract_f;
        b.push_back(v);
    }

    if (release != nullptr && release->valid) {
        for (size_t i = 0; i + 1 < release->path.size(); ++i) {
            MotionBlock r;
            r.kind = MotionBlock::Kind::Linear;
            r.path_mm = std::hypot(release->path[i + 1].x - release->path[i].x,
                                   release->path[i + 1].y - release->path[i].y);
            r.feed_mm_min = release->f_mm_min;
            b.push_back(r);
        }
    }

    {
        MotionBlock z_up;
        z_up.kind = MotionBlock::Kind::Linear;
        z_up.path_mm = std::max(0.0, params.lift_z_mm);
        z_up.feed_mm_min = params.lift_f;
        b.push_back(z_up);
    }
    macro(); // M1002

    return b;
}

std::vector<MotionBlock> blocks_from_gcode(const std::string& gcode,
                                           double initial_z,
                                           const FiberPoint* initial_xy,
                                           std::vector<size_t>* line_offsets)
{
    std::vector<MotionBlock> out;
    // Byte offset of the START of the line each block came from, so a caller can
    // splice a scheduled command in front of that line. Recorded for every block
    // kind, including Macros, so insert_index indexes the same list either way.
    auto push_block = [&](const MotionBlock& b, size_t off) {
        out.push_back(b);
        if (line_offsets)
            line_offsets->push_back(off);
    };

    // Modal feed, in mm/min, as G-code carries it: one F word applies to every
    // move until the next one.
    double modal_f = 0.0;
    // Previous XY position, and whether one is known at all.
    bool   have_prev = initial_xy != nullptr;
    double px = initial_xy ? initial_xy->x : 0.0;
    double py = initial_xy ? initial_xy->y : 0.0;
    // Previous Z: supplied by the caller when the text cannot carry it.
    bool   have_z = std::isfinite(initial_z);
    double z = have_z ? initial_z : 0.0;

    auto flush_line = [&](const std::string& line_in, size_t line_off) {
        // Strip a trailing comment; the clock reads commands, not prose.
        const size_t sc = line_in.find(';');
        std::string line = (sc == std::string::npos) ? line_in : line_in.substr(0, sc);
        // Upper-case copy so lowercase words still parse.
        for (char& c : line)
            c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));

        std::istringstream ss(line);
        std::string word;
        if (!(ss >> word))
            return; // blank

        const bool is_g1 = word == "G1" || word == "G0";
        const bool is_g4 = word == "G4";
        const bool is_m109 = word == "M109" || word == "M190" || word == "M191";

        if (!is_g1 && !is_g4 && !is_m109) {
            // Anything else the clock cannot model is opaque: M2800, M400, the
            // window markers, tool changes, fan and temperature sets.
            if (line.size() > 1) {
                MotionBlock m;
                m.kind = MotionBlock::Kind::Macro;
                push_block(m, line_off);
            }
            return;
        }

        double dx = 0.0, dy = 0.0, dz = 0.0;
        bool   has_x = false, has_y = false;
        double nx = 0.0, ny = 0.0;
        double mat = 0.0;      // max(|E|,|U|,|V|)
        bool   has_mat = false;
        bool   has_f = false;
        double f = 0.0;
        bool   has_p = false, has_s = false;
        double p_val = 0.0, s_val = 0.0;

        while (ss >> word) {
            if (word.size() < 2)
                continue;
            const char axis = word[0];
            const double val = std::atof(word.c_str() + 1);
            if (!std::isfinite(val))
                continue;
            switch (axis) {
            // A move's path is only knowable once the previous position is too.
            // The first XY of the slice has no known predecessor, so its length
            // is unknown and recorded as zero rather than invented from origin.
            case 'X': nx = val; has_x = true; break;
            case 'Y': ny = val; has_y = true; break;
            case 'Z': dz = val - (have_z ? z : val); z = val; have_z = true; break;
            case 'E': case 'U': case 'V':
                mat = std::max(mat, std::abs(val)); has_mat = true; break;
            case 'F': f = val; has_f = true; break;
            case 'P': p_val = val; has_p = true; break;
            case 'S': s_val = val; has_s = true; break;
            default: break;
            }
        }

        if (has_f && f > 0.0)
            modal_f = f;

        MotionBlock b;
        if (is_g4) {
            b.kind = MotionBlock::Kind::Dwell;
            // G4 P is milliseconds, S is seconds.
            b.dwell_s = has_p ? p_val / 1000.0 : (has_s ? s_val : 0.0);
            push_block(b, line_off);
            return;
        }
        if (is_m109) {
            b.kind = MotionBlock::Kind::TempWait; // zero by contract
            push_block(b, line_off);
            return;
        }

        const bool had_prev = have_prev;
        if (has_x && has_y) {
            // A move's path is only knowable once the previous position is too.
            // The first XY of the slice has no known predecessor, so its length
            // is recorded as zero rather than invented from origin.
            dx = had_prev ? (nx - px) : 0.0;
            dy = had_prev ? (ny - py) : 0.0;
            px = nx;
            py = ny;
            have_prev = true; // only a full XY pair ever establishes a position
        }
        const double xy_path = std::sqrt(dx * dx + dy * dy);
        const bool moved_xy = xy_path > 1e-9;
        const bool moved_z = std::abs(dz) > 1e-9;

        if (moved_xy && has_mat) {
            b.kind = MotionBlock::Kind::Deposit;
            b.path_mm = xy_path;
        } else if (moved_xy || moved_z) {
            b.kind = MotionBlock::Kind::Linear;
            b.path_mm = moved_xy ? xy_path : std::abs(dz);
        } else if (has_mat) {
            b.kind = MotionBlock::Kind::Stationary;
            b.max_material_mm = mat;
        } else {
            return; // a no-op line
        }
        b.feed_mm_min = modal_f;
        push_block(b, line_off);
    };

    // Walk the text by byte offset rather than with a stream, so every block
    // can carry the offset its command starts at. That is what lets the caller
    // splice a scheduled M104 into content that has already been emitted.
    size_t off = 0;
    for (;; ) {
        const size_t nl = gcode.find('\n', off);
        const size_t end = (nl == std::string::npos) ? gcode.size() : nl;
        std::string line = gcode.substr(off, end - off);
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        flush_line(line, off);
        if (nl == std::string::npos)
            break;
        off = nl + 1;
    }
    return out;
}


std::string schedule_preheat_into_plastic(const std::string& outgoing_plastic_gcode,
                                          int incoming_tool,
                                          int incoming_target_c,
                                          double lead_s,
                                          PreheatPlan& plan,
                                          double initial_z,
                                          const FiberPoint* initial_xy)
{
    std::vector<size_t> offsets;
    const std::vector<MotionBlock> blocks =
        blocks_from_gcode(outgoing_plastic_gcode, initial_z, initial_xy, &offsets);

    plan = plan_tool_preheat(blocks, incoming_tool, incoming_target_c, lead_s);
    if (!plan.valid || !plan.inserted)
        return outgoing_plastic_gcode;

    const std::string line = emit_tool_preheat(plan);
    if (line.empty())
        return outgoing_plastic_gcode;

    // insert_index == size() means the target landed at the very end of the
    // activation: the command goes after the last line rather than before it.
    if (plan.insert_index >= offsets.size()) {
        // Append after the final newline, keeping the text's own terminator.
        if (outgoing_plastic_gcode.empty())
            return line;
        const size_t last_nl = outgoing_plastic_gcode.rfind('\n');
        if (last_nl == std::string::npos)
            return outgoing_plastic_gcode + '\n' + line;
        return outgoing_plastic_gcode.substr(0, last_nl + 1) + line +
               outgoing_plastic_gcode.substr(last_nl + 1);
    }

    const size_t off = offsets[plan.insert_index];
    return outgoing_plastic_gcode.substr(0, off) + line +
           outgoing_plastic_gcode.substr(off);
}
} // namespace Fiber
} // namespace Slic3r
