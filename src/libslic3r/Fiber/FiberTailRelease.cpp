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

SupportVerdict PurgeCorridorSupport::query(const ReleaseSupportQuery& q) const
{
    if (q.path.size() < 2)
        return SupportVerdict::Unsupported;
    const double half = q.half_width_mm;
    const double tol  = q.tolerance_mm >= 0.0 ? q.tolerance_mm : 0.0;
    // Every footprint vertex must sit inside the validated corridor. The band is
    // widened by the tolerance and, at the rear, by nothing: the release starts at
    // the purge endpoint and only ever moves forward along +X.
    for (const FiberPoint& p : release_footprint(q.path, half)) {
        if (p.x < min_x - tol || p.x > max_x + tol ||
            p.y < min_y - tol || p.y > max_y + tol)
            return SupportVerdict::Unsupported;
    }
    return SupportVerdict::Supported;
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

    // What has to qualify, and why not "the opening run": the release retraces
    // the first R mm of the loop, and the blade cut plus the deposition that ends
    // at the strand endpoint sit immediately BEFORE the seam. The owner's rule is
    // that the final `anchor` mm of deposition AND the whole release lie on one
    // qualifying straight run, i.e. the contiguous interval
    //
    //     [total - anchor, total]  U  [0, R]
    //
    // which straddles the seam. The deterministic seam is placed R + 0.01 mm
    // before its run's forward end precisely so that this straddle interval fits
    // inside that run; requiring the run to START at the seam would reject the
    // very seam the seam policy exists to choose.
    //
    // The straddle is tested directly, by walking the path backwards `anchor` mm
    // and forwards `R` mm from the seam and measuring the heading spread and
    // chord deviation of that one contiguous polyline. find_straight_runs() is
    // not used here because it reports a run that wraps the closure point as two
    // pieces, which would reject a seam that is genuinely straight across it.
    const double need = p.anchor_mm + p.release_mm + 0.02;
    const double total = strand.total_path;
    bool qualified = false;
    {
        // Collect the straddle vertices: the tail of the path (last anchor mm)
        // followed by the head of the path (first R mm), seam vertex shared.
        std::vector<FiberPoint> band;
        // Backwards from the seam (= path end for a closed strand) over anchor mm.
        {
            double back = p.anchor_mm;
            band.push_back(strand.pts.front()); // seam == front == back
            for (size_t i = strand.pts.size() - 1; i > 0 && back > 1e-9; --i) {
                const FiberPoint& a = strand.pts[i - 1];
                const FiberPoint& b = strand.pts[i];
                const double len = seg_len(a, b);
                if (len <= 1e-9)
                    continue;
                if (len <= back + 1e-9) {
                    band.insert(band.begin(), a);
                    back -= len;
                } else {
                    const double t = back / len;
                    FiberPoint q{b.x + (a.x - b.x) * t, b.y + (a.y - b.y) * t};
                    band.insert(band.begin(), q);
                    back = 0.0;
                }
            }
        }
        // Forwards from the seam over R mm (skip pts[0] == seam, already in band).
        {
            double fwd = p.release_mm;
            for (size_t i = 0; i + 1 < strand.pts.size() && fwd > 1e-9; ++i) {
                const FiberPoint& a = strand.pts[i];
                const FiberPoint& b = strand.pts[i + 1];
                const double len = seg_len(a, b);
                if (len <= 1e-9)
                    continue;
                if (len <= fwd + 1e-9) {
                    band.push_back(b);
                    fwd -= len;
                } else {
                    const double t = fwd / len;
                    FiberPoint q{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
                    band.push_back(q);
                    fwd = 0.0;
                }
            }
        }
        // Measure the band: total length, cumulative heading spread, max
        // perpendicular deviation from the band chord.
        double band_len = 0.0, spread_deg = 0.0, dev = 0.0;
        for (size_t i = 0; i + 1 < band.size(); ++i)
            band_len += seg_len(band[i], band[i + 1]);
        for (size_t i = 0; i + 2 < band.size(); ++i) {
            const double h0 = std::atan2(band[i + 1].y - band[i].y, band[i + 1].x - band[i].x);
            const double h1 = std::atan2(band[i + 2].y - band[i + 1].y, band[i + 2].x - band[i + 1].x);
            spread_deg += std::fabs(rad2deg(wrap_pi(h1 - h0)));
        }
        if (band.size() >= 2) {
            const FiberPoint& a = band.front();
            const FiberPoint& b = band.back();
            const double cl = seg_len(a, b);
            for (const FiberPoint& q : band) {
                double d;
                if (cl <= 1e-9)
                    d = seg_len(a, q);
                else
                    d = std::fabs((b.y - a.y) * q.x - (b.x - a.x) * q.y +
                                  b.x * a.y - a.x * b.y) / cl;
                if (d > dev)
                    dev = d;
            }
        }
        qualified = (band_len >= p.anchor_mm + p.release_mm - 1e-6) &&
                    (spread_deg <= 2.0 + 1e-6) && (dev <= 0.05 + 1e-6);
        if (!qualified)
            out.findings.push_back({FsCode::ReleaseUnsupported,
                                    "seam straddle is " + fmt3("%.3f", band_len) + " mm with spread " +
                                    fmt3("%.3f", spread_deg) + " deg and deviation " +
                                    fmt3("%.3f", dev) + " mm, short of " +
                                    fmt3("%.3f", p.anchor_mm + p.release_mm) + " mm / 2 deg / 0.05 mm"});
    }
    if (!qualified) {
        if (error)
            *error = "FS_RELEASE_UNSUPPORTED: no straight run of " + fmt3("%.3f", need) +
                     " mm carries the anchor and the release across the seam";
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
    // The margin ADDS to the nominal tail. finalize() cuts at
    // total_path - tail_length_mm, so a LONGER tail cuts EARLIER, which is what
    // the spec requires: the severed tail becomes T+M of post-cut deposition
    // while the strand endpoint - and therefore the part - is untouched.
    //
    // Subtracting here is the inverted-sign defect this replaces: it moved the
    // blade M mm LATER, made the post-cut deposition T-M, and so variant C
    // measured 53.8 mm where the spec demands 55.8 mm. The purge table pins the
    // direction independently: cut X 35.2 at M=0 and 34.2 at M=1, i.e. the cut
    // moves toward the deposition START as M grows.
    shifted.tail_length_mm = strand.tail_length_mm + margin_mm;
    if (!(shifted.tail_length_mm > 0.0)) {
        if (error)
            *error = "apply_tail_margin: margin " + fmt3("%.3f", margin_mm) +
                     " mm is not a positive finite length";
        return false;
    }
    // The strand must still carry a body before the cut: S > T + M. finalize()
    // rejects the strand outright in that case, and naming the cause here is what
    // lets the caller report an unprintable feature instead of silently
    // shortening the tail.
    if (!(shifted.tail_length_mm < strand.total_path) && strand.total_path > 0.0) {
        if (error)
            *error = "apply_tail_margin: margin " + fmt3("%.3f", margin_mm) +
                     " mm leaves no body before the cut (S <= T + M)";
        return false;
    }    // Re-finalize the copy. finalize() clears the move lists on failure, so the
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
// ---------------------------------------------------------------------------
// Whole-file predictive-preheat pass
// ---------------------------------------------------------------------------
//
// Why this exists: the per-layer scheduler can only see one layer of outgoing
// material, and the per-window fibre scheduler cannot see the entry moves the
// acceptance clock counts. Both therefore under-lead transitions whose outgoing
// activation spans more than the text they were handed. This pass runs over the
// FINAL exported bytes, reconstructs the real activation between physical
// switches, and re-places each marked preheat to the position the deterministic
// clock says is min(lead, available) seconds before the outgoing endpoint. It
// mirrors tools/verify_fs_shook.py line for line so the schedule it writes is
// the schedule the verifier measures.

namespace {

struct PassLine {
    std::string raw;
    std::string cmd;
    std::string comment;
    std::map<char, double> w;
    double x = std::nan(""), y = std::nan(""), z = std::nan(""), f = std::nan("");
    int tool = -1;
    bool station = false;
    double e = 0.0, u = 0.0, v = 0.0;
};

bool pass_is_move(const std::string& cmd) { return cmd == "G0" || cmd == "G1"; }

bool pass_is_t(const std::string& cmd, int& tool)
{
    if (cmd.size() < 2 || cmd[0] != 'T')
        return false;
    for (size_t i = 1; i < cmd.size(); ++i)
        if (!std::isdigit(static_cast<unsigned char>(cmd[i])))
            return false;
    tool = std::atoi(cmd.c_str() + 1);
    return true;
}

// Tokenize one line the way the verifier does: first whitespace token is the
// command (uppercased), the rest are single-letter words with numeric values.
void pass_tokenize(const std::string& line, std::string& cmd,
                   std::map<char, double>& w, std::string& comment)
{
    const size_t semi = line.find(';');
    const std::string body = semi == std::string::npos ? line : line.substr(0, semi);
    if (semi != std::string::npos)
        comment = line.substr(semi + 1);
    std::istringstream ss(body);
    std::string tok;
    bool first = true;
    while (ss >> tok) {
        if (first) {
            cmd = tok;
            for (auto& ch : cmd)
                ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
            first = false;
            continue;
        }
        if (tok.empty() || !std::isalpha(static_cast<unsigned char>(tok[0])))
            continue;
        try {
            size_t used = 0;
            const double val = std::stod(tok.substr(1), &used);
            if (used == tok.size() - 1)
                w[static_cast<char>(std::toupper(static_cast<unsigned char>(tok[0])))] = val;
        } catch (...) {}
    }
}

std::vector<PassLine> pass_parse(const std::string& text)
{
    std::vector<PassLine> lines;
    std::istringstream ss(text);
    std::string raw;
    double x = std::nan(""), y = std::nan(""), z = std::nan(""), f = std::nan("");
    int tool = -1;
    bool station = false;
    while (std::getline(ss, raw)) {
        if (!raw.empty() && raw.back() == '\r')
            raw.pop_back();
        PassLine c;
        c.raw = raw;
        pass_tokenize(raw, c.cmd, c.w, c.comment);
        if (pass_is_move(c.cmd)) {
            auto it = c.w.find('F'); if (it != c.w.end()) f = it->second;
            it = c.w.find('X'); if (it != c.w.end()) x = it->second;
            it = c.w.find('Y'); if (it != c.w.end()) y = it->second;
            it = c.w.find('Z'); if (it != c.w.end()) z = it->second;
            c.e = c.w.count('E') ? c.w['E'] : 0.0;
            c.u = c.w.count('U') ? c.w['U'] : 0.0;
            c.v = c.w.count('V') ? c.w['V'] : 0.0;
        } else if (c.cmd == "MOVE_TO_BRUSH_STATION") {
            station = true;
        } else if (c.cmd == "MOVE_OUT_BRUSH_STATION") {
            station = false;
        } else if (c.cmd == "G92" && c.w.count('X') && c.w.count('Y')) {
            x = c.w['X']; y = c.w['Y'];
        } else {
            int t;
            if (pass_is_t(c.cmd, t))
                tool = t;
        }
        c.x = x; c.y = y; c.z = z; c.f = f;
        c.tool = tool; c.station = station;
        lines.push_back(std::move(c));
    }
    return lines;
}

// Nominal seconds over [a, b) exactly as the verifier's clock: translating
// blocks 60*dXYZ/F, stationary 60*max(|E|,|U|,|V|)/F, G4 its P word, everything
// else zero. The first motion block seeds the position and counts zero. Returns
// false when a translating block has no resolvable feed (an error, not zero).
bool pass_clock(const std::vector<PassLine>& lines, size_t a, size_t b,
                double& seconds, bool& feed_error)
{
    seconds = 0.0;
    feed_error = false;
    // Seed from the modal position the line BEFORE the range established, so the
    // first motion block after a non-motion command is charged its real travel
    // time (the spec clock charges every translating block). Falls back to
    // zeroing the first block only when no prior position is known.
    bool have_prev = false;
    double px = 0.0, py = 0.0, pz = 0.0;
    if (a > 0 && std::isfinite(lines[a - 1].x)) {
        px = lines[a - 1].x; py = lines[a - 1].y; pz = lines[a - 1].z;
        have_prev = true;
    }
    for (size_t i = a; i < b; ++i) {
        const PassLine& c = lines[i];
        if (c.cmd == "G4") {
            auto it = c.w.find('P');
            if (it == c.w.end()) it = c.w.find('S');
            if (it != c.w.end()) seconds += it->second;
            continue;
        }
        if (!pass_is_move(c.cmd))
            continue;
        if (!std::isfinite(c.f) || c.f <= 0.0) {
            feed_error = true;
            continue;
        }
        if (!have_prev) {
            px = c.x; py = c.y; pz = c.z;
            have_prev = true;
            continue;
        }
        const double d = std::sqrt((c.x - px) * (c.x - px) +
                                   (c.y - py) * (c.y - py) +
                                   (c.z - pz) * (c.z - pz));
        const double mat = std::max(std::abs(c.e), std::max(std::abs(c.u), std::abs(c.v)));
        if (d > 1e-9)
            seconds += 60.0 * d / c.f;
        else if (mat > 1e-9)
            seconds += 60.0 * mat / c.f;
        px = c.x; py = c.y; pz = c.z;
    }
    return true;
}

} // anonymous namespace

// The time a single motion block contributes on its own, using the modal
// position of the preceding line as its start. The range clock above zeroes the
// first block it is handed (it seeds the position there); placement needs the
// block's own duration, so it is computed separately here.
double pass_block_seconds(const std::vector<PassLine>& lines, size_t i, bool& feed_error)
{
    feed_error = false;
    const PassLine& c = lines[i];
    if (c.cmd == "G4") {
        auto it = c.w.find('P');
        if (it == c.w.end()) it = c.w.find('S');
        return it == c.w.end() ? 0.0 : it->second;
    }
    if (!pass_is_move(c.cmd))
        return 0.0;
    if (!std::isfinite(c.f) || c.f <= 0.0) {
        feed_error = true;
        return 0.0;
    }
    const double px = i > 0 ? lines[i - 1].x : c.x;
    const double py = i > 0 ? lines[i - 1].y : c.y;
    const double pz = i > 0 ? lines[i - 1].z : c.z;
    const double d = std::sqrt((c.x - px) * (c.x - px) +
                               (c.y - py) * (c.y - py) +
                               (c.z - pz) * (c.z - pz));
    const double mat = std::max(std::abs(c.e), std::max(std::abs(c.u), std::abs(c.v)));
    if (d > 1e-9)
        return 60.0 * d / c.f;
    if (mat > 1e-9)
        return 60.0 * mat / c.f;
    return 0.0;
}

// Predict the time the verifier's clock will charge to the TAIL half of a split
// block, using the same rounding the renderer applies (XYZ to 2 decimals, the
// extrusion axes to 3). Returns false when the block cannot be split.
bool pass_split_predict(const std::vector<PassLine>& lines, size_t i, double want_head,
                        double& predicted_tail_seconds)
{
    predicted_tail_seconds = -1.0;
    const PassLine& c = lines[i];
    if (c.cmd == "G4") {
        auto it = c.w.find('P');
        if (it == c.w.end()) it = c.w.find('S');
        if (it == c.w.end())
            return false;
        if (want_head <= 0.0 || want_head >= it->second)
            return false;
        predicted_tail_seconds = it->second - std::round(want_head * 1000.0) / 1000.0;
        return true;
    }
    if (!pass_is_move(c.cmd) || !std::isfinite(c.f) || c.f <= 0.0)
        return false;
    const double px = i > 0 ? lines[i - 1].x : c.x;
    const double py = i > 0 ? lines[i - 1].y : c.y;
    const double pz = i > 0 ? lines[i - 1].z : c.z;
    const double dx = c.x - px, dy = c.y - py, dz = c.z - pz;
    const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double mat = std::max(std::abs(c.e), std::max(std::abs(c.u), std::abs(c.v)));
    const double block_s = d > 1e-9 ? 60.0 * d / c.f : (mat > 1e-9 ? 60.0 * mat / c.f : 0.0);
    if (block_s <= 0.0 || want_head <= 0.0 || want_head >= block_s)
        return false;
    if (d > 1e-9) {
        // Head endpoint rounded the way the renderer rounds it: the head runs
        // from the established position to that rounded point, which is exactly
        // what the verifier's seeded clock will charge it.
        const double frac = want_head / block_s;
        const double hx = std::round((px + dx * frac) * 100.0) / 100.0;
        const double hy = std::round((py + dy * frac) * 100.0) / 100.0;
        const double hz = std::round((pz + dz * frac) * 100.0) / 100.0;
        // Measure the tail from the ROUNDED head endpoint to the untouched
        // original endpoint: that is what the seeded clock charges after the
        // preheat line, which is the number the acceptance check compares.
        const double dt = std::sqrt((c.x - hx) * (c.x - hx) + (c.y - hy) * (c.y - hy) +
                                    (c.z - hz) * (c.z - hz));
        if (dt < 0.005)
            return false;
        predicted_tail_seconds = 60.0 * dt / c.f;
    } else {
        const double v_head = std::round(mat * (want_head / block_s) * 1000.0) / 1000.0;
        if (std::abs(v_head) < 0.0005 || std::abs(v_head) > std::abs(mat) - 0.0005)
            return false;
        predicted_tail_seconds = 60.0 * (std::abs(mat) - std::abs(v_head)) / c.f;
    }
    return predicted_tail_seconds > 0.0;
}


// Split one motion block so its head carries exactly head_seconds, and render
// the two halves. Geometry and total extrusion are preserved: the tail keeps the
// original endpoint verbatim, the head takes the exact remainder, and the
// extrusion words are split so head + tail sums to the original value. Returns
// false when the block cannot be split (dwell, or a head that would round away).
bool pass_split_block(const std::vector<PassLine>& lines, size_t i,
                      double head_seconds, std::string& head_out,
                      std::string& tail_out, const std::string& eol)
{
    const PassLine& c = lines[i];
    if (c.cmd == "G4") {
        auto it = c.w.find('P');
        const bool used_p = it != c.w.end();
        if (!used_p) it = c.w.find('S');
        if (it == c.w.end())
            return false;
        const double total = it->second;
        if (head_seconds <= 0.0 || head_seconds >= total)
            return false;
        char buf[96];
        std::snprintf(buf, sizeof(buf), "G4 P%.3f", head_seconds);
        head_out = buf;
        std::snprintf(buf, sizeof(buf), "G4 P%.3f", total - head_seconds);
        tail_out = buf;
        return true;
    }
    if (!pass_is_move(c.cmd) || !std::isfinite(c.f) || c.f <= 0.0)
        return false;

    const double px = i > 0 ? lines[i - 1].x : c.x;
    const double py = i > 0 ? lines[i - 1].y : c.y;
    const double pz = i > 0 ? lines[i - 1].z : c.z;
    const double dx = c.x - px, dy = c.y - py, dz = c.z - pz;
    const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double mat = std::max(std::abs(c.e), std::max(std::abs(c.u), std::abs(c.v)));
    const double block_s = d > 1e-9 ? 60.0 * d / c.f : (mat > 1e-9 ? 60.0 * mat / c.f : 0.0);
    if (block_s <= 0.0 || head_seconds <= 0.0 || head_seconds >= block_s)
        return false;
    const double frac = head_seconds / block_s;   // head share of the block

    char buf[192];
    std::string head, tail;
    if (d > 1e-9) {
        // Translating: split along the segment. The tail keeps the original
        // endpoint exactly, so the polyline through the pair is unchanged.
        const double hx = px + dx * frac;
        const double hy = py + dy * frac;
        const double hz = pz + dz * frac;
        if (std::abs(hx - px) < 0.005 && std::abs(hy - py) < 0.005 && std::abs(hz - pz) < 0.005)
            return false;   // head would round away: let the caller use whole-line placement
        std::snprintf(buf, sizeof(buf), "G1 X%.2f Y%.2f", hx, hy);
        head = buf;
        if (std::abs(dz) > 1e-9) {
            std::snprintf(buf, sizeof(buf), " Z%.2f", hz);
            head += buf;
        }
        tail = "G1";
        if (c.w.count('X')) { std::snprintf(buf, sizeof(buf), " X%.2f", c.x); tail += buf; }
        if (c.w.count('Y')) { std::snprintf(buf, sizeof(buf), " Y%.2f", c.y); tail += buf; }
        if (c.w.count('Z')) { std::snprintf(buf, sizeof(buf), " Z%.2f", c.z); tail += buf; }
    } else {
        head = "G1";
        tail = "G1";
    }

    // Extrusion words: the tail takes its proportional share rounded to the
    // file's three decimals, the head carries the exact remainder, so the pair
    // sums to the original value with no drift.
    struct Ax { char letter; double total; bool present; };
    const Ax axes[3] = {{'E', c.e, c.w.count('E') > 0},
                        {'U', c.u, c.w.count('U') > 0},
                        {'V', c.v, c.w.count('V') > 0}};
    for (const Ax& a : axes) {
        if (!a.present || std::abs(a.total) < 1e-12)
            continue;
        const double rr = std::round(a.total * frac * 1000.0) / 1000.0;
        const double r1 = std::abs(rr) < 0.0005 ? 0.0 : rr;   // head share
        const double r2 = a.total - r1;                        // exact remainder
        std::snprintf(buf, sizeof(buf), " %c%.3f", a.letter, r1);
        head += buf;
        std::snprintf(buf, sizeof(buf), " %c%.3f", a.letter, r2);
        tail += buf;
    }
    // Any other axis word (P carries the pressure-advance preamble) stays on the
    // tail, which keeps the tail's own semantics intact.
    for (const auto& kv : c.w) {
        if (kv.first == 'X' || kv.first == 'Y' || kv.first == 'Z' || kv.first == 'F' ||
            kv.first == 'E' || kv.first == 'U' || kv.first == 'V')
            continue;
        std::snprintf(buf, sizeof(buf), " %c%.3f", kv.first, kv.second);
        tail += buf;
    }
    std::snprintf(buf, sizeof(buf), " F%.0f", c.f);
    head += buf;
    tail += buf;

    head_out = head + eol;
    tail_out = tail + eol;
    return true;
}


std::string apply_preheat_schedule_pass(const std::string& gcode,
                                        double lead_s,
                                        bool& changed,
                                        std::string& report)
{
    changed = false;
    if (!std::isfinite(lead_s) || lead_s <= 0.0)
        return gcode;

    const std::vector<PassLine> lines = pass_parse(gcode);
    if (lines.empty())
        return gcode;

    // Station brackets from the actual macros, not comments.
    std::vector<std::pair<size_t, size_t>> brackets;
    {
        long long open_at = -1;
        for (size_t i = 0; i < lines.size(); ++i) {
            if (lines[i].cmd == "MOVE_TO_BRUSH_STATION" && open_at < 0)
                open_at = static_cast<long long>(i);
            else if (lines[i].cmd == "MOVE_OUT_BRUSH_STATION" && open_at >= 0) {
                brackets.emplace_back(static_cast<size_t>(open_at), i);
                open_at = -1;
            }
        }
    }

    // Physical transitions: bare T words whose previous tool differs.
    struct Trans { size_t idx; int frm; int to; };
    std::vector<Trans> trans;
    {
        int prev = -1;
        for (size_t i = 0; i < lines.size(); ++i) {
            int t;
            if (!pass_is_t(lines[i].cmd, t))
                continue;
            if (prev >= 0 && prev != t)
                trans.push_back({i, prev, t});
            prev = t;
        }
    }

    std::vector<bool> del(lines.size(), false);
    std::map<size_t, std::string> ins;   // insert this line BEFORE index
    std::string pending;                 // trailing insert (index == size)
    const std::string eol = gcode.find("\r\n") != std::string::npos ? "\r\n" : "\n";

    for (size_t k = 1; k < trans.size(); ++k) {   // k==0: startup, S07 owns it
        const Trans& tr = trans[k];
        const size_t switch_idx = tr.idx;
        const int in_head = tr.to;

        // The visit that pays this switch's wait: the last bracket ending at or
        // just before the switch (fan clause and mode resets sit between).
        const Trans& prev_tr = trans[k - 1];
        size_t best_end = size_t(-1);
        size_t entry_idx = size_t(-1);
        for (const auto& br : brackets) {
            if (br.first < switch_idx && switch_idx <= br.second + 12 &&
                (best_end == size_t(-1) || br.second > best_end)) {
                best_end = br.second;
                entry_idx = br.first;
            }
        }
        if (entry_idx == size_t(-1))
            continue;   // no visit: S09 reports it; the pass cannot fix what has no station wait

        // Timing endpoint: end of the outgoing deposition plus release.
        size_t end_idx = size_t(-1);
        if (tr.frm == 0) {
            for (size_t i = entry_idx; i-- > 0;) {
                if (lines[i].cmd == "M1002") { end_idx = i; break; }
                if (i == 0) break;
            }
        } else {
            const size_t floor_i = entry_idx > 400 ? entry_idx - 400 : 0;
            for (size_t i = entry_idx; i >= floor_i; --i) {
                if (pass_is_move(lines[i].cmd) && lines[i].e > 1e-6) { end_idx = i; break; }
                if (i == 0) break;
            }
        }
        if (end_idx == size_t(-1) || end_idx <= prev_tr.idx)
            continue;

        const size_t start_idx = prev_tr.idx + 1;
        double avail = 0.0; bool ferr = false;
        pass_clock(lines, start_idx, end_idx, avail, ferr);
        if (ferr)
            continue;
        const double required = std::min(lead_s, avail);

        // Existing marked preheat for the incoming head in this activation.
        long long marker = -1;
        for (size_t i = start_idx; i <= switch_idx; ++i) {
            const PassLine& c = lines[i];
            if (c.cmd == "M104" && c.comment.find("FS_PREHEAT") != std::string::npos &&
                c.w.count('T') && static_cast<int>(c.w.at('T')) == in_head)
                marker = static_cast<long long>(i);
        }

        // Target temperature: the blocking wait inside the visit is authoritative.
        double target = 0.0;
        long long wait_idx = -1;
        for (size_t i = entry_idx; i <= best_end; ++i) {
            const PassLine& c = lines[i];
            if (c.cmd == "M109" && c.w.count('T') &&
                static_cast<int>(c.w.at('T')) == in_head) {
                target = c.w.count('S') ? c.w.at('S') : 0.0;
                wait_idx = static_cast<long long>(i);
                break;
            }
        }
        if (target <= 0.0)
            continue;

        char line_buf[160];
        std::snprintf(line_buf, sizeof(line_buf),
                      "M104 S%d T%d ; FS_PREHEAT next_tool=%d lead_s=%.0f",
                      static_cast<int>(target), in_head, in_head, lead_s);
        const std::string preheat_line(line_buf);

        double measured = -1.0;
        if (marker >= 0)
            pass_clock(lines, static_cast<size_t>(marker), end_idx, measured, ferr);

        size_t place = size_t(-1);
        size_t split_idx = size_t(-1);
        double split_tail = 0.0;
        if (marker < 0 || measured < 0.0 || std::abs(measured - required) > 0.05) {
            // Walk backwards block by block. Inserting before a block zeroes that
            // block in the verifier's clock (it seeds the position on the first
            // move of the range), so the lead achievable at block i is exactly the
            // time accumulated AFTER it. When the required lead falls INSIDE a
            // block, whole-line placement cannot reach it: spec section 4 item 4
            // then requires splitting that block without changing geometry or
            // total extrusion, which pass_split_block() does.
            // The lead achievable by inserting BEFORE line i is exactly what the
            // verifier's range clock charges to [i, end): it zeroes the first
            // motion block of the range, so a plain boundary's lead is NOT the sum
            // of the block times after it. Plain boundaries are tried first; a
            // split is taken only when it predicts a smaller error, and its
            // prediction is computed from the ROUNDED head endpoint so it is what
            // the verifier will actually measure.
            double best_lead = 0.0;
            {
                bool fe0 = false;
                pass_clock(lines, end_idx, end_idx, best_lead, fe0);
            }
            double best_err = best_lead - required;
            if (best_err < 0.0) best_err = -best_err;
            place = end_idx;
            for (size_t i = end_idx; i-- > start_idx;) {
                double lead = 0.0; bool fe2 = false;
                pass_clock(lines, i, end_idx, lead, fe2);
                const double err = std::abs(lead - required);
                if (err < best_err - 1e-12) {
                    best_err = err;
                    place = i;
                    split_idx = size_t(-1);
                }
                if (best_err <= 0.02)
                    break;   // already inside the acceptance gate with room to spare
                if (lead > required) {
                    // The target sits inside this block. Inserting before it is
                    // worth 'lead', after it only 'after', so split it and put the
                    // command between the halves: the tail then carries exactly
                    // required - after, which means the head carries the block's
                    // own time minus that shortfall.
                    double after = 0.0; bool fe3 = false;
                    pass_clock(lines, i + 1, end_idx, after, fe3);
                    double t_self = 0.0; bool fe4 = false;
                    pass_clock(lines, i, i + 1, t_self, fe4);
                    const double want_tail = required - after;
                    const double want_head = t_self - want_tail;
                    double pred_tail = -1.0;
                    if (want_head > 0.0 && want_tail > 0.0 &&
                        pass_split_predict(lines, i, want_head, pred_tail)) {
                        const double serr = std::abs(after + pred_tail - required);
                        if (serr < best_err - 1e-12) {
                            best_err = serr;
                            place = i;
                            split_idx = i;
                            split_tail = want_head;   // renderer takes the HEAD share
                        }
                    }
                    break;   // further back only lengthens the lead
                }
            }
            if (marker >= 0 && split_idx == size_t(-1) && static_cast<size_t>(marker) == place) {
                place = size_t(-1);      // already exactly there
                split_idx = size_t(-1);
            }
        }

        if (place != size_t(-1)) {
            if (marker >= 0)
                del[static_cast<size_t>(marker)] = true;
            std::string payload = preheat_line + eol;
            if (split_idx != size_t(-1)) {
                std::string head, tail;
                if (pass_split_block(lines, split_idx, split_tail, head, tail, eol)) {
                    del[split_idx] = true;
                    // The command belongs BETWEEN the halves: before the head it
                    // would still pay for the whole block.
                    payload = head + preheat_line + eol + tail;
                } else {
                    split_idx = size_t(-1);   // unspittable: plain insertion
                }
            }
            if (place >= lines.size())
                pending += payload;
            else
                ins[place] += payload;
            measured = -1.0;   // recomputed below for the report
        }

        // Clobber guard: no standby drop for the incoming head between the
        // scheduled preheat and its wait. The both-heads clause is still met by
        // the working pre-charge that follows the deleted standby line.
        const size_t ph_pos = place != size_t(-1) ? place
                            : (marker >= 0 ? static_cast<size_t>(marker) : size_t(-1));
        if (ph_pos != size_t(-1) && wait_idx >= 0) {
            for (size_t i = ph_pos + 1; i < static_cast<size_t>(wait_idx); ++i) {
                const PassLine& c = lines[i];
                if (c.cmd == "M104" && c.w.count('T') &&
                    static_cast<int>(c.w.at('T')) == in_head &&
                    c.w.count('S') && c.w.at('S') < target - 1e-9 &&
                    c.comment.find("standby") != std::string::npos)
                    del[i] = true;
            }
        }

        // Report line.
        double final_meas = 0.0; bool e3 = false;
        const size_t report_pos = place != size_t(-1) ? place
                                : (marker >= 0 ? static_cast<size_t>(marker) : start_idx);
        pass_clock(lines, report_pos, end_idx, final_meas, e3);
        char rep[256];
        std::snprintf(rep, sizeof(rep),
                      "  T%d->T%d switch=%zu preheat=%s required=%.2f avail=%.2f measured=%.2f %s\n",
                      tr.frm, tr.to, switch_idx + 1,
                      place != size_t(-1) ? "moved" : (marker >= 0 ? "kept" : "missing"),
                      required, avail, final_meas,
                      std::abs(final_meas - required) <= 0.10 ? "ok" : "OFF-SCHEDULE");
        report += rep;
    }

    // Rebuild the text.
    std::string out;
    out.reserve(gcode.size() + 1024);
    for (size_t i = 0; i < lines.size(); ++i) {
        auto it = ins.find(i);
        if (it != ins.end())
            out += it->second;
        if (del[i])
            continue;   // a split block is deleted but its replacement rode in with the insert
        out += lines[i].raw;
        out += eol;
    }
    out += pending;

    changed = !ins.empty() || !pending.empty();
    for (size_t i = 0; i < lines.size(); ++i)
        if (del[i]) { changed = true; break; }

    return changed ? out : gcode;
}

bool apply_preheat_schedule_file(const std::string& path,
                                 double lead_s,
                                 bool& changed,
                                 std::string& report,
                                 std::string* error)
{
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        if (error) *error = "cannot open " + path;
        return false;
    }
    std::ostringstream buf;
    buf << in.rdbuf();
    in.close();
    const std::string text = buf.str();
    const std::string out = apply_preheat_schedule_pass(text, lead_s, changed, report);
    if (!changed)
        return true;
    std::ofstream o(path, std::ios::binary | std::ios::trunc);
    if (!o) {
        if (error) *error = "cannot rewrite " + path;
        return false;
    }
    o << out;
    o.close();
    return true;
}
} // namespace Fiber
} // namespace Slic3r
