// License: GNU AGPLv3 or higher

#include "FiberEmitter.hpp"

#include <cmath>
#include <cstdio>
#include <algorithm>

namespace Slic3r {
namespace Fiber {

namespace {

// All emitted material values go through the run's printed precision so the
// g-code itself remains the single source of truth for window accounting.
std::string fmt(const char* f, double v)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), f, v);
    return std::string(buf);
}

// Shared failure shape for the emitters: report, clear the output, reject.
bool fail_into(std::string& out, std::string* error, const std::string& msg)
{
    if (error) {
        *error = msg;
    }
    out.clear();
    return false;
}
} // namespace

double zone_feed_mm_min(const FiberEmitParams& params, double dist_mm, double total_mm, double fallback_f)
{
    const double start  = params.start_speed_mm_s;
    const double finish = params.finish_speed_mm_s;
    // The start zone wins where the two overlap on a strand shorter than both:
    // a freshly restarted strand has to be anchored before anything else.
    if (start > 0.0 && params.start_length_mm > 0.0 && dist_mm < params.start_length_mm)
        return start * 60.0;
    if (finish > 0.0 && params.finish_length_mm > 0.0 && dist_mm >= total_mm - params.finish_length_mm)
        return finish * 60.0;
    return params.normal_speed_mm_s > 0.0 ? params.normal_speed_mm_s * 60.0 : fallback_f;
}

bool emit_strand(const FiberStrand& strand, const FiberEmitParams& params, std::string& out, std::string* error)
{
    auto fail = [error, &out](const char* msg) {
        if (error) {
            *error = msg;
        }
        out.clear();
        return false;
    };

    if (!strand.finalized) {
        return fail("emit_strand: strand must be finalized before emission");
    }
    if (strand.body_pts.empty() || strand.body_pts.size() != strand.body_u.size()) {
        return fail("emit_strand: finalized body move list is empty or inconsistent");
    }
    if (strand.tail_pts.empty() || strand.tail_pts.size() != strand.tail_v.size()) {
        return fail("emit_strand: finalized tail move list is empty or inconsistent");
    }
    if (!(strand.feed_mm_min > 0.0) || !(strand.tail_length_mm > 0.0)) {
        return fail("emit_strand: strand feed and calibrated tail must be > 0");
    }
    if (!(params.restart_feed_mm > 0.0) || !(params.prime_v_mm >= 0.0) ||
        !(params.retract_v_mm >= 0.0) || !(params.restart_z_mm >= 0.0) || !(params.lift_z_mm >= 0.0)) {
        return fail("emit_strand: invalid FiberEmitParams (feeds/lengths must be finite and non-negative as applicable)");
    }

    const double restart = FiberRun::round3(params.restart_feed_mm);
    const double p_r     = FiberRun::round3(strand.ratio_p);
    const double z_hi    = strand.z + std::max(params.restart_z_mm, params.lift_z_mm);

    std::string s;
    if (params.emit_layer_marker) {
        s += "; LAYER:" + std::to_string(strand.layer_id) + " [" + fmt("%.2f", strand.z) + "]\n";
    }

    const FiberPoint& p0 = strand.pts.front();
    // Optional vendor-shaped entity comments (fs_fiber_verbose_comments). A closed
    // loop is an Inset XF; an open path is Fiber infill. The seam comment names
    // the restart XY the cutter and the tail will share.
    if (params.verbose_comments) {
        const FiberPoint& pe = strand.pts.back();
        const bool closed = std::hypot(pe.x - p0.x, pe.y - p0.y) < 0.05;
        s += closed ? "; Inset XF start\n" : "; Fiber infill start\n";
        s += "; SEAM Fiber at X" + fmt("%.3f", p0.x) + " Y" + fmt("%.3f", p0.y) +
             " Z" + fmt("%.3f", strand.z) + "\n";
    }

    // 1. Open the window. Budget counts commanded forward U only: the tail
    // suffix carries no U (the blade is upstream of everything that could drive
    // it), so it never enters L.
    s += "M1001 L" + std::to_string(strand.budget_L(params.restart_feed_mm)) + "\n";

    // 2. Above-layer restart feed at the strand start (contract s8.2), then
    // 3. V-only matrix prime. Identical to the per-run lifecycle so downstream
    // pause/rewind handling sees one dialect.
    s += "G1 F" + fmt("%.0f", params.lift_f) + " Z" + fmt("%.2f", z_hi) + "\n";
    s += "G1 X" + fmt("%.2f", p0.x) + " Y" + fmt("%.2f", p0.y) + " F" + fmt("%.0f", params.lift_f) + "\n";
    s += "G1 F" + fmt("%.0f", params.restart_feed_f) + " U" + fmt("%.3f", restart) + " ; Extrude restart\n";
    s += "G1 F" + fmt("%.0f", params.lift_f) + " Z" + fmt("%.2f", strand.z) + "\n";
    // Stationary V is two accounted quantities: recover the previous run's
    // retract, then any extra configured as an anchor prime. When they sum to
    // the same millimetres as the old single V4 they are still one physical
    // start, but the G-code names the two purposes instead of burying recover
    // inside the prime.
    {
        const double recover = FiberRun::round3(params.retract_v_mm);
        const double extra   = FiberRun::round3(std::max(0.0, params.prime_v_mm - params.retract_v_mm));
        if (recover > 0.0)
            s += "G1 F" + fmt("%.0f", params.prime_f) + " V" + fmt("%.3f", recover) + " ; Recover matrix retract\n";
        if (extra > 0.0)
            s += "G1 F" + fmt("%.0f", params.prime_f) + " V" + fmt("%.3f", extra) + " ; Matrix prime\n";
        if (recover <= 0.0 && extra <= 0.0)
            s += "G1 F" + fmt("%.0f", params.prime_f) + " V" + fmt("%.3f", FiberRun::round3(params.prime_v_mm)) + " ; Matrix prime\n";
    }

    // 4. Body joint deposits up to the scheduled cut position. V is derived
    // from the PRINTED U so V == U * P holds at emitted precision (R11).
    // Deposit moves carry NO Z word: the fiber is attached between M1001 and
    // M1002, so Z stays constant for the whole window (operator ruling; the
    // vendor reference files are flat-Z inside windows too).
    // Distance along the deposited path, carried through the cut into the tail
    // so the three-zone ramp measures one continuous strand.
    double        dist   = 0.0;
    FiberPoint    prev   = p0;
    const double  total  = strand.total_path;
    for (size_t i = 0; i < strand.body_u.size(); ++i) {
        const double u = strand.body_u[i];
        const double v = FiberRun::round3(u * p_r);
        const FiberPoint& pe = strand.body_pts[i];
        s += "G1 X" + fmt("%.2f", pe.x) + " Y" + fmt("%.2f", pe.y);
        s += " V" + fmt("%.3f", v) + " U" + fmt("%.3f", u) +
             " P" + fmt("%.3f", p_r) +
             " F" + fmt("%.0f", zone_feed_mm_min(params, dist, total, strand.feed_mm_min)) + "\n";
        dist += std::hypot(pe.x - prev.x, pe.y - prev.y);
        prev  = pe;
    }

    // 5. Cut INSIDE the deposition path (operator ruling 2026-10-01): the blade
    // fires at path length (total - calibrated tail) so the severed tail
    // becomes the strand's final deposited section. M2800 is cutter-only
    // (fibre_servo.cfg: no feed, no motion) so an early position is legal; the
    // dwelled pulse is always followed by M400 (contract s6).
    s += "; Start to cut\n";
    s += "M2800\n";
    s += "M400\n";
    s += ";CUT DISTANCE " + fmt("%.1f", FiberStrand::round3(strand.tail_length_mm)) + "\n";

    // 6. Tail deposition: V-bearing, U-free moves along the remaining path to
    // the strand endpoint. The matrix extruder, downstream of the blade, pays
    // the severed tail out under drag (contract s8.7 / U17); it is deliberately
    // deposited into the planned structure - NO separation move here.
    for (size_t i = 0; i < strand.tail_v.size(); ++i) {
        const FiberPoint& pe = strand.tail_pts[i];
        s += "G1 X" + fmt("%.2f", pe.x) + " Y" + fmt("%.2f", pe.y) + " V" + fmt("%.3f", strand.tail_v[i]) +
             " F" + fmt("%.0f", zone_feed_mm_min(params, dist, total, strand.feed_mm_min)) + "\n";
        dist += std::hypot(pe.x - prev.x, pe.y - prev.y);
        prev  = pe;
    }

    // 7. Deferred-pause cut-boundary handshake after the tail is consumed
    // (matches the vendor position: last material move -> handshake -> retract).
    s += "; Cutting completed.\n";

    // 8. Release at the strand ENDPOINT: V-only retract, Z lift, close window.
    s += "G1 F" + fmt("%.0f", params.retract_f) + " V-" + fmt("%.3f", FiberRun::round3(params.retract_v_mm)) + " ; Retract\n";
    // Forward dry release, between the tail retract and the Z lift. The block
    // carries XY and F only: no U, no V, no E, no Z. It deposits nothing, so it
    // reserves nothing and never claims a measured clearance.
    if (params.release != nullptr && params.release->valid && !params.release->path.empty())
        s += emit_fiber_release(*params.release);
    // The departure Z lift. When this window is closed by a paired tool change,
    // the lift is DEFERRED out of the window and into that block, immediately
    // after the departure matrix withdrawal and before station entry. The owner
    // transition contract (section 3 steps 4-5, and check S06) pins the order as
    //     V-1 -> release -> M1002 -> V-4 -> lift -> station entry
    // and lifting before the window closes puts the carriage above the part while
    // the matrix is still being withdrawn through it. The withdrawal is a V-axis
    // move on the head that is still selected, so it stays legal either way; the
    // lift is what has to move. Unwrapped output (no paired tool change) keeps the
    // lift here, because nothing downstream would emit it.
    if (!params.defer_departure_lift)
        s += "G1 F" + fmt("%.0f", params.lift_f) + " Z" + fmt("%.2f", strand.z + params.lift_z_mm) + "\n";
    s += "M1002\n";

    out = s;
    return true;
}

bool plan_fiber_prime_line(const FiberPoint& from, const FiberPoint& to, double z, size_t layer_id,
                           double ratio_p, double fiber_rate, double feed_mm_min,
                           double tail_length_mm, FiberPrimeLine& out, std::string* error)
{
    auto fail = [error, &out](const char* msg) {
        if (error) {
            *error = msg;
        }
        out = FiberPrimeLine{};
        return false;
    };

    if (!std::isfinite(from.x) || !std::isfinite(from.y) || !std::isfinite(to.x) || !std::isfinite(to.y) ||
        !std::isfinite(z))
        return fail("plan_fiber_prime_line: non-finite priming line geometry");
    if (layer_id == 0)
        return fail("plan_fiber_prime_line: layer id must be >= 1");
    if (!(ratio_p > 0.0) || !std::isfinite(ratio_p))
        return fail("plan_fiber_prime_line: matrix:fiber ratio P must be finite and > 0");
    if (!(fiber_rate > 0.0) || !std::isfinite(fiber_rate))
        return fail("plan_fiber_prime_line: fiber rate must be finite and > 0");
    if (!(feed_mm_min > 0.0) || !std::isfinite(feed_mm_min))
        return fail("plan_fiber_prime_line: deposit feedrate must be finite and > 0");
    if (!(tail_length_mm > 0.0) || !std::isfinite(tail_length_mm))
        return fail("plan_fiber_prime_line: calibrated tail must be finite and > 0");
    // The physical limit on how short the line may be: a strand carries a body
    // before the cut as well as the severed tail after it, so the line has to
    // outlast fs_tail_length. finalize() enforces it; naming it here gives the
    // operator the actionable message (lengthen the line) instead of the
    // generic strand one.
    if (std::hypot(to.x - from.x, to.y - from.y) <= tail_length_mm)
        return fail("plan_fiber_prime_line: priming line is not longer than the calibrated tail, so it cannot carry a body before the cut");

    out.from           = from;
    out.to             = to;
    out.z              = z;
    out.layer_id       = layer_id;
    out.ratio_p        = ratio_p;
    out.fiber_rate     = fiber_rate;
    out.feed_mm_min    = feed_mm_min;
    out.tail_length_mm = tail_length_mm;
    return true;
}

bool emit_fiber_prime_line(const FiberPrimeLine& line, const FiberEmitParams& params,
                           bool tool_wrap, std::string& out, std::string* error)
{
    out.clear();

    FiberStrand s;
    s.layer_id       = line.layer_id;
    s.z              = line.z;
    s.pts            = {line.from, line.to};
    s.ratio_p        = line.ratio_p;
    s.fiber_rate     = line.fiber_rate;
    s.feed_mm_min    = line.feed_mm_min;
    s.tail_length_mm = line.tail_length_mm;
    // The priming line is sacrificial, so it is laid at the plate's own matrix
    // payout: no tail-factor discount, which exists to soften the tail of a
    // strand bonded into real material.
    s.tail_v_factor  = 1.0;

    std::string err;
    if (!s.finalize(&err))
        return fail_into(out, error, "emit_fiber_prime_line: " + err);

    // The priming line is not part of any layer, so it carries no layer marker:
    // the validator then reports it as the priming window it is (WARN R13)
    // rather than as fiber belonging to a layer.
    FiberEmitParams pp      = params;
    pp.emit_layer_marker    = false;
    pp.verbose_comments     = false;

    std::string body;
    if (!emit_strand(s, pp, body, &err))
        return fail_into(out, error, "emit_fiber_prime_line: " + err);

    out = tool_wrap ? "T0 ; switch extruder type to:FIBER\n" + body + "T1 ; switch extruder type to:PLASTIC\n" : body;
    return true;
}

std::string fiber_enforce_violation(size_t layer_id, size_t num_rings, size_t num_runs, size_t num_skipped)
{
    // Skipped candidate paths mean the layer would print partially reinforced.
    if (num_skipped > 0) {
        return "layer " + std::to_string(layer_id) + ": " + std::to_string(num_skipped) +
               " fiber path(s) could not be reconstructed into a complete window";
    }
    // External-perimeter rings present but nothing finalized: the layer is
    // fiber-capable yet would print with no fiber at all.
    if (num_rings > 0 && num_runs == 0) {
        return "layer " + std::to_string(layer_id) + ": no fiber window could be formed on any external perimeter";
    }
    return std::string();
}

} // namespace Fiber
} // namespace Slic3r
