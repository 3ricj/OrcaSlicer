// License: GNU AGPLv3 or higher
//
// FibreSeeker3 fiber emission: renders a finalized FiberStrand into the
// composite-deposition dialect.
//
//   [; LAYER marker]            optional, emit_layer_marker
//   M1001 L<budget>             open fiber window (advisory L)
//   Z lift, travel to strand start above the layer
//   G1 F<f> U<restart>          U-only restart feed above the layer
//   descend to layer Z
//   G1 F<f> V<recover>          recover previous matrix retract
//   G1 F<f> V<prime-recover>    remaining anchor prime
//   G1 X.. Y.. U.. V=U*P P.. F..   joint deposits up to the cut
//   ; Start to cut / M2800 / M400 / ;CUT DISTANCE <tail>
//   tail deposition: V-bearing, U-free (severed tow paid out by the matrix)
//   ; Cutting completed.        firmware-parsed cut-boundary handshake
//   G1 F<f> V-<retract>         V-only retract (U is never negative)
//   Z lift, then M1002          close window
//
// The same file renders the composite priming line (emit_fiber_prime_line): a
// sacrificial strand laid on bare bed, so the composite head is charged before
// the first strand of the plate. It is a real strand lifecycle, cut included.
//
// Numeric constants come from FiberEmitParams, filled from fs_* keys at the
// G-code call site. The emitter is pure: it never emits tool changes or
// dock/brush moves, except the optional T0/T1 bracket of the priming line,
// which the caller asks for explicitly.
#pragma once

#include <string>
#include <vector>

#include "FiberRun.hpp"
#include "FiberStrand.hpp"

namespace Slic3r {
namespace Fiber {

struct FiberEmitParams
{
    // Feedrates mm/min, lengths mm. Defaults match the FibreSeeker3 SK3
    // machine profile and are overridden from fs_* keys at the call site.
    double restart_feed_mm  = 55.0;
    double restart_feed_f   = 1200.0;
    double restart_z_mm     = 1.2;
    double prime_v_mm       = 4.0;
    double prime_f          = 600.0;
    double tail_length_mm   = 54.8;
    double separation_mm    = 10.0;
    double separation_f     = 1200.0;
    double retract_v_mm     = 1.0;
    double retract_f        = 600.0;
    double lift_z_mm        = 0.6;
    double lift_f           = 1200.0;
    bool   emit_layer_marker = true;
    // Three-zone deposition speed (fs_fiber_speed_*), mm/s, measured along the
    // whole deposited path. A zone speed of 0 means "not configured" and
    // leaves those moves at the strand's own feed_mm_min, so all-zero is the
    // single-feedrate path.
    //
    // The zone of a move is decided by where the move STARTS. This emitter
    // never splits a segment at a zone boundary, so the boundary lands on the
    // next vertex. Set fs_fiber_max_arc_seg to bound how far that can drift.
    double start_speed_mm_s  = 0.0;
    double start_length_mm   = 0.0;
    double normal_speed_mm_s = 0.0;
    double finish_speed_mm_s = 0.0;
    double finish_length_mm  = 0.0;
    // Optional entity comments for preview tooling (fs_fiber_verbose_comments).
    // Off by default so the G-code stays lean.
    bool verbose_comments = false;
};

double zone_feed_mm_min(const FiberEmitParams& params, double dist_mm, double total_mm, double fallback_f);

// Emits the complete lifecycle for one finalized FiberStrand.
//   [; LAYER marker] / M1001 / lift+travel / U restart / descend / V prime
//   body joint deposits up to the cut position (segment split if needed)
//   ; Start to cut / M2800 / M400 / ;CUT DISTANCE <tail>
//   tail deposition: V-bearing, U-free
//   ; Cutting completed. / V retract / Z lift / M1002
// Budget L = floor(restart + sum of printed body U); tail carries no U.
// params.tail_length_mm is ignored - the strand carries its own calibrated
// tail_length_mm. Returns false with `error` set and nothing partial produced.
bool emit_strand(const FiberStrand& strand, const FiberEmitParams& params, std::string& out, std::string* error = nullptr);

// Composite-head priming (fs_fiber_prime): the composite head is charged by
// laying one sacrificial strand on bare bed before the plate's real deposition
// starts. It is deliberately a real FiberStrand rather than a bespoke block:
// the dialect requires every M1001 window to contain a cut, and the strand
// model is the only producer of that shape, so a priming line is a straight
// two-point strand laid off the part. Same restart, same matrix prime, same
// joint deposits, same cut and tail, same budget accounting - nothing new for
// the firmware or the validator to learn.
//
// Placement and mode are the caller's; this is the plan + emission of the line.
struct FiberPrimeLine
{
    FiberPoint from;            // line start, absolute bed mm
    FiberPoint to;              // line end; |to - from| is the line length
    double     z = 0.0;         // deposit Z
    size_t     layer_id = 1;    // layer id carried by the window's marker
    double     ratio_p = 0.0;   // plate's matrix:fiber ratio
    double     fiber_rate = 0.0;// plate's fiber feed per mm of path
    double     feed_mm_min = 0.0; // plate's deposit feedrate
    double     tail_length_mm = 0.0; // plate's calibrated tail
};

// Validates a priming line and fills `out`. Pure and total: returns false with
// `error` set when the line cannot become a strand - non-finite geometry, a
// non-positive ratio / rate / feed / tail, or a line shorter than the calibrated
// tail. The last rule is the physical one: a strand must carry a body before the
// cut as well as the severed tail after it, so a priming line has to be longer
// than fs_tail_length. Everything past that is left to FiberStrand::finalize,
// whose error is surfaced by emit_fiber_prime_line rather than duplicated here.
bool plan_fiber_prime_line(const FiberPoint& from, const FiberPoint& to, double z, size_t layer_id,
                           double ratio_p, double fiber_rate, double feed_mm_min,
                           double tail_length_mm, FiberPrimeLine& out, std::string* error = nullptr);

// Emits a priming line as the full strand lifecycle (window, restart, matrix
// prime, joint deposits, cut, tail, retract, close) using `params` for the
// machine-side constants, so the priming window is indistinguishable from a
// strand window except for where it lies.
//
// `tool_wrap` brackets the block in the machine-dialect T0/T1 tool changes. The
// exporter does NOT use it: the startup purge is entered and left by the paired
// tool-change sequence (Fiber/FiberToolChange), which owns both tool lines, and
// the fibre->plastic half has to issue its matrix withdrawal while T0 is still
// selected, which a closing bracket would make illegal. The bracket stays
// available because it is the complete standalone block - a caller that only
// needs the window and its tool bracket, and no switch handling, can still ask
// for it - and the wrapped form is what the priming fixture pins.
bool emit_fiber_prime_line(const FiberPrimeLine& line, const FiberEmitParams& params,
                           bool tool_wrap, std::string& out, std::string* error = nullptr);

// Enforcement policy for fs_fiber_enforce. Returns a human-readable violation
// when an enforced slice must fail for this layer, or empty when acceptable.
// A producer skip (path that could have been a window but was not emitted)
// or rings with no finalized strand is a violation. A layer without external
// perimeter rings is never a violation.
std::string fiber_enforce_violation(size_t layer_id, size_t num_rings, size_t num_runs, size_t num_skipped);

} // namespace Fiber
} // namespace Slic3r
