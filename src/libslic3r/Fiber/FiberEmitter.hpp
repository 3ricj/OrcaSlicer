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
// Numeric constants come from FiberEmitParams, filled from fs_* keys at the
// G-code call site. The emitter is pure: it never emits tool changes or
// dock/brush moves.

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

// Enforcement policy for fs_fiber_enforce. Returns a human-readable violation
// when an enforced slice must fail for this layer, or empty when acceptable.
// A producer skip (path that could have been a window but was not emitted)
// or rings with no finalized strand is a violation. A layer without external
// perimeter rings is never a violation.
std::string fiber_enforce_violation(size_t layer_id, size_t num_rings, size_t num_runs, size_t num_skipped);

} // namespace Fiber
} // namespace Slic3r
