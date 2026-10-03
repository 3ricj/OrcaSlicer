// License: GNU AGPLv3 or higher
//
// FibreSeeker3 continuous-fiber STRAND model (operator ruling 2026-10-01).
//
// A FiberStrand is one physically continuous piece of reinforcement deposited
// over a polyline path at one layer: many geometric segments, exactly one cut
// event scheduled INSIDE the deposition path, and a final tail suffix deposited
// after severance under matrix drag (contract s8.7, U17). This replaces the
// per-chord FiberRun lifecycle for the strand production mode: tessellation,
// intersections and direction changes must never create lifecycle boundaries.
//
// Early-cut schedule:
//     cut_position = total_path_length - tail_length
// where tail_length is the CALIBRATED physical fiber length downstream of the
// blade at cut time (config only, U04/U17 lineage - never inferred from U/V
// command values). The final tail_length mm of cumulative path - which may span
// several short segments and validated turns - is deposited V-only (no U: the
// upstream feeder cannot drive a severed tail; the matrix extruder, downstream
// of the blade, pays it out). The release sequence (retract, lift, M1002)
// belongs to the strand ENDPOINT, not to the cut event.
//
// Invariants mirror FiberRun: once finalize() succeeds the strand is immutable
// and must never be reordered, reversed, retracted, simplified or interrupted
// without explicit replanning. All lengths mm, feeds mm/min, absolute bed mm.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "FiberRun.hpp" // FiberPoint + shared printed-precision rounding

namespace Slic3r {
namespace Fiber {

class FiberStrand
{
public:
    // ---- planning inputs (mutable until finalize()) --------------------------
    size_t              layer_id    = 0;   // 1-based layer index for markers
    double              z           = 0.0; // layer Z at which the strand deposits
    std::vector<FiberPoint> pts;           // polyline, >= 2 distinct points
    double              ratio_p     = 0.0; // matrix:fiber ratio; body V = U * P per move
    double              fiber_rate  = 0.0; // fiber (U) mm per mm of path
    double              feed_mm_min = 0.0; // deposit feedrate (body and tail moves)
    double              tail_length_mm = 0.0; // calibrated tail (cutter->nozzle path), must be > 0
    double              tail_v_factor  = 1.0; // tail matrix payout as a fraction of the body matrix
                                              // rate: tail V/mm = tail_v_factor * fiber_rate * ratio_p.
                                              // 1.0 = auto (same V/XY as joint U/V deposits), (0,1]

    // ---- finalized outputs (read-only after finalize()) ----------------------
    // Body moves: joint deposits ending at body_pts[i] with U = body_u[i]
    // (V printed as round3(U * P) by the emitter). Tail moves: V-only deposits
    // ending at tail_pts[i] with V = tail_v[i], following "; Start to cut".
    std::vector<FiberPoint> body_pts;
    std::vector<double>     body_u;
    std::vector<FiberPoint> tail_pts;
    std::vector<double>     tail_v;
    double total_path      = 0.0; // total polyline length on printed-rounded geometry
    double total_u_feed    = 0.0; // sum of body_u (tail carries no U)
    double total_v_tail    = 0.0; // sum of tail_v
    bool   finalized       = false;

    // Validates the strand and computes the printed body/tail move lists with
    // the cut scheduled at total_path - tail_length (splitting the segment the
    // cut falls inside). Returns false and fills `error` if the strand can
    // never be emitted safely (e.g. the path cannot carry both a body and the
    // reserved tail suffix - an island too short for a viable strand).
    bool finalize(std::string* error = nullptr);

    size_t num_body_moves() const { return finalized ? body_u.size() : 0; }
    size_t num_tail_moves() const { return finalized ? tail_v.size() : 0; }

    // Window budget: floor(restart_printed + sum of printed body U). Tail
    // moves carry no U, so they never enter the budget (U is forward-only and
    // zero after the blade).
    long budget_L(double restart_feed) const;

    double path_length() const;

    // Same printed precision as FiberRun; all accounting is on printed values.
    static double round3(double v) { return FiberRun::round3(v); }
};

} // namespace Fiber
} // namespace Slic3r
