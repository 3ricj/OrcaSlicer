// License: GNU AGPLv3 or higher
//
// FibreSeeker3 continuous-fiber run model.
//
// A FiberRun is the atom of composite (matrix + fiber) deposition: one continuous
// fiber feed window over a polyline path at one layer, terminated by the cutter
// cycle. See docs/superpowers/fibreseeker3/machine_contract.md section 8 for the
// lifecycle the run is emitted against, and the project invariants: once
// finalize() has succeeded the run is immutable data and must never be
// reordered, reversed, retracted, simplified or interrupted without an explicit
// replanning that builds a new run.
//
// All lengths are millimeters, feeds mm/min. Coordinates are absolute bed mm
// (tool offsets are macro-owned on this machine and never baked here, contract
// section 12.3).

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace Slic3r {
namespace Fiber {

struct FiberPoint
{
    double x = 0.0;
    double y = 0.0;
};

class FiberRun
{
public:
    // ---- planning inputs (mutable until finalize()) --------------------------
    size_t              layer_id      = 0;     // 1-based layer index for markers
    double              z             = 0.0;   // layer Z at which the run deposits
    std::vector<FiberPoint> pts;               // polyline, >= 2 distinct points
    double              ratio_p       = 0.0;   // matrix:fiber ratio; V = U * P per deposit move
    double              fiber_rate    = 0.0;   // fiber (U) mm per mm of path
    double              feed_mm_min   = 0.0;   // deposit feedrate

    // ---- finalized outputs (read-only after finalize()) ----------------------
    std::vector<double> u_feed;                // per-segment U feed, as it will be printed
    double              total_u_feed  = 0.0;   // sum of u_feed
    bool                finalized     = false;

    // Computes per-segment feeds and validates every emission invariant.
    // Returns false and fills `error` if the run can never be emitted safely.
    bool finalize(std::string* error = nullptr);

    // Number of deposit segments (== pts.size() - 1) after a successful finalize().
    size_t num_segments() const { return finalized ? u_feed.size() : 0; }

    // Total printed fiber feed including the U-only restart: the window budget is
    // floor(restart_feed + total_u_feed), matching the L<n> parameter of M1001
    // (contract section 4: advisory, exporter-side bookkeeping).
    long budget_L(double restart_feed) const;

    double path_length() const;

    // Rounding used for every printed material value; the window budget is
    // computed over printed values so exporter accounting matches the g-code.
    static double round3(double v);
};

} // namespace Fiber
} // namespace Slic3r
