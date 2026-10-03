// License: GNU AGPLv3 or higher

#include "FiberRun.hpp"

#include <cmath>

namespace Slic3r {
namespace Fiber {

double FiberRun::round3(double v)
{
    // Emitted material values carry three decimals; the printed value is the
    // source of truth for the window budget, so all accounting is done on the
    // printed (rounded) numbers, never on raw planner floats.
    if (std::abs(v) < 0.0005) {
        return 0.0; // normalize -0.000 away
    }
    return std::round(v * 1000.0) / 1000.0;
}

double FiberRun::path_length() const
{
    double len = 0.0;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const double dx = pts[i + 1].x - pts[i].x;
        const double dy = pts[i + 1].y - pts[i].y;
        len += std::sqrt(dx * dx + dy * dy);
    }
    return len;
}

long FiberRun::budget_L(double restart_feed) const
{
    // L = floor(restart_printed + sum(u_printed)); 1e-9 absorbs binary error of
    // summing 3-decimal decimal values (the budget may never under-count).
    const double total = round3(restart_feed) + total_u_feed;
    return static_cast<long>(std::floor(total + 1e-9));
}

bool FiberRun::finalize(std::string* error)
{
    auto fail = [error](const char* msg) {
        if (error) {
            *error = msg;
        }
        return false;
    };

    if (!std::isfinite(z) || layer_id == 0) {
        return fail("FiberRun: layer_id must be >= 1 and z finite");
    }
    if (pts.size() < 2) {
        return fail("FiberRun: path needs at least two points");
    }
    if (!(ratio_p > 0.0) || !std::isfinite(ratio_p)) {
        return fail("FiberRun: matrix:fiber ratio P must be finite and > 0");
    }
    if (!(fiber_rate > 0.0) || !std::isfinite(fiber_rate)) {
        return fail("FiberRun: fiber_rate must be finite and > 0");
    }
    if (!(feed_mm_min > 0.0) || !std::isfinite(feed_mm_min)) {
        return fail("FiberRun: feed_mm_min must be finite and > 0");
    }

    u_feed.clear();
    double total = 0.0;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        if (!std::isfinite(pts[i].x) || !std::isfinite(pts[i].y) || !std::isfinite(pts[i + 1].x) || !std::isfinite(pts[i + 1].y)) {
            return fail("FiberRun: non-finite path coordinate");
        }
        const double dx = pts[i + 1].x - pts[i].x;
        const double dy = pts[i + 1].y - pts[i].y;
        const double seg = std::sqrt(dx * dx + dy * dy);
        if (seg < 1e-6) {
            // Dropping or merging a segment here would silently simplify a
            // finalized run, which the model forbids. Reject instead.
            return fail("FiberRun: zero-length segment (paths must not repeat points)");
        }
        const double u = round3(seg * fiber_rate);
        if (!(u > 0.0)) {
            return fail("FiberRun: segment feed rounds to zero (fiber without matrix would be emitted)");
        }
        u_feed.push_back(u);
        total += u;
    }

    total_u_feed = round3(total);
    finalized    = true;
    return true;
}

} // namespace Fiber
} // namespace Slic3r
