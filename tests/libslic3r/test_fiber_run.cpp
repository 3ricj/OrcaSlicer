// FiberRun model + enforce-policy tests.

#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Fiber/FiberEmitter.hpp"
#include "libslic3r/Fiber/FiberRun.hpp"

#include <limits>
#include <string>

using Catch::Matchers::WithinAbs;
using namespace Slic3r::Fiber;

namespace {

FiberRun make_square_run()
{
    FiberRun r;
    r.layer_id    = 3;
    r.z           = 0.30;
    r.pts         = {{100.0, 100.0}, {110.0, 100.0}, {110.0, 110.0}, {100.0, 110.0}, {100.0, 100.0}};
    r.ratio_p     = 2.0;
    r.fiber_rate  = 0.25;
    r.feed_mm_min = 1200.0;
    return r;
}

} // namespace

TEST_CASE("FiberRun finalize computes printed feeds and budget", "[Fiber]")
{
    FiberRun r = make_square_run();
    std::string err;
    REQUIRE(r.finalize(&err));
    REQUIRE(err.empty());
    REQUIRE(r.finalized);
    REQUIRE(r.num_segments() == 4);
    for (double u : r.u_feed) {
        REQUIRE_THAT(u, WithinAbs(2.5, 1e-12));
        REQUIRE(u > 0.0);
    }
    REQUIRE_THAT(r.total_u_feed, WithinAbs(10.0, 1e-12));
    REQUIRE(r.budget_L(55.0) == 65);
    FiberRun s;
    s.layer_id = 1;
    s.z        = 0.2;
    s.pts      = {{0.0, 0.0}, {4.444, 0.0}};
    s.ratio_p  = 2.38;
    s.fiber_rate = 0.25;
    s.feed_mm_min = 600.0;
    REQUIRE(s.finalize());
    REQUIRE(s.budget_L(55.0) == 56);
}

TEST_CASE("FiberRun finalize rejects unsafe run shapes", "[Fiber]")
{
    std::string err;

    FiberRun few = make_square_run();
    few.pts.resize(1);
    REQUIRE_FALSE(few.finalize(&err));
    REQUIRE_FALSE(few.finalized);

    FiberRun dup = make_square_run();
    dup.pts[1] = dup.pts[0];
    REQUIRE_FALSE(dup.finalize(&err));

    FiberRun nop = make_square_run();
    nop.ratio_p = 0.0;
    REQUIRE_FALSE(nop.finalize(&err));

    FiberRun norate = make_square_run();
    norate.fiber_rate = 0.0;
    REQUIRE_FALSE(norate.finalize(&err));

    FiberRun nofeed = make_square_run();
    nofeed.feed_mm_min = 0.0;
    REQUIRE_FALSE(nofeed.finalize(&err));

    FiberRun tiny = make_square_run();
    tiny.pts = {{0.0, 0.0}, {0.001, 0.0}};
    tiny.fiber_rate = 0.25;
    REQUIRE_FALSE(tiny.finalize(&err));

    FiberRun nanp = make_square_run();
    nanp.pts[2].y = std::numeric_limits<double>::quiet_NaN();
    REQUIRE_FALSE(nanp.finalize(&err));
}

TEST_CASE("fiber_enforce_violation implements the enforce truth table", "[Fiber]")
{
    REQUIRE(fiber_enforce_violation(3, 2, 2, 0).empty());
    REQUIRE(fiber_enforce_violation(7, 0, 0, 0).empty());
    REQUIRE_FALSE(fiber_enforce_violation(4, 2, 2, 1).empty());
    REQUIRE(fiber_enforce_violation(4, 2, 2, 1).find("layer 4") != std::string::npos);
    REQUIRE(fiber_enforce_violation(4, 2, 2, 1).find("1 fiber path(s)") != std::string::npos);
    REQUIRE_FALSE(fiber_enforce_violation(12, 3, 0, 0).empty());
    REQUIRE(fiber_enforce_violation(12, 3, 0, 0).find("layer 12") != std::string::npos);
    REQUIRE(fiber_enforce_violation(9, 1, 0, 2).find("layer 9") != std::string::npos);
    REQUIRE(fiber_enforce_violation(9, 1, 0, 2).find("2 fiber path(s)") != std::string::npos);
}
