// License: GNU AGPLv3 or higher
//
// FiberModePlan unit tests ([Fiber]). Contract under test: the RocketSlicer
// mode selector resolves to deterministic strand-planner parameters - the
// spacing law belongs to the fiber infill pattern (isogrid ribs three beads
// apart at full density, solid one bead apart, legacy rectilinear one bead
// divided by the density), fortified forces 100 percent, the laydown angle
// cycles over FIBER-EMITTED layers through the parsed fs_fiber_fill_angles
// list, the fill inset passes through, and coverage 0 switches the interior
// fill off so the fiber perimeter stands alone. The functions are pure: no
// print, no config object needed.

#include <catch2/catch_all.hpp>

#include <cmath>
#include <string>
#include <vector>

#include "libslic3r/Fiber/FiberModePlan.hpp"

using namespace Slic3r;
using namespace Slic3r::Fiber;
using Catch::Approx;

namespace {

// Reference machine bead widths: the composite nozzle is 0.7 mm but lays a
// 0.8 mm roving bead (InsetXFEWMM / InfillFSolidEW on the Reinforced levels);
// its Fortified profile drops the solid bead to 0.7 mm.
constexpr double W  = 0.8;
constexpr double WF = 0.7;

} // namespace

TEST_CASE("FiberModePlan: angle list parses the slash spec tolerantly", "[Fiber][FiberModePlan]")
{
    CHECK(parse_fiber_angle_list("0/90/0") == std::vector<double>{0.0, 90.0, 0.0});
    CHECK(parse_fiber_angle_list("45") == std::vector<double>{45.0});
    CHECK(parse_fiber_angle_list(" 45.5 , -30 ") == std::vector<double>{45.5, -30.0});
    CHECK(parse_fiber_angle_list("0//90") == std::vector<double>{0.0, 90.0}); // empty tokens skipped
    CHECK(parse_fiber_angle_list("").empty());
    CHECK(parse_fiber_angle_list("junk").empty());
    CHECK(parse_fiber_angle_list("30/60").size() == 2); // vendor Tetra angles
}

TEST_CASE("FiberModePlan: coverage percent maps to clamped rectilinear pitch", "[Fiber][FiberModePlan]")
{
    CHECK(fiber_pitch_for_coverage(80.0, W) == Approx(W / 0.8));
    CHECK(fiber_pitch_for_coverage(60.0, W) == Approx(W / 0.6));
    CHECK(fiber_pitch_for_coverage(100.0, W) == Approx(W));
    // Lower clamp 0.5 * bead width: coverage 200 wants 0.4 -> exactly the floor.
    CHECK(fiber_pitch_for_coverage(200.0, W) == Approx(0.4));
    CHECK(fiber_pitch_for_coverage(1000.0, W) == Approx(0.4));
    // Upper clamp 20 mm: a sliver of coverage.
    CHECK(fiber_pitch_for_coverage(0.5, W) == Approx(20.0));
    // Coverage off: positive fallback = bead width (caller disables the fill).
    CHECK(fiber_pitch_for_coverage(0.0, W) == Approx(W));
    CHECK(fiber_pitch_for_coverage(-5.0, W) == Approx(W));
    // Non-finite bead width: defensive positive fallback, never NaN to the planner.
    CHECK(fiber_pitch_for_coverage(80.0, std::nan("")) > 0.0);
}

// The rib spacing law measured on the reference machine's Benchy Reinforced
// levels: a 0.8 mm bead gives 3.00 mm ribs at 80 percent density and 4.00 mm at
// 60 percent. This is the number the whole density response hangs on.
TEST_CASE("FiberModePlan: isogrid density maps to the measured rib pitch", "[Fiber][FiberModePlan]")
{
    CHECK(fiber_isogrid_pitch(80.0, W) == Approx(3.0));
    CHECK(fiber_isogrid_pitch(60.0, W) == Approx(4.0));
    CHECK(fiber_isogrid_pitch(30.0, W) == Approx(8.0));
    CHECK(fiber_isogrid_pitch(100.0, W) == Approx(2.4));
    // Density off: positive fallback, ribs are switched off by the caller.
    CHECK(fiber_isogrid_pitch(0.0, W) == Approx(W));
    // A rib pair can never be closer than one bead, and the pitch is bounded.
    CHECK(fiber_isogrid_pitch(10000.0, W) == Approx(W));
    CHECK(fiber_isogrid_pitch(0.001, W) == Approx(200.0));
    CHECK(fiber_isogrid_pitch(80.0, std::nan("")) > 0.0);
}

TEST_CASE("FiberModePlan: reinforced isogrid resolves the vendor level ladder", "[Fiber][FiberModePlan]")
{
    const std::vector<double> angles{0.0};
    const auto level = [&angles](double cov) {
        return resolve_fiber_mode_layer(FiberMode::fmWalls, FiberInfillPattern::fipIsogrid,
                                        angles, cov, W, 0.3, 0);
    };
    // Vendor Reinforced L1-L5 carry InfillFIsogridFillDensity 20/10/30/60/80.
    CHECK(level(20.0).pitch_mm == Approx(12.0));
    CHECK(level(10.0).pitch_mm == Approx(24.0));
    CHECK(level(30.0).pitch_mm == Approx(8.0));
    CHECK(level(60.0).pitch_mm == Approx(4.0));
    CHECK(level(80.0).pitch_mm == Approx(3.0));
    const FiberModeLayer l5 = level(80.0);
    CHECK(l5.fill_enabled);
    CHECK(l5.pattern == FiberInfillPattern::fipIsogrid);
    CHECK(l5.fill_angle_deg == Approx(0.0));
    CHECK(l5.fill_outer_inset == Approx(0.3));
}

TEST_CASE("FiberModePlan: fortified solid lays passes one bead apart", "[Fiber][FiberModePlan]")
{
    const std::vector<double> angles{0.0, 90.0, 0.0};
    // Vendor Fortified: InfillFType 0 (solid), InfillFSolidEW 0.7, angle list
    // 0/90/0, ExtendIntoPerimeters 0. The coverage key is inert under fortified.
    const FiberModeLayer f0 = resolve_fiber_mode_layer(FiberMode::fmSolid, FiberInfillPattern::fipSolid,
                                                       angles, 10.0, WF, 0.0, 0);
    CHECK(f0.fill_enabled);
    CHECK(f0.pattern == FiberInfillPattern::fipSolid);
    CHECK(f0.pitch_mm == Approx(WF)); // adjacent passes, one bead apart
    CHECK(f0.fill_angle_deg == Approx(0.0));
    CHECK(f0.fill_outer_inset == Approx(0.0));
    // The angle list cycles over fiber-emitted layers: 0, 90, 0, then wraps.
    CHECK(resolve_fiber_mode_layer(FiberMode::fmSolid, FiberInfillPattern::fipSolid, angles,
                                   10.0, WF, 0.0, 1).fill_angle_deg == Approx(90.0));
    CHECK(resolve_fiber_mode_layer(FiberMode::fmSolid, FiberInfillPattern::fipSolid, angles,
                                   10.0, WF, 0.0, 2).fill_angle_deg == Approx(0.0));
    CHECK(resolve_fiber_mode_layer(FiberMode::fmSolid, FiberInfillPattern::fipSolid, angles,
                                   10.0, WF, 0.0, 3).fill_angle_deg == Approx(0.0));
    // Fortified forces full density, so even an isogrid selection resolves to
    // the 100 percent rib pitch rather than the (ignored) coverage key.
    CHECK(resolve_fiber_mode_layer(FiberMode::fmSolid, FiberInfillPattern::fipIsogrid, angles,
                                   10.0, W, 0.0, 0).pitch_mm == Approx(2.4));
}

TEST_CASE("FiberModePlan: rectilinear stays the legacy default pattern", "[Fiber][FiberModePlan]")
{
    // A project that never set fs_fiber_infill_pattern keeps the legacy spacing
    // law, so turning a mode on does not silently re-pattern an existing job.
    const FiberModeLayer l = resolve_fiber_mode_layer(FiberMode::fmWalls,
                                                      FiberInfillPattern::fipRectilinear,
                                                      {0.0}, 80.0, W, 0.3, 0);
    CHECK(l.pattern == FiberInfillPattern::fipRectilinear);
    CHECK(l.pitch_mm == Approx(W / 0.8));
}

TEST_CASE("FiberModePlan: coverage zero leaves the fiber perimeter alone", "[Fiber][FiberModePlan]")
{
    // No interior fiber at all; the pitch returned is a positive fallback so the
    // planner's parameter validation still passes.
    const FiberModeLayer l = resolve_fiber_mode_layer(FiberMode::fmWalls,
                                                      FiberInfillPattern::fipIsogrid,
                                                      {0.0}, 0.0, W, 0.3, 5);
    CHECK_FALSE(l.fill_enabled);
    CHECK(l.pitch_mm == Approx(W));
}

TEST_CASE("FiberModePlan: empty angle list and a negative inset degrade safely", "[Fiber][FiberModePlan]")
{
    const FiberModeLayer l = resolve_fiber_mode_layer(FiberMode::fmWalls,
                                                      FiberInfillPattern::fipIsogrid,
                                                      {}, 80.0, W, 0.3, 7);
    CHECK(l.fill_angle_deg == Approx(0.0)); // empty list falls back to 0 degrees
    // Negative inset clamps to zero rather than dilating the fill region.
    CHECK(resolve_fiber_mode_layer(FiberMode::fmWalls, FiberInfillPattern::fipIsogrid,
                                   {}, 80.0, W, -1.0, 0).fill_outer_inset == Approx(0.0));
}
