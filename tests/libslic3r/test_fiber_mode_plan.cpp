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
// ---- Schedule: does a layer receive fiber? (fs_fiber_schedule) -------------
// The exporter applies these rules inside the layer loop; the composite-head
// priming decision needs the same answer for the WHOLE plate before the first
// layer is written, so the rules live here as a pure function. A priming line
// that disagrees with the export about whether a plate carries fiber would
// either waste roving on a plastic-only plate or leave a CF plate unprimed.

namespace {

FiberScheduleParams sched(FiberSchedule s, FiberMode m, double macro_h = 0.24)
{
    FiberScheduleParams p;
    p.schedule              = s;
    p.mode                  = m;
    p.macro_layer_height_mm = macro_h;
    return p;
}

} // namespace

TEST_CASE("FiberModePlan: plastic_only never schedules a layer, under any schedule", "[Fiber][FiberModePlan]")
{
    for (const FiberSchedule s : {FiberSchedule::fsEveryLayer, FiberSchedule::fsBand, FiberSchedule::fsMacroLayer})
        for (size_t id = 0; id < 5; ++id)
            CHECK_FALSE(fiber_layer_scheduled(sched(s, FiberMode::fmPlasticOnly), id, 0.12 * (id + 1), 0.12, false, 0.12));
}

TEST_CASE("FiberModePlan: every_layer keeps the plastic first and last layer outside solid", "[Fiber][FiberModePlan]")
{
    const FiberScheduleParams s = sched(FiberSchedule::fsEveryLayer, FiberMode::fmOff);
    CHECK_FALSE(fiber_layer_scheduled(s, 0, 0.28, 0.28, false, 0.28));  // bed stays FFF
    CHECK(fiber_layer_scheduled(s, 1, 0.40, 0.12, false, 0.28));
    CHECK_FALSE(fiber_layer_scheduled(s, 9, 1.48, 0.12, true, 0.28));   // top skin stays FFF
    // Solid (Fortified) is allowed on the bed and the top skin.
    const FiberScheduleParams solid = sched(FiberSchedule::fsEveryLayer, FiberMode::fmSolid);
    CHECK(fiber_layer_scheduled(solid, 0, 0.28, 0.28, false, 0.28));
    CHECK(fiber_layer_scheduled(solid, 9, 1.48, 0.12, true, 0.28));
}

TEST_CASE("FiberModePlan: macro_layer fires once per composite bead", "[Fiber][FiberModePlan]")
{
    // The reference pairing: 0.12 mm plastic layers, 0.24 mm composite bead.
    const FiberScheduleParams s = sched(FiberSchedule::fsMacroLayer, FiberMode::fmWalls, 0.24);
    size_t hits = 0;
    for (size_t id = 1; id + 1 < 12; ++id) // id 0 is the bed, id 11 the top skin
        hits += fiber_layer_scheduled(s, id, 0.12 * (id + 1), 0.12, false, 0.12) ? 1 : 0;
    CHECK(hits == 5); // every second layer, and neither end
    CHECK(fiber_layer_scheduled(s, 2, 0.36, 0.12, false, 0.12));
    CHECK_FALSE(fiber_layer_scheduled(s, 3, 0.48, 0.12, false, 0.12));
    // A bead that is a whole number of layers tall: 3 layers per 0.36 mm bead.
    const FiberScheduleParams s3 = sched(FiberSchedule::fsMacroLayer, FiberMode::fmWalls, 0.36);
    CHECK(fiber_layer_scheduled(s3, 3, 0.48, 0.12, false, 0.12));
    CHECK_FALSE(fiber_layer_scheduled(s3, 4, 0.60, 0.12, false, 0.12));
}

TEST_CASE("FiberModePlan: band confines fiber to the Z band on macro steps", "[Fiber][FiberModePlan]")
{
    FiberScheduleParams s = sched(FiberSchedule::fsBand, FiberMode::fmWalls, 0.24);
    s.band_z_min_mm = 0.2;
    s.band_z_max_mm = 3.8;
    // print_z measured against a 0.28 mm bottom: rel lands on 0.24 steps at ids 1,3,5,...
    CHECK(fiber_layer_scheduled(s, 1, 0.52, 0.12, false, 0.28));   // rel 0.24, inside the band
    CHECK_FALSE(fiber_layer_scheduled(s, 2, 0.64, 0.12, false, 0.28)); // rel 0.36: off-step
    CHECK(fiber_layer_scheduled(s, 3, 0.76, 0.12, false, 0.28));   // rel 0.48
    CHECK_FALSE(fiber_layer_scheduled(s, 33, 4.36, 0.12, false, 0.28)); // rel 4.08: above the band
    // An inverted band asks for nothing; that must not read as "everything".
    FiberScheduleParams empty_band = s;
    empty_band.band_z_min_mm = 5.0;
    empty_band.band_z_max_mm = 1.0;
    CHECK_FALSE(fiber_layer_scheduled(empty_band, 3, 0.76, 0.12, false, 0.28));
}

TEST_CASE("FiberModePlan: degenerate schedule inputs degrade to every-layer, never to a crash", "[Fiber][FiberModePlan]")
{
    // Zero / non-finite layer height or macro height makes the Z tests
    // meaningless, so the schedule constraint is dropped rather than guessed
    // at: the plate falls back to reinforcing every layer but the ends.
    const FiberScheduleParams s = sched(FiberSchedule::fsBand, FiberMode::fmWalls, 0.0);
    CHECK_FALSE(fiber_layer_scheduled(s, 0, 0.28, 0.0, false, 0.28)); // the bed rule still holds
    CHECK(fiber_layer_scheduled(s, 4, 1.0, 0.0, false, 0.28));
    CHECK(fiber_layer_scheduled(s, 4, 1.0, std::nan(""), false, 0.28));
    CHECK(fiber_layer_scheduled(sched(FiberSchedule::fsMacroLayer, FiberMode::fmWalls, std::nan("")),
                                4, 1.0, 0.12, false, 0.28));
    CHECK_FALSE(fiber_layer_scheduled(s, 4, 1.0, 0.12, true, 0.28));  // and the top skin rule
}

// ---------------------------------------------------------------------------
// The fiber lane (fiber_lane_inset_mm). The exporter harvests the plastic
// outer-wall CENTERLINE as the fiber ring, so the inset is the single quantity
// that decides whether the roving hides behind plastic or lands on the visible
// surface. A zero or negative inset is the bug this guards: fiber tracing the
// edge of the part instead of a core under plastic walls.
// ---------------------------------------------------------------------------

TEST_CASE("FiberModePlan: the fiber lane is always held behind at least one plastic wall", "[Fiber][FiberModePlan]")
{
    // Reference machine numbers: 0.4 mm plastic wall pitch and bead, 0.8 mm
    // composite bead, 0.1 mm deliberate bond. One wall outboard puts the lane
    // center 0.4 + (0.4 + 0.8)/2 - 0.1 = 0.9 mm off the wall centerline.
    CHECK(fiber_lane_inset_mm(1, 0.4, 0.4, W, 0.1) == Approx(0.9));
    // Asking for zero (or a negative count) still keeps one plastic wall: the
    // exterior belongs to plastic, so the answer never collapses onto the skin.
    CHECK(fiber_lane_inset_mm(0, 0.4, 0.4, W, 0.1) == Approx(0.9));
    CHECK(fiber_lane_inset_mm(-3, 0.4, 0.4, W, 0.1) == Approx(0.9));
    // More walls outboard push the lane deeper, one wall pitch per wall.
    CHECK(fiber_lane_inset_mm(2, 0.4, 0.4, W, 0.1) == Approx(0.9 + 0.4));
    CHECK(fiber_lane_inset_mm(3, 0.4, 0.4, W, 0.1) == Approx(0.9 + 2 * 0.4));
    // The bond overlap pulls the lane back out but cannot undo the wall pack.
    CHECK(fiber_lane_inset_mm(1, 0.4, 0.4, W, 0.0) == Approx(1.0));
}

TEST_CASE("FiberModePlan: a degenerate lane request degrades to a positive inset", "[Fiber][FiberModePlan]")
{
    // An overlap larger than the geometry would put the lane back on the wall:
    // clamped to zero rather than allowed to negate the lane.
    CHECK(fiber_lane_inset_mm(1, 0.4, 0.4, W, 5.0) == Approx(0.0));
    CHECK(fiber_lane_inset_mm(1, 0.4, 0.4, W, -1.0) == Approx(fiber_lane_inset_mm(1, 0.4, 0.4, W, 0.0)));
    // Missing wall pitch falls back to the plastic bead width, never to zero.
    CHECK(fiber_lane_inset_mm(1, 0.0, 0.4, W, 0.1) == Approx(0.4 + 0.5 * (0.4 + W) - 0.1));
    CHECK(fiber_lane_inset_mm(1, -2.0, 0.4, W, 0.1) == Approx(0.4 + 0.5 * (0.4 + W) - 0.1));
    // Garbage inputs stay finite and non-negative: the planner rejects a
    // negative inset outright, so the helper must never produce one.
    CHECK(fiber_lane_inset_mm(1, std::nan(""), 0.4, W, 0.1) >= 0.0);
    CHECK(fiber_lane_inset_mm(1, 0.4, 0.4, std::nan(""), 0.1) == 0.0); // no bead width to place
    CHECK(fiber_lane_inset_mm(1, 0.4, 0.4, 0.0, 0.1) == 0.0);
    CHECK(fiber_lane_inset_mm(1, 0.4, std::nan(""), W, 0.1) == Approx(0.4 + W - 0.1));
}
