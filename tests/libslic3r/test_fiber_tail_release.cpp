// FibreSeeker3 fibre-tail release and preheat-ordering unit tests (owner
// specification v1.0).
//
// What is pinned here, and why each pin is the one that matters:
//   - The three distances. cut = S-(T+M), post-cut deposition = T+M, total
//     post-cut XY = T+M+R, with the worked 100 mm fixture table reproduced
//     exactly for variants A, B and C. The margin moves the cut EARLIER and
//     never extends the footprint past the strand end.
//   - S <= T+M is rejected, not rescued. Silently shortening the tail would
//     hide an unprintable feature.
//   - The seam is deterministic and the margin does not influence its ranking,
//     which is what makes shook_B and shook_C differ only in M.
//   - A release with no qualifying candidate FAILS. It never silently becomes
//     R = 0, and variant A stays independently exportable.
//   - The emitted release block carries XY and F and no U, V, E or Z.
//   - The preheat clock is nominal: temperature waits and macros contribute
//     zero, and the plan is clamped into the outgoing activation.
//   - The E ledger never stacks a redundant travel withdrawal on a pending
//     tool-change withdrawal, and recovery is owed exactly once.

#include <catch2/catch_all.hpp>
using Catch::Approx;

#include "libslic3r/Fiber/FiberTailRelease.hpp"

#include <cmath>
#include <string>
#include <vector>

using namespace Slic3r::Fiber;

namespace {

// A rectangle the release footprint must lie inside. Stands in for the
// exporter's real per-layer deposited footprint, which the pure planner cannot
// see. Records the query it was asked about so a test can assert the footprint
// width came from the composite bead width and not from an inherited width.
class RectSupport : public ReleaseSupportModel
{
public:
    RectSupport(double x0, double y0, double x1, double y1) : m_x0(x0), m_y0(y0), m_x1(x1), m_y1(y1) {}

    SupportVerdict query(const ReleaseSupportQuery& q) const override
    {
        m_queried = true;
        m_last_half_width = q.half_width_mm;
        for (const FiberPoint& p : q.path) {
            // Containment of the centreline is enough for this stand-in; the
            // exporter does the buffered test properly.
            if (p.x < m_x0 - q.tolerance_mm || p.x > m_x1 + q.tolerance_mm
                || p.y < m_y0 - q.tolerance_mm || p.y > m_y1 + q.tolerance_mm)
                return SupportVerdict::Unsupported;
        }
        return SupportVerdict::Supported;
    }

    bool queried() const { return m_queried; }
    double last_half_width() const { return m_last_half_width; }

private:
    double m_x0, m_y0, m_x1, m_y1;
    mutable bool m_queried = false;
    mutable double m_last_half_width = -1.0;
};

// A 100 mm straight strand from X0 to X100 at Y10.
std::vector<FiberPoint> straight_100()
{
    return {FiberPoint{0.0, 10.0}, FiberPoint{100.0, 10.0}};
}

// A closed 40 x 20 rectangle starting at the origin, explicitly closed.
std::vector<FiberPoint> rect_closed()
{
    return {FiberPoint{0.0, 0.0}, FiberPoint{40.0, 0.0}, FiberPoint{40.0, 20.0},
            FiberPoint{0.0, 20.0}, FiberPoint{0.0, 0.0}};
}

TailReleaseParams params_a()
{
    TailReleaseParams p; // R 0, M 0
    return p;
}

TailReleaseParams params_b()
{
    TailReleaseParams p;
    p.release_mm = 6.8;
    return p;
}

TailReleaseParams params_c()
{
    TailReleaseParams p;
    p.release_mm = 6.8;
    p.margin_mm = 1.0;
    return p;
}

MotionBlock deposit(double path_mm, double feed)
{
    MotionBlock b;
    b.kind = MotionBlock::Kind::Deposit;
    b.path_mm = path_mm;
    b.feed_mm_min = feed;
    return b;
}

MotionBlock stationary(double material_mm, double feed)
{
    MotionBlock b;
    b.kind = MotionBlock::Kind::Stationary;
    b.max_material_mm = material_mm;
    b.feed_mm_min = feed;
    return b;
}

MotionBlock macro_block()
{
    MotionBlock b;
    b.kind = MotionBlock::Kind::Macro;
    return b;
}

MotionBlock temp_wait()
{
    MotionBlock b;
    b.kind = MotionBlock::Kind::TempWait;
    b.feed_mm_min = 600.0; // deliberately non-zero: it must still cost zero
    return b;
}

} // namespace

TEST_CASE("FiberTailRelease: the 100 mm fixture table holds for A, B and C", "[Fiber][FiberTailRelease]")
{
    // The spec's own worked table. These are measurement choices, so the
    // numbers are pinned rather than derived.
    TailReleaseDistances d;
    std::string err;

    REQUIRE(plan_tail_release_distances(100.0, params_a(), d, &err));
    CHECK(d.cut_distance_mm == Approx(45.2).margin(1e-6));
    CHECK(d.post_cut_deposit_mm == Approx(54.8).margin(1e-6));
    CHECK(d.post_cut_xy_mm == Approx(54.8).margin(1e-6)); // R = 0

    // post_cut_xy_mm is the spec's TOTAL PLANNED POST-CUT XY, i.e. the distance
    // travelled after the cut (T + M + R), not the absolute endpoint. On this
    // fixture the endpoint lands at X106.8 because the cut is at X45.2.
    REQUIRE(plan_tail_release_distances(100.0, params_b(), d, &err));
    CHECK(d.cut_distance_mm == Approx(45.2).margin(1e-6));
    CHECK(d.post_cut_deposit_mm == Approx(54.8).margin(1e-6));
    CHECK(d.post_cut_xy_mm == Approx(61.6).margin(1e-6));            // 54.8 + 6.8
    CHECK(d.cut_distance_mm + d.post_cut_xy_mm == Approx(106.8).margin(1e-6)); // endpoint

    REQUIRE(plan_tail_release_distances(100.0, params_c(), d, &err));
    CHECK(d.cut_distance_mm == Approx(44.2).margin(1e-6));  // margin moved the cut earlier
    CHECK(d.post_cut_deposit_mm == Approx(55.8).margin(1e-6));
    CHECK(d.post_cut_xy_mm == Approx(62.6).margin(1e-6));   // 55.8 + 6.8
    // The endpoint is unchanged by the margin: M moves the cut, not the end.
    CHECK(d.cut_distance_mm + d.post_cut_xy_mm == Approx(106.8).margin(1e-6));
}

TEST_CASE("FiberTailRelease: the release feed is the configured speed as an F word", "[Fiber][FiberTailRelease]")
{
    TailReleaseParams p = params_b();
    p.release_speed_mm_s = 10.0;
    TailReleaseDistances d;
    std::string err;
    REQUIRE(plan_tail_release_distances(100.0, p, d, &err));
    CHECK(d.release_f == Approx(600.0).margin(1e-6)); // 10 mm/s emits F600

    p.release_speed_mm_s = 20.0;
    REQUIRE(plan_tail_release_distances(100.0, p, d, &err));
    CHECK(d.release_f == Approx(1200.0).margin(1e-6));
}

TEST_CASE("FiberTailRelease: a strand not longer than tail plus margin is rejected", "[Fiber][FiberTailRelease]")
{
    TailReleaseDistances d;
    std::string err;
    // Exactly T + M leaves a zero body.
    CHECK_FALSE(plan_tail_release_distances(54.8, params_a(), d, &err));
    CHECK(err.find("FS_RELEASE_UNSUPPORTED") != std::string::npos);
    CHECK(d.planned_length_mm == 0.0); // zeroed, not half-filled

    // T + M + R is still fine: R is not deposition.
    CHECK(plan_tail_release_distances(54.9, params_b(), d, &err));

    // Non-finite and non-positive lengths are rejected too.
    CHECK_FALSE(plan_tail_release_distances(0.0, params_a(), d, &err));
    CHECK_FALSE(plan_tail_release_distances(std::nan(""), params_a(), d, &err));
}

TEST_CASE("FiberTailRelease: out-of-range parameters are a configuration error", "[Fiber][FiberTailRelease]")
{
    std::string err;
    TailReleaseParams p;
    CHECK(validate_tail_release_params(p, &err));

    p.margin_mm = 3.0001;
    CHECK_FALSE(validate_tail_release_params(p, &err));
    p = params_a();
    p.margin_mm = -0.001;
    CHECK_FALSE(validate_tail_release_params(p, &err));

    p = params_a();
    p.release_mm = 10.001;
    CHECK_FALSE(validate_tail_release_params(p, &err));

    p = params_a();
    p.release_speed_mm_s = 0.5; // below the 1..20 band
    CHECK_FALSE(validate_tail_release_params(p, &err));

    p = params_a();
    p.anchor_mm = 1.0; // below the 2..20 band
    CHECK_FALSE(validate_tail_release_params(p, &err));

    // Boundary values are inside the band.
    p = params_a();
    p.margin_mm = 3.0;
    p.release_mm = 10.0;
    p.release_speed_mm_s = 1.0;
    p.anchor_mm = 20.0;
    CHECK(validate_tail_release_params(p, &err));
}

TEST_CASE("FiberTailRelease: R = 0 is the feature disabled, not a failure", "[Fiber][FiberTailRelease]")
{
    // Variant A must stay independently exportable, so a disabled release plans
    // valid with no path rather than reporting FS_RELEASE_UNSUPPORTED.
    ReleasePlan plan;
    std::string err;
    REQUIRE(plan_fiber_release(straight_100(), false, 0.2, params_a(), nullptr, plan, &err));
    CHECK(plan.valid);
    CHECK(plan.path.empty());
    CHECK(err.empty());
    CHECK(emit_fiber_release(plan).empty());
}

TEST_CASE("FiberTailRelease: an open release continues the final tangent and needs support", "[Fiber][FiberTailRelease]")
{
    TailReleaseParams p = params_b();
    ReleasePlan plan;
    std::string err;

    // Supported: the release stays inside deposited material of the layer.
    RectSupport inside(-5.0, -5.0, 120.0, 30.0);
    REQUIRE(plan_fiber_release(straight_100(), false, 0.2, p, &inside, plan, &err));
    CHECK(plan.valid);
    CHECK(plan.support == SupportVerdict::Supported);
    REQUIRE(plan.path.size() == 2);
    CHECK(plan.path.front().x == Approx(100.0).margin(1e-6));
    CHECK(plan.path.back().x == Approx(106.8).margin(1e-6));
    CHECK(plan.path.front().y == Approx(10.0).margin(1e-6));
    CHECK(plan.planned_mm == Approx(6.8).margin(1e-6));
    CHECK(inside.queried());
    // Half the shipped 0.7 mm composite bead, not an inherited T1 WIDTH.
    CHECK(inside.last_half_width() == Approx(0.35).margin(1e-6));
    // And it tracks the configured width rather than being a constant.
    TailReleaseParams wide = params_b();
    wide.bead_width_mm = 1.2;
    ReleasePlan wplan;
    err.clear();
    REQUIRE(plan_fiber_release(straight_100(), false, 0.2, wide, &inside, wplan, &err));
    CHECK(inside.last_half_width() == Approx(0.6).margin(1e-6));

    // A missing composite width is a configuration error, not a guess.
    TailReleaseParams nowidth = params_b();
    nowidth.bead_width_mm = 0.0;
    ReleasePlan nplan;
    err.clear();
    CHECK_FALSE(plan_fiber_release(straight_100(), false, 0.2, nowidth, &inside, nplan, &err));

    // Unsupported: the footprint leaves the deposited material. The export must
    // fail rather than shorten R or fall back to R = 0.
    RectSupport outside(-5.0, -5.0, 103.0, 30.0);
    ReleasePlan bad;
    err.clear();
    CHECK_FALSE(plan_fiber_release(straight_100(), false, 0.2, p, &outside, bad, &err));
    CHECK(err.find("FS_RELEASE_UNSUPPORTED") != std::string::npos);
    CHECK(bad.valid == false);

    // No oracle at all is also a failure: an open-air extension is never
    // guessed at.
    ReleasePlan nogeom;
    err.clear();
    CHECK_FALSE(plan_fiber_release(straight_100(), false, 0.2, p, nullptr, nogeom, &err));
    CHECK(err.find("FS_RELEASE_UNSUPPORTED") != std::string::npos);
    CHECK(nogeom.support == SupportVerdict::NotEvaluated);
}

TEST_CASE("FiberTailRelease: an open strand whose final run is shorter than the anchor is refused", "[Fiber][FiberTailRelease]")
{
    // The final run is 5 mm, short of the 8 mm anchor, so there is no qualified
    // straight approach to continue. Note a 90 degree turn at the START of a
    // long-enough final run is NOT this case: the run itself is straight, so
    // its tangent is well defined and the release is legal.
    std::vector<FiberPoint> pts{FiberPoint{0.0, 0.0}, FiberPoint{50.0, 0.0}, FiberPoint{50.0, 5.0}};
    TailReleaseParams p = params_b();
    RectSupport everywhere(-100.0, -100.0, 200.0, 200.0);
    ReleasePlan plan;
    std::string err;
    CHECK_FALSE(plan_fiber_release(pts, false, 0.2, p, &everywhere, plan, &err));
    CHECK(err.find("FS_RELEASE_UNSUPPORTED") != std::string::npos);

    // Same geometry with a longer final run: now the anchor is satisfied and the
    // release follows the vertical tangent.
    std::vector<FiberPoint> long_run{FiberPoint{0.0, 0.0}, FiberPoint{50.0, 0.0}, FiberPoint{50.0, 40.0}};
    ReleasePlan ok;
    err.clear();
    REQUIRE(plan_fiber_release(long_run, false, 0.2, p, &everywhere, ok, &err));
    CHECK(ok.valid);
    CHECK(ok.path.back().y == Approx(46.8).margin(1e-6)); // straight on up the tangent
    CHECK(ok.path.back().x == Approx(50.0).margin(1e-6));
}

TEST_CASE("FiberTailRelease: the emitted release block is dry", "[Fiber][FiberTailRelease]")
{
    TailReleaseParams p = params_b();
    RectSupport inside(-5.0, -5.0, 120.0, 30.0);
    ReleasePlan plan;
    std::string err;
    REQUIRE(plan_fiber_release(straight_100(), false, 0.2, p, &inside, plan, &err));

    const std::string block = emit_fiber_release(plan);
    CHECK(block.find("; FS_RELEASE_BEGIN length_mm=6.800 speed_mm_s=10.000") != std::string::npos);
    CHECK(block.find("G1 X106.800 Y10.000 F600") != std::string::npos);
    CHECK(block.find("; FS_RELEASE_END physical_clearance=unmeasured") != std::string::npos);

    // The whole point: no material word anywhere in the block.
    CHECK(block.find(" U") == std::string::npos);
    CHECK(block.find(" V") == std::string::npos);
    CHECK(block.find(" E") == std::string::npos);
    CHECK(block.find(" Z") == std::string::npos);
    // And the end comment never claims a measured clearance.
    CHECK(block.find("physical_clearance=unmeasured") != std::string::npos);
}

TEST_CASE("FiberTailRelease: straight runs are found and a wrapping run is seen once", "[Fiber][FiberTailRelease]")
{
    const auto runs = find_straight_runs(rect_closed(), true);
    REQUIRE_FALSE(runs.empty());
    // The 40 mm bottom edge is the longest single straight run.
    double longest = 0.0;
    for (const StraightRun& r : runs)
        longest = std::max(longest, r.length_mm);
    CHECK(longest == Approx(40.0).margin(1e-3));
    for (const StraightRun& r : runs) {
        CHECK(r.heading_spread_deg <= 2.0 + 1e-9);
        CHECK(r.max_deviation_mm <= 0.05 + 1e-9);
    }
}

TEST_CASE("FiberTailRelease: the seam is deterministic and the margin does not rank it", "[Fiber][FiberTailRelease]")
{
    // B and C differ only in M. The seam must be identical, which is what makes
    // the comparison exports a clean test of the margin.
    const auto pts = rect_closed();
    ReleaseSeam b_seam, c_seam;
    std::string err;
    REQUIRE(select_release_seam(pts, true, params_b(), b_seam, &err));
    REQUIRE(select_release_seam(pts, true, params_c(), c_seam, &err));
    CHECK(b_seam.valid);
    CHECK(c_seam.valid);
    CHECK(b_seam.seam_distance_mm == Approx(c_seam.seam_distance_mm).margin(1e-9));
    CHECK(b_seam.run_first_seg == c_seam.run_first_seg);
    CHECK(b_seam.run_length_mm == Approx(c_seam.run_length_mm).margin(1e-9));

    // The seam sits R + 0.01 before the chosen run's forward end.
    CHECK(b_seam.seam_distance_mm ==
          Approx(b_seam.run_length_mm - (params_b().release_mm + 0.01)).margin(1e-6));
}

TEST_CASE("FiberTailRelease: a run too short for anchor plus release fails the export", "[Fiber][FiberTailRelease]")
{
    // A 10 x 6 rectangle: no run reaches anchor 8 + release 6.8 + 0.02.
    std::vector<FiberPoint> small{FiberPoint{0.0, 0.0}, FiberPoint{10.0, 0.0}, FiberPoint{10.0, 6.0},
                                  FiberPoint{0.0, 6.0}, FiberPoint{0.0, 0.0}};
    ReleaseSeam seam;
    std::string err;
    CHECK_FALSE(select_release_seam(small, true, params_b(), seam, &err));
    CHECK(err.find("FS_RELEASE_UNSUPPORTED") != std::string::npos);
    CHECK(seam.valid == false);
    CHECK_FALSE(seam.rejected.empty()); // the reason is recorded, not swallowed
}

TEST_CASE("FiberTailRelease: seam selection refuses an open strand rather than guessing", "[Fiber][FiberTailRelease]")
{
    ReleaseSeam seam;
    std::string err;
    CHECK_FALSE(select_release_seam(straight_100(), false, params_b(), seam, &err));
    CHECK(err.find("FS_RELEASE_UNSUPPORTED") != std::string::npos);
}

TEST_CASE("FiberTailRelease: rotating a closed path preserves the locus and the direction", "[Fiber][FiberTailRelease]")
{
    const auto pts = rect_closed();
    ReleaseSeam seam;
    std::string err;
    REQUIRE(select_release_seam(pts, true, params_b(), seam, &err));

    const auto rotated = rotate_closed_path(pts, seam.seam_distance_mm);
    REQUIRE(rotated.size() >= 2);
    // Still closed.
    const double gap = std::hypot(rotated.front().x - rotated.back().x,
                                  rotated.front().y - rotated.back().y);
    CHECK(gap <= 0.05);
    // Same total length: rotation moves the start, never the geometry.
    double orig = 0.0, rot = 0.0;
    for (size_t i = 0; i + 1 < pts.size(); ++i)
        orig += std::hypot(pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y);
    for (size_t i = 0; i + 1 < rotated.size(); ++i)
        rot += std::hypot(rotated[i + 1].x - rotated[i].x, rotated[i + 1].y - rotated[i].y);
    CHECK(rot == Approx(orig).margin(1e-6));
    // The seam lies on the path, so the rotated start is a point of the original.
    bool on_path = false;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const double ax = pts[i].x, ay = pts[i].y;
        const double dx = pts[i + 1].x - ax, dy = pts[i + 1].y - ay;
        const double len2 = dx * dx + dy * dy;
        if (len2 <= 0.0)
            continue;
        double t = ((rotated.front().x - ax) * dx + (rotated.front().y - ay) * dy) / len2;
        if (t < -1e-9 || t > 1.0 + 1e-9)
            continue;
        const double px = ax + dx * t, py = ay + dy * t;
        if (std::hypot(px - rotated.front().x, py - rotated.front().y) < 1e-6)
            on_path = true;
    }
    CHECK(on_path);
}

TEST_CASE("FiberTailRelease: a closed release retraces the first R mm forward", "[Fiber][FiberTailRelease]")
{
    const auto pts = rect_closed();
    ReleaseSeam seam;
    std::string err;
    REQUIRE(select_release_seam(pts, true, params_b(), seam, &err));
    const auto rotated = rotate_closed_path(pts, seam.seam_distance_mm);

    TailReleaseParams p = params_b();
    ReleasePlan plan;
    REQUIRE(plan_fiber_release(rotated, true, 0.2, p, nullptr, plan, &err));
    CHECK(plan.valid);
    CHECK(plan.planned_mm == Approx(6.8).margin(1e-3));
    CHECK(plan.support == SupportVerdict::Supported); // over its own deposition
    // Forward, not reversed: the release leaves the seam in the path's own
    // traversal direction.
    CHECK(plan.path.size() >= 2);
    CHECK(std::hypot(plan.path.back().x - plan.path.front().x,
                     plan.path.back().y - plan.path.front().y) > 0.0);
}

TEST_CASE("FiberTailRelease: the preheat clock costs temperature waits and macros nothing", "[Fiber][FiberTailRelease]")
{
    CHECK(motion_block_nominal_seconds(temp_wait()) == 0.0);
    CHECK(motion_block_nominal_seconds(macro_block()) == 0.0);

    // 120 mm at F3000 = 2.4 s.
    CHECK(motion_block_nominal_seconds(deposit(120.0, 3000.0)) == Approx(2.4).margin(1e-9));
    // Stationary uses max material, not path.
    CHECK(motion_block_nominal_seconds(stationary(4.0, 600.0)) == Approx(0.4).margin(1e-9));

    MotionBlock dwell;
    dwell.kind = MotionBlock::Kind::Dwell;
    dwell.dwell_s = 3.0;
    CHECK(motion_block_nominal_seconds(dwell) == Approx(3.0).margin(1e-9));

    // Degenerate inputs invent nothing.
    CHECK(motion_block_nominal_seconds(deposit(100.0, 0.0)) == 0.0);
    CHECK(motion_block_nominal_seconds(deposit(-5.0, 3000.0)) == 0.0);
}

TEST_CASE("FiberTailRelease: the preheat lands lead_s before the activation endpoint", "[Fiber][FiberTailRelease]")
{
    // Six 3-second deposit blocks: a 18 s activation.
    std::vector<MotionBlock> act;
    for (int i = 0; i < 6; ++i)
        act.push_back(deposit(150.0, 3000.0)); // 3 s each
    REQUIRE(nominal_activation_seconds(act) == Approx(18.0).margin(1e-9));

    const PreheatPlan p = plan_tool_preheat(act, 1, 250, 15.0);
    CHECK(p.valid);
    CHECK(p.inserted);
    CHECK(p.tool == 1);
    CHECK(p.target_c == 250);
    CHECK(p.endpoint_s == Approx(18.0).margin(1e-9));
    // Target time 3 s: the insert goes before the block starting at 3 s.
    CHECK(p.insert_index == 1);
    CHECK(p.nominal_lead_s == Approx(15.0).margin(1e-9));
    CHECK(p.clamped_to_activation == false);

    const std::string line = emit_tool_preheat(p);
    CHECK(line.find("M104 S250 T1 ; FS_PREHEAT next_tool=1 lead_s=15") != std::string::npos);
}

TEST_CASE("FiberTailRelease: a lead longer than the activation clamps to its start", "[Fiber][FiberTailRelease]")
{
    // The spec's own short-purge case: a 4 s activation cannot carry a 15 s
    // lead, so the preheat goes right after the entry setup and the clamp is
    // recorded rather than the preheat dropped.
    std::vector<MotionBlock> act{deposit(200.0, 3000.0)}; // 4 s
    const PreheatPlan p = plan_tool_preheat(act, 1, 250, 15.0);
    CHECK(p.valid);
    CHECK(p.inserted);
    CHECK(p.insert_index == 0);
    CHECK(p.clamped_to_activation);
    CHECK(p.nominal_lead_s == Approx(4.0).margin(1e-9));
}

TEST_CASE("FiberTailRelease: lead zero still sets the target", "[Fiber][FiberTailRelease]")
{
    // Zero means no predictive lead, not removal of temperature protection.
    std::vector<MotionBlock> act{deposit(150.0, 3000.0), deposit(150.0, 3000.0)};
    const PreheatPlan p = plan_tool_preheat(act, 1, 250, 0.0);
    CHECK(p.valid);
    CHECK(p.inserted);
    CHECK(p.insert_index == 2); // at the end: the target is still commanded
    CHECK(p.requested_lead_s == 0.0);
    CHECK_FALSE(emit_tool_preheat(p).empty());
}

TEST_CASE("FiberTailRelease: a disabled thermal target emits no heat command", "[Fiber][FiberTailRelease]")
{
    // fs_t0_temp = 0 means the machine owns that head. A disabled gate must not
    // become a heater command.
    std::vector<MotionBlock> act{deposit(150.0, 3000.0)};
    const PreheatPlan p = plan_tool_preheat(act, 0, 0, 15.0);
    CHECK_FALSE(p.valid);
    CHECK_FALSE(p.inserted);
    CHECK(emit_tool_preheat(p).empty());
    REQUIRE_FALSE(p.findings.empty());
    CHECK(std::string(fs_code_name(p.findings.front().code)) == "THERMAL_MANAGEMENT_DISABLED");
}

TEST_CASE("FiberTailRelease: an empty activation still gets the target at index zero", "[Fiber][FiberTailRelease]")
{
    // Nothing modelled at all (a purge with no timed blocks). The plan must not
    // read past the end of the activation.
    const PreheatPlan p = plan_tool_preheat({}, 1, 250, 15.0);
    CHECK(p.valid);
    CHECK(p.insert_index == 0);
    CHECK(p.clamped_to_activation);
    CHECK(p.endpoint_s == 0.0);
}

TEST_CASE("FiberTailRelease: the E ledger never stacks and recovers exactly once", "[Fiber][FiberTailRelease]")
{
    EWithdrawalLedger led;
    CHECK_FALSE(led.owed());
    CHECK(led.recover() == 0.0);

    led.withdraw(10.0); // the tool-change withdrawal
    CHECK(led.owed());
    CHECK(led.pending_mm() == Approx(10.0).margin(1e-9));
    // A 1 mm travel retraction on top of a pending 10 mm is redundant.
    CHECK(led.covers(1.0));
    CHECK_FALSE(led.covers(10.5));

    // Ordinary travel on top of a real pending amount accumulates, it does not
    // double-book the same travel.
    led.withdraw(2.0);
    CHECK(led.pending_mm() == Approx(12.0).margin(1e-9));

    CHECK(led.recover() == Approx(12.0).margin(1e-9));
    CHECK_FALSE(led.owed());
    CHECK(led.pending_mm() == 0.0);
    CHECK(led.recover() == 0.0); // exactly once

    // Garbage in is ignored rather than corrupting the ledger.
    led.withdraw(-3.0);
    led.withdraw(std::nan(""));
    CHECK_FALSE(led.owed());
}

TEST_CASE("FiberTailRelease: code names and severities match the spec", "[Fiber][FiberTailRelease]")
{
    CHECK(std::string(fs_code_name(FsCode::WaitOutsideStation)) == "FS_WAIT_OUTSIDE_STATION");
    CHECK(std::string(fs_code_name(FsCode::CoolBeforeRelease)) == "FS_COOL_BEFORE_RELEASE");
    CHECK(std::string(fs_code_name(FsCode::CoolBeforeClean)) == "FS_COOL_BEFORE_CLEAN");
    CHECK(std::string(fs_code_name(FsCode::PreheatClobbered)) == "FS_PREHEAT_CLOBBERED");
    CHECK(std::string(fs_code_name(FsCode::ReleaseNotExecuted)) == "FS_RELEASE_NOT_EXECUTED");
    CHECK(std::string(fs_code_name(FsCode::ReleaseExtrusion)) == "FS_RELEASE_EXTRUSION");
    CHECK(std::string(fs_code_name(FsCode::ReleaseUnsupported)) == "FS_RELEASE_UNSUPPORTED");
    CHECK(std::string(fs_code_name(FsCode::TailDistanceMismatch)) == "FS_TAIL_DISTANCE_MISMATCH");
    CHECK(std::string(fs_code_name(FsCode::ERecoveryLocation)) == "FS_E_RECOVERY_LOCATION");
    CHECK(std::string(fs_code_name(FsCode::TailSharpTurn)) == "FS_TAIL_SHARP_TURN");
    CHECK(std::string(fs_code_name(FsCode::MacroContractUnverified)) == "FS_MACRO_CONTRACT_UNVERIFIED");
    CHECK(std::string(fs_code_name(FsCode::NonblockingHeatUnsupported)) == "FS_NONBLOCKING_HEAT_UNSUPPORTED");
    CHECK(std::string(fs_code_name(FsCode::ThermalManagementDisabled)) == "THERMAL_MANAGEMENT_DISABLED");
    CHECK(std::string(fs_code_name(FsCode::PurgeReleaseOutOfBounds)) == "FS_PURGE_RELEASE_OUT_OF_BOUNDS");

    // Exactly two WARN codes; everything else is an ERROR.
    CHECK(fs_code_is_warning(FsCode::TailSharpTurn));
    CHECK(fs_code_is_warning(FsCode::MacroContractUnverified));
    CHECK_FALSE(fs_code_is_warning(FsCode::ReleaseUnsupported));
    CHECK_FALSE(fs_code_is_warning(FsCode::WaitOutsideStation));
    CHECK_FALSE(fs_code_is_warning(FsCode::TailDistanceMismatch));
    CHECK_FALSE(fs_code_is_warning(FsCode::ReleaseExtrusion));
}

TEST_CASE("FiberTailRelease: the evidence record keeps its honesty fields", "[Fiber][FiberTailRelease]")
{
    TailReleaseEvidence e;
    e.object_id = "shook";
    e.window_id = "W3";
    e.selected_seam = "run@0.000";
    e.margin_mm = 1.0;
    e.requested_release_mm = 6.8;
    e.actual_release_mm = 6.8;
    e.post_cut_deposit_mm = 55.8;
    e.support_result = "NOT_EVALUATED";

    const std::string j = evidence_to_json(e);
    CHECK(j.find("\"object_id\": \"shook\"") != std::string::npos);
    CHECK(j.find("\"margin_mm\": 1.000") != std::string::npos);
    // The two fields the spec pins, and the default that keeps them honest.
    CHECK(j.find("\"physical_tail_clearance\": \"unmeasured\"") != std::string::npos);
    CHECK(j.find("\"macro_contract\": \"MACRO_CONTRACT_UNVERIFIED\"") != std::string::npos);
    CHECK(j.find("\"support_result\": \"NOT_EVALUATED\"") != std::string::npos);
}

TEST_CASE("FiberTailRelease: the manifest states the clock and its limits", "[Fiber][FiberTailRelease]")
{
    std::vector<TailReleaseEvidence> recs(2);
    recs[0].window_id = "W1";
    recs[1].window_id = "W2";
    const std::string j = evidence_manifest_to_json(recs, "shook_C_release_margin_1mm");
    CHECK(j.find("\"variant\": \"shook_C_release_margin_1mm\"") != std::string::npos);
    CHECK(j.find("\"record_count\": 2") != std::string::npos);
    CHECK(j.find("temperature waits and opaque macro calls contribute zero") != std::string::npos);
    CHECK(j.find("acceleration, heater behaviour and hidden macro duration are not modelled") != std::string::npos);
    CHECK(j.find("UNTESTED until the operator supplies them") != std::string::npos);
    CHECK(j.find("\"window_id\": \"W1\"") != std::string::npos);
    CHECK(j.find("\"window_id\": \"W2\"") != std::string::npos);
}

TEST_CASE("FiberTailRelease: degenerate geometry never crashes and never invents a plan", "[Fiber][FiberTailRelease]")
{
    TailReleaseParams p = params_b();
    ReleasePlan plan;
    std::string err;

    // Fewer than two points.
    CHECK_FALSE(plan_fiber_release({}, false, 0.2, p, nullptr, plan, &err));
    CHECK_FALSE(plan_fiber_release({FiberPoint{1.0, 1.0}}, false, 0.2, p, nullptr, plan, &err));

    // A repeated point (zero-length final segment) has no tangent to continue.
    std::vector<FiberPoint> dup{FiberPoint{0.0, 0.0}, FiberPoint{50.0, 0.0}, FiberPoint{50.0, 0.0}};
    RectSupport everywhere(-100.0, -100.0, 200.0, 200.0);
    CHECK_FALSE(plan_fiber_release(dup, false, 0.2, p, &everywhere, plan, &err));

    // Non-finite parameter values are a configuration error, not a crash.
    TailReleaseParams nan_p = params_b();
    nan_p.release_mm = std::nan("");
    CHECK_FALSE(plan_fiber_release(straight_100(), false, 0.2, nan_p, &everywhere, plan, &err));

    // Rotation of a degenerate path returns nothing rather than a bogus path.
    CHECK(rotate_closed_path({}, 5.0).empty());
    CHECK(rotate_closed_path({FiberPoint{2.0, 2.0}}, 5.0).empty());

    // A footprint of zero width is no footprint.
    CHECK(release_footprint(straight_100(), 0.0).empty());
    CHECK(release_footprint(straight_100(), 0.35).size() == 4);
}
