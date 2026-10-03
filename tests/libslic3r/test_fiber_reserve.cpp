// License: GNU AGPLv3 or higher
//
// FiberReserve unit tests ([Fiber]). Contract under test: plastic is removed
// from the exclusion band d(w) = composite/2 + w/2 - bond_overlap around the
// FINAL accepted strand geometry (body AND post-cut tail), per planned plastic
// bead width; unhit entities keep their exact pointers; d(w) <= 0 disables
// subtraction; fragments keep role / width / height; the outer_wall diagnostic
// drops external perimeters only.

#include <catch2/catch_all.hpp>
using Catch::Approx;   // Catch2 v3 scopes Approx under Catch:: (v2 had it at global scope)

#include <cmath>
#include <memory>
#include <vector>

#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Fiber/FiberReserve.hpp"
#include "libslic3r/Fiber/FiberStrand.hpp"
#include "libslic3r/Fiber/FiberStrandPlanner.hpp"

using namespace Slic3r;
using namespace Slic3r::Fiber;

namespace {

// Strand carrying explicit printed body/tail geometry (what the reserve pass
// consumes; finalize() bookkeeping is the planner's contract, tested there).
FiberStrand strand_from_pts(const std::vector<FiberPoint>& body,
                            const std::vector<FiberPoint>& tail)
{
    FiberStrand s;
    s.layer_id = 1;
    s.z        = 1.0;
    s.body_pts = body;
    s.tail_pts = tail;
    return s;
}

ReserveParams params(double composite = 0.4, double overlap = 0.1, Vec2d origin = {0.0, 0.0})
{
    ReserveParams rp;
    rp.composite_width_mm = composite;
    rp.bond_overlap_mm    = overlap;
    rp.origin_offset_mm   = origin;
    return rp;
}

// Horizontal fiber strand along y=10, x in [0,100], no tail.
std::vector<FiberStrand> fiber_line_y10()
{
    return { strand_from_pts({{0.0, 10.0}, {100.0, 10.0}}, {}) };
}

// mm-space leaf path with explicit planned width.
ExtrusionPath* make_path(ExtrusionRole role, double width_mm,
                         const std::vector<std::pair<double, double>>& xy)
{
    Points3 pts3;
    pts3.reserve(xy.size());
    for (const auto& q : xy)
        // new_scale, not emplace_back(coord_t, coord_t, 0): the int32/int64/
        // double Point3 overloads are ambiguous for mixed coord_t/int arguments.
        pts3.push_back(Point3::new_scale(q.first, q.second, 0.0));
    ExtrusionPath* p = new ExtrusionPath(role, 0.1, float(width_mm), 0.2f);
    p->polyline = Polyline3(std::move(pts3));
    return p;
}

ExtrusionEntityCollection make_collection(ExtrusionEntitiesPtr&& leaves)
{
    ExtrusionEntityCollection col;
    col.append(std::move(leaves));
    return col;
}

// Collection frame coordinates are scaled bed mm; these tests use origin (0,0)
// unless stated otherwise, so path coordinates equal bed mm.
size_t count_leaves(const ExtrusionEntityCollection& col)
{
    size_t n = 0;
    for (const ExtrusionEntity* e : col.entities)
        n += e->is_collection() ? count_leaves(static_cast<const ExtrusionEntityCollection&>(*e)) : 1;
    return n;
}

double total_path_length(const ExtrusionEntityCollection& col)
{
    double len = 0.0;
    for (const ExtrusionEntity* e : col.entities) {
        if (e->is_collection())
            len += total_path_length(static_cast<const ExtrusionEntityCollection&>(*e));
        else if (!e->is_loop())
            // MultiPoint::length() returns SCALED units in this fork; the
            // assertions below are in millimeters.
            len += unscale<double>(e->length());
    }
    return len;
}

} // namespace

TEST_CASE("FiberReserve: unhit plastic survives untouched", "[Fiber][FiberReserve]")
{
    auto col = make_collection({ make_path(erPerimeter, 0.4, {{0.0, 0.0}, {100.0, 0.0}}) });
    const size_t removed = subtract_reserve_bands(col, fiber_line_y10(), params());
    REQUIRE(removed == 0);
    REQUIRE(count_leaves(col) == 1);
}

TEST_CASE("FiberReserve: crossing path fragments around the band", "[Fiber][FiberReserve]")
{
    // Fiber along y=10 (x 0..100). Band half-width d(0.4) = 0.2+0.2-0.1 = 0.3.
    // Vertical path (20,5)->(20,15) must split at y = 9.7 and 10.3.
    auto col = make_collection({ make_path(erPerimeter, 0.4, {{20.0, 5.0}, {20.0, 15.0}}) });
    const size_t removed = subtract_reserve_bands(col, fiber_line_y10(), params());
    REQUIRE(removed == 1);
    REQUIRE(count_leaves(col) == 2);
    double lens[2] = { 0.0, 0.0 };
    size_t i = 0;
    for (const ExtrusionEntity* e : col.entities) {
        REQUIRE(e->role() == erPerimeter);
        REQUIRE(static_cast<const ExtrusionPath*>(e)->width == Approx(0.4));
        lens[i++] = unscale<double>(e->length());   // scaled units -> mm
    }
    REQUIRE(lens[0] == Approx(4.7).margin(0.02));
    REQUIRE(lens[1] == Approx(4.7).margin(0.02));
}

TEST_CASE("FiberReserve: clipped plastic shorter than 1 mm is dropped", "[Fiber][FiberReserve]")
{
    // Band ends at y=10.3. A crossing that only just clears it would leave a
    // 0.15 mm stub; that is not a bead, so only the long remainder survives.
    auto col = make_collection({ make_path(erSolidInfill, 0.4, {{50.0, 5.0}, {50.0, 10.45}}) });
    const size_t removed = subtract_reserve_bands(col, fiber_line_y10(), params());
    REQUIRE(removed == 1);
    REQUIRE(count_leaves(col) == 1);
    CHECK(unscale<double>(col.entities.front()->length()) == Approx(4.7).margin(0.2));
}

TEST_CASE("FiberReserve: fully engulfed path is removed", "[Fiber][FiberReserve]")
{
    // Collinear with the fiber: entirely inside the capsule.
    auto col = make_collection({ make_path(erInternalInfill, 0.4, {{10.0, 10.0}, {90.0, 10.0}}) });
    const size_t removed = subtract_reserve_bands(col, fiber_line_y10(), params());
    REQUIRE(removed == 1);
    REQUIRE(count_leaves(col) == 0);
}

TEST_CASE("FiberReserve: ring strand reserves its own perimeter loop", "[Fiber][FiberReserve]")
{
    // Closed ring strand (closing duplicate preserved) matching an external
    // perimeter loop: the loop must be fragmented away entirely.
    const std::vector<FiberPoint> ring{ {10.0, 10.0}, {50.0, 10.0}, {50.0, 30.0}, {10.0, 30.0}, {10.0, 10.0} };
    std::vector<FiberStrand> strands{ strand_from_pts(ring, {}) };

    ExtrusionPaths loop_paths;
    loop_paths.emplace_back(erExternalPerimeter, 0.1, 0.4f, 0.2f);
    {
        // Point3 in this fork is not brace-initializable from three coord_t
        // values (copy-list-init picks the explicit Eigen ctor); use new_scale.
        Points3 pts3{ Point3::new_scale(10.0, 10.0, 0.0),
                      Point3::new_scale(50.0, 10.0, 0.0),
                      Point3::new_scale(50.0, 30.0, 0.0),
                      Point3::new_scale(10.0, 30.0, 0.0),
                      Point3::new_scale(10.0, 10.0, 0.0) };
        loop_paths.front().polyline = Polyline3(std::move(pts3));
    }
    ExtrusionEntityCollection col;
    col.append(ExtrusionEntitiesPtr{ new ExtrusionLoop(std::move(loop_paths)) });

    const size_t removed = subtract_reserve_bands(col, strands, params());
    REQUIRE(removed == 1);
    REQUIRE(count_leaves(col) == 0);
}

TEST_CASE("FiberReserve: d(w) <= 0 disables subtraction", "[Fiber][FiberReserve]")
{
    // overlap 0.5 >= composite/2 + w/2 = 0.4 for w = 0.4: touching allowed.
    auto col = make_collection({ make_path(erPerimeter, 0.4, {{10.0, 10.0}, {90.0, 10.0}}) });
    const size_t removed = subtract_reserve_bands(col, fiber_line_y10(), params(0.4, 0.5));
    REQUIRE(removed == 0);
    REQUIRE(count_leaves(col) == 1);
}

TEST_CASE("FiberReserve: exclusion distance is per bead width", "[Fiber][FiberReserve]")
{
    // Parallel offsets at y = 10.25 (thin 0.2 -> d = 0.2 < 0.25: intact) and
    // y = 10.3 (wide 0.6 -> d = 0.4 > 0.3: engulfed).
    ExtrusionEntitiesPtr leaves;
    leaves.push_back(make_path(erPerimeter, 0.2, {{0.0, 10.25}, {100.0, 10.25}}));
    leaves.push_back(make_path(erPerimeter, 0.6, {{0.0, 10.3}, {100.0, 10.3}}));
    auto col = make_collection(std::move(leaves));
    const size_t removed = subtract_reserve_bands(col, fiber_line_y10(), params());
    REQUIRE(removed == 1);
    REQUIRE(count_leaves(col) == 1);
    REQUIRE(static_cast<const ExtrusionPath*>(col.entities[0])->width == Approx(0.2));
}

TEST_CASE("FiberReserve: post-cut tail reserves too", "[Fiber][FiberReserve]")
{
    // Body stops at x = 90, tail continues to x = 110 (operator requirement
    // R2.1-2). A crossing path over the tail-only zone must be clipped.
    std::vector<FiberStrand> strands{ strand_from_pts(
        { {0.0, 10.0}, {90.0, 10.0} }, { {90.0, 10.0}, {110.0, 10.0} }) };
    auto col = make_collection({ make_path(erSolidInfill, 0.4, {{100.0, 5.0}, {100.0, 15.0}}) });
    const size_t removed = subtract_reserve_bands(col, strands, params());
    REQUIRE(removed == 1);
    REQUIRE(count_leaves(col) == 2);

    // Control: the same crossing at x = 100 with an empty tail is untouched.
    auto col2 = make_collection({ make_path(erSolidInfill, 0.4, {{100.0, 5.0}, {100.0, 15.0}}) });
    std::vector<FiberStrand> body_only{ strand_from_pts({ {0.0, 10.0}, {90.0, 10.0} }, {}) };
    REQUIRE(subtract_reserve_bands(col2, body_only, params()) == 0);
}

TEST_CASE("FiberReserve: the strand's opening segment reserves too", "[Fiber][FiberReserve]")
{
    // body_pts holds move DESTINATIONS, so the first deposit runs from
    // pts.front() to body_pts.front() and that opening segment lays fiber like
    // any other. Reserving from body_pts alone left it uncovered, and plastic
    // survived directly on the strand's first segment.
    FiberStrand s = strand_from_pts({ {20.0, 10.0}, {100.0, 10.0} }, {});
    s.pts = { {0.0, 10.0}, {20.0, 10.0}, {100.0, 10.0} };
    std::vector<FiberStrand> strands{ s };

    // A crossing path at x = 10 sits on the opening segment only.
    auto col = make_collection({ make_path(erSolidInfill, 0.4, {{10.0, 5.0}, {10.0, 15.0}}) });
    const size_t removed = subtract_reserve_bands(col, strands, params());
    REQUIRE(removed == 1);
    REQUIRE(count_leaves(col) == 2);   // fragments either side of the band

    // Control: the same strand with no planned polyline reserves from
    // body_pts alone, so x = 10 is genuinely outside the deposited geometry.
    auto col2 = make_collection({ make_path(erSolidInfill, 0.4, {{10.0, 5.0}, {10.0, 15.0}}) });
    std::vector<FiberStrand> body_only{ strand_from_pts({ {20.0, 10.0}, {100.0, 10.0} }, {}) };
    REQUIRE(subtract_reserve_bands(col2, body_only, params()) == 0);
}

// A multipath is neither a loop nor a collection, so the pass has to recognise
// it by type. Reading one as a plain path yields a garbage bead width, which
// used to turn into an unbounded offset and abort the slice.
TEST_CASE("FiberReserve: a multipath crossing the band is clipped", "[Fiber][FiberReserve]")
{
    ExtrusionMultiPath* mp = new ExtrusionMultiPath();
    for (const auto& seg : { std::make_pair(5.0, 20.0), std::make_pair(20.0, 40.0) }) {
        std::unique_ptr<ExtrusionPath> p(make_path(erInternalInfill, 0.4,
                                                   {{seg.first, 10.0}, {seg.second, 10.0}}));
        mp->paths.push_back(*p);
    }
    auto col = make_collection({ mp });
    REQUIRE(count_leaves(col) == 1);
    const size_t removed = subtract_reserve_bands(col, fiber_line_y10(), params());
    REQUIRE(removed == 1);
    // The fiber runs along y = 10 from x = 0 to x = 100, so the whole
    // multipath lies inside the band and nothing survives.
    CHECK(count_leaves(col) == 0);
    CHECK(total_path_length(col) == 0.0);
}

TEST_CASE("FiberReserve: origin offset frame respected", "[Fiber][FiberReserve]")
{
    // Fiber bed y=10, origin (5,5): collection frame fiber line at y=5.
    // Plastic path (5,0)-(5,15) crosses the frame line at (5,5): it must be
    // clipped there (proving the band was placed in the collection frame, not
    // the bed frame, which would have missed this path entirely).
    auto col = make_collection({ make_path(erPerimeter, 0.4, {{5.0, 0.0}, {5.0, 15.0}}) });
    const size_t removed = subtract_reserve_bands(col, fiber_line_y10(), params(0.4, 0.1, {5.0, 5.0}));
    REQUIRE(removed == 1);
    REQUIRE(count_leaves(col) == 2);   // fragments below y=4.7 and above y=5.3
}

// The reason this pass needs no "replace refused fiber with plastic solid"
// backfill (the vendor ships one): the band is a pure function of the ACCEPTED
// strands, so a refused strand reserves nothing and the plastic that would have
// been displaced by it is left exactly as the plastic generator produced it.
TEST_CASE("FiberReserve: a refused strand reserves nothing", "[Fiber][FiberReserve]")
{
    Fiber::StrandLayerParams p;
    p.layer_id       = 7;
    p.z              = 1.44;
    p.ratio_p        = 4.0;
    p.fiber_rate     = 0.23;
    p.feed_mm_min    = 600.0;
    p.tail_length_mm = 54.8;
    p.pitch_mm       = 2.0;
    // Pitch 150 erodes empty at level 1, so the interior fill produces no
    // chords at all: the only strand is the boundary trace near the walls.
    p.pitch_mm = 150.0;
    const std::vector<FiberPoint> ring{ {0.0, 0.0}, {60.0, 0.0}, {60.0, 40.0}, {0.0, 40.0} };
    const Fiber::StrandLayerResult res = Fiber::build_layer_strands({ ring }, p);
    REQUIRE(res.strands.size() == 1);

    // Plastic down the middle of the rectangle, where the refused fill would
    // have run, well clear of the surviving boundary trace.
    ExtrusionEntity* mid = make_path(erInternalInfill, 0.4, {{30.0, 10.0}, {30.0, 30.0}});
    auto col = make_collection({ mid });
    CHECK(subtract_reserve_bands(col, res.strands, params()) == 0);
    REQUIRE(col.entities.size() == 1);
    CHECK(col.entities.front() == mid);   // same object: nothing to backfill
}

TEST_CASE("FiberReserve: drop_external_perimeters is diagnostic-only scope", "[Fiber][FiberReserve]")
{
    ExtrusionEntitiesPtr inner;
    inner.push_back(make_path(erExternalPerimeter, 0.4, {{0.0, 0.0}, {10.0, 0.0}}));
    inner.push_back(make_path(erPerimeter, 0.4, {{0.0, 1.0}, {10.0, 1.0}}));
    inner.push_back(make_path(erInternalInfill, 0.4, {{0.0, 2.0}, {10.0, 2.0}}));
    ExtrusionEntityCollection* sub = new ExtrusionEntityCollection;
    sub->append(std::move(inner));
    auto col = make_collection({ sub });

    const size_t removed = drop_external_perimeters(col);
    REQUIRE(removed == 1);
    REQUIRE(count_leaves(col) == 2);

    // A sub-collection containing only externals is pruned entirely.
    ExtrusionEntityCollection* sub2 = new ExtrusionEntityCollection;
    sub2->append(ExtrusionEntitiesPtr{ make_path(erExternalPerimeter, 0.4, {{0.0, 0.0}, {10.0, 0.0}}) });
    auto col2 = make_collection({ sub2 });
    REQUIRE(drop_external_perimeters(col2) == 1);
    REQUIRE(col2.empty());
}

TEST_CASE("FiberReserve: determinism and untouched pointer identity", "[Fiber][FiberReserve]")
{
    ExtrusionEntity* hit  = make_path(erPerimeter, 0.4, {{20.0, 5.0}, {20.0, 15.0}});
    ExtrusionEntity* miss = make_path(erPerimeter, 0.4, {{0.0, 0.0}, {100.0, 0.0}});
    auto col = make_collection(ExtrusionEntitiesPtr{ hit, miss });
    const size_t removed = subtract_reserve_bands(col, fiber_line_y10(), params());
    REQUIRE(removed == 1);
    // The unhit entity is the very same object.
    REQUIRE(col.entities.size() == 3);
    bool miss_kept = false, hit_gone = true;
    for (ExtrusionEntity* e : col.entities) {
        if (e == miss)
            miss_kept = true;
        if (e == hit)
            hit_gone = false;
    }
    REQUIRE(miss_kept);
    REQUIRE(hit_gone);
    // Determinism: same inputs, same leaf count and total length.
    auto col2 = make_collection({ make_path(erPerimeter, 0.4, {{20.0, 5.0}, {20.0, 15.0}}),
                                  make_path(erPerimeter, 0.4, {{0.0, 0.0}, {100.0, 0.0}}) });
    REQUIRE(subtract_reserve_bands(col2, fiber_line_y10(), params()) == 1);
    REQUIRE(total_path_length(col2) == Approx(total_path_length(col)).margin(0.05));
}

