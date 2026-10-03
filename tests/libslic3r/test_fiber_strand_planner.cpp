// License: GNU AGPLv3 or higher
//
// FiberStrandPlanner unit tests ([Fiber]). Contract under test: layer perimeter
// rings become finalized, profile-following FiberStrands (concentric closed
// loops, deterministic order, reject-don't-simplify accounting), and each
// strand emits exactly one early-cut lifecycle via Fiber::emit_strand with the
// tail deposited along the strand's own path (never floating).

#include <catch2/catch_all.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "libslic3r/Fiber/FiberEmitter.hpp"
#include "libslic3r/Fiber/FiberStrandPlanner.hpp"

using namespace Slic3r;
using namespace Slic3r::Fiber;

namespace {

size_t count_sub(const std::string& hay, const std::string& needle)
{
    size_t n = 0;
    for (size_t pos = hay.find(needle); pos != std::string::npos; pos = hay.find(needle, pos + needle.size()))
        ++n;
    return n;
}

std::vector<std::string> split_lines(const std::string& s)
{
    std::vector<std::string> out;
    size_t start = 0;
    while (true) {
        const size_t nl = s.find('\n', start);
        if (nl == std::string::npos) {
            if (start < s.size())
                out.push_back(s.substr(start));
            break;
        }
        out.push_back(s.substr(start, nl - start));
        start = nl + 1;
    }
    return out;
}

// Closed ring (no duplicated closing vertex) from flat x,y pairs in mm.
std::vector<FiberPoint> ring(std::initializer_list<double> xy)
{
    std::vector<FiberPoint> r;
    for (auto it = xy.begin(); it != xy.end(); ++it) {
        const double x = *it;
        ++it;
        r.push_back({x, *it});
    }
    return r;
}

std::vector<FiberPoint> rect(double x0, double y0, double x1, double y1)
{
    return ring({x0, y0, x1, y0, x1, y1, x0, y1});
}

constexpr double TAU_HALF = 3.14159265358979323846; // local, PrintConfig.hpp also defines PI

// Block-S perimeter, uniform 3 mm thickness, 30 x 60 mm envelope, perimeter
// 288 mm (verified vertex-by-vertex below). Every corridor is thinner than a
// 2 mm concentric erosion, so the planner must emit exactly one strand: the S
// profile itself.
std::vector<FiberPoint> s_ring()
{
    return ring({10, 60, 40, 60, 40, 57, 13, 57, 13, 31, 40, 31,
                 40, 0,  10, 0,  10, 3,  37, 3,  37, 28, 10, 28});
}

double ring_len(const std::vector<FiberPoint>& r)
{
    double len = 0.0;
    const size_t n = r.size(); // fixture ring: no closing duplicate, closed edge implied
    for (size_t i = 0; i < n; ++i) {
        const FiberPoint& a = r[i];
        const FiberPoint& b = r[(i + 1) % n];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        len += std::sqrt(dx * dx + dy * dy);
    }
    return len;
}

StrandLayerParams base_params()
{
    StrandLayerParams p;
    p.layer_id       = 7;
    p.z              = 1.44;
    p.ratio_p        = 4.0;
    p.fiber_rate     = 0.23;
    p.feed_mm_min    = 600.0;
    p.tail_length_mm = 54.8;
    p.pitch_mm       = 2.0;
    return p;
}

double loop_signed_area(const std::vector<FiberPoint>& closed)
{
    double a = 0.0;
    const size_t n = closed.size() - 1; // loop, closing vertex duplicated
    for (size_t i = 0; i < n; ++i) {
        const FiberPoint& p = closed[i];
        const FiberPoint& q = closed[(i + 1) % n];
        a += p.x * q.y - q.x * p.y;
    }
    return 0.5 * a;
}

} // namespace

TEST_CASE("FiberStrandPlanner: S profile becomes one long strand", "[Fiber][FiberStrandPlanner]")
{
    const std::vector<FiberPoint> s = s_ring();
    REQUIRE(std::abs(ring_len(s) - 288.0) < 0.01); // the fixture itself

    std::string err;
    StrandLayerResult res = build_layer_strands({s}, base_params(), &err);
    CHECK(err.empty());
    CHECK(res.islands == 1);
    CHECK(res.viable_rings == 1);
    CHECK(res.rejected_short == 0);
    CHECK(res.rejected_other == 0);
    CHECK(res.skipped_tiny == 0);
    // Thin band: eroded empty at level 1, so exactly the profile trace.
    REQUIRE(res.strands.size() == 1);
    const FiberStrand& st = res.strands.front();
    CHECK(st.finalized);
    CHECK(st.total_path == Catch::Approx(288.0).margin(0.5));
    CHECK(st.total_path > 150.0); // a long strand, not a comb of ticks
    // U is paid only along the body prefix: the severed tail suffix is V-only
    // (matrix), so body feed is (path - tail) * rate, per-mm truncated.
    CHECK(st.total_u_feed == Catch::Approx(std::floor((st.total_path - 54.8) * 0.23 * 1000.0) / 1000.0).margin(0.1));
    CHECK(st.tail_pts.size() >= 1);
    // Tail is paid out at the body matrix rate (review finding #1): V = tail * rate * P.
    CHECK(st.total_v_tail == Catch::Approx(54.8 * 0.23 * 4.0).margin(0.02));
    // Deterministic start: canonical lexicographically smallest vertex (10,0).
    CHECK(st.pts.front().x == Catch::Approx(10.0));
    CHECK(st.pts.front().y == Catch::Approx(0.0));
    // Closed loop: the deposition returns to its start.
    CHECK(st.pts.back().x == Catch::Approx(st.pts.front().x).margin(1e-9));
    CHECK(st.pts.back().y == Catch::Approx(st.pts.front().y).margin(1e-9));
}

TEST_CASE("FiberStrandPlanner: thick island becomes concentric strands", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params(); // pitch 2
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 40, 30)}, p, &err);
    CHECK(err.empty());
    CHECK(res.islands == 1);
    CHECK(res.viable_rings == 1);
    // 40x30 eroded 2 mm per side per level: perimeters 140,124,108,92,76,60
    // are viable (>= 54.8 + 1 margin); 44 and 28 are short-feature rejects;
    // the next level (8 x -2) is empty and ends the schedule.
    CHECK(res.strands.size() == 6);
    CHECK(res.rejected_short == 2);
    for (const FiberStrand& st : res.strands)
        CHECK(st.finalized);
    // Outermost strand is the part boundary itself (140 mm perimeter).
    CHECK(res.strands.front().total_path == Catch::Approx(140.0).margin(0.5));
    // Strictly shrinking loops: each deeper strand is shorter.
    for (size_t i = 1; i < res.strands.size(); ++i)
        CHECK(res.strands[i].total_path < res.strands[i - 1].total_path);
}

TEST_CASE("FiberStrandPlanner: annulus reinforces hole contour too", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    // Both input rings arrive CCW (as the real perimeter producer emits them):
    // the planner must classify the inner ring as a hole by containment depth,
    // not by input winding. Pitch 150 > half the 90 mm annulus band, so the
    // plate erodes empty at level 1: exactly the two boundary traces remain.
    p.pitch_mm = 150.0;
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 200, 200), rect(90, 90, 110, 110)}, p, &err);
    CHECK(err.empty());
    CHECK(res.islands == 1);       // nesting -> one island with a hole
    CHECK(res.viable_rings == 2);   // outer boundary + hole boundary
    REQUIRE(res.strands.size() == 2);
    // Level 0: outer (800) then hole (80), ordered by lexicographic minimum.
    CHECK(res.strands[0].total_path == Catch::Approx(800.0).margin(1.0));
    CHECK(res.strands[1].total_path == Catch::Approx(80.0).margin(1.0));
    // Nothing is deposited across the hole: the hole loop traces its edge.
    for (const FiberPoint& fp : res.strands[1].pts)
        CHECK((fp.x <= 90.0 + 1e-6 || fp.x >= 110.0 - 1e-6 || fp.y <= 90.0 + 1e-6 || fp.y >= 110.0 - 1e-6));
}

TEST_CASE("FiberStrandPlanner: reject-don't-simplify accounting", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    // 1 mm blob: a real island, but far too short to carry a tail: counted as
    // a short-feature plastic fallback, never "simplified" into a stub strand.
    std::vector<FiberPoint> blob;
    for (int i = 0; i < 12; ++i) {
        const double a = 2.0 * TAU_HALF * double(i) / 12.0;
        blob.push_back({10.0 + 0.5 * std::cos(a), 10.0 + 0.5 * std::sin(a)});
    }
    StrandLayerResult short_res = build_layer_strands({blob}, p);
    CHECK(short_res.strands.empty());
    CHECK(short_res.islands == 1);
    CHECK(short_res.viable_rings == 0);   // not enforceable: physically unreinforceable
    CHECK(short_res.rejected_short == 1);
    CHECK(short_res.rejected_other == 0);

    // Dust: below the island threshold -> plastic-only, separate tally.
    StrandLayerResult dust = build_layer_strands({ring({5, 5, 5.1, 5, 5.05, 5.08})}, p);
    CHECK(dust.strands.empty());
    CHECK(dust.islands == 0);
    CHECK(dust.skipped_tiny == 1);
    CHECK(dust.viable_rings == 0);

    // Malformed ring (non-finite) is an "other" rejection, never silent.
    std::string err;
    std::vector<FiberPoint> bad = rect(0, 0, 10, 10);
    bad[2].x = std::nan("");
    StrandLayerResult bad_res = build_layer_strands({bad}, p, &err);
    CHECK(bad_res.strands.empty());
    CHECK(bad_res.rejected_other == 1);
    CHECK_FALSE(err.empty());
}

TEST_CASE("FiberStrandPlanner: direction alternates between islands", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    std::vector<std::vector<FiberPoint>> rings = {rect(0, 0, 40, 30), rect(60, 0, 100, 30)};
    StrandLayerResult res = build_layer_strands(rings, p);
    REQUIRE(res.strands.size() >= 2);
    // Equal areas -> bbox order: the x=0 island first. Both level-0 loops share
    // one geometry, so opposite deposition direction means opposite orientation.
    const double a0 = loop_signed_area(res.strands[0].pts);
    const double a1 = loop_signed_area(res.strands[1].pts);
    CHECK(a0 * a1 < 0.0);
}

TEST_CASE("FiberStrandPlanner: emission gives one early cut per strand, tail on path", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    StrandLayerResult res = build_layer_strands({rect(0, 0, 40, 30)}, p);
    REQUIRE(res.strands.size() == 6);

    FiberEmitParams ep;
    ep.restart_feed_mm = 55.0;
    std::string all;
    size_t emitted = 0;
    for (const FiberStrand& st : res.strands) {
        std::string out;
        ep.emit_layer_marker = emitted == 0;
        REQUIRE(emit_strand(st, ep, out));
        all += out;
        ++emitted;
    }
    // Exactly one cut lifecycle per strand - no per-chord cut storm.
    CHECK(count_sub(all, "M2800") == 6);
    CHECK(count_sub(all, "M1001 ") == 6);
    CHECK(count_sub(all, "; LAYER:") == 1); // one marker per layer
    // Tail section: V-bearing U-free moves along the remaining path, between
    // the cut and the handshake (never a separation move into empty air).
    const std::vector<std::string> lines = split_lines(all);
    bool in_tail = false;
    size_t tail_moves = 0;
    for (const std::string& ln : lines) {
        if (ln.find("; Start to cut") != std::string::npos)
            in_tail = true;
        else if (ln.find("; Cutting completed.") != std::string::npos)
            in_tail = false;
        else if (in_tail && ln.compare(0, 2, "G1") == 0) {
            CHECK(ln.find('V') != std::string::npos);
            CHECK(ln.find('U') == std::string::npos);
            ++tail_moves;
        }
    }
    CHECK(tail_moves >= 6); // at least one deposited tail move per strand
}

TEST_CASE("FiberStrandPlanner: deterministic for identical input", "[Fiber][FiberStrandPlanner]")
{
    const std::vector<std::vector<FiberPoint>> rings = {s_ring(), rect(60, 0, 100, 30)};
    StrandLayerResult a = build_layer_strands(rings, base_params());
    StrandLayerResult b = build_layer_strands(rings, base_params());
    REQUIRE(a.strands.size() == b.strands.size());
    for (size_t i = 0; i < a.strands.size(); ++i) {
        REQUIRE(a.strands[i].pts.size() == b.strands[i].pts.size());
        CHECK(a.strands[i].total_path == b.strands[i].total_path);
        for (size_t k = 0; k < a.strands[i].pts.size(); ++k) {
            CHECK(a.strands[i].pts[k].x == b.strands[i].pts[k].x);
            CHECK(a.strands[i].pts[k].y == b.strands[i].pts[k].y);
        }
    }
}

// ---------------------------------------------------------------------------
// Serpentine interior fill (fill build, operator ruling 2026-10-01: "still
// not filling with fiber").
// ---------------------------------------------------------------------------

TEST_CASE("FiberStrandPlanner: fill produces interior chords in one continuous snake", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params(); // pitch 2, tail 54.8
    p.fill_enabled = true;
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 60, 40)}, p, &err);
    CHECK(err.empty());
    CHECK(res.islands == 1);
    CHECK(res.viable_rings == 1);
    CHECK(res.rejected_other == 0);
    // Clip region = rect eroded 1 mm: x in [1,59], y in [1,39]. Horizontal
    // chords struck at y = 2,4,...,38: 19 scanlines, every chord 58 mm long
    // (>= min seg 3): the interior is covered at uniform 2 mm pitch.
    CHECK(res.fill_chords == 19);
    CHECK(res.fill_chords_dropped == 0);
    // A convex rect chains into exactly one continuous snake, no breaks.
    CHECK(res.fill_breaks == 0);
    REQUIRE(res.strands.size() == 2); // level-0 trace + one fill strand
    const FiberStrand& trace = res.strands[0];
    const FiberStrand& fill = res.strands[1];
    CHECK(trace.finalized);
    CHECK(fill.finalized);
    CHECK(trace.total_path == Catch::Approx(200.0).margin(0.5)); // boundary still owns the edge
    // Open polyline: the fill does not close on itself.
    CHECK((fill.pts.front().x != fill.pts.back().x || fill.pts.front().y != fill.pts.back().y));
    // 19 chords of 58 mm + 18 connectors of 2 mm.
    CHECK(fill.total_path == Catch::Approx(19 * 58.0 + 18 * 2.0).margin(1.0));
    // The interior finally carries fiber: deposited path dwarfs the profile trace.
    CHECK(fill.total_path > 3.0 * trace.total_path);
    // On-part invariant: every deposited vertex at least half a pitch inside
    // the boundary (fill region is the 1 mm erosion).
    for (const FiberPoint& fp : fill.pts) {
        CHECK(fp.x >= 1.0 - 1e-6);
        CHECK(fp.x <= 59.0 + 1e-6);
        CHECK(fp.y >= 1.0 - 1e-6);
        CHECK(fp.y <= 39.0 + 1e-6);
    }
    // Tail is the last 54.8 mm of the path itself: V-only, on-part, paid at the
    // body matrix rate (tail * rate * P).
    CHECK(fill.total_v_tail == Catch::Approx(54.8 * 0.23 * 4.0).margin(0.02));
    CHECK(fill.tail_pts.size() >= 1);
    for (const FiberPoint& fp : fill.tail_pts) {
        CHECK(fp.x >= 1.0 - 1e-6);
        CHECK(fp.x <= 59.0 + 1e-6);
        CHECK(fp.y >= 1.0 - 1e-6);
        CHECK(fp.y <= 39.0 + 1e-6);
    }
}

TEST_CASE("FiberStrandPlanner: alternating angle reorients the fill", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled = true;
    StrandLayerResult h = build_layer_strands({rect(0, 0, 60, 40)}, p);
    p.fill_angle_deg = 90.0; // layer-parity rotation as the producer applies it
    StrandLayerResult v = build_layer_strands({rect(0, 0, 60, 40)}, p);
    // Same island, quarter-turn: chords now run vertically, so the long axis
    // is scanned: 29 chords vs 19. The interior of BOTH layers is filled.
    CHECK(h.fill_chords == 19);
    CHECK(v.fill_chords == 29);
    CHECK_FALSE(h.fill_chords == v.fill_chords);
    REQUIRE(v.strands.size() == 2);
    // Vertical chords: 29 * 38 mm + 28 connectors of 2 mm.
    CHECK(v.strands[1].total_path == Catch::Approx(29 * 38.0 + 28 * 2.0).margin(1.0));
}

TEST_CASE("FiberStrandPlanner: U-shape breaks the strand at the slot, never crosses it", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled = true;
    // U: 30 x 30 with an 8-wide slot open at the top (slot x in [8,22], y in [8,30]).
    const std::vector<FiberPoint> u = ring({0, 0, 30, 0, 30, 30, 22, 30, 22, 8, 8, 8, 8, 30, 0, 30});
    std::string err;
    StrandLayerResult res = build_layer_strands({u}, p, &err);
    CHECK(err.empty());
    CHECK(res.rejected_other == 0);
    // Eroded 1 mm: base band [1,29]x[1,7]; legs x in [1,7] and [23,29] up to
    // y=29. Scanlines y=2,4,6 are full-width chords (28 mm each); y=8..28 give
    // two 6 mm leg chords each: 3 + 22 = 25 admitted.
    CHECK(res.fill_chords == 25);
    // The walk snakes the base band (last line exits at x=29), dives into the
    // near right leg (2 mm turns inside material), tops out, and cannot cross
    // the slot void to the other leg: exactly ONE break, the far leg opens a
    // new strand. No faked turns.
    CHECK(res.fill_breaks == 1);
    REQUIRE(res.strands.size() == 3); // trace + (band+near leg) + far leg
    // Contour perimeter of the U is 164: viable trace.
    CHECK(res.strands[0].total_path == Catch::Approx(164.0).margin(1.0));
    CHECK(res.strands[1].finalized);
    CHECK(res.strands[2].finalized);
    // band + near leg: 3 x 28 + 11 x 6 + 13 connectors x 2 = 176
    CHECK(res.strands[1].total_path == Catch::Approx(3 * 28.0 + 11 * 6.0 + 13 * 2.0).margin(1.5));
    // far leg alone: 11 x 6 + 10 x 2 = 86
    CHECK(res.strands[2].total_path == Catch::Approx(11 * 6.0 + 10 * 2.0).margin(1.0));
    // Nothing deposited over the slot: every vertex of every strand outside the
    // (grown) slot void x in (8,22), y in (8,30).
    auto outside_slot = [](const FiberStrand& st) {
        for (const FiberPoint& fp : st.pts)
            if (fp.x > 8.0 && fp.x < 22.0 && fp.y > 8.0)
                return false;
        return true;
    };
    // The level-0 trace rides the boundary, so slot edges touch y=8/x=8/x=22
    // but never enter the void interior.
    for (const FiberStrand& st : res.strands)
        CHECK(outside_slot(st));
}

TEST_CASE("FiberStrandPlanner: fill accounting and min-seg drop are honest", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled = true;
    p.fill_min_seg_mm = 100.0; // longer than any chord across the 58 mm region
    StrandLayerResult res = build_layer_strands({rect(0, 0, 60, 40)}, p);
    // Every chord dropped, all counted; interior then falls to the boundary
    // trace alone (reject-don't-simplify: no stub strand is invented).
    CHECK(res.fill_chords == 0);
    CHECK(res.fill_chords_dropped == 19);
    REQUIRE(res.strands.size() == 1);
    CHECK(res.strands.front().total_path == Catch::Approx(200.0).margin(0.5));
    // Fill off: fill accounting stays zero (existing concentric behavior untouched).
    StrandLayerParams q = base_params();
    StrandLayerResult off = build_layer_strands({rect(0, 0, 60, 40)}, q);
    CHECK(off.fill_chords == 0);
    CHECK(off.fill_chords_dropped == 0);
    CHECK(off.fill_breaks == 0);
}

TEST_CASE("FiberStrandPlanner: fill strands emit one early-cut lifecycle each", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled = true;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 60, 40)}, p);
    REQUIRE(res.strands.size() == 2);
    FiberEmitParams ep;
    ep.restart_feed_mm = 55.0;
    std::string all;
    for (size_t i = 0; i < res.strands.size(); ++i) {
        std::string out;
        ep.emit_layer_marker = i == 0;
        REQUIRE(emit_strand(res.strands[i], ep, out));
        all += out;
    }
    CHECK(count_sub(all, "M2800") == 2);   // one cut per strand, no storm
    CHECK(count_sub(all, "M1001 ") == 2);  // one budget per strand
    CHECK(count_sub(all, "; LAYER:") == 1);
    // Tail discipline on the open fill strand: V-only U-free moves between the
    // cut and the handshake.
    const std::vector<std::string> lines = split_lines(all);
    bool in_tail = false;
    size_t tail_moves = 0;
    for (const std::string& ln : lines) {
        if (ln.find("; Start to cut") != std::string::npos)
            in_tail = true;
        else if (ln.find("; Cutting completed.") != std::string::npos)
            in_tail = false;
        else if (in_tail && ln.compare(0, 2, "G1") == 0) {
            CHECK(ln.find('V') != std::string::npos);
            CHECK(ln.find('U') == std::string::npos);
            ++tail_moves;
        }
    }
    CHECK(tail_moves >= 2);
}

// ---------------------------------------------------------------------------
// Fill admission gate (operator ruling 2026-10-01, concert build): the
// serpentine fill is for large areas only; thin walls belong to the trace.
// ---------------------------------------------------------------------------

TEST_CASE("FiberStrandPlanner: thin wall is owned by the trace, fill admits no chords", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled = true;
    p.fill_min_wall_width = 4.0; // opening diameter: nothing thinner than 4 mm is fillable
    p.fill_min_area_mm2 = 100.0;
    // 30x30 frame with a 3 mm wall (hole 24x24): 324 mm^2 of material passes
    // the area pre-filter but every band is only 3 mm < 4 mm wide.
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 30, 30), rect(3, 3, 27, 27)}, p, &err);
    CHECK(err.empty());
    CHECK(res.fill_chords == 0);
    CHECK(res.fill_chords_dropped == 0); // rejected by admission, not by min-seg
    // The trace still reinforces the wall: both level-0 loops finalize.
    CHECK(res.islands == 1);
    CHECK(res.viable_rings == 2);
    REQUIRE(res.strands.size() == 2);
    CHECK(res.strands[0].finalized);
    CHECK(res.strands[1].finalized);
    // Area pre-filter alone: with the opening disabled but a 1000 mm^2 floor,
    // the same 324 mm^2 frame still produces no chords.
    StrandLayerParams q = base_params();
    q.fill_enabled = true;
    q.fill_min_area_mm2 = 1000.0;
    StrandLayerResult res2 = build_layer_strands({rect(0, 0, 30, 30), rect(3, 3, 27, 27)}, q);
    CHECK(res2.fill_chords == 0);
}

TEST_CASE("FiberStrandPlanner: thick island keeps its fill through the opening gate", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled = true;
    p.fill_min_wall_width = 4.0;
    p.fill_min_area_mm2 = 100.0;
    // 60x40 solid: opening by r=2 restores the full rectangle (erode/dilate are
    // inverse on a convex block), so half-pitch erosion gives [1,59]x[1,39] ->
    // exactly the ungated result: 19 horizontal chords of 58 mm, one snake.
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 60, 40)}, p, &err);
    CHECK(err.empty());
    CHECK(res.fill_chords == 19);
    CHECK(res.fill_chords_dropped == 0);
    CHECK(res.fill_breaks == 0);
    REQUIRE(res.strands.size() == 2); // trace + fill, concert on one island
    CHECK(res.strands[0].finalized);
    CHECK(res.strands[1].finalized);
    CHECK(res.strands[1].total_path > 3.0 * res.strands[0].total_path);
    // Every deposited vertex stays at least half a pitch inside material.
    for (const FiberPoint& fp : res.strands[1].pts) {
        CHECK(fp.x >= 1.0 - 1e-6);
        CHECK(fp.x <= 59.0 + 1e-6);
        CHECK(fp.y >= 1.0 - 1e-6);
        CHECK(fp.y <= 39.0 + 1e-6);
    }
}

// ---------------------------------------------------------------------------
// Concentric wall loops (operator ruling 2026-10-01, second correction): a
// thin wall is not "trace only" - the wall thickness is carried by multiple
// loops hugging the shape, at the dedicated wall pitch. Admitted (thick)
// islands keep the serpentine concert byte-identical.
// ---------------------------------------------------------------------------

TEST_CASE("FiberStrandPlanner: refused thin wall fills its thickness with concentric loops", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled = true;
    p.fill_min_wall_width = 4.0;
    p.fill_min_area_mm2 = 100.0;
    p.wall_loops_enabled = true; // wall_pitch default 0.7
    // Same 3 mm-wall frame the admission test refuses: 30x30 outer, 24x24 hole.
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 30, 30), rect(3, 3, 27, 27)}, p, &err);
    CHECK(err.empty());
    CHECK(res.fill_chords == 0);      // wall mode: no serpentine attempted
    CHECK(res.fill_chords_dropped == 0);
    CHECK(res.islands == 1);
    CHECK(res.viable_rings == 2);     // level-0 trace accounting unchanged
    // Level 1 erodes 0.7 mm: outer 28.6, hole 25.4. Level 2 erodes 1.4 mm:
    // outer 27.2, hole 26.8 (0.4 mm band). Level 3 inverts and stops. Four
    // loops, each over the 55.8 mm viability floor: the 3 mm wall thickness
    // is carried, hugging the shape.
    CHECK(res.wall_loops == 4);
    REQUIRE(res.strands.size() == 6); // 2 trace + 4 wall loops
    for (const FiberStrand& st : res.strands)
        CHECK(st.finalized);
    // Every wall-loop vertex stays inside material, at least one wall pitch
    // off the boundary: the loops hug the shape, they do not drift into air.
    for (size_t i = 2; i < res.strands.size(); ++i)
        for (const FiberPoint& fp : res.strands[i].pts) {
            CHECK(fp.x >= p.wall_pitch_mm - 1e-6);
            CHECK(fp.x <= 30.0 - p.wall_pitch_mm + 1e-6);
            CHECK(fp.y >= p.wall_pitch_mm - 1e-6);
            CHECK(fp.y <= 30.0 - p.wall_pitch_mm + 1e-6);
        }
    // Wall loops off on the same island: single trace line, unchanged.
    StrandLayerParams q = base_params();
    q.fill_enabled = true;
    q.fill_min_wall_width = 4.0;
    q.fill_min_area_mm2 = 100.0;
    StrandLayerResult off = build_layer_strands({rect(0, 0, 30, 30), rect(3, 3, 27, 27)}, q);
    CHECK(off.wall_loops == 0);
    CHECK(off.strands.size() == 2);
}

TEST_CASE("FiberStrandPlanner: admitted island is untouched by wall loops", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled = true;
    p.fill_min_wall_width = 4.0;
    p.fill_min_area_mm2 = 100.0;
    p.wall_loops_enabled = true;
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 60, 40)}, p, &err);
    CHECK(err.empty());
    CHECK(res.wall_loops == 0);       // admitted: serpentine owns the interior
    CHECK(res.fill_chords == 19);
    REQUIRE(res.strands.size() == 2); // trace + fill, byte-identical to gate-only
    StrandLayerResult ungated = build_layer_strands({rect(0, 0, 60, 40)}, p);
    CHECK(res.strands[0].total_path == Catch::Approx(ungated.strands[0].total_path));
    CHECK(res.strands[1].total_path == Catch::Approx(ungated.strands[1].total_path));
}

TEST_CASE("FiberStrandPlanner: wall loops reject a non-positive pitch", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled = true;
    p.wall_loops_enabled = true;
    p.wall_pitch_mm = 0.0;
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 30, 30)}, p, &err);
    CHECK_FALSE(err.empty());
    CHECK(res.strands.empty());
}

TEST_CASE("FiberStrandPlanner: wall loop emission is deterministic", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled = true;
    p.fill_min_wall_width = 4.0;
    p.fill_min_area_mm2 = 100.0;
    p.wall_loops_enabled = true;
    StrandLayerResult a = build_layer_strands({rect(0, 0, 30, 30), rect(3, 3, 27, 27)}, p);
    StrandLayerResult b = build_layer_strands({rect(0, 0, 30, 30), rect(3, 3, 27, 27)}, p);
    REQUIRE(a.strands.size() == b.strands.size());
    CHECK(a.wall_loops == b.wall_loops);
    for (size_t i = 0; i < a.strands.size(); ++i) {
        CHECK(a.strands[i].pts.size() == b.strands[i].pts.size());
        CHECK(a.strands[i].total_path == Catch::Approx(b.strands[i].total_path).epsilon(0.0));
        if (!a.strands[i].pts.empty()) {
            CHECK(a.strands[i].pts.front().x == b.strands[i].pts.front().x);
            CHECK(a.strands[i].pts.front().y == b.strands[i].pts.front().y);
        }
    }
}

// Deposit moves carry no Z word at all: the fiber is attached between M1001 and
// M1002, so Z stays constant for the whole window (operator ruling; matches the
// vendor reference files). Enforced here so the dialect cannot regress.
TEST_CASE("FiberStrandPlanner: emitted body moves never carry a Z word", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    std::string err;
    StrandLayerResult res = build_layer_strands({s_ring()}, p, &err);
    REQUIRE(res.strands.size() == 1);
    FiberEmitParams ep;
    ep.restart_feed_mm = 55.0;
    std::string out;
    REQUIRE(emit_strand(res.strands[0], ep, out));
    size_t body = 0;
    for (const std::string& ln : split_lines(out)) {
        if (ln.find(" P") == std::string::npos || ln.compare(0, 2, "G1") != 0)
            continue;
        ++body;
        CHECK(ln.find('Z') == std::string::npos);
    }
    CHECK(body >= 10);
}

// ---------------------------------------------------------------------------
// RocketSlicer mode parity (plan rev 2, M2): fill_outer_inset keeps the chord
// fill a set distance short of the walls (CF inside, FFF outer skin), and the
// wall-loop clamp emits a bounded loop count on every island the wall producer
// runs on - admitted (filled) islands included. Defaults (0/0/0) keep the
// legacy behavior byte-identical, covered by all tests above.
// ---------------------------------------------------------------------------

TEST_CASE("FiberStrandPlanner: fill outer inset keeps chords short of the walls", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled = true;
    p.fill_min_wall_width = 4.0;
    p.fill_min_area_mm2 = 100.0;
    p.fill_outer_inset = 2.0;
    // 60x40 solid: pitch erosion gives [1,59]x[1,39], the inset gives
    // [3,57]x[3,37] -> scanlines at y = 4,6,...,36: 17 chords of 54 mm.
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 60, 40)}, p, &err);
    CHECK(err.empty());
    CHECK(res.fill_chords == 17); // one fewer pair than the 19-chord inset-0 case
    REQUIRE(res.strands.size() == 2); // trace + fill
    for (const FiberPoint& fp : res.strands[1].pts) {
        CHECK(fp.x >= 3.0 - 1e-6); // inset applied: chords never reach x < 3
        CHECK(fp.x <= 57.0 + 1e-6);
        CHECK(fp.y >= 3.0 - 1e-6);
        CHECK(fp.y <= 37.0 + 1e-6);
    }
    // An inset that consumes the fillable region refuses the island: with wall
    // loops off only the boundary trace remains.
    StrandLayerParams q = p;
    q.fill_outer_inset = 29.0;
    StrandLayerResult deep = build_layer_strands({rect(0, 0, 60, 40)}, q);
    CHECK(deep.fill_chords == 0);
    CHECK(deep.strands.size() == 1);
}

TEST_CASE("FiberStrandPlanner: a negative fill outer inset is rejected", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled = true;
    p.fill_outer_inset = -1.0;
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 30, 30)}, p, &err);
    CHECK_FALSE(err.empty());
    CHECK(res.strands.empty());
}

TEST_CASE("FiberStrandPlanner: wall loop clamp runs on filled islands too", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled = true;
    p.fill_min_wall_width = 4.0;
    p.fill_min_area_mm2 = 100.0;
    p.wall_loops_enabled = true;
    p.wall_outer_loops = 2;
    p.wall_inner_loops = 2; // clamp = 4 levels
    // Admitted 60x40 island: the clamp makes the wall producer run alongside
    // the serpentine (vendor Reinforced carries F. loops AND fill). Levels
    // 1..3 at wall pitch 0.7 are all viable single loops; level 4 would be
    // level index 3 (clamp-1), so exactly 3 wall loops are emitted.
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 60, 40)}, p, &err);
    CHECK(err.empty());
    CHECK(res.fill_chords == 19); // fill untouched by the clamp
    CHECK(res.wall_loops == 3);
    REQUIRE(res.strands.size() == 5); // trace + 3 wall loops + fill snake
    // Outer boundary first, then the inner continuation: loop i+1 is the
    // erosion of loop i by one wall pitch, so perimeters shrink monotonically.
    CHECK(res.strands[1].total_path > res.strands[2].total_path);
    CHECK(res.strands[2].total_path > res.strands[3].total_path);
    // Clamp 1 (outer only): the single loop IS the boundary trace, so the wall
    // loop counter (erosion levels only) stays 0 and only trace + fill remain.
    StrandLayerParams q = p;
    q.wall_outer_loops = 1;
    q.wall_inner_loops = 0;
    StrandLayerResult one = build_layer_strands({rect(0, 0, 60, 40)}, q);
    CHECK(one.wall_loops == 0);
    CHECK(one.strands.size() == 2);
}

TEST_CASE("FiberStrandPlanner: clamp caps wall loops on a refused thin wall", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled = true;
    p.fill_min_wall_width = 4.0;
    p.fill_min_area_mm2 = 100.0;
    p.wall_loops_enabled = true;
    p.wall_outer_loops = 1;
    p.wall_inner_loops = 1; // clamp = 2
    // The 3 mm-wall frame the concert test carries to 4 wall loops unclamped:
    // the clamp stops after level 1 (outer + hole contour = 2 loops).
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 30, 30), rect(3, 3, 27, 27)}, p, &err);
    CHECK(err.empty());
    CHECK(res.fill_chords == 0);  // refused: still wall mode, no serpentine
    CHECK(res.wall_loops == 2);
    REQUIRE(res.strands.size() == 4); // 2 trace + 2 clamped wall loops
}

TEST_CASE("FiberStrandPlanner: clamp caps concentric levels when fill is off", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled = false;
    p.wall_outer_loops = 2;
    p.wall_inner_loops = 1; // clamp = 3
    // 30x30 solid at pitch 2: boundary + 2 erosion levels. Legacy (clamp 0)
    // erodes until viability runs out: level k perimeter = 120 - 16k, floor
    // 55.8 -> levels 0..4 viable (k=4: 56, k=5: 40 fails): 5 strands.
    StrandLayerResult clamped = build_layer_strands({rect(0, 0, 30, 30)}, p);
    CHECK(clamped.strands.size() == 3);
    StrandLayerParams q = base_params();
    StrandLayerResult legacy = build_layer_strands({rect(0, 0, 30, 30)}, q);
    CHECK(legacy.strands.size() == 5);
}

TEST_CASE("FiberStrandPlanner: clamp without wall loops is inert", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled = true;
    p.fill_min_wall_width = 4.0;
    p.fill_min_area_mm2 = 100.0;
    p.wall_loops_enabled = false; // clamp has nothing to drive
    p.wall_outer_loops = 2;
    p.wall_inner_loops = 2;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 60, 40)}, p);
    CHECK(res.wall_loops == 0);
    CHECK(res.fill_chords == 19);
    CHECK(res.strands.size() == 2); // trace + fill, byte-identical concert
}

// ---------------------------------------------------------------------------
// The fiber lane (boundary_inset_mm): the harvested rings are the plastic
// outer-wall centerline, so the whole fiber region has to move inboard of them
// or the roving lands on the very plastic bead it is meant to hide behind.
// Inset 0 is the legacy lane and is covered by every test above.
// ---------------------------------------------------------------------------

TEST_CASE("FiberStrandPlanner: boundary inset moves the whole fiber lane inboard", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params(); // pitch 2, tail 54.8 -> 55.8 mm floor
    p.boundary_inset_mm = 1.0;
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 60, 40)}, p, &err);
    CHECK(err.empty());
    CHECK(res.islands == 1);
    CHECK(res.rejected_thin == 0);
    REQUIRE(res.strands.size() >= 1);
    // Level 0 is the ring eroded 1 mm, so the outermost strand is
    // 2 * (58 + 38) = 192 mm instead of the 200 mm boundary itself.
    CHECK(res.strands.front().total_path == Catch::Approx(192.0).margin(0.5));
    for (const FiberPoint& fp : res.strands.front().pts) {
        CHECK(fp.x >= 1.0 - 1e-6);
        CHECK(fp.x <= 59.0 + 1e-6);
        CHECK(fp.y >= 1.0 - 1e-6);
        CHECK(fp.y <= 39.0 + 1e-6);
    }
    // Deeper levels compose from the lane: erosion 1 + 2k, perimeter 192 - 16k,
    // viable for k = 0..8; k = 9 is 48 mm and falls back to plastic.
    CHECK(res.strands.size() == 9);
    CHECK(res.rejected_short == 1);
    // Without the inset the lane rides the input ring and reaches one level
    // deeper: perimeter 200 - 16k, viable for k = 0..9.
    StrandLayerResult legacy = build_layer_strands({rect(0, 0, 60, 40)}, base_params());
    CHECK(legacy.strands.front().total_path == Catch::Approx(200.0).margin(0.5));
    CHECK(legacy.strands.size() == 10);
}

TEST_CASE("FiberStrandPlanner: a wall too thin for the fiber lane falls back to plastic", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.boundary_inset_mm = 2.0;
    // 3 mm wall (30x30 outer, 24x24 hole). Holding the fiber 2 mm off both
    // faces of a 3 mm band leaves no lane at all, so the feature carries no
    // fiber: plastic-only fallback, counted rather than squeezed in.
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 30, 30), rect(3, 3, 27, 27)}, p, &err);
    CHECK(err.empty());
    CHECK(res.islands == 1);
    CHECK(res.rejected_thin == 1);
    CHECK(res.viable_rings == 0);
    CHECK(res.strands.empty());
}

TEST_CASE("FiberStrandPlanner: a negative boundary inset is rejected", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.boundary_inset_mm = -1.0;
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 30, 30)}, p, &err);
    CHECK_FALSE(err.empty());
    CHECK(res.strands.empty());
}

// ---------------------------------------------------------------------------
// Isogrid interior fill: three rib families 60 degrees apart, each rib a twin
// pass one bead wide, at the rib pitch the density law resolves to (see
// test_fiber_mode_plan.cpp for the density -> pitch law itself).
//
// The fixture is a 100 x 14 mm strip with a 20 mm minimum segment: only the
// family running along the long axis produces chords that survive, because the
// other two cross the 13.2 mm band and are at most 13.2 / sin(60) = 15.2 mm
// long. That isolates one family and makes its rib geometry exact.
// ---------------------------------------------------------------------------

namespace {

StrandLayerParams isogrid_params(double rib_pitch_mm, double base_angle_deg)
{
    StrandLayerParams p = base_params();
    p.fill_enabled     = true;
    p.infill_pattern   = FiberInfillPattern::fipIsogrid;
    p.fiber_width_mm   = 0.8;
    p.pitch_mm         = rib_pitch_mm;
    p.fill_angle_deg   = base_angle_deg;
    p.fill_min_seg_mm  = 20.0;
    return p;
}

// Sorted distinct y coordinates of a strand's deposited vertices.
std::vector<double> distinct_y(const FiberStrand& st)
{
    std::vector<double> ys;
    for (const FiberPoint& fp : st.pts)
        ys.push_back(fp.y);
    std::sort(ys.begin(), ys.end());
    ys.erase(std::unique(ys.begin(), ys.end(),
                         [](double a, double b) { return std::abs(a - b) < 1e-6; }),
             ys.end());
    return ys;
}

} // namespace

TEST_CASE("FiberStrandPlanner: isogrid lays twin ribs one bead apart at the rib pitch", "[Fiber][FiberStrandPlanner]")
{
    // Families land on 0, 60 and 120 degrees; the 0 degree one survives.
    StrandLayerParams p = isogrid_params(3.0, -30.0);
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 100, 14)}, p, &err);
    CHECK(err.empty());
    // Clearance is half a BEAD (0.4 mm), not half a rib pitch: a 3 mm rib pitch
    // must not push the ribs 1.5 mm off the wall. Clip region is
    // [0.4, 99.6] x [0.4, 13.6], so rib centers fall at y = 1.9, 4.9, 7.9, 10.9
    // and each carries a twin pass 0.8 mm above it.
    CHECK(res.fill_chords == 8);        // 4 ribs x 2 passes
    CHECK(res.fill_chords_dropped > 0); // the two diagonal families
    CHECK(res.fill_breaks == 0);        // ribs chain into one continuous strand
    REQUIRE(res.strands.size() == 2);   // boundary trace + one chained snake
    CHECK(res.strands[0].total_path == Catch::Approx(228.0).margin(1.0)); // 2 * (100 + 14)
    // 8 chords of 99.2 mm, chained by four 0.8 mm twin turns and three 2.2 mm
    // rib-to-rib turns.
    CHECK(res.strands[1].total_path == Catch::Approx(8 * 99.2 + 4 * 0.8 + 3 * 2.2).margin(1.0));
    const std::vector<double> ys = distinct_y(res.strands[1]);
    REQUIRE(ys.size() == 8);
    for (size_t rib = 0; rib < 4; ++rib)
        CHECK(ys[2 * rib + 1] - ys[2 * rib] == Catch::Approx(0.8).margin(1e-5)); // twin pass: one bead
    for (size_t rib = 0; rib + 1 < 4; ++rib)
        CHECK(ys[2 * rib + 2] - ys[2 * rib] == Catch::Approx(3.0).margin(1e-5)); // rib pitch
}

TEST_CASE("FiberStrandPlanner: isogrid rib pitch follows the resolved density", "[Fiber][FiberStrandPlanner]")
{
    // 60 percent density at a 0.8 mm bead resolves to a 4 mm rib pitch, which
    // fits three ribs in the same strip instead of four.
    StrandLayerParams p = isogrid_params(4.0, -30.0);
    StrandLayerResult res = build_layer_strands({rect(0, 0, 100, 14)}, p);
    CHECK(res.fill_chords == 6); // 3 ribs x 2 passes
    REQUIRE(res.strands.size() == 2);
    const std::vector<double> ys = distinct_y(res.strands[1]);
    REQUIRE(ys.size() == 6);
    for (size_t rib = 0; rib < 3; ++rib)
        CHECK(ys[2 * rib + 1] - ys[2 * rib] == Catch::Approx(0.8).margin(1e-5));
    for (size_t rib = 0; rib + 1 < 3; ++rib)
        CHECK(ys[2 * rib + 2] - ys[2 * rib] == Catch::Approx(4.0).margin(1e-5));
}

TEST_CASE("FiberStrandPlanner: isogrid rib families sit 60 degrees apart", "[Fiber][FiberStrandPlanner]")
{
    // The families are base+30, base+90 and base+150, so a 60 degree rotation
    // of the laydown angle maps the set onto itself. Whichever family slot ends
    // up running along the strip, the surviving rib set has the same count,
    // the same rib pitch and the same twin spacing.
    const double base = GENERATE(-30.0, 30.0, 90.0);
    StrandLayerParams p = isogrid_params(3.0, base);
    StrandLayerResult res = build_layer_strands({rect(0, 0, 100, 14)}, p);
    CAPTURE(base);
    CHECK(res.fill_chords == 8);
    REQUIRE(res.strands.size() == 2);
    const std::vector<double> ys = distinct_y(res.strands[1]);
    REQUIRE(ys.size() == 8);
    for (size_t rib = 0; rib < 4; ++rib)
        CHECK(std::abs(ys[2 * rib + 1] - ys[2 * rib]) == Catch::Approx(0.8).margin(1e-5));
    for (size_t rib = 0; rib + 1 < 4; ++rib)
        CHECK(std::abs(ys[2 * rib + 2] - ys[2 * rib]) == Catch::Approx(3.0).margin(1e-5));
}

TEST_CASE("FiberStrandPlanner: isogrid covers an open area from three directions", "[Fiber][FiberStrandPlanner]")
{
    // On an area wide enough for every family the three twin-pass rib families
    // put down far more fiber than the single-pass serpentine at the same
    // spacing - that is what the density response is made of.
    StrandLayerParams p = base_params();
    p.fill_enabled   = true;
    p.infill_pattern = FiberInfillPattern::fipIsogrid;
    p.fiber_width_mm = 0.8;
    p.pitch_mm       = 3.0;
    StrandLayerResult iso = build_layer_strands({rect(0, 0, 60, 60)}, p);
    StrandLayerParams q = p;
    q.infill_pattern = FiberInfillPattern::fipRectilinear;
    StrandLayerResult rect_fill = build_layer_strands({rect(0, 0, 60, 60)}, q);
    CHECK(rect_fill.fill_chords == 19); // clearance 1.5 mm, scanlines 3..57
    CHECK(iso.fill_chords > 3 * rect_fill.fill_chords);
}

TEST_CASE("FiberStrandPlanner: isogrid without a bead width is rejected", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled   = true;
    p.infill_pattern = FiberInfillPattern::fipIsogrid;
    p.fiber_width_mm = 0.0; // the twin-pass offset and the clearance need it
    std::string err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 60, 40)}, p, &err);
    CHECK_FALSE(err.empty());
    CHECK(res.strands.empty());
}

TEST_CASE("FiberStrandPlanner: solid fill is the serpentine driven at one bead", "[Fiber][FiberStrandPlanner]")
{
    // The vendor Fortified pattern: adjacent passes one 0.7 mm bead apart. The
    // producer is the single-pass serpentine, so the spacing is the pitch the
    // mode resolver hands over and the chords stay half a bead inside material.
    StrandLayerParams p = base_params();
    p.fill_enabled    = true;
    p.infill_pattern  = FiberInfillPattern::fipSolid;
    p.fiber_width_mm  = 0.7;
    p.pitch_mm        = 0.7;
    p.fill_min_seg_mm = 5.0;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 60, 14)}, p);
    // Clip region [0.35, 59.65] x [0.35, 13.65], scanlines every 0.7 mm from
    // y = 0.7: 19 of them fit below 13.65.
    CHECK(res.fill_chords == 19);
    REQUIRE(res.strands.size() == 2);
    const std::vector<double> ys = distinct_y(res.strands[1]);
    REQUIRE(ys.size() == 19);
    for (size_t i = 0; i + 1 < ys.size(); ++i)
        CHECK(ys[i + 1] - ys[i] == Catch::Approx(0.7).margin(1e-5));
}

// ---------------------------------------------------------------------------
// Feasibility limits (fs_fiber_min_radius / fs_fiber_max_arc_seg), both 0 =
// off, so every test above exercises the legacy behavior.
// ---------------------------------------------------------------------------

TEST_CASE("FiberStrandPlanner: a turn tighter than the minimum radius is reported", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params(); // pitch 2
    p.fill_enabled = true;
    p.min_turn_radius_mm = 12.0;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 60, 40)}, p);
    // The serpentine reverses over a 2 mm connector, a 1 mm fillet radius. Under
    // the default keep policy the strand is deposited as planned and only the
    // tight joints are counted, so the operator can size the cost of splitting.
    CHECK(res.fill_chords == 19);
    // Keep now fillets each reversal to the 1 mm radius that fits in the 2 mm
    // connector. Those U-turns still cannot meet 12 mm, so they remain counted.
    CHECK(res.tight_turns > 0);
    CHECK(res.tight_turn_splits == 0);
    // The boundary trace's right-angle corners sit on 40 and 60 mm legs, a
    // 20 mm fillet radius, so it contributes no tight joint.
    REQUIRE(res.strands.size() == 2);
    // Raising the limit past the trace's own corners reports its corners too
    // without dropping it.
    StrandLayerParams q = p;
    q.min_turn_radius_mm = 25.0;
    StrandLayerResult all = build_layer_strands({rect(0, 0, 60, 40)}, q);
    CHECK(all.strands.size() == 2);
    CHECK(all.tight_turns == 40); // the 36 reversal corners + the trace's 4
}

TEST_CASE("FiberStrandPlanner: the split policy cuts the strand at a tight turn", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.pitch_mm           = 150.0; // erodes empty at level 1: the boundary trace alone
    p.min_turn_radius_mm = 25.0;  // tighter than the trace's own 20 mm corners
    p.tight_turn_policy  = FiberTightTurnPolicy::fttSplit;
    std::string      err;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 60, 40)}, p, &err);
    CHECK(res.tight_turns == 4);
    CHECK(res.tight_turn_splits == 4);
    CHECK_FALSE(err.empty());
    // Four cuts open the 200 mm ring into its four sides. Only the two 60 mm
    // sides carry the 54.8 mm tail plus margin; the 40 mm sides are too short.
    REQUIRE(res.strands.size() == 2);
    for (const FiberStrand& s : res.strands)
        CHECK(s.total_path == Catch::Approx(60.0).margin(0.5));
    CHECK(res.rejected_short == 2);

    // Keep is the default: one strand, corners filleted to the largest radius
    // that fits (20 mm on this rectangle) instead of depositing the square tips.
    StrandLayerParams q = p;
    q.tight_turn_policy = FiberTightTurnPolicy::fttKeep;
    StrandLayerResult kept = build_layer_strands({rect(0, 0, 60, 40)}, q);
    CHECK(kept.tight_turn_splits == 0);
    REQUIRE(kept.strands.size() == 1);
    CHECK(kept.strands.front().total_path == Catch::Approx(40.0 + 2.0 * 3.141592653589793 * 20.0).margin(2.0));
}

TEST_CASE("FiberStrandPlanner: chaining joins a boundary loop into its island fill",
          "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.fill_enabled        = true;
    p.pitch_mm            = 4.0;
    p.fill_min_seg_mm     = 3.0;
    p.tail_length_mm      = 20.0; // short enough that a 40x30 rect carries body + tail
    // Off: one boundary strand plus one or more fill strands.
    StrandLayerResult off = build_layer_strands({rect(0, 0, 40, 30)}, p);
    REQUIRE(off.strands.size() >= 2);
    CHECK(off.chained_loops == 0);

    // On: the boundary continues into the fill, so the layer spends one fewer cut.
    p.chain_loops_into_fill = true;
    StrandLayerResult on = build_layer_strands({rect(0, 0, 40, 30)}, p);
    CHECK(on.chained_loops >= 1);
    CHECK(on.strands.size() == off.strands.size() - on.chained_loops);
    // Same fiber laid: the connector is a short hop inside material, so the
    // total path across the layer grows by that hop, not by a second loop.
    double off_len = 0.0, on_len = 0.0;
    for (const FiberStrand& s : off.strands) off_len += s.total_path;
    for (const FiberStrand& s : on.strands)  on_len  += s.total_path;
    CHECK(on_len == Catch::Approx(off_len).margin(p.pitch_mm * on.chained_loops + 1.0));
}

TEST_CASE("FiberStrandPlanner: the seam position moves the loop start without moving the loop",
          "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.pitch_mm = 150.0; // erodes empty at level 1: the boundary trace alone
    const std::vector<std::vector<FiberPoint>> in = {rect(0, 0, 60, 40)};

    // Aligned is the default: the canonical lexicographically smallest vertex.
    StrandLayerResult aligned = build_layer_strands(in, p);
    REQUIRE(aligned.strands.size() == 1);
    const FiberStrand& a = aligned.strands.front();
    CHECK(a.pts.front().x == Catch::Approx(0.0));
    CHECK(a.pts.front().y == Catch::Approx(0.0));

    // The longest sides are the two 60 mm runs; the seam lands at the midpoint
    // of the one the canonical walk meets first, so halfway along x on a long side.
    p.seam_position = FiberSeamPosition::fspLongestEdge;
    StrandLayerResult longest = build_layer_strands(in, p);
    REQUIRE(longest.strands.size() == 1);
    const FiberStrand& l = longest.strands.front();
    CHECK(l.pts.front().x == Catch::Approx(30.0));
    CHECK(std::min(std::abs(l.pts.front().y), std::abs(l.pts.front().y - 40.0)) < 1e-6);
    CHECK(l.total_path == Catch::Approx(a.total_path).margin(1e-6));
    CHECK(l.pts.back().x == Catch::Approx(l.pts.front().x).margin(1e-9));
    CHECK(l.pts.back().y == Catch::Approx(l.pts.front().y).margin(1e-9));

    // Scattered moves the seam off the aligned start and is reproducible, but a
    // different layer breaks somewhere else.
    p.seam_position = FiberSeamPosition::fspScattered;
    StrandLayerResult s1 = build_layer_strands(in, p);
    StrandLayerResult s2 = build_layer_strands(in, p);
    REQUIRE(s1.strands.size() == 1);
    REQUIRE(s2.strands.size() == 1);
    REQUIRE(s1.strands.front().pts.size() == s2.strands.front().pts.size());
    for (size_t i = 0; i < s1.strands.front().pts.size(); ++i) {
        CHECK(s1.strands.front().pts[i].x == Catch::Approx(s2.strands.front().pts[i].x));
        CHECK(s1.strands.front().pts[i].y == Catch::Approx(s2.strands.front().pts[i].y));
    }
    CHECK(s1.strands.front().total_path == Catch::Approx(a.total_path).margin(1e-6));

    p.layer_id = base_params().layer_id + 1;
    StrandLayerResult next = build_layer_strands(in, p);
    REQUIRE(next.strands.size() == 1);
    const FiberPoint& n0 = next.strands.front().pts.front();
    const FiberPoint& s0 = s1.strands.front().pts.front();
    CHECK(std::hypot(n0.x - s0.x, n0.y - s0.y) > 1.0);
}

TEST_CASE("FiberStrandPlanner: the maximum segment length resamples without moving the path", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.pitch_mm = 150.0; // erodes empty at level 1: the boundary trace alone
    StrandLayerResult off = build_layer_strands({rect(0, 0, 60, 40)}, p);
    REQUIRE(off.strands.size() == 1);

    p.max_arc_seg_mm = 3.0;
    StrandLayerResult res = build_layer_strands({rect(0, 0, 60, 40)}, p);
    REQUIRE(res.strands.size() == 1);
    const FiberStrand& st = res.strands.front();
    // Same geometry, same length: the extra vertices are on the path.
    CHECK(st.total_path == Catch::Approx(off.strands.front().total_path).margin(1e-3));
    CHECK(st.pts.size() > off.strands.front().pts.size());
    for (size_t i = 0; i + 1 < st.pts.size(); ++i) {
        const double len = std::hypot(st.pts[i + 1].x - st.pts[i].x, st.pts[i + 1].y - st.pts[i].y);
        CHECK(len <= 3.0 + 1e-6);
    }
}

TEST_CASE("FiberStrandPlanner: negative feasibility limits are rejected", "[Fiber][FiberStrandPlanner]")
{
    StrandLayerParams p = base_params();
    p.min_turn_radius_mm = -1.0;
    std::string err;
    CHECK(build_layer_strands({rect(0, 0, 30, 30)}, p, &err).strands.empty());
    CHECK_FALSE(err.empty());

    StrandLayerParams q = base_params();
    q.max_arc_seg_mm = -1.0;
    std::string err2;
    CHECK(build_layer_strands({rect(0, 0, 30, 30)}, q, &err2).strands.empty());
    CHECK_FALSE(err2.empty());
}
