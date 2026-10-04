// FibreSeeker3 continuous-fiber STRAND model + emitter unit tests
// (operator ruling 2026-10-01: the severed tail is deposition material, the
// cut fires INSIDE the deposition path).
//
// Acceptance criterion from the ruling: many sub-5 mm segments (total path
// >= 150 mm) must still produce ONE strand, ONE correctly positioned early
// cut, and a final ~55 mm tail deposited into the planned structure - the
// chord-grid test below is that acceptance test as a unit test.
//
// Invariants under test: finalize() rejects every strand shape that could not
// be emitted safely (path too short for a tail, inert tail, repeated points);
// the emitted window carries exactly one M2800 placed before the path end;
// every post-cut deposition move is V-bearing and U-free (the severed tail is
// paid out by the matrix, never commanded by U); the release (handshake,
// retract, lift, M1002) happens at the strand ENDPOINT, not at the cut.

#include <catch2/catch_all.hpp>
using Catch::Approx;

#include "libslic3r/Fiber/FiberEmitter.hpp"
#include "libslic3r/Fiber/FiberStrand.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r::Fiber;

namespace {

size_t count_sub(const std::string& hay, const std::string& needle)
{
    size_t n   = 0;
    size_t pos = 0;
    while ((pos = hay.find(needle, pos)) != std::string::npos) {
        ++n;
        pos += needle.size();
    }
    return n;
}

std::vector<std::string> split_lines(const std::string& s)
{
    std::vector<std::string> lines;
    std::istringstream       in(s);
    std::string              line;
    while (std::getline(in, line)) {
        if (!line.empty()) {
            lines.push_back(line);
        }
    }
    return lines;
}

// A long chord-grid serpentine: 20 lanes of 10 mm with 2 mm inter-lane links
// (out-and-back pairs so every link is a real 2 mm deposited step, not an
// attached travel). 40 points, 39 segments, total path 20*10 + 19*2 = 238 mm.
// This is the ruling's "many sub-5 mm segments" acceptance geometry.
FiberStrand make_chord_grid()
{
    FiberStrand s;
    s.layer_id       = 7;
    s.z              = 0.70;
    s.ratio_p        = 2.0;
    s.fiber_rate     = 0.25;
    s.feed_mm_min    = 1200.0;
    s.tail_length_mm = 54.8;
    s.tail_v_factor  = 1.0;
    // (50,100) -> (60,100) -> (60,102) -> (50,102) -> (50,104) -> ...
    for (int lane = 0; lane < 20; ++lane) {
        const double y   = 100.0 + 2.0 * lane;
        const double x0  = (lane % 2 == 0) ? 50.0 : 60.0;
        const double x1  = (lane % 2 == 0) ? 60.0 : 50.0;
        s.pts.push_back({x0, y});
        s.pts.push_back({x1, y});
    }
    return s;
}

} // namespace

TEST_CASE("FiberStrand early-cut schedule on a straight path", "[Fiber]")
{
    FiberStrand s;
    s.layer_id       = 3;
    s.z              = 0.30;
    s.pts            = {{100.0, 100.0}, {200.0, 100.0}};
    s.ratio_p        = 2.0;
    s.fiber_rate     = 0.25;
    s.feed_mm_min    = 1200.0;
    s.tail_length_mm = 54.8;

    std::string err;
    REQUIRE(s.finalize(&err));
    REQUIRE(err.empty());
    REQUIRE(s.finalized);
    REQUIRE(s.path_length() == Approx(100.0));
    // One body move ending at the cut position 100 - 54.8 = 45.2 mm in.
    REQUIRE(s.num_body_moves() == 1);
    REQUIRE(s.body_pts[0].x == Approx(145.2));
    REQUIRE(s.body_u[0] == Approx(11.3)); // 45.2 * 0.25
    // One tail move depositing the reserved tail at the body matrix rate
    // (tail_v_factor 1.0 => V/mm = fiber_rate * ratio_p = 0.5).
    REQUIRE(s.num_tail_moves() == 1);
    REQUIRE(s.tail_pts[0].x == Approx(200.0));
    REQUIRE(s.tail_v[0] == Approx(27.4)); // 54.8 * 0.25 * 2.0
    REQUIRE(s.total_u_feed == Approx(11.3));
    REQUIRE(s.total_v_tail == Approx(27.4));
    // Budget counts commanded U only; the tail suffix never enters L.
    REQUIRE(s.budget_L(55.0) == 66);

    FiberEmitParams params; // defaults
    std::string     out;
    REQUIRE(emit_strand(s, params, out, &err));
    REQUIRE(err.empty());

    const std::string golden =
        "; LAYER:3 [0.30]\n"
        "M1001 L66\n"
        "G1 F1200 Z1.50\n"
        "G1 X100.00 Y100.00 F1200\n"
        "G1 F1200 U55.000 ; Extrude restart\n"
        "G1 F1200 Z0.30\n"
        "G1 F600 V1.000 ; Recover matrix retract\n"
        "G1 F600 V3.000 ; Matrix prime\n"
        "G1 X145.20 Y100.00 V22.600 U11.300 P2.000 F1200\n"
        "; Start to cut\n"
        "M2800\n"
        "M400\n"
        ";CUT DISTANCE 54.8\n"
        "G1 X200.00 Y100.00 V27.400 F1200\n"
        "; Cutting completed.\n"
        "G1 F600 V-1.000 ; Retract\n"
        "G1 F1200 Z0.90\n"
        "M1002\n";
    REQUIRE(out == golden);
}

TEST_CASE("The fiber speed zones pick the feedrate by distance along the strand", "[Fiber][FiberEmitter]")
{
    FiberEmitParams p;
    // Unconfigured: every zone falls back to the strand's own feed.
    CHECK(zone_feed_mm_min(p, 0.0, 100.0, 1200.0) == Approx(1200.0));
    CHECK(zone_feed_mm_min(p, 99.0, 100.0, 1200.0) == Approx(1200.0));

    p.start_speed_mm_s  = 5.0;
    p.start_length_mm   = 15.0;
    p.normal_speed_mm_s = 30.0;
    p.finish_speed_mm_s = 3.0;
    p.finish_length_mm  = 10.0;
    CHECK(zone_feed_mm_min(p, 0.0, 100.0, 1200.0) == Approx(300.0));    // start
    CHECK(zone_feed_mm_min(p, 14.99, 100.0, 1200.0) == Approx(300.0));
    CHECK(zone_feed_mm_min(p, 15.0, 100.0, 1200.0) == Approx(1800.0));  // normal
    CHECK(zone_feed_mm_min(p, 89.99, 100.0, 1200.0) == Approx(1800.0));
    CHECK(zone_feed_mm_min(p, 90.0, 100.0, 1200.0) == Approx(180.0));   // finish
    // A strand shorter than both zones is all start: it still has to anchor.
    CHECK(zone_feed_mm_min(p, 0.0, 8.0, 1200.0) == Approx(300.0));
    // Only the start zone configured: the rest keeps the strand's own feed.
    FiberEmitParams q;
    q.start_speed_mm_s = 5.0;
    q.start_length_mm  = 15.0;
    CHECK(zone_feed_mm_min(q, 0.0, 100.0, 1200.0) == Approx(300.0));
    CHECK(zone_feed_mm_min(q, 20.0, 100.0, 1200.0) == Approx(1200.0));
}

TEST_CASE("emit_strand writes vendor entity comments when verbose comments are on", "[Fiber][FiberEmitter]")
{
    FiberStrand closed;
    closed.layer_id       = 5;
    closed.z              = 1.20;
    closed.pts            = {{0.0, 0.0}, {60.0, 0.0}, {60.0, 40.0}, {0.0, 40.0}, {0.0, 0.0}};
    closed.ratio_p        = 2.0;
    closed.fiber_rate     = 0.25;
    closed.feed_mm_min    = 1200.0;
    closed.tail_length_mm = 54.8;
    REQUIRE(closed.finalize());

    FiberEmitParams quiet;
    std::string out_off;
    REQUIRE(emit_strand(closed, quiet, out_off));
    CHECK(out_off.find("; Inset XF start") == std::string::npos);
    CHECK(out_off.find("; SEAM Fiber at") == std::string::npos);

    FiberEmitParams loud = quiet;
    loud.verbose_comments = true;
    std::string out_on;
    REQUIRE(emit_strand(closed, loud, out_on));
    CHECK(out_on.find("; Inset XF start\n") != std::string::npos);
    CHECK(out_on.find("; SEAM Fiber at X0.000 Y0.000 Z1.200\n") != std::string::npos);
    CHECK(out_on.find("; Fiber infill start") == std::string::npos);

    FiberStrand open = closed;
    open.pts = {{0.0, 0.0}, {80.0, 0.0}, {80.0, 40.0}, {0.0, 40.0}}; // open: no return
    REQUIRE(open.finalize());
    std::string out_fill;
    REQUIRE(emit_strand(open, loud, out_fill));
    CHECK(out_fill.find("; Fiber infill start\n") != std::string::npos);
    CHECK(out_fill.find("; Inset XF start") == std::string::npos);
}

TEST_CASE("emit_strand ramps the deposition feedrate through the speed zones", "[Fiber][FiberEmitter]")
{
    FiberStrand s;
    s.layer_id       = 3;
    s.z              = 0.30;
    s.pts            = {{100.0, 100.0}, {130.0, 100.0}, {200.0, 100.0}};
    s.ratio_p        = 2.0;
    s.fiber_rate     = 0.25;
    s.feed_mm_min    = 1200.0;
    s.tail_length_mm = 54.8;
    std::string err;
    REQUIRE(s.finalize(&err));

    FiberEmitParams p;
    p.start_speed_mm_s  = 5.0;   // F300 for the first 15 mm
    p.start_length_mm   = 15.0;
    p.normal_speed_mm_s = 30.0;  // F1800 after it
    std::string out;
    REQUIRE(emit_strand(s, p, out, &err));
    // The first body move starts at 0 mm, so it is a start-zone move; the
    // second starts 30 mm in and cruises. The tail is past both.
    CHECK(out.find("G1 X130.00 Y100.00 V15.000 U7.500 P2.000 F300\n") != std::string::npos);
    CHECK(out.find(" F1800\n") != std::string::npos);
    // Default params leave the single planned feedrate everywhere.
    std::string plain;
    REQUIRE(emit_strand(s, FiberEmitParams{}, plain, &err));
    CHECK(plain.find(" F300\n") == std::string::npos);
    CHECK(plain.find(" F1800\n") == std::string::npos);
}

TEST_CASE("emit_strand uses the strand's calibrated tail, not the params field", "[Fiber]")
{
    FiberStrand s;
    s.layer_id       = 3;
    s.z              = 0.30;
    s.pts            = {{100.0, 100.0}, {200.0, 100.0}};
    s.ratio_p        = 2.0;
    s.fiber_rate     = 0.25;
    s.feed_mm_min    = 1200.0;
    s.tail_length_mm = 54.8;
    std::string err;
    REQUIRE(s.finalize(&err));

    FiberEmitParams params;
    params.tail_length_mm = 999.0; // must be ignored: the strand owns the calibration
    std::string out;
    REQUIRE(emit_strand(s, params, out, &err));
    REQUIRE(out.find(";CUT DISTANCE 54.8\n") != std::string::npos);
    REQUIRE(out.find("999") == std::string::npos);
}

TEST_CASE("FiberStrand acceptance: chord grid -> one strand, one early cut, deposited tail", "[Fiber]")
{
    FiberStrand s = make_chord_grid();
    std::string err;
    REQUIRE(s.finalize(&err));
    REQUIRE(s.path_length() == Approx(238.0));
    REQUIRE(s.num_body_moves() >= 1);
    REQUIRE(s.num_tail_moves() >= 1);

    // Ruling economics: commanded fiber ~= (path - tail) * rate; the tail is matrix
    // feed at the body payout rate (review finding #1): V/mm = rate * P = 0.5.
    REQUIRE(s.total_u_feed == Approx((238.0 - 54.8) * 0.25).margin(0.05));
    REQUIRE(s.total_v_tail == Approx(54.8 * 0.25 * 2.0).margin(0.05));

    FiberEmitParams params;
    std::string     out;
    REQUIRE(emit_strand(s, params, out, &err));

    // One window, one cut for 39 sub-chords: tessellation created no lifecycle.
    REQUIRE(count_sub(out, "M1001") == 1);
    REQUIRE(count_sub(out, "M2800") == 1);
    REQUIRE(count_sub(out, "M1002") == 1);
    REQUIRE(count_sub(out, "; Cutting completed.") == 1);

    // Cut fires before the path end: at least one deposition move follows it,
    // and every post-cut move is V-bearing and U-free (no U drives a severed
    // tail; no inert travel either). No E anywhere (composite dialect).
    const auto lines = split_lines(out);
    size_t     i_cut = 0, i_done = 0;
    bool found_cut = false, found_done = false;
    for (size_t i = 0; i < lines.size(); ++i) {
        if (lines[i].rfind(";CUT DISTANCE", 0) == 0) {
            i_cut = i;
            found_cut = true;
        }
        if (lines[i] == "; Cutting completed.") {
            i_done = i;
            found_done = true;
        }
        // No E word on the code portion of any line: this dialect speaks U/V
        // only (comments such as "; Extrude restart" may contain letters).
        if (lines[i].rfind("G1", 0) == 0) {
            const size_t sc = lines[i].find(';');
            const std::string code = (sc == std::string::npos) ? lines[i] : lines[i].substr(0, sc);
            REQUIRE(code.find('E') == std::string::npos);
        }
    }
    REQUIRE(found_cut);
    REQUIRE(found_done);
    REQUIRE(i_done > i_cut + 1); // at least one deposition move between cut and handshake
    bool tail_moves_seen = false;
    for (size_t i = i_cut + 1; i < i_done; ++i) {
        const std::string& l = lines[i];
        REQUIRE(l.rfind("G1 ", 0) == 0);
        REQUIRE(l.find(" V") != std::string::npos);
        REQUIRE(l.find(" U") == std::string::npos);
        tail_moves_seen = true;
    }
    REQUIRE(tail_moves_seen);

    // The tail is deposited INTO the structure: the last post-cut move ends at
    // the strand endpoint (50, 138), and the release follows at that endpoint.
    REQUIRE(lines[i_done - 1].find("X50.00 Y138.00") != std::string::npos);
    REQUIRE(lines[i_done + 1].find("V-1.000 ; Retract") != std::string::npos);
    REQUIRE(lines.back() == "M1002");
}

TEST_CASE("FiberStrand finalize rejects non-strand shapes", "[Fiber]")
{
    auto base = []() {
        FiberStrand s;
        s.layer_id       = 1;
        s.z              = 0.2;
        s.ratio_p        = 2.0;
        s.fiber_rate     = 0.25;
        s.feed_mm_min    = 1200.0;
        s.tail_length_mm = 54.8;
        return s;
    };

    // Isolated feature shorter than the calibrated tail: report / plastic-only.
    {
        FiberStrand s    = base();
        s.pts            = {{0.0, 0.0}, {40.0, 0.0}}; // 40 < 54.8
        std::string err;
        REQUIRE_FALSE(s.finalize(&err));
        REQUIRE(err.find("path shorter than the calibrated tail") != std::string::npos);
        REQUIRE_FALSE(s.finalized);
    }
    // An uncalibrated (zero) tail is not a strand.
    {
        FiberStrand s         = base();
        s.pts                 = {{0.0, 0.0}, {100.0, 0.0}};
        s.tail_length_mm      = 0.0;
        std::string err;
        REQUIRE_FALSE(s.finalize(&err));
        REQUIRE(err.find("tail_length_mm") != std::string::npos);
    }
    // tail_v_factor outside (0,1] would over/under-run the calibrated payout.
    {
        FiberStrand s   = base();
        s.pts           = {{0.0, 0.0}, {100.0, 0.0}};
        s.tail_v_factor = 0.0;
        std::string err;
        REQUIRE_FALSE(s.finalize(&err));
        REQUIRE(err.find("tail_v_factor") != std::string::npos);
    }
    // Repeated points: silent simplification of a planned strand is forbidden.
    {
        FiberStrand s = base();
        s.pts         = {{0.0, 0.0}, {100.0, 0.0}, {100.0, 0.0}};
        std::string err;
        REQUIRE_FALSE(s.finalize(&err));
        REQUIRE(err.find("zero-length segment") != std::string::npos);
    }
}

TEST_CASE("FiberStrand tail payout equals the body matrix rate", "[Fiber]")
{
    // Payout model (G-code review finding #1, 2026-10-01): tail V per mm of tail path
    // is tail_v_factor * fiber_rate * ratio_p, so at the default factor 1.0 the
    // post-cut V/XY ratio matches the joint U/V body deposits instead of jumping to 1.
    auto make = [](double factor) {
        FiberStrand s;
        s.layer_id       = 1;
        s.z              = 0.2;
        s.pts            = {{100.0, 100.0}, {200.0, 100.0}};
        s.ratio_p        = 2.0;
        s.fiber_rate     = 0.25;
        s.feed_mm_min    = 1200.0;
        s.tail_length_mm = 54.8;
        s.tail_v_factor  = factor;
        return s;
    };
    std::string err;
    {
        FiberStrand s = make(1.0);
        REQUIRE(s.finalize(&err));
        REQUIRE(s.num_tail_moves() == 1);
        REQUIRE(s.tail_v[0] / 54.8 == Approx(0.25 * 2.0)); // V/mm equals the body matrix payout
    }
    {
        FiberStrand s = make(0.5);
        REQUIRE(s.finalize(&err));
        REQUIRE(s.tail_v[0] == Approx(54.8 * 0.5 * 0.25 * 2.0)); // factor halves the payout
    }
}

TEST_CASE("emit_strand refuses unfinalized strands and inconsistent lists", "[Fiber]")
{
    FiberStrand s;
    s.layer_id       = 1;
    s.z              = 0.2;
    s.pts            = {{0.0, 0.0}, {100.0, 0.0}};
    s.ratio_p        = 2.0;
    s.fiber_rate     = 0.25;
    s.feed_mm_min    = 1200.0;
    s.tail_length_mm = 54.8;

    FiberEmitParams params;
    std::string     out;
    std::string     err;
    REQUIRE_FALSE(emit_strand(s, params, out, &err)); // not finalized yet
    REQUIRE(out.empty());
    REQUIRE(err.find("finalized") != std::string::npos);

    REQUIRE(s.finalize(&err));
    // Tampering after finalize must be caught, not emitted.
    s.body_u.pop_back();
    REQUIRE_FALSE(emit_strand(s, params, out, &err));
    REQUIRE(out.empty());
}

// ---- Composite-head priming (fs_fiber_prime) --------------------------------
// The priming line is a strand, so the tests below pin the two things that
// matter: the emitted block is the reference priming window byte-for-byte, and
// a line that cannot carry a body plus the calibrated tail is refused rather
// than silently shortened.

TEST_CASE("emit_fiber_prime_line writes the reference priming window", "[Fiber][FiberEmitter]")
{
    // Reference machine constants (the shipped CF-nozzle profile).
    FiberEmitParams p; // restart 55 @F1200, hop 1.2, prime V4, retract V1, lift 0.6

    FiberPrimeLine line;
    std::string    plan_err;
    REQUIRE(plan_fiber_prime_line({10.0, 10.0}, {90.0, 10.0}, 0.20, 1, 0.034, 0.98, 1200.0, 54.8,
                                  line, &plan_err));
    REQUIRE(plan_err.empty());

    std::string out;
    std::string err;
    REQUIRE(emit_fiber_prime_line(line, p, true, out, &err));
    REQUIRE(err.empty());

    // Budget: floor(55 + 24.696) = 79. Body ends at the cut, 25.20 mm in; the
    // severed 54.8 mm tail is paid out to the line end under matrix drag.
    const std::string golden =
        "T0 ; switch extruder type to:FIBER\n"
        "M1001 L79\n"
        "G1 F1200 Z1.40\n"
        "G1 X10.00 Y10.00 F1200\n"
        "G1 F1200 U55.000 ; Extrude restart\n"
        "G1 F1200 Z0.20\n"
        "G1 F600 V1.000 ; Recover matrix retract\n"
        "G1 F600 V3.000 ; Matrix prime\n"
        "G1 X35.20 Y10.00 V0.840 U24.696 P0.034 F1200\n"
        "; Start to cut\n"
        "M2800\n"
        "M400\n"
        ";CUT DISTANCE 54.8\n"
        "G1 X90.00 Y10.00 V1.826 F1200\n"
        "; Cutting completed.\n"
        "G1 F600 V-1.000 ; Retract\n"
        "G1 F1200 Z0.80\n"
        "M1002\n"
        "T1 ; switch extruder type to:PLASTIC\n";
    REQUIRE(err == golden);
}

TEST_CASE("emit_fiber_prime_line leaves the tool bracket out when unwrapped", "[Fiber][FiberEmitter]")
{
    FiberEmitParams p;
    FiberPrimeLine  line;
    std::string     err;
    REQUIRE(plan_fiber_prime_line({10.0, 10.0}, {90.0, 10.0}, 0.20, 1, 0.034, 0.98, 1200.0, 54.8,
                                  line, &err));

    std::string wrapped;
    std::string bare;
    REQUIRE(emit_fiber_prime_line(line, p, true, wrapped, &err));
    REQUIRE(emit_fiber_prime_line(line, p, false, bare, &err));
    // The wrap is only the bracket: the window between the tool changes is identical.
    const std::string open  = "T0 ; switch extruder type to:FIBER\n";
    const std::string close = "T1 ; switch extruder type to:PLASTIC\n";
    REQUIRE(wrapped.size() > open.size() + close.size());
    CHECK(wrapped.compare(0, open.size(), open) == 0);
    CHECK(wrapped.compare(wrapped.size() - close.size(), close.size(), close) == 0);
    CHECK(bare == wrapped.substr(open.size(), wrapped.size() - open.size() - close.size()));
    // Unwrapped output is the window alone: it still opens and closes exactly once.
    CHECK(count_sub(bare, "M1001 ") == 1);
    CHECK(count_sub(bare, "M1002") == 1);
    CHECK(bare.find("T0") == std::string::npos);
    // No layer marker: the priming window belongs to no layer.
    CHECK(bare.find("; LAYER:") == std::string::npos);
}

TEST_CASE("plan_fiber_prime_line refuses a line that cannot carry a body and a tail", "[Fiber][FiberEmitter]")
{
    FiberPrimeLine line;
    std::string    err;
    // Exactly the tail length: no body before the cut.
    CHECK_FALSE(plan_fiber_prime_line({0.0, 0.0}, {54.8, 0.0}, 0.2, 1, 0.034, 0.98, 1200.0, 54.8, line, &err));
    CHECK(err.find("tail") != std::string::npos);
    CHECK(line.tail_length_mm == 0.0); // refused plan leaves nothing half-built

    CHECK_FALSE(plan_fiber_prime_line({0.0, 0.0}, {100.0, 0.0}, 0.2, 0, 0.034, 0.98, 1200.0, 54.8, line, &err));
    CHECK_FALSE(plan_fiber_prime_line({0.0, 0.0}, {100.0, 0.0}, 0.2, 1, 0.0, 0.98, 1200.0, 54.8, line, &err));
    CHECK_FALSE(plan_fiber_prime_line({0.0, 0.0}, {100.0, 0.0}, 0.2, 1, 0.034, 0.0, 1200.0, 54.8, line, &err));
    CHECK_FALSE(plan_fiber_prime_line({0.0, 0.0}, {100.0, 0.0}, 0.2, 1, 0.034, 0.98, 0.0, 54.8, line, &err));
    CHECK_FALSE(plan_fiber_prime_line({0.0, 0.0}, {0.0, 0.0}, 0.2, 1, 0.034, 0.98, 1200.0, 54.8, line, &err));
    CHECK_FALSE(plan_fiber_prime_line({0.0, 0.0}, {100.0, 0.0}, 0.2, 1, 0.034, 0.98, 1200.0, 0.0, line, &err));
    CHECK_FALSE(plan_fiber_prime_line({0.0, 0.0}, {100.0, 0.0}, std::nan(""), 1, 0.034, 0.98, 1200.0, 54.8, line, &err));
}

TEST_CASE("emit_fiber_prime_line surfaces the strand's own refusal", "[Fiber][FiberEmitter]")
{
    // A plan that passes the cheap gates but cannot be finalized: the feed
    // rounds to zero, which would be fiber without matrix.
    FiberPrimeLine line;
    std::string    err;
    REQUIRE(plan_fiber_prime_line({0.0, 0.0}, {100.0, 0.0}, 0.2, 1, 0.0001, 0.98, 1200.0, 54.8, line, &err));
    std::string out;
    CHECK_FALSE(emit_fiber_prime_line(line, FiberEmitParams{}, false, out, &err));
    CHECK(out.empty());
    CHECK_FALSE(err.empty());
}
