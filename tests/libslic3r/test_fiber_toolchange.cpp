// FibreSeeker3 paired composite tool-change sequence unit tests (owner ruling
// 2026-10-04: the vendor machine supplies the whole tool-change sequence at
// EVERY switch, so the exporter must too).
//
// What is pinned here, and why each pin is the one that matters:
//   - Both halves of the sequence exist and are PAIRED: a withdrawal going out,
//     a standby pair for both heads, a blocking readiness wait, a brush visit,
//     and the matching matrix withdrawal plus temperature restore coming back.
//   - The brush visit is emitted against the head being PUT AWAY, i.e. while
//     that head is still the selected tool, so the clean happens before the
//     switch rather than after it.
//   - Fan outputs carry an explicit P word. A bare M106 drives fan3 and fan4
//     on this firmware, which is the un-routed behaviour being replaced.
//   - The stationary-V ledger closes at the vendor's -1 mm, and the arithmetic
//     is exposed as a number rather than asserted in prose.
//   - Nothing is invented: no thermal lines when the machine owns thermals, no
//     fan lines when the cooling demand is unresolved.

#include <catch2/catch_all.hpp>
using Catch::Approx;

#include "libslic3r/Fiber/FiberToolChange.hpp"

#include <cmath>
#include <string>

using namespace Slic3r::Fiber;

namespace {

// Count occurrences of `needle` in `hay`.
int count_sub(const std::string& hay, const std::string& needle)
{
    int n = 0;
    for (size_t pos = hay.find(needle); pos != std::string::npos; pos = hay.find(needle, pos + needle.size()))
        ++n;
    return n;
}

// Count LINES that start with `prefix`. Counting a tool line with a plain
// substring search would also match the T word of an M104/M109 temperature
// line, which is exactly the confusion this counter exists to avoid.
int count_line_start(const std::string& hay, const std::string& prefix)
{
    int n = 0;
    size_t pos = 0;
    while (pos < hay.size()) {
        const size_t eol = hay.find(0x0A /* LF */, pos);
        const size_t end = eol == std::string::npos ? hay.size() : eol;
        if (hay.compare(pos, prefix.size(), prefix) == 0)
            ++n;
        if (eol == std::string::npos)
            break;
        pos = eol + 1;
    }
    return n;
}

// True when the block ends with `line` plus exactly one newline.
bool ends_with_line(const std::string& hay, const std::string& line)
{
    if (hay.size() < line.size() + 1 || hay.back() != 0x0A)
        return false;
    return hay.compare(hay.size() - line.size() - 1, line.size(), line) == 0;
}

// The shipped FibreSeeker3 SK3 CF profile, as the exporter resolves it at a
// mid-plate switch: T0 hot, T1 on its filament working temperature, cooling
// demand resolved.
FiberToolChangeParams profile_params()
{
    FiberToolChangeParams p;
    p.t0_temp_c               = 270;
    p.t0_standby_c            = 180;
    p.t1_standby_c            = 150;
    p.t1_working_c            = 250;
    p.toolchange_retract_v_mm = 4.0;
    p.toolchange_retract_v_f  = 600.0;
    p.brush_on_toolchange     = true;
    p.part_cooling_pct        = 100;
    return p;
}

} // namespace

TEST_CASE("FiberToolChange: plastic -> fibre emits the full outgoing sequence", "[Fiber][FiberToolChange]")
{
    const std::string s = emit_toolchange_to_fiber(profile_params());

    // No M400 inside this block: on this half the vendor flush precedes the
    // caller outgoing withdrawal, so the caller emits it (see GCode::process_layer).
    CHECK(s.find("M400") == std::string::npos);
    // BOTH heads carry a standby target, which is the clause the owner objective
    // states literally: the plastic head being put away and the composite head
    // being activated. The composite head then ALSO gets a working-temperature
    // pre-charge, which is what the vendor actually sends (measured: M104 S150 T1
    // then M104 S270 T0 on every one of 315 reference entries, 0/315 with a
    // standby target on the activated head). The extra line is a measured no-op
    // and the divergence is recorded in the header and the HLSD doc.
    CHECK(s.find("M104 S150 T1 ; standby") != std::string::npos);
    CHECK(s.find("M104 S180 T0 ; standby") != std::string::npos);
    CHECK(s.find("M104 S270 T0 ; pre-charge") != std::string::npos);
    // Both standby targets precede the pre-charge that overrides one of them.
    CHECK(s.find("M104 S180 T0 ; standby") < s.find("M104 S270 T0 ; pre-charge"));
    // Blocking readiness wait for the composite head, and it is M109 not M104:
    // selecting T0 without waiting is the "one initial wait per head" defect.
    CHECK(s.find("M109 S270 T0") != std::string::npos);
    // Brush visit, three macro lines, exactly once.
    CHECK(count_sub(s, "MOVE_TO_BRUSH_STATION") == 1);
    CHECK(count_sub(s, "CLEAN_NOZZLE") == 1);
    CHECK(count_sub(s, "MOVE_OUT_BRUSH_STATION") == 1);
    // The tool line is bare T0 with the firmware-parsed comment, and last.
    REQUIRE(!s.empty());
    CHECK(ends_with_line(s, "T0 ; switch extruder type to:FIBER"));
    // Every line terminated. Ten lines of standby/charge/wait/brush/fan
    // routing plus the two auxiliary fan lines the switch adds.
    CHECK(count_sub(s, "\n") == 12);
}

TEST_CASE("FiberToolChange: fibre -> plastic emits the matching return sequence", "[Fiber][FiberToolChange]")
{
    const std::string s = emit_toolchange_to_plastic(profile_params());

    // The second matrix withdrawal, at the configured feedrate, negative V.
    CHECK(s.find("G1 F600 V-4.000 ; Toolchange matrix retract") != std::string::npos);
    // Standby on the head being put away, working temperature restored on the
    // head being activated WITH A BLOCKING WAIT. Without the restore T1 resumes
    // at standby; without the wait it resumes printing AT standby.
    CHECK(s.find("M400") == 0);
    CHECK(s.find("M104 S180 T0 ; standby") != std::string::npos);
    CHECK(s.find("M104 S250 T1 ; pre-charge") != std::string::npos);
    CHECK(s.find("M109 S250 T1") != std::string::npos);    // Cleaned on the way out, before the switch.
    CHECK(count_sub(s, "CLEAN_NOZZLE") == 1);
    REQUIRE(!s.empty());
    CHECK(ends_with_line(s, "T1 ; switch extruder type to:PLASTIC"));
    // The withdrawal comes BEFORE the tool line: V is a T0-channel axis.
    CHECK(s.find("Toolchange matrix retract") < s.find("T1 ;"));
    CHECK(s.find("M104 S180 T0") < s.find("T1 ;"));
}

TEST_CASE("FiberToolChange: fan outputs are routed per head with an explicit P word", "[Fiber][FiberToolChange]")
{
    const std::string to_fiber   = emit_toolchange_to_fiber(profile_params());
    const std::string to_plastic = emit_toolchange_to_plastic(profile_params());

    // No bare M106 anywhere: on this firmware it drives fan3 AND fan4 at once.
    CHECK(to_fiber.find("M106 S") == std::string::npos);
    CHECK(to_plastic.find("M106 S") == std::string::npos);

    // Entering fibre: the fibre-side output (P2/fan4) runs, the part-cooling
    // output (P1/fan3) is zeroed rather than left running.
    CHECK(to_fiber.find("M106 P2 S255") != std::string::npos);
    CHECK(to_fiber.find("M106 P1 S0") != std::string::npos);
    // Leaving fibre: the mirror image.
    CHECK(to_plastic.find("M106 P2 S0") != std::string::npos);
    CHECK(to_plastic.find("M106 P1 S255") != std::string::npos);
    // Each output names the head that owns it, so the routing is attributable
    // per head in the emitted bytes rather than only in the source. The PORT is
    // shared (HardwareInfo 7.3: P1->fan3, P2->fan4, neither per-head), so this
    // is per-head attribution of a shared output - see the PENDING OWNER RULING
    // block in docs/HLSD/continuous_fiber_gcode.md.
    CHECK(to_fiber.find("M106 P2 S255 ; fibre-side cooling, fan4, owned by T0 (depositing)") != std::string::npos);
    CHECK(to_fiber.find("M106 P1 S0 ; part-cooling, fan3, owned by T1 (idle)") != std::string::npos);
    CHECK(to_plastic.find("M106 P2 S0 ; fibre-side cooling, fan4, owned by T0 (idle)") != std::string::npos);
    CHECK(to_plastic.find("M106 P1 S255 ; part-cooling, fan3, owned by T1 (depositing)") != std::string::npos);

    // The demand is routed, not invented: half cooling demand halves both.
    FiberToolChangeParams half = profile_params();
    half.part_cooling_pct = 50;
    const std::string s = emit_toolchange_to_fiber(half);
    CHECK(s.find("M106 P2 S127") != std::string::npos);
    CHECK(s.find("M106 P1 S0") != std::string::npos);
}

TEST_CASE("FiberToolChange: nothing is invented when a source has no value", "[Fiber][FiberToolChange]")
{
    // Cooling demand unresolved: emit no DEMAND-DERIVED fan lines rather than
    // guess a speed. The auxiliary ports are not demand-derived - the vendor
    // fixes them at 255 for the fibre pass - so this state does not invent them
    // and they stay emitted.
    FiberToolChangeParams nofan = profile_params();
    nofan.part_cooling_pct = -1;
    const std::string s = emit_toolchange_to_fiber(nofan);
    CHECK(s.find("M106 P1") == std::string::npos);
    CHECK(s.find("M106 P2") == std::string::npos);
    CHECK(s.find("M106 P3 S255") != std::string::npos);
    CHECK(s.find("M106 P5 S255") != std::string::npos);
    // The switch itself still happens and is still cleaned.
    CHECK(s.find("T0 ;") != std::string::npos);
    CHECK(count_sub(s, "CLEAN_NOZZLE") == 1);

    // fs_t0_temp = 0: the machine start gcode owns T0 thermals, so no
    // temperature lines from us - but the clean, the fans and the switch stay.
    FiberToolChangeParams nothermal = profile_params();
    nothermal.t0_temp_c = 0;
    const std::string f = emit_toolchange_to_fiber(nothermal);
    const std::string r = emit_toolchange_to_plastic(nothermal);
    CHECK(f.find("M104") == std::string::npos);
    CHECK(f.find("M109") == std::string::npos);
    CHECK(r.find("M104") == std::string::npos);
    CHECK(f.find("T0 ;") != std::string::npos);
    CHECK(r.find("T1 ;") != std::string::npos);
    // The matrix withdrawal is not a thermal command: it stays.
    CHECK(r.find("Toolchange matrix retract") != std::string::npos);

    // Restore temperature unknown (no writer filament, so the caller cannot
    // resolve it): the plastic head is NOT parked, because the return half has
    // nothing to bring it back to. Parking it anyway would leave the head that
    // prints plastic stranded at standby for the rest of the plate.
    FiberToolChangeParams norestore = profile_params();
    norestore.t1_working_c = 0;
    const std::string nr = emit_toolchange_to_fiber(norestore);
    CHECK(nr.find("M104 S150 T1") == std::string::npos);
    // The composite head is still charged and waited for, and the switch still
    // happens and is still cleaned. Its standby target is present too: it lives
    // inside the readiness gate, and this half still has the M109 to undo it.
    CHECK(nr.find("M104 S180 T0 ; standby") != std::string::npos);
    CHECK(nr.find("M104 S270 T0 ; pre-charge") != std::string::npos);
    CHECK(nr.find("M109 S270 T0") != std::string::npos);
    CHECK(count_sub(nr, "CLEAN_NOZZLE") == 1);
    CHECK(nr.find("T0 ;") != std::string::npos);

    // Zero tool-change withdrawal: the withdrawal line disappears, everything
    // else is untouched.
    FiberToolChangeParams no_v = profile_params();
    no_v.toolchange_retract_v_mm = 0.0;
    CHECK(emit_toolchange_to_plastic(no_v).find("Toolchange matrix retract") == std::string::npos);

    // Brush switched off: only the brush lines go.
    FiberToolChangeParams nobrush = profile_params();
    nobrush.brush_on_toolchange = false;
    const std::string nb = emit_toolchange_to_fiber(nobrush);
    CHECK(nb.find("CLEAN_NOZZLE") == std::string::npos);
    CHECK(nb.find("MOVE_TO_BRUSH_STATION") == std::string::npos);
    CHECK(nb.find("M109 S270 T0") != std::string::npos);
}

TEST_CASE("FiberToolChange: a withheld readiness wait still parks the outgoing head", "[Fiber][FiberToolChange]")
{
    // The readiness gate is off only for the first window of a plate that was NOT
    // primed, where the preamble preheat is still standing. (A PRIMED plate pays the
    // wait on its first model window, because the priming window's exit parks T0 like
    // any other exit - see the startup-purge case below.)
    FiberToolChangeParams first = profile_params();
    first.emit_readiness_wait = false;
    const std::string s = emit_toolchange_to_fiber(first);
    CHECK(s.find("M109") == std::string::npos);
    // Only the T0 charge/wait pair is skipped. The plastic head being put away
    // is STILL dropped to standby: M104 does not block, and leaving the head that
    // just printed plastic at working temperature through the fibre window is the
    // ooze this whole sequence exists to stop.
    CHECK(s.find("M104 S150 T1 ; standby") != std::string::npos);
    CHECK(s.find("M104 S270 T0") == std::string::npos);
    // And the activated head gets NO standby target here either. This is the one
    // place the objective clause yields: with no M109 to bring the head back,
    // parking it would strand the composite head cold, which is worse than the
    // vendor and worse than not emitting the line at all.
    CHECK(s.find("M104 S180 T0") == std::string::npos);
    // Pairing is per window, not per plate: the clean and the switch stay.
    CHECK(count_sub(s, "CLEAN_NOZZLE") == 1);
    CHECK(s.find("T0 ;") != std::string::npos);
    CHECK(s.find("M106 P2") != std::string::npos);

    // The return half of that same window still pays the incoming wait: the
    // standby dwell it is waiting for happened, so T1 must not resume cold.
    const std::string r = emit_toolchange_to_plastic(first);
    CHECK(r.find("M104 S180 T0 ; standby") != std::string::npos);
    CHECK(r.find("M109 S250 T1") != std::string::npos);
}
TEST_CASE("FiberToolChange: auxiliary ports P3 and P5 are driven on both halves", "[Fiber][FiberToolChange]")
{
    // The owner table row: "Both Rocket files eventually command P3 and P5 to
    // 255" against our output "commands P3 and P5 to 0, with no later enable".
    // The machine start gcode zeroes both ports, so before this clause nothing
    // raised them again for the rest of the plate.
    const std::string in  = emit_toolchange_to_fiber(profile_params());
    const std::string out = emit_toolchange_to_plastic(profile_params());

    // Going into fibre both ports run; coming out both are explicitly zeroed
    // rather than left running, so no window inherits the previous one's state.
    CHECK(count_line_start(in, "M106 P3 S255") == 1);
    CHECK(count_line_start(in, "M106 P5 S255") == 1);
    CHECK(count_line_start(out, "M106 P3 S0") == 1);
    CHECK(count_line_start(out, "M106 P5 S0") == 1);
    // Exactly one command per port per half, so a window cannot stack them.
    CHECK(count_line_start(in, "M106 P3") == 1);
    CHECK(count_line_start(in, "M106 P5") == 1);
    CHECK(count_line_start(out, "M106 P3") == 1);
    CHECK(count_line_start(out, "M106 P5") == 1);
    // Named in the emitted bytes, so the routing is attributable at the switch.
    CHECK(in.find("M106 P3 S255 ; auxiliary fan on while T0 deposits") != std::string::npos);
    CHECK(out.find("M106 P5 S0 ; exhaust fan off while T1 deposits") != std::string::npos);
    // Both precede the tool line: the ports are set up for the pass that follows.
    // The tool line is the LAST line, and it is located from the end rather than
    // by find("T0 ;"), which would also match the T word of the standby line.
    const std::string tool_in  = "T0 ; switch extruder type to:FIBER";
    const std::string tool_out = "T1 ; switch extruder type to:PLASTIC";
    CHECK(ends_with_line(in, tool_in));
    CHECK(ends_with_line(out, tool_out));
    CHECK(in.find("M106 P3") < in.size() - tool_in.size() - 1);
    CHECK(out.find("M106 P5") < out.size() - tool_out.size() - 1);
    // The value is the vendor constant, NOT the cooling demand: half cooling
    // demand halves P1/P2 and leaves P3/P5 at full.
    FiberToolChangeParams half = profile_params();
    half.part_cooling_pct = 50;
    const std::string h = emit_toolchange_to_fiber(half);
    CHECK(h.find("M106 P2 S127") != std::string::npos);
    CHECK(h.find("M106 P3 S255") != std::string::npos);
    CHECK(h.find("M106 P5 S255") != std::string::npos);
    // Switched off: only the two auxiliary lines go; the routing and the switch
    // are untouched.
    FiberToolChangeParams off = profile_params();
    off.aux_fans_on_toolchange = false;
    const std::string o = emit_toolchange_to_fiber(off);
    CHECK(count_line_start(o, "M106 P3") == 0);
    CHECK(count_line_start(o, "M106 P5") == 0);
    CHECK(count_line_start(o, "M106 P2") == 1);
    CHECK(ends_with_line(o, "T0 ; switch extruder type to:FIBER"));
}

TEST_CASE("FiberToolChange: the stationary V ledger is the measured arithmetic, not a claim", "[Fiber][FiberToolChange]")
{
    // Prime 4 in, window retract 1 out, tool-change withdrawal 4 out and never
    // recovered. The vendor is NOT at -1 mm: measured across all six reference
    // exports in Test_files, Rocket nets 0.000 mm per fibre window in every
    // file, decomposing as +5 restart/feed, -1 window retract, -4 tool-change.
    // At the shipped fs_prime_v of 4 we therefore net -1.000 mm, one millimetre
    // MORE withdrawn than the vendor, and the whole difference is the prime.
    // Closing it is fs_prime_v 4 -> 5, a profile value this change deliberately
    // does not touch. The -1 mm in the original brief is the vendor two
    // withdrawals without its prime, not the vendor net.
    CHECK(fiber_cycle_net_stationary_v_mm(4.0, 1.0, 4.0) == Approx(-1.0));
    // Without the tool-change withdrawal the cycle is net POSITIVE by 3 mm:
    // this is the number the owner measured as the leading ooze candidate.
    CHECK(fiber_cycle_net_stationary_v_mm(4.0, 1.0, 0.0) == Approx(+3.0));
    // The ledger is linear in the withdrawal, so a profile can be tuned without
    // changing the shape of the argument.
    CHECK(fiber_cycle_net_stationary_v_mm(4.0, 1.0, 3.0) == Approx(0.0));
    CHECK(fiber_cycle_net_stationary_v_mm(4.0, 1.0, 5.0) == Approx(-2.0));
    // Degenerate inputs give a finite answer rather than NaN.
    CHECK(fiber_cycle_net_stationary_v_mm(std::nan(""), 1.0, 4.0) == Approx(0.0));
}

TEST_CASE("FiberToolChange: the struct defaults are the measured vendor values", "[Fiber][FiberToolChange]")
{
    // The defaults are measurements off the reference exports in Test_files,
    // not numbers from the brief. The brief carried 150 / 120 for the standby
    // pair; the six exports on disk carry 180 / 150, and the 150 for the
    // plastic head is constant across all six including the PLA one, while the
    // composite park tracks material (180 at a 270 working temperature, 100 at
    // a 230 one). Pinned so a future edit cannot quietly restore the brief.
    FiberToolChangeParams d;
    CHECK(d.t0_standby_c == 180);
    CHECK(d.t1_standby_c == 150);
    CHECK(d.toolchange_retract_v_mm == Approx(4.0));
    CHECK(d.toolchange_retract_v_f == Approx(600.0));
    CHECK(d.brush_on_toolchange == true);
    // The vendor raises P3/P5 for the fibre pass in every reference export, so
    // the clause is on by default rather than opt-in.
    CHECK(d.aux_fans_on_toolchange == true);
    // A default-constructed block must therefore carry the measured numbers.
    d.t0_temp_c      = 270;
    d.t1_working_c   = 250;
    d.part_cooling_pct = 0;
    const std::string in  = emit_toolchange_to_fiber(d);
    const std::string out = emit_toolchange_to_plastic(d);
    CHECK(in.find("M104 S150 T1 ; standby")  != std::string::npos);
    CHECK(in.find("M104 S120 T1")            == std::string::npos);
    CHECK(out.find("M104 S180 T0 ; standby") != std::string::npos);
    CHECK(out.find("M104 S150 T0")           == std::string::npos);
}

TEST_CASE("FiberToolChange: the two halves are paired, not one-sided", "[Fiber][FiberToolChange]")
{
    const FiberToolChangeParams p = profile_params();
    const std::string in  = emit_toolchange_to_fiber(p);
    const std::string out = emit_toolchange_to_plastic(p);

    // One clean per half, so a fibre window costs exactly two brush visits -
    // the vendor's 24 cleans for 15 windows in the S-hook shape.
    CHECK(count_sub(in, "CLEAN_NOZZLE") == 1);
    CHECK(count_sub(out, "CLEAN_NOZZLE") == 1);
    // Exactly one tool line per half, and each half names the OTHER tool.
    // Line-initial on purpose: "T1 ;" also occurs inside "M104 S150 T1 ; standby".
    CHECK(count_line_start(in, "T0 ;") == 1);
    CHECK(count_line_start(in, "T1 ;") == 0);
    CHECK(count_line_start(out, "T1 ;") == 1);
    CHECK(count_line_start(out, "T0 ;") == 0);
    // Both heads are targeted on both halves: the head being put away is always
    // dropped to standby, and the head being activated is brought to temperature
    // (M104 pre-charge + M109 wait going in, M109 wait coming back).
    //
    // The counts are deliberately asymmetric. Going INTO fibre the block carries
    // a standby target for BOTH heads, which is the clause the owner objective
    // states literally for this half, so three M104: T1 standby, T0 standby, then
    // the T0 working-temperature pre-charge. Coming OUT of fibre the objective
    // asks only for the second matrix withdrawal and the recovery, so two M104:
    // T0 standby plus the T1 pre-charge. Adding a T1 standby on the return half
    // would be a third no-op line for a clause nobody asked about, so it is not
    // emitted.
    CHECK(count_sub(in, "M104") == 3);
    CHECK(count_sub(in, "M109") == 1);
    CHECK(count_sub(out, "M104") == 2);
    CHECK(count_sub(out, "M109") == 1);    // Every emitted line is newline terminated.
    CHECK(in.back() == '\n');
    CHECK(out.back() == '\n');
}

TEST_CASE("FiberToolChange: the startup purge pays both halves of the sequence", "[Fiber][FiberToolChange]")
{
    // The startup purge is a fibre window like any other, so it is entered and
    // left by this same pair. Two things make its parameters differ from a
    // mid-plate switch, and both are pinned here.
    //
    //   1. The cooling demand is NOT resolved yet - the purge runs before any
    //      layer has been through CoolingBuffer - so part_cooling_pct is -1.
    //   2. The readiness wait is ON on BOTH halves. On the entry because the
    //      preamble's M104 only asked for the temperature and this is the one
    //      blocking wait the plate pays for T0; on the exit because this half
    //      really does park T0, unlike the model's first window, which withholds
    //      the park for a head that is provably hot.
    FiberToolChangeParams p = profile_params();
    p.part_cooling_pct    = -1;
    p.emit_readiness_wait = true;

    const std::string in  = emit_toolchange_to_fiber(p);
    const std::string out = emit_toolchange_to_plastic(p);

    // The defect this closes: the purge exit used to be the strand's own
    // V-retract, Z lift, M1002 and a bare T1, so the composite head stayed
    // commanded at its working target for the whole dwell until the first model
    // fibre strand. The exit now parks it.
    CHECK(out.find("M104 S180 T0 ; standby") != std::string::npos);
    // And the plastic head, dropped to standby by the entry, is brought back
    // with a blocking wait rather than left to resume cold.
    CHECK(out.find("M109 S250 T1") != std::string::npos);
    // The entry parks the plastic head, so it is not left at its printing
    // temperature through the purge dwell either.
    CHECK(in.find("M104 S150 T1 ; standby") != std::string::npos);
    // The per-window matrix withdrawal is issued while T0 is still selected,
    // which is why the purge window is emitted unwrapped and the tool lines come
    // from these two helpers rather than from the emitter's bracket.
    CHECK(count_line_start(out, "T1 ;") == 1);
    CHECK(count_line_start(out, "T0 ;") == 0);
    CHECK(out.find("V-4.000") != std::string::npos);
    // One clean per half: the purge takes the head to the brush station on the
    // way in and on the way out, so tow residue is not carried to the plate.
    CHECK(count_sub(in, "CLEAN_NOZZLE") == 1);
    CHECK(count_sub(out, "CLEAN_NOZZLE") == 1);

    // Unresolved cooling suppresses the DEMAND-DERIVED routing only. The aux
    // ports carry a vendor constant, not a demand, so they are driven here too -
    // the start gcode zeroes them and nothing else ever raised them.
    CHECK(in.find("M106 P1") == std::string::npos);
    CHECK(in.find("M106 P2") == std::string::npos);
    CHECK(out.find("M106 P1") == std::string::npos);
    CHECK(out.find("M106 P2") == std::string::npos);
    CHECK(in.find("M106 P3 S255") != std::string::npos);
    CHECK(in.find("M106 P5 S255") != std::string::npos);
    CHECK(out.find("M106 P3 S0") != std::string::npos);
    CHECK(out.find("M106 P5 S0") != std::string::npos);

    // The dependency the exit park creates: because T0 was parked, the plate's
    // first MODEL window is entered with the readiness wait back ON, so it carries
    // a fresh reheat-and-wait for the composite head rather than assuming it is
    // still hot from startup. That is the same block as profile_params() with the
    // gate open, pinned by the outgoing-sequence case above; named here so the
    // dependency is stated next to the park that creates it.
    FiberToolChangeParams rearmed = profile_params();
    rearmed.emit_readiness_wait = true;
    const std::string re = emit_toolchange_to_fiber(rearmed);
    CHECK(re.find("M104 S270 T0 ; pre-charge") != std::string::npos);
    CHECK(re.find("M109 S270 T0") != std::string::npos);
}
