// License: GNU AGPLv3 or higher
//
// FibreSeeker3 composite tool-change sequence.
//
// The vendor machine supplies the whole tool-change sequence itself; a slicer
// that emits a bare `T0` / `T1` and nothing else leaves the heads in whatever
// state the last window left them in. That is the stringing defect the owner
// measured against Rocket Slicer: one withdrawal per PLATE instead of one per
// FIBRE WINDOW, no standby targets, no cleaning, no per-head fan routing.
//
// Rocket pairs both sides of every switch. The two blocks below are copied
// verbatim from the reference exports in FibreSeeker3/Test_files (Rocket
// Slicer v1.3.1.480) and the coverage is measured, not remembered: across
// 720 harvested change-extruder blocks in 6 continuous-fiber references,
// 351/351 plastic->fibre blocks and 358/358 fibre->plastic blocks carry M400,
// an M104 for BOTH heads, a blocking M109, a brush visit and an M106 with an
// explicit P word.
//
//   plastic -> fibre   M400 / M104 S270 T0 / M104 S150 T1 / M106 P1 S229 /
//                      G1 F1200 E-5 / Z lift / brush / T0 / M106 P2 S255 /
//                      M109 S270 T0 / M106 P1 S0
//   fibre -> plastic   M400 / M104 S250 T1 / M104 S180 T0 / M106 P2 S204 /
//                      G1 F600 V-4 / Z lift / T1 / M106 P1 S255 / brush /
//                      M109 S250 T1 / M106 P2 S0
//
// Both heads get a STANDBY target at every switch, which is what the owner
// objective asks for: T1 -> 150 and T0 -> 180 going into fibre, T0 -> 180 and
// T1 -> 150 coming out of it. The head being ACTIVATED additionally gets a
// WORKING-temperature pre-charge (T0 -> 270 going in, T1 -> 250 coming out)
// immediately before its blocking M109, because charging it only to standby
// and then waiting for working temperature would just lengthen the wait.
//
// The vendor divergence is recorded, not copied. Measured over 636
// M104-bearing reference blocks (315 plastic->fibre, 321 fibre->plastic): the
// vendor emits exactly TWO M104 per block, one standby for the head being put
// away and one working-temperature pre-charge for the head being activated,
// and 0/636 give the activated head a standby target. So the activated head's
// standby line we emit is a measured NO-OP, not a behaviour change: that head
// is already parked at that exact temperature, because the opposite half
// dropped it there when the previous window closed, and M104 does not block.
// The clause is therefore IMPLEMENTED, with the vendor divergence recorded
// here and in docs/HLSD/continuous_fiber_gcode.md. It is emitted only inside
// the readiness gate, where an M109 follows to undo it: outside that gate a
// standby target with no way back would strand the head cold, which would
// make the print worse than the vendor, so the clause yields to correctness
// there and only there.
//
// The three V quantities are what close the stationary-V ledger. emit_strand
// already ends a window at V-<fs_retract_v> and opens the next one by
// recovering that same amount plus any prime remainder, so within-window V is
// self-pairing. The tool-change withdrawal is a THIRD quantity, issued on the
// way out and never recovered, and adding it is what moves the per-window net
// from +3 mm to -1 mm.
//
// The vendor number is NOT -1 mm. Measured with one method on all six
// reference exports, Rocket nets 0.000 mm per fibre window in every single
// file, decomposing as +5 restart/feed, -1 window retract, -4 tool-change
// withdrawal. Our canonical single-window ledger is +4 -1 -4 = -1 mm, i.e.
// one millimetre MORE withdrawn than the vendor, and the entire difference is
// the prime: fs_prime_v is 4 where the vendor primes 5. That is a profile
// value, deliberately untouched here, and reported for the owner to rule on.
// The -1 mm in the original brief is Rocket's two withdrawals without its
// prime, not Rocket's net.
//
// Both functions are pure: they take the numbers, return the block, and touch
// no printer state. The E-side withdrawal and its recovery are NOT here,
// because they belong to the GCodeWriter's filament model (retract_length_
// toolchange, deretraction speed) and are emitted by the caller around these
// blocks.
//
// Everything stays behind fs_fiber_enabled / fs_t0_wrap at the call site, so a
// non-continuous-fiber export never reaches this code.

#pragma once

#include <string>

namespace Slic3r {
namespace Fiber {

// Everything the tool-change block needs, resolved from fs_* keys and the
// filament temperatures by the caller. Defaults are the FibreSeeker3 SK3 CF
// values the vendor reference files show.
struct FiberToolChangeParams
{
    // Active composite (T0) temperature, degrees. Also the M109 blocking
    // target. 0 means "the machine start gcode owns T0 thermals": the
    // caller must not emit the block at all.
    int t0_temp_c = 270;
    // Standby target dropped on the composite head when its window closes,
    // degrees. Keeps the hotend charged enough to come back fast without
    // cooking matrix in the nozzle. Measured vendor value 180, for a working
    // temperature of 270; the PLA reference parks at 100 for a working 230,
    // so this tracks material rather than sitting at a fixed offset.
    int t0_standby_c = 180;
    // Standby target dropped on the plastic head while fibre is running,
    // degrees. Measured vendor value 150, constant across all six references
    // including the PLA one, so it is a fixed park rather than a fraction of
    // the working temperature.
    int t1_standby_c = 150;
    // Working temperature restored on the plastic head when fibre ends,
    // degrees. Resolved by the caller from the filament profile, because the
    // CF path never goes through the exporter's own tool-change temperature
    // handling and nothing else will put T1 back on its working temperature.
    // 0 = the caller could not resolve it: emit neither the restore nor the
    // standby drop, because parking a head that cannot be brought back would
    // strand it at standby for the rest of the plate.
    int t1_working_c = 0;

    // Second matrix (V) withdrawal issued when leaving fibre, mm, at
    // toolchange_retract_v_f. This is the move that closes the per-cycle
    // stationary-V ledger against the window prime.
    double toolchange_retract_v_mm = 4.0;
    double toolchange_retract_v_f  = 600.0;

    // Visit the brush station at the switch. The clean is emitted against the
    // head being PUT AWAY, while it is still the selected tool, because that
    // is the head that carries fresh material to the switch station.
    bool brush_on_toolchange = true;

    // Drive the auxiliary fan ports P3 and P5 at the switch. The vendor
    // reference exports command both to 255 while the composite head deposits
    // and back to 0 for the plastic pass, and the machine start gcode leaves
    // them at 0, so without this clause they never run at all. Unlike the
    // P1/P2 routing this is NOT derived from the cooling demand: 255 is the
    // measured vendor value, a fixed on/off per head, so the clause is gated
    // only by this flag and by nothing else.
    bool aux_fans_on_toolchange = true;

    // Part-cooling demand at the switch, percent 0..100, resolved by the
    // caller from the cooling system's current value. The exporter's generic
    // fan writer emits a bare M106 with no P word, which on this firmware
    // drives fan3 AND fan4 at once (FibreSeeker3 Exploration/HardwareInfo.md
    // 7.3 and its fans.cfg remap note: plain M106 sets both; P1 -> fan3,
    // P2 -> fan4), so a fibre window also spins the part-cooling output and a
    // plastic window also spins the fibre-side one. The demand is routed to the
    // output the depositing material needs and the other one is zeroed. Negative
    // means "cooling demand not known yet": emit no fan commands at all.
    int part_cooling_pct = -1;

    // Whether this switch emits the BLOCKING readiness wait (M109) and the
    // standby pre-charge of the head being activated. False only for the first
    // window of a plate that was NOT primed, where the preamble preheat is still
    // standing and nothing has parked the head since. A primed plate pays the
    // wait on its first model window: the priming window is closed by
    // emit_toolchange_to_plastic below, which parks T0 like any other exit.
    //
    // This gates the WAIT and the incoming head's pre-charge, and NOTHING ELSE.
    // Parking the head being put away is not gated: it is free (M104 does not
    // block), it is the line that stops a parked nozzle cooking matrix, and
    // skipping it on the first window would leave the head that just printed
    // plastic at working temperature through the fibre pass - the exact ooze
    // this sequence exists to prevent. The brush visit and the fan routing are
    // likewise ungated: the pairing is per window, not per plate.
    bool emit_readiness_wait = true;
};

// The plastic -> fibre (T1 -> T0) half of the sequence, everything except the
// caller's outgoing E withdrawal: standby targets, the blocking readiness
// wait, per-head fan routing and the brush visit, then the bare T0 line.
// Returns an empty string when the parameters ask for none of it.
std::string emit_toolchange_to_fiber(const FiberToolChangeParams& p);

// The fibre -> plastic (T0 -> T1) half: the second matrix withdrawal, the
// standby/restore temperature pair, fan routing, the brush visit, the bare T1
// line. The caller's E recovery goes after this block, because E movement is
// illegal while T0 is selected.
std::string emit_toolchange_to_plastic(const FiberToolChangeParams& p);

// Net commanded STATIONARY V movement for one complete fibre cycle at these
// parameters: the window prime puts `prime_v_mm` in, the window-end retract
// takes `retract_v_mm` out, the tool-change withdrawal takes
// `toolchange_retract_v_mm` out and is never recovered. Exposed so the ledger
// is a number the tests and the report can name rather than a claim. Measured
// vendor net is 0.000 mm per window in all six reference exports (+5 -1 -4).
// At the shipped fs_prime_v of 4 this returns -1.000 mm, one mm more withdrawn
// than the vendor; with no tool-change withdrawal it returns +3.000 mm.
// Closing the last millimetre is a prime change, i.e. a profile decision.
double fiber_cycle_net_stationary_v_mm(double prime_v_mm, double retract_v_mm, double toolchange_retract_v_mm);

} // namespace Fiber
} // namespace Slic3r
