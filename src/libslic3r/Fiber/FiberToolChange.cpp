// License: GNU AGPLv3 or higher

#include "FiberToolChange.hpp"

#include <cmath>
#include <cstdio>

namespace Slic3r {
namespace Fiber {

namespace {

// Append one complete line. Newlines are added here rather than written into
// the format strings, so no string literal in this file carries an escape.
void put(std::string& s, const std::string& line)
{
    s += line;
    s += '\n';
}

void append_standby(std::string& s, int temp_c, int tool)
{
    if (temp_c > 0)
        put(s, "M104 S" + std::to_string(temp_c) + " T" + std::to_string(tool) + " ; standby");
}

// The brush triple, emitted against the head currently selected - which the
// callers arrange to be the head being put away. Macro-owned motion (machine
// contract s10): the slicer names the station, the firmware moves to it.
void append_brush(std::string& s, bool enable)
{
    if (!enable)
        return;
    put(s, "MOVE_TO_BRUSH_STATION");
    put(s, "CLEAN_NOZZLE");
    put(s, "MOVE_OUT_BRUSH_STATION");
}

// Explicit cooling-output routing at a switch. FibreSeeker3
// Exploration/HardwareInfo.md 7.3 maps P1 -> fan3 (doc term: part-cooling fan,
// PB5) and P2 -> fan4 (doc term: fiber-related fan, PC9), and marks both
// functions as carrying uncertainty; neither is documented as a per-head fan, so
// the emitted comments name the port and the doc term rather than a head.
// Routing means: the fibre-side output runs while fibre deposits and the
// part-cooling output while plastic does, and the output not in use is explicitly
// zeroed. The generic writer's bare M106 drives fan3 and fan4 together,
// which is the un-routed behaviour this replaces on the fibre path; the plastic
// path keeps its own cooling logic.
void append_head_fans(std::string& s, const FiberToolChangeParams& p, bool fiber_active)
{
    // Negative demand means the cooling system has not resolved a speed yet;
    // emitting a fan command then would invent a value the print never asked
    // for, so the routing is simply skipped.
    if (p.part_cooling_pct < 0)
        return;
    const int demand = p.part_cooling_pct > 100 ? 100 : p.part_cooling_pct;
    // The output the depositing material needs keeps the cooling demand; the
    // other one is explicitly zeroed rather than left running.
    const int fiber_pct   = fiber_active ? demand : 0;
    const int plastic_pct = fiber_active ? 0 : demand;
    // Name the head that owns each output, in the emitted line itself, so each
    // head's cooling output is attributable at the switch: fibre deposits from T0
    // (left composite head) and plastic from T1 (right plastic head). The PORT is
    // not per-head - HardwareInfo 7.3 maps P1 to fan3 and P2 to fan4 as shared
    // outputs - so this is per-head ATTRIBUTION of a shared output, which is the
    // reading of "routed explicitly per head" implemented here. See the PENDING
    // OWNER RULING block in docs/HLSD/continuous_fiber_gcode.md.
    const char* dep  = fiber_active ? "depositing" : "idle";
    const char* idle = fiber_active ? "idle" : "depositing";
    char buf[96];
    std::snprintf(buf, sizeof(buf), "M106 P2 S%d ; fibre-side cooling, fan4, owned by T0 (%s)",
                  (int)(255.0 * fiber_pct / 100.0), dep);
    put(s, buf);
    std::snprintf(buf, sizeof(buf), "M106 P1 S%d ; part-cooling, fan3, owned by T1 (%s)",
                  (int)(255.0 * plastic_pct / 100.0), idle);
    put(s, buf);
}

// Auxiliary fan ports P3 and P5 at a switch. The vendor reference exports
// drive both to 255 for the fibre pass and back to 0 for the plastic pass;
// the machine start gcode zeroes them and nothing else ever raised them,
// which is the "commanded to 0 with no later enable" defect. The value is a
// fixed vendor constant, not the cooling demand, so this is deliberately
// independent of part_cooling_pct: an unresolved cooling state must not
// suppress a command whose value the vendor fixes.
void append_aux_fans(std::string& s, const FiberToolChangeParams& p, bool fiber_active)
{
    if (!p.aux_fans_on_toolchange)
        return;
    const char* state = fiber_active ? "on" : "off";
    const char* dep   = fiber_active ? "T0 deposits" : "T1 deposits";
    const int   pwm   = fiber_active ? 255 : 0;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "M106 P3 S%d ; auxiliary fan %s while %s", pwm, state, dep);
    put(s, buf);
    std::snprintf(buf, sizeof(buf), "M106 P5 S%d ; exhaust fan %s while %s", pwm, state, dep);
    put(s, buf);
}

} // namespace

std::string emit_toolchange_to_fiber(const FiberToolChangeParams& p)
{
    std::string s;
    // NOTE: no M400 here, unlike emit_toolchange_to_plastic below. The vendor
    // leads every tool-change block with a planner flush (181 occurrences in
    // Benchy_renforced_level5.gcode; present in 136/136 and 37/37 blocks in the
    // other reference exports), and on this half the flush has to precede the
    // caller's outgoing E withdrawal, so the caller emits it. See the call site
    // in GCode::process_layer.
    // Standby target for the plastic head being put away: emitted when the
    // slicer owns thermals, because M104 does not block and a parked nozzle left
    // at working temperature is the ooze source this sequence exists to remove.
    // Gated on the restore temperature being KNOWN: parking the head is only
    // safe if the return half can put it back, and the caller passes 0 when the
    // filament working temperature cannot be resolved. The composite head is
    // charged and waited for only when the caller asks for the readiness wait; on
    // the first window of a plate that was NOT primed the head is provably hot
    // from the preamble preheat, and dropping it to standby without an M109 to
    // bring it back would make it print cold. A PRIMED plate is the opposite case:
    // the priming window is closed by emit_toolchange_to_plastic below, which
    // parks T0 like any other exit, so the first model window owes the wait again.
    if (p.t0_temp_c > 0) {
        if (p.t1_working_c > 0)
            append_standby(s, p.t1_standby_c, 1);
        if (p.emit_readiness_wait) {
            // Standby target for the head being ACTIVATED, so that this block
            // carries a target for BOTH heads as the objective requires. It is a
            // measured no-op: the composite head is already parked at this exact
            // temperature, because the fibre->plastic half dropped it here on the
            // way out of the previous window, and 0/315 reference blocks send it.
            // It is harmless (M104 does not block) and it is overridden two lines
            // later by the working-temperature pre-charge below, so the vendor
            // thermal behaviour is unchanged. It sits INSIDE the readiness gate on
            // purpose: where no M109 follows, dropping the head to standby with no
            // way to bring it back would strand it cold, so the gate keeps the
            // clause satisfiable without ever making the print worse.
            append_standby(s, p.t0_standby_c, 0);
            // Pre-charge at WORKING temperature, not standby: the vendor sends
            // M104 S<t0_temp> T0 here so the head is already climbing when the
            // blocking M109 below is reached. Charging it to standby and then
            // waiting for working temperature would only make the wait longer.
            put(s, "M104 S" + std::to_string(p.t0_temp_c) + " T0 ; pre-charge");
            put(s, "M109 S" + std::to_string(p.t0_temp_c) + " T0");
        }
    }
    // Clean against the head being put away (T1) while it is still selected.
    append_brush(s, p.brush_on_toolchange);
    append_head_fans(s, p, true);
    append_aux_fans(s, p, true);
    put(s, "T0 ; switch extruder type to:FIBER");
    return s;
}

std::string emit_toolchange_to_plastic(const FiberToolChangeParams& p)
{
    std::string s;
    put(s, "M400"); // see emit_toolchange_to_fiber: leads every vendor block
    // The second matrix withdrawal: issued while T0 is still selected (V is a
    // T0-channel axis) and never recovered by the window prime, which is what
    // pulls the per-cycle stationary-V ledger to the vendor's net negative.
    if (p.toolchange_retract_v_mm > 0.0 && std::isfinite(p.toolchange_retract_v_mm)) {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "G1 F%.0f V-%.3f ; Toolchange matrix retract",
                      p.toolchange_retract_v_f, p.toolchange_retract_v_mm);
        put(s, buf);
    }
    // Standby on the head being put away (T0), always emitted when the slicer
    // owns thermals. Then the plastic head's working temperature is RESTORED
    // with a blocking M109, not M104: the incoming head is T1, it was dropped to
    // standby when this window opened, and nothing else on this path puts it back
    // (the fibre block bypasses the exporter's own tool-change temperature
    // handling), so resuming plastic deposition without the wait would extrude
    // cold. This is the "incoming temperature readiness" half of the vendor
    // sequence and it is not gated by emit_readiness_wait: the standby dwell this
    // wait pays for happens on EVERY window, including the first.
    if (p.t0_temp_c > 0) {
        append_standby(s, p.t0_standby_c, 0);
        if (p.t1_working_c > 0) {
            // Pre-charge the plastic head before waiting on it, matching the
            // vendor's M104 S<working> T1 / M109 S<working> T1 pair.
            put(s, "M104 S" + std::to_string(p.t1_working_c) + " T1 ; pre-charge");
            put(s, "M109 S" + std::to_string(p.t1_working_c) + " T1");
        }
    }
    // Clean against the head being put away (T0) while it is still selected.
    append_brush(s, p.brush_on_toolchange);
    append_head_fans(s, p, false);
    append_aux_fans(s, p, false);
    put(s, "T1 ; switch extruder type to:PLASTIC");
    return s;
}

double fiber_cycle_net_stationary_v_mm(double prime_v_mm, double retract_v_mm, double toolchange_retract_v_mm)
{
    if (!std::isfinite(prime_v_mm) || !std::isfinite(retract_v_mm) || !std::isfinite(toolchange_retract_v_mm))
        return 0.0;
    return prime_v_mm - retract_v_mm - toolchange_retract_v_mm;
}

} // namespace Fiber
} // namespace Slic3r
