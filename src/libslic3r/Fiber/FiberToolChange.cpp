// License: GNU AGPLv3 or higher

#include "FiberToolChange.hpp"

#include <cmath>
#include <cstdio>
#include <cctype>
#include <sstream>
#include <string>
#include <vector>

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
//
// `inside_visit` is emitted between the clean and the exit, i.e. while the
// carriage is parked at the station. It exists for the owner's transition-
// ordering rule (2026-10-05): a blocking temperature wait is only ever paid
// at the station, never on the way to it and never over the part. The wait
// goes AFTER the clean so the head is still hot enough to shed matrix onto
// the brush rather than after it has been parked at standby.
//
// When the visit is disabled there is nowhere to pay the wait, so the block
// is emitted in place instead. That is deliberately not silent: the station-
// only rule is unsatisfiable without a station, and printing cold is worse
// than a wait in the open, so the wait wins and the analyzer reports
// FS_WAIT_OUTSIDE_STATION against the export rather than the code quietly
// dropping a readiness wait the print needs.
void append_brush(std::string& s, bool enable, const std::string& inside_visit = std::string())
{
    if (!enable) {
        if (!inside_visit.empty())
            s += inside_visit;
        return;
    }
    put(s, "MOVE_TO_BRUSH_STATION");
    put(s, "CLEAN_NOZZLE");
    if (!inside_visit.empty())
        s += inside_visit;
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
    // Every thermal line of this half is built into its own string first, so
    // the whole thermal block can be placed INSIDE the brush visit. That is the
    // owner's transition-ordering rule (2026-10-05): no blocking wait before
    // station entry, and no cooling of the selected head before its clean.
    //
    // Order inside the visit, and why:
    //   CLEAN_NOZZLE      first, while the outgoing head is still at working
    //                     temperature, so matrix sheds onto the brush molten
    //                     rather than being parked half-cooked in the nozzle.
    //   M104 T1 standby   only now is the outgoing plastic head parked. Doing it
    //                     before the clean is the FS_COOL_BEFORE_CLEAN defect.
    //   M104 T0 standby   the both-heads clause, a measured no-op (see below).
    //   M104 T0 working   pre-charge, so the head is climbing before it is waited
    //                     for; charging to standby then waiting for working only
    //                     lengthens the wait.
    //   M109 T0           the blocking readiness wait, paid at the station.
    std::string thermal;
    if (p.t0_temp_c > 0) {
        // Standby target for the plastic head being put away: emitted when the
        // slicer owns thermals, because M104 does not block and a parked nozzle
        // left at working temperature is the ooze source this sequence exists to
        // remove. Gated on the restore temperature being KNOWN: parking the head
        // is only safe if the return half can put it back, and the caller passes 0
        // when the filament working temperature cannot be resolved.
        if (p.t1_working_c > 0)
            append_standby(thermal, p.t1_standby_c, 1);
        if (p.emit_readiness_wait) {
            // Standby target for the head being ACTIVATED, so that this block
            // carries a target for BOTH heads as the objective requires. It is a
            // measured no-op: the composite head is already parked at this exact
            // temperature, because the fibre->plastic half dropped it here on the
            // way out of the previous window, and 0/315 reference blocks send it.
            // It is harmless (M104 does not block) and it is overridden two lines
            // later by the working-temperature pre-charge, so the vendor thermal
            // behaviour is unchanged. It sits INSIDE the readiness gate on purpose:
            // where no M109 follows, dropping the head to standby with no way to
            // bring it back would strand it cold, so the gate keeps the clause
            // satisfiable without ever making the print worse.
            // Suppressed when the scheduler already preheated this head: see
            // incoming_preheat_pending. The both-heads clause is still met by the
            // working-temperature pre-charge emitted next.
            if (!p.incoming_preheat_pending)
                append_standby(thermal, p.t0_standby_c, 0);
            put(thermal, "M104 S" + std::to_string(p.t0_temp_c) + " T0 ; pre-charge");
            put(thermal, "M109 S" + std::to_string(p.t0_temp_c) + " T0");
        }
    }
    // Clean against the head being put away (T1) while it is still selected,
    // and pay the composite head's readiness wait while parked there.
    append_brush(s, p.brush_on_toolchange, thermal);
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
    // Departure lift, after the withdrawal and before the station visit. See
    // departure_lift_z_mm in the header for why the lift lives here rather than
    // at the end of the fibre window.
    if (p.departure_lift_z_mm > 0.0 && std::isfinite(p.departure_lift_z_mm)) {
        char lift_buf[96];
        std::snprintf(lift_buf, sizeof(lift_buf), "G1 F%.0f Z%.2f",
                      p.departure_lift_f, p.departure_lift_z_mm);
        put(s, lift_buf);
    }
    // Thermal order inside the visit, per the owner transition contract
    // (2026-10-05, section 3 of the tail/release spec):
    //
    //   1. the composite head being put away is parked to standby, AFTER its
    //      clean (dropping the selected head to standby before the brush visit
    //      is the FS_COOL_BEFORE_CLEAN defect) and BEFORE the incoming wait;
    //   2. the plastic head's working temperature is RESTORED with a blocking
    //      M109, not M104: the incoming head is T1, it was dropped to standby
    //      when this window opened, and nothing else on this path puts it back
    //      (the fibre block bypasses the exporter's own tool-change temperature
    //      handling), so resuming plastic deposition without the wait would
    //      extrude cold. This is the "incoming temperature readiness" half of
    //      the vendor sequence and it is not gated by emit_readiness_wait: the
    //      standby dwell this wait pays for happens on EVERY window.
    //
    // The standby-first ordering is the whole point of this half: the outgoing
    // head must be off its working target before the printer starts spending
    // wall-clock waiting for the incoming one, otherwise the composite head
    // cooks matrix for the whole duration of the M109. The blocking wait is
    // paid while parked at the station rather than over the part.
    std::string thermal;
    if (p.t0_temp_c > 0) {
        // Park the composite head being put away, AFTER its clean and BEFORE the
        // incoming wait.
        append_standby(thermal, p.t0_standby_c, 0);
        if (p.t1_working_c > 0) {
            // Pre-charge the plastic head before waiting on it, matching the
            // vendor's M104 S<working> T1 / M109 S<working> T1 pair.
            put(thermal, "M104 S" + std::to_string(p.t1_working_c) + " T1 ; pre-charge");
            put(thermal, "M109 S" + std::to_string(p.t1_working_c) + " T1");
        }
    }
    // Clean against the head being put away (T0) while it is still selected.
    append_brush(s, p.brush_on_toolchange, thermal);
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


// Startup park: hoist the FIRST blocking hotend wait in the machine start g-code
// into the station bracket that follows it.
//
// Why this exists: the reviewed export's one unparked thermal wait was the vendor
// startup M109 S250 T1, which sits between M400 and T1, i.e. before the
// MOVE_TO_BRUSH_STATION / CLEAN_NOZZLE / MOVE_OUT_BRUSH_STATION bracket a few lines
// later. Every one of the 32 model-window waits was correctly bracketed; the startup
// one was inherited from the vendor profile untouched. The rule the owner set is that
// a hotend wait happens at an established park, and the startup case is not exempt
// just because it is the first one.
//
// The transform is deliberately narrow:
//   * Bed (M190) and chamber (M191) waits are NOT touched. Soaking a bed or a chamber
//     at a brush station is meaningless, and the owner put them outside this rule
//     explicitly.
//   * The wait only moves DOWN into an existing station bracket, never to an invented
//     coordinate. If the profile has no station-entry macro after the wait, the text
//     is returned unchanged and the caller reports it, rather than us guessing a park
//     position we cannot prove is safe.
//   * The wait lands directly after the entry macro, so the head is parked while it
//     waits and the cleaning that follows is still hot, exactly as the vendor intended.
std::string park_initial_hotend_wait(const std::string& machine_start_gcode, bool& out_parked)
{
    out_parked = false;
    if (machine_start_gcode.empty())
        return machine_start_gcode;

    std::vector<std::string> lines;
    {
        std::istringstream ss(machine_start_gcode);
        std::string        line;
        while (std::getline(ss, line))
            lines.push_back(line);
    }

    // First blocking HOTEND wait: M109. M190/M191 are the bed and chamber equivalents
    // and are out of scope by the rule above.
    auto command_of = [](const std::string& l) {
        const size_t sc = l.find(';');
        std::string  s  = (sc == std::string::npos) ? l : l.substr(0, sc);
        const size_t a  = s.find_first_not_of(" \t\r\n");
        if (a == std::string::npos)
            return std::string();
        const size_t b = s.find_first_of(" \t\r\n", a);
        std::string  c = s.substr(a, b == std::string::npos ? std::string::npos : b - a);
        for (char& ch : c)
            ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
        return c;
    };

    size_t wait_idx = std::string::npos;
    for (size_t i = 0; i < lines.size(); ++i) {
        if (command_of(lines[i]) == "M109") {
            wait_idx = i;
            break;
        }
    }
    if (wait_idx == std::string::npos)
        return machine_start_gcode;

    // First station-entry macro AFTER the wait.
    size_t entry_idx = std::string::npos;
    for (size_t i = wait_idx + 1; i < lines.size(); ++i) {
        const std::string c = command_of(lines[i]);
        if (c.find("MOVE_TO_BRUSH_STATION") != std::string::npos ||
            c.find("PARK") != std::string::npos) {
            entry_idx = i;
            break;
        }
    }
    if (entry_idx == std::string::npos)
        return machine_start_gcode;

    const std::string wait_line = lines[wait_idx];
    lines.erase(lines.begin() + wait_idx);
    // Erasing above the entry shifted it down by one.
    lines.insert(lines.begin() + entry_idx, wait_line);

    std::string out;
    for (const std::string& l : lines)
        out += l + "\n";
    out_parked = true;
    return out;
}

} // namespace Fiber
} // namespace Slic3r
