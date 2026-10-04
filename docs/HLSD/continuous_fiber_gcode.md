# Continuous-fiber G-code dialect

The emitter (`Fiber::emit_strand`) writes one window per finalized strand.
Window shape:

1. Optional `; LAYER:<n> [z]`
2. `M1001 L<budget>` — open. L is advisory: `floor(restart + sum of printed body U)`.
3. Lift, XY travel to the start above the layer, U-only restart, descend.
4. V-only prime at the start point.
5. Joint deposits `G1 X.. Y.. U.. V.. P.. F..` up to the cut. V is derived from
   the printed U and the 3-decimal ratio P.
6. `; Start to cut` / `M2800` / `M400` / `;CUT DISTANCE <tail>`
7. Tail deposition along the remaining path: V-bearing, U-free.
8. `; Cutting completed.` — firmware-parsed cut-boundary handshake.
9. V-only retract, Z lift, `M1002` — close.

U is never negative. E never appears on T0 moves; U/V never appear on T1 moves.
`M1001`/`M1002` have no firmware handler; they exist so the independent
validator (`tests/fibreseeker/fs_gcode_validator.py`) can bound a window.

Tool changes and dock/brush motion are not the emitter's job. When
`fs_t0_wrap` is on, the G-code writer wraps a layer's fiber block in `T0`/`T1`
and the machine start/end macros own offsets. The one emission exception is the
composite priming line, which asks for the wrap explicitly: it is emitted
from the plate preamble, whose tool context is the plastic head.

## Paired tool-change sequence

The vendor machine supplies a complete tool-change sequence at EVERY
plastic<->fibre switch, so the exporter does too. The sequence is emitted per
fibre WINDOW, not once per plate, and both halves are paired. It is rendered by
`Fiber::emit_toolchange_to_fiber` / `emit_toolchange_to_plastic`
(`Fiber/FiberToolChange.{hpp,cpp}`), which are pure: numbers in, block out, no
printer state touched.

Plastic -> fibre, in order:

1. `M400` - planner flush. The vendor leads EVERY tool-change block with it
   (181 occurrences in `Benchy_renforced_level5.gcode`; present in 136/136 and
   37/37 blocks across the reference exports), so no queued extrusion is still
   in flight when the temperature lines land. It precedes the withdrawal, so the
   caller emits it rather than the helper.
2. Outgoing withdrawal of the plastic channel, at `retract_length_toolchange`,
   while T1 is still selected - E is illegal under T0.
3. `M104 S<fs_t1_standby_temp> T1` - park the head being put away.
4. `M104 S<fs_t0_standby_temp> T0` - the standby target for the incoming head,
   so the block carries a standby target for BOTH heads as the objective
   requires. Measured no-op (the vendor does not send it, 0/636 blocks; the head
   is already parked here by the previous window exit) and overridden by step 5.
5. `M104 S<fs_t0_temp> T0` then `M109 S<fs_t0_temp> T0` - pre-charge and then
   wait for the incoming head. The pre-charge is at WORKING temperature, not
   standby: the vendor sends `M104 S270 T0` / `M109 S270 T0` on every entry into
   fibre, so the head is already climbing when the blocking wait is reached.
   Steps 4 and 5 are skipped together on the first window of a primed plate,
   where the preamble preheat plus the priming line already paid the wait, and
   where parking the head with no blocking wait to undo it would strand it cold.

   > **Standby-target clause: IMPLEMENTED, vendor divergence recorded.** The
   > objective asks for "standby targets for both heads" at the plastic->fibre
   > switch, and the emitted block now carries both of them:
   > `M104 S150 T1 ; standby` and `M104 S180 T0 ; standby`, followed by the
   > working-temperature pre-charge and the blocking wait. The divergence from
   > the vendor is measured, not assumed: across 636 M104-bearing reference
   > blocks (315 plastic->fibre, 321 fibre->plastic) the vendor emits exactly
   > two M104 per block and **0/636** give the ACTIVATED head a standby target.
   > The activated-head standby line is therefore a **no-op**, not a behaviour
   > change: that head is already parked at that temperature, because the
   > opposite half dropped it there when the previous window closed, and M104
   > does not block. It is emitted only inside the readiness gate, where a
   > blocking M109 follows to undo it. On the first window of a primed plate,
   > where no M109 is emitted, the line is withheld, because parking a head with
   > no way back would strand the composite head cold. That single gated
   > exception is the only place the clause yields, and it yields to not making
   > the print worse than the vendor.
6. Brush triple, against the head being put away, while it is still selected.
7. `M106 P2 S<n>` / `M106 P1 S<n>` - the demand routed to the cooling output
   the depositing material needs (P2 -> fan4, fibre-side; P1 -> fan3
   part-cooling, per Exploration/HardwareInfo.md 7.3), the other output
   explicitly zeroed.

   > **Fan clause: per-head attribution emitted; PENDING OWNER RULING on the
   > wording.** The objective asks for fan outputs "routed explicitly per head".
   > Taken literally as one fan per head, that is not satisfiable on this
   > machine: P1 -> fan3 is part-cooling and P2 -> fan4 is fibre-side
   > (Exploration/HardwareInfo.md 7.3 and its fans.cfg remap note), and
   > **neither is documented as a per-head output**, so there is no per-head fan
   > output to route a signal to. The emitted bytes now make the per-head
   > attribution explicit in the line itself:
   > `M106 P2 S255 ; fibre-side cooling, fan4, owned by T0 (depositing)` and
   > `M106 P1 S0 ; part-cooling, fan3, owned by T1 (idle)`, mirrored on the way
   > out. So each head cooling output is attributable at the switch, the output
   > the depositing material needs carries the demand, the unused one is
   > explicitly zeroed, no window inherits a fan state from the previous one,
   > and the fibre path emits zero bare `M106`. **One sentence for the ruling:
   > the ports are shared, so this is per-head attribution plus per-head
   > activation and deactivation of a shared output, not a physically separate
   > fan per head** - literal per-head fans would be a firmware fans.cfg
   > question, not an exporter one.
7b. `M106 P3 S255` / `M106 P5 S255` - the auxiliary ports, raised for the fibre
   pass and zeroed on the way back out. Both Rocket reference files eventually
   command P3 and P5 to 255, and the machine start gcode ends by zeroing both,
   so before this clause an export commanded them to 0 and never enabled them
   again for the rest of the plate. The value is deliberately NOT the cooling
   demand: 255 is the measured vendor constant and the clause is a fixed on/off
   per head, so an unresolved cooling state suppresses the P1/P2 routing and
   leaves these two lines standing. Behind `fs_aux_fans_on_toolchange`.
8. `T0 ; switch extruder type to:FIBER`

Fibre -> plastic mirrors it: the second matrix withdrawal
(`G1 F<fs_toolchange_retract_v_speed> V-<fs_toolchange_retract_v>`, issued while
T0 is still selected because V is a T0-channel axis), `M104` standby on T0, then
`M104 S<working> T1` / `M109 S<working> T1` restoring the plastic head - the
incoming head is pre-charged and then brought to temperature with a BLOCKING
wait, because the standby dwell that wait pays for happens on every window - the
brush triple, the mirrored fan pair plus the auxiliary pair zeroed to
`M106 P3 S0` / `M106 P5 S0`, `T1`, then the recovery of step 2. That
recovery is the exporter's own `unretract()`, which is unlift plus unretract, so
the switch is self-contained: the Z hop registered by the outgoing withdrawal is
released at the station rather than being left pending for the next extrusion.
The pristine base emitted neither half at the switch.

Two consequences worth naming. The tool-change withdrawal in the return half is
a THIRD matrix quantity, issued on the way out and never recovered, so the net
commanded stationary V per fibre window goes from +3 mm to -1 mm;
`Fiber::fiber_cycle_net_stationary_v_mm` exposes that ledger as a number rather
than a claim. The vendor is NOT at -1 mm. Measured with one method across all
six reference exports in `Test_files`, Rocket nets **0.000 mm per window in
every file**: +5 restart/feed, -1 window retract, -4 tool-change withdrawal.
At the shipped `fs_prime_v` of 4 we net -1.000 mm, i.e. one millimetre MORE
withdrawn than the vendor, and the whole difference is the prime. Closing it is
`fs_prime_v` 4 -> 5, a profile value deliberately untouched here and reported
for ruling. And because the standby pair now runs on every window, the
plastic head is no longer left at working temperature while fibre prints - the
`M109` restore is what makes that safe to resume.
## Composite priming line

`fs_fiber_prime` charges the composite head before the plate's real deposition
by laying one sacrificial strand on bare bed. It is a strand and not a bespoke
block, because the dialect requires every window to contain a cut: the block is
the ordinary lifecycle over a two-point path, so nothing new is introduced for
the firmware or the validator. It carries no `; LAYER:` marker (it belongs to no
layer), which the validator reports as WARN R13, and no layer marker is also
what keeps it out of layer accounting.

The line is planned by `Fiber::plan_fiber_prime_line` and rendered by
`Fiber::emit_fiber_prime_line`, both in the emitter. It is refused - reported in
the G-code as `; FIBER PRIME:`, never silently shortened - when it is no longer
than `fs_tail_length`, because a strand must carry a body before the cut as well
as the severed tail after it.

`fs_fiber_prime = when_fiber` restricts the cost to prints that carry fiber. The
gate is `Fiber::print_carries_fiber`, which asks the same
`Fiber::fiber_layer_scheduled` the exporter asks per layer, so the priming
decision and the export cannot disagree about whether a plate is reinforced.
Priming also requires `fs_t0_wrap`: the preamble's tool context is the plastic
head, and an unwrapped block would put U/V moves on T1.

`fs_*` keys are banned from the G-code config dump so a stock FFF slice stays
byte-identical in the header as well as the body.
