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
and the machine start/end macros own offsets and cleaning. The one exception is
the composite priming line, which asks for the wrap explicitly: it is emitted
from the plate preamble, whose tool context is the plastic head.

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