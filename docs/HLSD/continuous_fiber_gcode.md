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
and the machine start/end macros own offsets and cleaning.

`fs_*` keys are banned from the G-code config dump so a stock FFF slice stays
byte-identical in the header as well as the body.
