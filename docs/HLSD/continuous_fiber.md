# Continuous fiber

OrcaSlicer can generate continuous-fiber (composite) deposition for machines that
have a separate fiber channel. The capability is off by default
(`fs_fiber_enabled` false). Every `fs_*` key defaults to a value that leaves a
plain FFF slice byte-identical.

## Machine model

The FibreSeeker3 SK3 is the first shipped target. It has two heads:

- T0, the composite head, deposits fiber on U and matrix on V. It has no E axis.
- T1, the plastic head, deposits thermoplastic on E. It has no U or V.

Fiber is never mixed with E on the same move. U is forward-only. After a
restart prime, joint deposit moves carry both U and V with V = U * P at the
printed precision.

## Slice path

When the capability is on and `fs_fiber_mode` is not `plastic_only`, G-code
export plans strands from that layer's external perimeters
(`Fiber::build_layer_strands`) and optionally subtracts a plastic reservation
band around the accepted strand geometry (`Fiber::subtract_reserve_bands`).
Reservation is applied to a copy of the sliced collections for that export
only; the stored slice is restored afterwards.

`fs_fiber_mode`:

| Value | Effect |
|---|---|
| `off` | Fiber follows the external perimeter on scheduled layers. |
| `plastic_only` | No fiber, even if the capability flag is on. |
| `walls` | Interior fiber at `fs_fiber_coverage_percent`; plastic keeps the outer skin. |
| `solid` | 100 percent interior fiber inside the outer plastic shell. |

A path that cannot carry body plus the calibrated tail is rejected and counted,
never shortened. `fs_fiber_enforce` aborts the slice if a reinforceable layer
would print without a complete fiber window.

## Composite-head priming

`fs_fiber_prime` lays one sacrificial composite strand on bare bed, in the plate
preamble, before the plate's real deposition. The composite head is charged by
the same restart-plus-prime lifecycle a strand uses, so the first strand of a
plate does not start on the stale tow left by the last one.

The gate for `when_fiber` is `Fiber::print_carries_fiber`, which evaluates the
layer schedule over the whole plate. It calls `Fiber::fiber_layer_scheduled`,
the same producer the exporter calls per layer, so the priming decision and the
export never disagree about whether a plate is reinforced; a schedule rule that
lives in only one of the two places is the bug class that would prime a
plastic-only plate or leave a reinforced one unprimed. `plastic_only` is not a
print with CF features.

## Config ownership
Process presets own the pattern (mode, coverage, infill, speeds, schedule).
Printer presets own capability and hardware lifecycle (restart feed, tail
length, nozzle diameter, wrap). `full_fff_config()` applies the printer last,
so a key that lives on both lists is overwritten by the machine default.
Pattern keys are therefore registered only on the print list.
