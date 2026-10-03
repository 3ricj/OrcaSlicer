# FibreSeeker3 profiles

Vendor bundle: `resources/profiles/FibreSeeker3.json` and
`resources/profiles/FibreSeeker3/`.

The SK3 is a two-head Klipper machine. Two nozzle variants exist:

- `0.4` — composite-capable (T0 fiber + T1 plastic).
- `0.4FFF` — plastic-only; `fs_fiber_enabled` stays off.

Process presets encode the fiber modes (`plastic_only`, `walls`, `solid`) and
coverage ladders. The selectable ladder is `0.12mm` so plastic layer height is
half of the 0.24 mm fiber bead (`fs_fiber_z_step`), matching Rocket Slicer's
default plastic step and making `fs_fiber_schedule=macro_layer` land on every
second layer. Filament presets for FibreSeek plastics and continuous tows
(`X-CCF`, `X-CGF`) are printer-specific. Tow filaments are declarative: T0
temperature is owned by the machine macros.

Moonraker is the print host. Discovery sends an SSDP M-SEARCH that includes
`X-SPECIAL-MARKER: FIBRESEEK3D` because that firmware ignores unmarked
searches. A reply is accepted if it identifies as Moonraker; the marker does
not have to be echoed.
