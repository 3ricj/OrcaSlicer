# FibreSeeker continuous-fiber fork

This branch adds **native continuous-fiber composite G-code** to OrcaSlicer for the **FibreSeeker3 SK3**. The feature is strictly opt-in: with **Continuous fiber capability** off (`fs_fiber_enabled` false), defaults on every `fs_*` key are chosen so a normal FFF slice stays unchanged, including the G-code config header (those keys are omitted from the dump).

Work on this path has been validated mainly by **physical prints** and **inspection of exported G-code** for compatibility with the machine’s expected dialect.

---

## Overview

Continuous fiber is not a special infill pattern bolted onto FFF. It is a **second deposition channel** that runs **after** the layer’s ordinary plastic extrusions are planned and emitted.

Implementation lives under `src/libslic3r/Fiber/` and hooks into G-code export in `GCode.cpp`. High-level behavior is also summarized in `docs/HLSD/continuous_fiber.md` and `docs/HLSD/continuous_fiber_gcode.md`.

---

## Machine model: two heads, three feeders

The SK3 is modeled as a **two-tool** Klipper machine in G-code, with **three independent material feed paths**:

| Tool | Role | Axes | Material |
|------|------|------|----------|
| **T0** | Composite (“fiber”) head | **U** (tow), **V** (matrix) — **no E** | Continuous tow + impregnating matrix |
| **T1** | Plastic head | **E** — **no U/V** | Standard FFF filament |

Physically the composite carriage combines **matrix and fiber orifices** (commonly **0.7 mm** on the reference machine) with a separate **0.4 mm plastic head** on T1. Orca’s usual **Nozzle diameter** field describes **plastic only**; composite orifice and **deposited bead width** (`fs_fiber_nozzle_diameter`, `fs_fiber_bead_width`) are separate expert settings.

**Deposition rules:**

- **U is forward-only** (never negative). Fresh tow is fed at strand start (`fs_restart_feed`), then consumed along the path at `fs_fiber_rate` (mm tow per mm path).
- **Joint moves** carry **U and V** with **V = U × P** (`fs_matrix_ratio`).
- After the cut, the **tail** is **V-only** (severed tow paid out by the matrix channel).
- **E never appears on T0**; **U/V never appear on T1**.

Vendor profiles ship two **capability variants** (not orifice-size labels in the UI):

- **Plastic+CF** — composite-capable; machine preset enables fiber when the process asks for it.
- **Plastic** — plastic-only; fiber capability stays off.

The setup wizard and nozzle list show those as **Plastic+CF** and **Plastic** instead of raw variant tokens (`0.4CF` / `0.4FFF`).

---

## How fiber support works in the slicer

On fiber-capable layers the pipeline:

1. **Harvests geometry** from that layer’s **external perimeters** (closed loops in bed coordinates).
2. **Plans composite strands** (`Fiber::build_layer_strands`) — polylines with a fixed **one-cut-per-strand** lifecycle (body with joint U+V deposit, then cut, then matrix-only tail).
3. **Optionally reserves plastic** by clipping FFF paths away from the composite bead (`Fiber::subtract_reserve_bands`), so plastic and composite do not occupy the same XY band.
4. **Exports plastic G-code** for the layer using the normal FFF path (T1 / E axis).
5. **Appends the fiber block** at the **end of the layer**: tool selection to the composite head, one **M1001…M1002 window** per strand via `Fiber::emit_strand`, then return to plastic.

### Config ownership

| Side | Owns |
|------|------|
| **Process** | What to reinforce: mode, coverage, infill pattern, rectilinear/isogrid settings, speeds, schedule (when exposed), rates, enforce |
| **Printer** | Capability flag, lifecycle calibration (restart, tail, prime, retract), tool wrap, composite nozzle/bead, reservation, bond overlap |

Pattern keys are registered on the **print** preset list so machine defaults cannot silently override user reinforcement choices (`full_print_config()` applies the printer last).

### Fiber modes (`fs_fiber_mode`)

| Mode | Behavior |
|------|----------|
| `off` | Follow external perimeter on scheduled layers (legacy profile trace). |
| `plastic_only` | No fiber block runs even if capability is on. |
| `walls` | Interior composite at `fs_fiber_coverage_percent`; outer skin stays plastic. First and last layer stay FFF-only. |
| `solid` | Full interior composite inside the plastic shell; may include first/last layers. |

### Layer schedule (`fs_fiber_schedule`)

- **Every layer** — every capable layer gets a fiber window.
- **Z band** — fiber only between `fs_fiber_band_z_min` / `fs_fiber_band_z_max`, on `fs_fiber_z_step` boundaries (stress-zone style reinforcement).
- **Macro layer** — one fiber layer per composite bead height (e.g. **0.24 mm** macro over **0.12 mm** plastic layers).

Optional **Enforce continuous fiber** aborts the slice if a layer that should receive fiber cannot form a complete window.

### Preview

The G-code processor treats moves with **U + XY inside an M1001…M1002 window** as **Continuous fiber** in the 3D preview. Time estimation still treats them as non-E travel so totals stay FFF-oriented. The cooling buffer does not rewrite U/V lines when fiber is enabled.

---

## Managing fiber vs dual heads each layer

Each reinforced layer is a **plastic pass, then a composite pass**:

1. **Before export grouping**, if the layer is scheduled and mode allows fiber, strands are built and **plastic collections are mutated in place** for reservation, then **restored** after export so a second export of the same slice does not double-cut plastic.
2. **FFF extrusions** run in the usual order on **T1**, with plastic removed from **capsule exclusion bands** around accepted strand polylines (body **and** post-cut tail), using planned bead widths and optional bond overlap (`fs_fiber_reserve`, `fs_fiber_bond_overlap`).
3. **End of layer**: safe plastic retract/lift when needed, optional composite hot wait (`fs_t0_temp`), **T0**, emit all strands, **T1**, then invalidate the motion cache so the next move is fully specified.

**Tool wrap** (`fs_t0_wrap`): brackets the layer’s fiber section with bare **`T0 ; switch extruder type to:FIBER`** / **`T1 ; switch extruder type to:PLASTIC`** so **machine start/end macros** own offsets, dock, brush, and withdrawal — not the strand emitter.

Plastic multi-material on T1 still uses Orca’s normal **filament_map** / extruder assignment; composite deposition is orthogonal and never mixes E with U/V on one move.

---

## How fiber path is determined

Path planning is in **`FiberStrandPlanner`** (`build_layer_strands`). Input is **external-perimeter rings**; output is finalized **`FiberStrand`** objects for **`emit_strand`**.

### Perimeters → islands

- Rings merge into **islands** (outer vs hole by nesting).
- Processing order is deterministic (area, then bounding box).
- Tiny islands below area thresholds are **plastic-only**, not enforcement failures.

### Boundary trace

- Each viable island gets a **level-0 boundary loop** with optional **inset** so roving sits **behind** the plastic skin (`boundary_inset_mm` / mode-driven inset from outer plastic wall count).
- Closed loops honor **seam placement** (`fs_fiber_seam_position`).
- Strands shorter than **body + calibrated tail** (`fs_tail_length`) are **rejected**, not shortened. `FiberStrand::finalize()` is the authority.

### Interior reinforcement

When fill is enabled (legacy **`fs_rectify_*`** in `off` mode, or **`FiberModePlan`** for `walls` / `solid`):

- **Rectilinear / solid** — parallel chords, clipped and **chained into serpentine strands**; illegal turns **split** the strand (no out-of-part travel).
- **Isogrid** — three rib families at 60°, same chaining rules.
- **Thin walls** — if the fill admission gate refuses an island, optional **concentric wall loops** (`fs_fiber_wall_loops`) fill wall thickness instead of trace-only.
- **`fs_fiber_chain_loops`** — optionally connect boundary trace to interior in one strand when the connector stays in material.

Angles: base fill angle plus **+90° on odd layers**; `walls`/`solid` can cycle **`fs_fiber_fill_angles`**. Coverage sets pitch for rectilinear; solid forces 100% interior density.

### Corners and segmentation

- **`fs_fiber_min_radius`** — tight corners are counted; policy **keep** or **split** into separate strands.
- **`fs_fiber_max_arc_seg`** — splits long edges for deposition resolution without changing path length.

### Strand → G-code window

Each accepted strand becomes one lifecycle:

1. **M1001 L&lt;budget&gt;** (advisory U budget)
2. Lift, travel, **U-only restart**, descend, **V prime**
3. **Joint G1** body moves to cut position
4. Cut (**M2800**) + **tail** (V-only)
5. **V retract**, lift, **M1002**

Cut position is **`total_path − tail_length`**.

---

## Printer discovery and network setup

The SK3 is intended to be used with **Moonraker** as the print host (`gcode_flavor`: Klipper).

### SSDP discovery

Generic UPnP or mDNS browsing often **does not** find FibreSeeker machines on the LAN. Orca’s Moonraker integration sends a marked **SSDP M-SEARCH** that includes:

```http
X-SPECIAL-MARKER: FIBRESEEK3D
```

along with the usual `MAN`, `ST: ssdp:all`, and related SSDP headers. The printer’s Moonraker SSDP stack is configured to answer **marked** probes; unmarked searches may be ignored.

Discovery behavior in Orca:

- Sends the probe **three times** (350 ms apart) to survive datagram loss.
- Transmits on **each local IPv4 interface**, to **multicast** (`239.255.255.250:1900`) and **subnet broadcast**, so typical AP quirks still reach the printer.
- Accepts replies whose `SERVER` header identifies **Moonraker** (the marker does not need to be echoed back).
- Reads **API port** from the SSDP `LOCATION` URL when present; otherwise defaults to **7125**.
- Optionally reads **X-MACHINE-NAME** and **X-MACHINE-ID** for the picker list.
- Confirms the host with a quick **`/server/info`** check before treating it as upload-ready.

### Adding the printer in the UI

1. Open **Printer settings** (or the physical printer connection dialog) and choose host type **Moonraker**.
2. Use **Discover** (SSDP scan). Pick the entry (e.g. **FibreSeeker 3** with IP and port).
3. Orca fills **`print_host`** as `http://<ip>:<port>` and sets **`print_host_webui`** to a sensible default (stripping redundant `:7125` when applicable).
4. Save the machine preset and verify upload/print through Moonraker as for any Klipper printer.

Ensure the PC and printer share a subnet and that local firewalls allow **UDP 1900** outbound/inbound for discovery if scans return empty.

---

## Printer and profile setup

### Vendor bundle

Shipped profiles live under **`resources/profiles/FibreSeeker3.json`** and **`resources/profiles/FibreSeeker3/`**.

Typical workflow:

1. **First-run wizard** — select vendor **FibreSeeker**, model **SK3**, variant **Plastic+CF** or **Plastic**.
2. **Machine preset** — Klipper flavor, printable area/height, **machine start G-code** that homes, heats **T1** plastic, runs brush/clean macros, and leaves the job in **PLASTIC** tool context. Composite presets enable **`fs_fiber_enabled`** and ship calibrated **`fs_*`** defaults (restart feed, tail length, tool wrap, deposit feed, matrix ratio, etc.).
3. **Process preset** — choose a fiber mode ladder (**plastic only**, **reinforced/walls**, **fortified/solid**). Layer height **0.12 mm** is paired with **`fs_fiber_z_step` 0.24 mm** so macro-layer scheduling aligns with a two-plastic-layers-per-composite-bead workflow.
4. **Filaments** — FibreSeek plastic filaments for **T1**; tow materials (**X-CCF**, **X-CGF**, etc.) are printer-specific and largely **declarative** (composite hotend temperature is driven by machine macros / optional `fs_t0_temp`, not tow preset temps alone).

### Where settings appear in Orca

| Tab | Group | Examples |
|-----|--------|----------|
| **Printer → Continuous Fiber** | Machine capability & calibration | `fs_fiber_enabled`, nozzle diameter, tool wrap, restart/tail/prime/retract, deposit feed, reserve, bond overlap |
| **Process → Strength → Continuous Fiber** | Reinforcement policy | `fs_fiber_mode`, coverage, infill pattern, wall inset, angles, enforce, rectilinear keys, `fs_fiber_rate`, `fs_matrix_ratio`, speed zones |

Expert/comExpert gating applies to many of these keys so casual FFF users never see them.

### G-code expectations

- **Plastic body** — standard Orca FFF on **T1** / **E**.
- **Fiber block** — appended per layer after FFF; **T0** windows with **U/V/P** dialect documented in `continuous_fiber_gcode.md`.
- **Machine macros** — start G-code, tool-change comments, and `fs_t0_wrap` must stay consistent; offsets and cleaning are not generated by the strand emitter.

When **`fs_fiber_enabled`** is off or the variant is **Plastic**, slicing behavior matches stock Orca for that model aside from profile-specific start G-code.

---

## End-to-end slice flow

```text
Slice layer (FFF geometry)
        │
        ▼
[If scheduled] harvest external perimeter rings
        │
        ▼
FiberStrandPlanner → FiberStrand[] (finalize: body + tail split)
        │
        ▼
[Optional] subtract_reserve_bands on perimeters/fills
        │
        ▼
Emit all FFF extrusions (T1 / E)
        │
        ▼
[If strands] T0 → emit_strand × N → T1
        │
        ▼
Next layer
```

---

## Related docs

- `docs/HLSD/continuous_fiber.md` — capability and slice path summary
- `docs/HLSD/continuous_fiber_gcode.md` — window shape and axis rules
- `docs/HLSD/continuous_fiber_preview.md` — preview and cooling-buffer constraints
- `docs/HLSD/fibreseeker3_profiles.md` — bundle layout and preset notes
