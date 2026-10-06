# FibreSeeker3 S-hook export acceptance run

This directory is the handoff for the owner follow-up of 2026-10-05. It contains
three **freshly sliced** exports of the real S-hook project, the manifest that
describes how they were produced, and the verifier output that measures them.
Nothing here is a renamed older file, and no check below is satisfied by a unit
test or by reading source.

Reproduce the verdict from this directory alone:

```bash
python tools/verify_fs_shook.py \
  --manifest docs/evidence/shook_acceptance/manifest.json \
  --json-out out/fs_shook/verification.json \
  --md-out   out/fs_shook/verification.md
```

## Status

```text
SOFTWARE_EXPORT_ACCEPTANCE: PASS
MACRO_CONTRACT:             UNVERIFIED   (conditional: see the macro note below)
PHYSICAL_PRINT_RESULT:      NOT_TESTED   (operator trials pending)
NEGATIVE_TESTS:             PASS 10/10
```

## What was measured

All numbers are recomputed from the emitted bytes by `tools/verify_fs_shook.py`.
Comments name phases but never measure them.

| variant | R | M | windows | post-cut deposition | dry release | cut..release-end | max preheat delta |
|---|---:|---:|---|---|---|---|---|
| `shook_A_park_wait` | 0 | 0 | 16 | 54.784 .. 54.810 mm | 0.000 mm | 54.8 mm | 0.049 s |
| `shook_B_forward_release` | 6.8 | 0 | 16 | 54.798 .. 54.811 mm | 6.795 .. 6.801 mm | 61.6 mm | 0.045 s |
| `shook_C_release_margin_1mm` | 6.8 | 1 | 16 | 55.800 .. 55.814 mm | 6.795 .. 6.801 mm | 62.6 mm | 0.045 s |

The startup purge is pinned numerically rather than relatively, so it gets its
own table:

| variant | cut X | deposition ends X | release ends X | Z |
|---|---:|---:|---:|---:|
| A | 35.20 | 90.00 | 90.00 | 0.24 |
| B | 35.20 | 90.00 | 96.80 | 0.24 |
| C | 34.20 | 90.00 | 96.80 | 0.24 |

Every mandatory check S01..S13 is evaluated and passes for all three files.
`verification.md` carries one row per fibre window and one row per physical
transition, with line numbers, endpoints, measured distances, the E ledger and
the station state. `verification.json` is the same data in machine form.

## Requirements, and where each is proved

| Requirement from the follow-up | Where it is proved |
|---|---|
| Every hotend wait executes in an established park, including startup | S07; the startup wait is hoisted into a station bracket by `Fiber::park_initial_hotend_wait` |
| Forward dry release before lift, 6.8 mm in B/C including the purge | S03, S04; the release sits between `V-1` and `M1002` at the strand Z |
| Real 15 s lookahead in both directions, clamped to the activation | S09; `Fiber::apply_preheat_schedule_pass` re-places each marked preheat over the whole-file activation, splitting a motion block when the target falls inside one |
| Outgoing head stays active through its clean; standby after the clean, before the incoming wait | S08 |
| Withdrawal stays pending through travel and is recovered at the next deposition start | S10; one shared ledger, amount matched per recovery |
| C cuts 1 mm earlier and deposits 55.8 mm post-cut | S02, S12; the margin ADDS to the tail rather than shortening it |
| Purge corridor validated through X96.8, U55 unchanged | S13 corridor branch; S11 for the U55 reload |
| Release footprint supported by material already deposited in that layer | S13, reconstructed independently from emitted coordinates |
| A/B/C differ only in R and M; B and C share the seam | S12, compared as polylines rather than line-for-line |

## The macro contract is UNVERIFIED

`MOVE_TO_BRUSH_STATION`, `MOVE_OUT_BRUSH_STATION` and `CLEAN_NOZZLE` are vendor
macros whose definitions are not in this repository. The implementation is
written against the adapter contract recorded in `manifest.json` under
`macro_contract_detail`: the entry macro parks, `M104` is nonblocking and
addresses the named heater, `M109` blocks. Software acceptance is therefore
**conditional** on that contract. No hardware behaviour is claimed, and a
successful export proves nothing about whether the physical tail clears the
nozzle.

## Negative tests

`tools/verify_fs_shook_negatives.py` mutates a passing export ten ways and
requires the **specific** semantic error ID, with each mutant's own manifest
hash refreshed so a hash mismatch cannot mask a broken detector. Hash-mismatch
detection is its own case rather than a side effect of another.

| mutation | required error ID |
|---|---|
| all 16 releases deleted (reconstructed 001021 profile) | `FS_RELEASE_NOT_EXECUTED` |
| one release move deleted, comments kept | `FS_RELEASE_NOT_EXECUTED` |
| E recovery moved before its destination travel | `FS_E_RECOVERY_LOCATION` |
| outgoing standby moved below the incoming wait | `FS_STANDBY_AFTER_WAIT` |
| outgoing standby moved above its own clean | `FS_COOL_BEFORE_CLEAN` |
| one `M109` moved outside its station bracket | `FS_WAIT_OUTSIDE_STATION` |
| one preheat command deleted, comment kept | `FS_PREHEAT_MISSING` |
| C's cut reverted to B's position | `FS_TAIL_DISTANCE_MISMATCH` |
| release translated 40 mm off deposited material | `FS_RELEASE_UNSUPPORTED` |
| content changed with a stale manifest hash | `FS_HASH_MISMATCH` |

The reviewed baseline `S Hook_20261005-001021.gcode` is **not present** in this
repository or on this machine. Case 1 reconstructs the fault profile that review
documents (zero release in all 16 windows) from the passing B export rather than
processing the original bytes, and its own record says so. That is a limitation
of this environment, not a claim about the original file.

## Still outstanding, and not cleared by this run

- **Physical tail clearance and print quality are unmeasured.** Only operator
  trials can close them.
- **Tight-turn behaviour** (`fs_fiber_min_radius`, policy `keep`) is reported by
  the exporter and unchanged by this work.
- **Rocket motion-setting differences** are untouched.
- **The missing-intermediate-solid-layer investigation is still open.** S01
  counts deposition layers from extrusion and reports 32, but a layer count does
  not clear a partially missing-infill defect.
- `MACRO_CONTRACT` stays UNVERIFIED until the vendor macro definitions can be
  inspected.
