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

Regenerate the whole set (slice, manifest, publish, verify) from the repo root:

```bash
python docs/evidence/shook_acceptance/run_export.py
python docs/evidence/shook_acceptance/make_manifest.py --evidence-dir docs/evidence/shook_acceptance
python tools/verify_fs_shook.py --manifest docs/evidence/shook_acceptance/manifest.json \
  --json-out docs/evidence/shook_acceptance/verification.json \
  --md-out   docs/evidence/shook_acceptance/verification.md
python tools/verify_fs_shook_negatives.py \
  --manifest docs/evidence/shook_acceptance/manifest.json \
  --workdir  docs/evidence/shook_acceptance/negatives
```

## Status

```text
SOFTWARE_EXPORT_ACCEPTANCE: PASS
MACRO_CONTRACT:             UNVERIFIED   (conditional: see the macro note below)
PHYSICAL_PRINT_RESULT:      NOT_TESTED   (operator trials pending)
NEGATIVE_TESTS:             PASS 13/13
```

## What was measured

All numbers are recomputed from the emitted bytes by `tools/verify_fs_shook.py`.
Comments name phases but never measure them.

| variant | R | M | windows | post-cut deposition | dry release | cut..release-end | max preheat delta | max footprint escape |
|---|---:|---:|---|---|---|---|---|---|
| `shook_A_park_wait` | 0 | 0 | 16 | 54.784 .. 54.810 mm | 0.000 mm | 54.8 mm | 0.049 s | n/a (no release) |
| `shook_B_forward_release` | 6.8 | 0 | 16 | 54.798 .. 54.811 mm | 6.795 .. 6.801 mm | 61.6 mm | 0.045 s | 0.0057 mm |
| `shook_C_release_margin_1mm` | 6.8 | 1 | 16 | 55.800 .. 55.814 mm | 6.795 .. 6.801 mm | 62.6 mm | 0.045 s | 0.0057 mm |

`max footprint escape` is the worst distance the buffered release strip (half the
actual 0.8 mm composite bead width) reaches past material already deposited in
the same physical layer. Negative or below the 0.02 mm tolerance means the whole
strip, not just the centreline, is supported.

The startup purge is pinned numerically rather than relatively, so it gets its
own table:

| variant | cut X | deposition ends at X | release ends at X | release Z |
|---|---:|---:|---:|---:|
| A | 35.20 | 90.00 | 90.00 | 0.24 |
| B | 35.20 | 90.00 | 96.80 | 0.24 |
| C | 34.20 | 90.00 | 96.80 | 0.24 |

## Checks

S01 through S13 are each evaluated and pass on all three files. The per-window
and per-transition tables in `verification.md` carry the line numbers, endpoint
coordinates, measured lengths and error IDs behind every verdict.

| ID | What it now proves |
|---|---|
| S01 | 32 model deposition layers Z0.24..Z3.96, 16 windows (1 purge + 15 model), 16 cuts, 16 closes, counted from extrusion |
| S02 | post-cut deposition per window against T (+M for C) |
| S03 | release actually executed, after V-1 and before M1002 and any lift |
| S04 | release uses F600, holds strand Z, carries no E/U/V |
| S05 | no U drive from cut through close |
| S06 | final deposition, V-1, release, M1002, departure V-4, lift, station entry |
| S07 | **every required hotend wait exists AND is parked** |
| S08 | outgoing head holds its active target; standby after cleaning, before the incoming wait |
| S09 | both transition directions meet the 15 s lead within 0.10 s |
| S10 | pending withdrawal survives travel and is recovered once at the next deposition start |
| S11 | **body and tail V payout match the configured formula; stationary V amounts are the contracted ones** |
| S12 | A/B/C differ only as listed; B/C deposited path and seam agree; C cuts 1 mm earlier |
| S13 | **the full buffered release footprint, not its centreline, lies inside material deposited earlier in the same layer** |

## Detector hardening (2026-10-06)

Three holes the owner measured in the previous revision of the verifier are
closed, and each is pinned by a negative test that failed against the old code:

| Hole | Old behaviour | Now | Negative test |
|---|---|---|---|
| S07 presence | deleting all 33 `M109` passed, reporting "all 0 hotend waits are parked" | S07 requires one wait per window per head plus the startup wait, and fails with `FS_WAIT_MISSING` | `t10_all_hotend_waits_deleted` |
| S11 payout | the V ledger was noted, never judged; a 30x tail payout passed | body rate and tail payout are judged against `fs_fiber_rate`, `fs_matrix_ratio` and `fs_tail_v_factor` from the effective config; stationary V is classified by value, not by comment | `t11_tail_payout_times_30` |
| S13 containment | centreline-to-centreline distance, so a release on a bead edge passed | the swept strip is sampled along and across and judged against each supporting segment's own half-width | `t12_release_on_bead_edge` |

The payout formula the verifier enforces is
`tail V = tail path x fs_tail_v_factor x fs_fiber_rate x fs_matrix_ratio`, with
the startup purge exempt at factor 1.0 because it is sacrificial. Measured
deviation on the shipped exports is at most 0.0093 mm against a 0.05 mm
tolerance, and the body rate is within 0.5 percent against a 2 percent
tolerance, so the gate is tight without being brittle.

## Export-script root

`run_export.py` and `make_manifest.py` previously derived the repository root by
counting `dirname()` calls. The count that is right for `out/fs_shook/` is wrong
for this directory, which sits one level deeper, so the published copies aborted
with `FATAL: built binary missing` while pointing at `docs/build/...`. Both now
search upward for the directory containing `build/src/Release` and
`resources/profiles`, which is correct wherever the script is placed.

## Macro contract

`MOVE_TO_BRUSH_STATION`, `CLEAN_NOZZLE` and `MOVE_OUT_BRUSH_STATION` are vendor
machine-start macros whose definitions are not in this repository. The
nonblocking-preheat and station-park requirements are therefore implemented
against the documented adapter contract recorded in `manifest.json`: the entry
macro establishes a park, `CLEAN_NOZZLE` is synchronous, `M104` is nonblocking
and addresses the heater zone named by its `T` word, and `M109` blocks on that
zone. **Software acceptance is conditional on that contract. No hardware
verification is claimed, and a successful export does not prove the physical tail
cleared the nozzle.**

## Still outstanding

- Physical tail clearance and print quality: unmeasured until operator trials.
- Tight-turn behaviour and Rocket motion-setting differences.
- The separate missing-intermediate-solid-layer investigation. A 32-layer count
  (S01) does not clear a partially missing infill defect.
