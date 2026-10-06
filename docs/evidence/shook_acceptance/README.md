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
python tests/fibreseeker/test_fs_departure_rule.py    # departure rule, both directions
python tests/fibreseeker/test_fs_s06_close_kinds.py   # S06 close kinds, 10 cases
python tests/fibreseeker/fs_validate_fixtures.py      # retained dialect fixtures
python tests/fibreseeker/fs_tail_release_analyzer.py  # retained ordering fixtures
```

## Status

```text
SOFTWARE_EXPORT_ACCEPTANCE: PASS
MACRO_CONTRACT:             UNVERIFIED   (conditional: see the macro note below)
PHYSICAL_PRINT_RESULT:      NOT_TESTED   (operator trials pending)
NEGATIVE_TESTS:             PASS 17/17
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
| S07 | **every managed activation is served by its own parked wait for the incoming head at its active temperature** |
| S08 | outgoing head holds its active target; standby after cleaning, before the incoming wait |
| S09 | both transition directions meet the 15 s lead within 0.10 s |
| S10 | pending withdrawal survives travel and is recovered once at the next deposition start |
| S11 | **body and tail V payout match the configured formula; stationary V amounts are the contracted ones; exactly one departure withdrawal per actual T0 departure** |
| S12 | A/B/C differ only as listed; B/C deposited path and seam agree; C cuts 1 mm earlier |
| S13 | **the full buffered release footprint, not its centreline, lies inside material deposited earlier in the same layer** |

## Detector hardening (2026-10-06)

Three holes the owner measured in the previous revision of the verifier are
closed, and each is pinned by a negative test that failed against the old code:

| Hole | Old behaviour | Now | Negative test |
|---|---|---|---|
| S07 presence | deleting all 33 `M109` passed, reporting "all 0 hotend waits are parked" | S07 requires a wait for every managed activation and fails with `FS_WAIT_MISSING` | `t10_all_hotend_waits_deleted` |
| S11 payout | the V ledger was noted, never judged; a 30x tail payout passed | body rate and tail payout are judged against `fs_fiber_rate`, `fs_matrix_ratio` and `fs_tail_v_factor` from the effective config; stationary V is classified by value, not by comment | `t11_tail_payout_times_30` |
| S13 containment | centreline-to-centreline distance, so a release on a bead edge passed | the swept strip is sampled along and across and judged against each supporting segment's own half-width | `t12_release_on_bead_edge` |

## Detector hardening, second round (2026-10-06)

The owner re-tested the hardened verifier and found two more holes that still
produced a full acceptance PASS. Both were real, both are closed, and both were
confirmed against the previous revision before being fixed. The owner's own
reproductions used complete three-file manifests with Python-captured return
codes; the single-file manifests in this section belong to the repository's
negative-test harness, which mutates one variant at a time.

| Hole | Old behaviour | Now | Negative test |
|---|---|---|---|
| S07 judged a file-wide tally | deleting the first model T0 activation's `M109` and duplicating it in the startup visit preserved the tally, so the activation ran with no temperature wait and S07 passed | obligations are derived per **activation**, not per file: 1 startup + 32 physical head changes = 33, each needing its own wait for the incoming head at its active temperature, served by the station visit that actually precedes it. Duplicate waits cannot discharge a later obligation | `t13_startup_wait_relocated` |
| S11 ledger stopped at `M1002` | the departure withdrawal is issued *after* the close, so it sat outside the judged range; deleting all 16 passed, S11 included | the ledger extends from `M1002` to the first lift and requires exactly one `V -fs_toolchange_retract_v` per **actual T0 departure**, and none for an inter-strand close that keeps T0 active | `t14_all_departure_withdrawals_deleted` |

Deriving obligations from physical head changes rather than from the window list
also fixes a latent over-strictness: several fibre windows inside one T0
activation need one wait, not one per window, because there is one physical
change into T0. The startup head is taken from the first `T` command in the file
rather than from the first recorded *change*, which is what left the vendor
startup `M109 S250 T1` with no obligation attached to it.

Two supporting defects surfaced while writing these tests:

- `make_manifest.py` stringified list-valued presets, so `nozzle_temperature`
  reached the manifest as the literal text `['250']`. T1's active temperature was
  therefore unresolvable and S07 could not judge the plastic head at all. Values
  now keep their list structure, and the filament preset is merged into the
  effective config because that is where the plastic nozzle temperature lives.
- The bash harness on this machine masks process exit codes to zero, so every
  `EXIT=` reading taken through it was meaningless. Exit codes are now captured
  inside Python; the verifier's own exit logic was correct.

The payout formula the verifier enforces is
`tail V = tail path x fs_tail_v_factor x fs_fiber_rate x fs_matrix_ratio`, with
the startup purge exempt at factor 1.0 because it is sacrificial. Measured
deviation on the shipped exports is at most 0.0093 mm against a 0.05 mm
tolerance, and the body rate is within 0.5 percent against a 2 percent
tolerance, so the gate is tight without being brittle.

## Detector hardening, third round (2026-10-06)

The owner found that the departure rule itself was wrong, not just its coverage.

S11 decided whether a window owed a departure withdrawal from the head selected
at the close:

    head = head_at(lines, w.close_idx)
    expect = (head == 0)

Being on T0 when a window closes does not establish that T0 is departing. An
inter-strand close, where the next strand is still T0, also happens while T0 is
selected, so the rule demanded a withdrawal there too. The rule now looks
forward, in `departure_obligations()`:

  - closes on a head other than T0: T0 is not the outgoing head, nothing owed
  - no physical head change at all after the close: T0 never leaves
  - the next fibre window opens before the next head change: T0 stays active
    between strands, nothing owed
  - otherwise the head change lands between windows, so T0 genuinely hands over
    and owes exactly one withdrawal

A missing withdrawal at a real departure reports `FS_DEPART_WITHDRAWAL_MISSING`;
a withdrawal where none is owed now reports its own code,
`FS_DEPART_WITHDRAWAL_SPURIOUS`, rather than sharing the missing code, so the
two directions of the same rule cannot be confused in a report.

Two further defects surfaced while building the fixture:

  - The post-close stationary-V scan was unbounded, so an inter-strand close
    scanned forward *through* the following window and collected that window's
    own recovery and prime stationary V, and even its later departure
    withdrawal, as if they belonged to the close. The scan now stops at the next
    window opening.
  - The scan only collected once it had found a lift. An inter-strand close has
    no lift, so a spurious withdrawal injected there was invisible. Collection
    is now independent of lift discovery.

Pinned by `tests/fibreseeker/test_fs_departure_rule.py` with the two-strand,
single-activation fixture `tests/fibreseeker/fixtures_departure/`
`good_interstrand_then_departure.gcode`: both closes are on T0, the first owes
nothing, the second owes exactly one `V -4.000`, and the test mutates the
fixture in both directions (inject a withdrawal at the inter-strand close,
delete the withdrawal at the real departure) and requires both to be detected.

On the shipped S-hook exports the corrected rule agrees with the old one for all
48 windows across A/B/C, because every window there really does end in a
departure. The correction is about the rule being right, not about these
results changing.

## Detector hardening, fourth round (2026-10-06)

The owner confirmed the S11 departure predicate and asked for the same
distinction one level up: **S06 required a lift and a station visit for every
fibre window**, which is only true of a physical tool departure.

S06 now consumes the one classification S11 already computes.
`departure_obligations()` became `classify_closes()` and returns a KIND, not a
boolean, because three situations exist and two of them owe no withdrawal:

| kind | how recognised | S06 requires |
|---|---|---|
| `departure` | head change lands between windows | M1002 -> withdrawal -> clearance lift -> station entry |
| `interstrand` | next window opens before the next head change | release + closure before repositioning; restart hop keeps its clearance; **no** station visit or departure lift owed |
| `terminal` | no head change at all after the close | release + closure before shutdown; no incoming-head transition invented |

Collapsing `terminal` into "expected is False, so inter-strand" would have been
the obvious shortcut and would have been wrong: both owe no withdrawal, but only
one is followed by another strand. The test proves the distinction by appending a
head change after the terminal window and requiring S06 to switch to the
departure rule and fail.

Every strand, whatever its kind, must complete final deposition -> V-1 -> the
configured release at printing Z -> M1002 before anything lifts. A Z move inside
that span is reported, and when a release was configured the message says the
release was truncated, because the release scan stops at the first Z move.

**A lift is now a displacement, not a Z word.** Re-issuing the height the
carriage already occupies moves nothing and clears nothing, so it is recorded as
`lift_z_repeat` and does not satisfy the clearance requirement. The required rise
is 0.60 mm, from `FiberEmitter.hpp` `lift_z_mm`, overridable by an effective
config key `fs_toolchange_lift_z`.

Pinned by `tests/fibreseeker/test_fs_s06_close_kinds.py` over the fixture
`tests/fibreseeker/fixtures_s06/s06_close_kinds.gcode`, which contains one
inter-strand close with no Z move, one inter-strand close with a valid 0.60 mm
restart hop, one real departure, and one terminal close. The test drives the
fixture through the verifier's normal CLI path and asserts **S06's own status**
per case, so an unrelated fixture failure cannot mask a broken S06 and a vacuous
S06 cannot hide behind an otherwise-green file. Ten cases: the four kinds pass
unmutated, and removing the departure lift, replacing it with an unchanged Z,
removing the station entry, lifting before the release completes, lifting before
M1002, weakening the inter-strand hop, and converting the terminal close into a
departure each fail S06. A spurious withdrawal at an inter-strand close is
asserted against S11, which owns that rule.

Two shipped-export negatives cover the same ground on real data:
`t15_all_departure_lifts_deleted` and `t16_lift_before_window_close`, both
requiring `FS_WINDOW_SE`UENCE` from S06. Negative tests are now 17.

On the shipped S-hook exports the new S06 agrees with the old one for all 48
windows, because every window there really does depart: the report's new `close`
column reads `departure/rise+0.60` for all sixteen windows of each variant. The
correction is about the rule being right, not about these results changing.

### Provenance fix found while re-running

`make_manifest.py` passed the git stdout through `subprocess.list2cmdline()`,
which given a string iterates its CHARACTERS. The recorded `source_revision` was
therefore `3 4 d 1 c c ...` rather than a checkoutable SHA. The revision now
records `34d1cc78a6efc74486bf5b15186bb11ea375084c`. Provenance that cannot be
checked out is not provenance.

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
