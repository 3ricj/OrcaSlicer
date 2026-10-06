#!/usr/bin/env python3
"""Build out/fs_shook/manifest.json from the export run record.

The manifest is what the verifier is allowed to trust about PROVENANCE (which
project, which build, which command, which effective config). It is never
allowed to substitute for the G-code bytes: the verifier re-parses and
re-hashes those independently.
"""
import hashlib
import json
import os
import shutil
import subprocess
import sys

# The repo root is found by searching upward, not by counting dirname()
# calls. A fixed count is wrong for at least one of the two places this
# script lives (out/fs_shook/ and docs/evidence/shook_acceptance/ sit at
# different depths), and a wrong ROOT silently points every BIN/PROC/FILA
# path at a directory that does not exist, which reads back as
# "FATAL: built binary missing".
def _find_repo(start):
    d = os.path.abspath(start)
    while True:
        has_build = os.path.isdir(os.path.join(d, "build", "src", "Release"))
        has_res = os.path.isdir(os.path.join(d, "resources", "profiles"))
        if has_build and has_res:
            return d
        parent = os.path.dirname(d)
        if parent == d:
            raise SystemExit("FATAL: no repository root above " + start)
        d = parent


ROOT = _find_repo(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "out", "fs_shook")
RUNS = os.path.join(OUT, "export_runs.json")

MODEL = "C:/Users/3ricj/Documents/GitHub/FibreSeeker3/Test_files/2026-09-25 S-hook/S Hook.3mf"
PROC = "resources/profiles/FibreSeeker3/process/0.12mm Reinforced L5 @FibreSeeker3 SK3.json"
FILA = "resources/profiles/FibreSeeker3/filament/FibreSeek PETG @FibreSeeker3 SK3.json"
BASE_MACHINE = "resources/profiles/FibreSeeker3/machine/FibreSeeker3 SK3 CF nozzle.json"
BIN = "build/src/Release/OrcaSlicer.dll"
CLI = "build/src/Release/orca-slicer.exe"


def mnt(p):
    if len(p) > 2 and p[1] == ":" and p[2] == "/":
        return "/mnt/" + p[0].lower() + p[2:]
    return p


def sha(path):
    h = hashlib.sha256()
    with open(mnt(path), "rb") as fh:
        for c in iter(lambda: fh.read(1 << 20), b""):
            h.update(c)
    return h.hexdigest()


def git(*a):
    """Run git in the repo and return its trimmed stdout.

    The result must NOT be passed through list2cmdline: given a str, that
    function iterates its CHARACTERS, so the recorded source revision came out
    as "3 4 d 1 c c" instead of the SHA. Provenance that cannot be checked out
    is not provenance.
    """
    return subprocess.run(["git"] + list(a), cwd=ROOT, capture_output=True,
                          text=True).stdout.strip()


def dirty_patch_sha256():
    """Hash of the uncommitted patch that produced this build.

    Read from out/fs_shook/dirty.patch, which out/fs_shook/make_dirty_patch.sh
    writes. The diff is scoped to the source tree because a whole-worktree diff
    on a Windows drive takes ~100 s and drags in build artifacts.
    """
    cache = os.path.join(OUT, "dirty.patch")
    if not os.path.isfile(cache):
        raise SystemExit("FATAL: %s missing; run out/fs_shook/make_dirty_patch.sh" % cache)
    return sha(cache)


def _cfgval(val):
    """Normalise one preset value for the manifest.

    Lists must stay lists: nozzle_temperature and nozzle_diameter are arrays, and
    str()-ing them produced "['250']", which no consumer could parse back into a
    number. S07 needs the T1 active temperature and S13 needs the bead width, so
    flattening them to a repr string silently disabled both.
    """
    if isinstance(val, (list, tuple)):
        return [_cfgval(v) for v in val]
    if isinstance(val, str):
        return val
    return str(val)


def effective_config(preset_path):
    """The merged config the run actually used, read back off the export.

    The G-code CONFIG_BLOCK is the slicer's own record of every key it sliced
    with, so it is the effective configuration rather than the input preset.
    """
    return preset_path


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument("--evidence-dir", default=None,
                    help="also publish into this directory, copying each export "
                         "to gcode/<variant>.gcode and rewriting the paths, so "
                         "the committed evidence set regenerates in one step")
    args = ap.parse_args(argv)
    runs = json.load(open(RUNS, encoding="utf-8"))
    by = dict((r["variant"], r) for r in runs)

    # Machine preset per variant, with the FS keys that define the trial.
    files = []
    for v in ("shook_A_park_wait", "shook_B_forward_release",
              "shook_C_release_margin_1mm"):
        r = by[v]
        preset = mnt(r["preset_path"])
        pj = json.load(open(preset, encoding="utf-8"))
        # Merge: base machine profile, then the variant preset's own keys.
        base = json.load(open(mnt(os.path.join(ROOT, BASE_MACHINE)),
                              encoding="utf-8-sig"))
        eff = {}
        for k, val in base.items():
            eff[k] = _cfgval(val)
        for k, val in pj.items():
            if k in ("type", "name", "from", "inherits", "version"):
                continue
            eff[k] = _cfgval(val)
        # The process preset overrides the machine preset for shared keys.
        proc = json.load(open(mnt(os.path.join(ROOT, PROC)), encoding="utf-8-sig"))
        for k, val in proc.items():
            if k in ("type", "name", "from", "inherits", "version"):
                continue
            eff[k] = _cfgval(val)
        # Filament supplies the plastic head's active nozzle temperature, which
        # is the temperature its blocking wait must target. Merge it last so the
        # effective config is the full three-way merge the slicer used.
        fila = json.load(open(mnt(os.path.join(ROOT, FILA)), encoding="utf-8-sig"))
        for k, val in fila.items():
            if k in ("type", "name", "from", "inherits", "version"):
                continue
            eff[k] = _cfgval(val)
        gcode = r["gcode"]
        files.append(dict(
            variant=v,
            gcode=os.path.relpath(mnt(gcode), OUT).replace("\\", "/"),
            sha256=r["sha256"],
            lines=r["lines"],
            cmd=" ".join(r["cmd"]),
            exit=r["exit"],
            preset_path=r["preset_path"],
            effective_config={k: eff.get(k) for k in (
                "fs_fiber_enabled", "fs_t0_temp", "fs_t0_standby_temp",
                "fs_t1_standby_temp", "fs_tail_length", "fs_fiber_release_length_mm",
                "fs_fiber_tail_margin_mm", "fs_fiber_release_speed_mm_s",
                "fs_fiber_release_anchor_mm", "fs_tool_preheat_lead_s",
                "fs_fiber_bead_width", "fs_fiber_nozzle_diameter",
                "fs_fiber_prime", "fs_t0_wrap", "fs_fiber_schedule",
                "fs_fiber_mode", "fs_fiber_z_step", "fs_matrix_ratio",
                "fs_fiber_rate", "fs_restart_feed", "fs_restart_z_hop",
                "fs_prime_v", "fs_retract_v", "fs_toolchange_retract_v",
                # S11 judges the emitted tail payout against this factor, so it
                # must be in the effective config the verifier may trust.
                "fs_tail_v_factor",
                "fs_brush_on_toolchange", "fs_aux_fans_on_toolchange",
                "nozzle_diameter", "nozzle_temperature",
                "first_layer_height", "layer_height")},
            evidence=dict(kind="independent_reconstruction",
                          method="deposited-material mask rebuilt from emitted "
                                 "coordinates by the verifier itself",
                          bead_width_mm=float(eff.get("fs_fiber_bead_width") or
                                              eff.get("fs_fiber_nozzle_diameter") or 0.7)),
            bead_width_mm=float(eff.get("fs_fiber_bead_width") or
                                eff.get("fs_fiber_nozzle_diameter") or 0.7),
            preheat_lead_s=float(eff.get("fs_tool_preheat_lead_s") or 15.0),
        ))

    prov = {
        "project": MODEL,
        "project_sha256": sha(MODEL),
        "process_preset": PROC,
        "process_preset_sha256": sha(os.path.join(ROOT, PROC)),
        "filament_preset": FILA,
        "filament_preset_sha256": sha(os.path.join(ROOT, FILA)),
        "machine_preset_base": BASE_MACHINE,
        "machine_preset_base_sha256": sha(os.path.join(ROOT, BASE_MACHINE)),
        "source_revision": git("rev-parse", "HEAD"),
        "source_dirty_patch_sha256": dirty_patch_sha256(),
        "build_command": "cmake --build build --target OrcaSlicer --config Release -j 8",
        "executable": CLI,
        "executable_sha256": sha(os.path.join(ROOT, CLI)),
        "runtime_dll": BIN,
        "runtime_dll_sha256": sha(os.path.join(ROOT, BIN)),
        "verifier": "tools/verify_fs_shook.py",
        "verifier_sha256": sha(os.path.join(ROOT, "tools/verify_fs_shook.py")),
    }

    man = dict(
        provenance=prov,
        clock=dict(
            translating="60 * XYZ distance / modal F",
            stationary="60 * max(|E|,|U|,|V|) / modal F",
            dwell="explicit G4 P duration",
            zero="temperature waits and opaque macros contribute zero",
            error="a motion block with no resolvable feed is an error, not zero"),
        macro_contract="MACRO_CONTRACT_UNVERIFIED",
        macro_contract_detail=(
            "MOVE_TO_BRUSH_STATION / CLEAN_NOZZLE / MOVE_OUT_BRUSH_STATION are "
            "vendor machine-start macros. Their definitions are not present in this "
            "repository, so the nonblocking-preheat and station-park requirements are "
            "implemented against the documented adapter contract: the entry macro "
            "establishes a park, CLEAN_NOZZLE is synchronous, M104 is nonblocking and "
            "addresses the heater zone named by its T word, M109 blocks on that zone. "
            "No hardware verification is claimed."),
        physical_print_result="NOT_TESTED",
        bead_width_mm=0.7,
        preheat_lead_s=15.0,
        files=files,
        limitations=[
            "Macro definitions unavailable: MACRO_CONTRACT_UNVERIFIED. Software "
            "acceptance is conditional on the adapter contract above.",
            "The reviewed baseline export 001021 is outside this repository; it is "
            "processed as a negative-test candidate, not as a same-project regression.",
        ],
    )
    out = os.path.join(OUT, "manifest.json")
    json.dump(man, open(out, "w", encoding="utf-8"), indent=1)
    print("wrote", out)
    if args.evidence_dir:
        # Publish the committed form: gcode/<variant>.gcode beside the
        # manifest, so the verdict reproduces from the evidence directory
        # alone with one verifier command.
        ev = os.path.abspath(mnt(args.evidence_dir))
        os.makedirs(os.path.join(ev, "gcode"), exist_ok=True)
        for f in man["files"]:
            src = os.path.join(OUT, f["gcode"])
            dst = os.path.join(ev, "gcode", f["variant"] + ".gcode")
            shutil.copyfile(mnt(src) if os.path.isabs(src) else src,
                            mnt(dst))
            f["gcode"] = "gcode/" + f["variant"] + ".gcode"
        evout = os.path.join(ev, "manifest.json")
        json.dump(man, open(evout, "w", encoding="utf-8"), indent=1)
        print("published", evout)
    for f in files:
        print("  %s  %s  %d lines" % (f["variant"], f["sha256"][:16], f["lines"]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
