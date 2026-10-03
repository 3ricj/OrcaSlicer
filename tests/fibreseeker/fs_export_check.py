#!/usr/bin/env python3
"""fs_export_check.py - FibreSeeker3 export slice (P2 plastic-only / P5 composite gates).

End-to-end check: headless OrcaSlicer slices the 20 mm cube with the FS3
presets (tests/fibreseeker/presets/*.json) through the real exporter
path, and the exported g-code is then re-parsed and validated by the
INDEPENDENT validator (fs_gcode_validator.py). The slice passes only when:

  1. the orca-slicer CLI exits with code 0,
  2. an executable block and a g-code file were produced,
  3. the machine start/end g-code blocks survived export verbatim
     (T1 activation, heaters, brush-station macros, fan P-lines),
  4. the validator reports 0 ERRORs,
  5. in --fiber mode (P5): the composite fiber markers survived export too
     (T0/T1 wrap, window open/close, restart/prime/retract/cut lines).

Warnings are printed but do not fail the gate; the reference PETG_8m
plastic-only file itself carries warnings (see p1_validation.md).

usage: fs_export_check.py <orca-slicer binary>
       [--keep-dir DIR]  keep the exported g-code in DIR on success too

Exit codes: 0 = pass, 3 = SKIP (prerequisite missing), else = fail.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import fs_gcode_validator  # noqa: E402  (independent validator, P1)

PRESETS = os.path.join(HERE, "presets")
MACHINE = os.path.join(PRESETS, "fs3_t1_plastic_machine.json")
PROCESS = os.path.join(PRESETS, "fs3_t1_plastic_process.json")
FILAMENT = os.path.join(PRESETS, "fs3_t1_plastic_filament.json")
MODEL = os.path.join(HERE, "data", "fs_20mm_cube.stl")

# Markers that must survive export verbatim (start block, tool activation, end block).
REQUIRED_MARKERS = [
    "SET_PRESSURE_ADVANCE EXTRUDER=extruder1 ADVANCE=0.04",
    "M104 S250 T1",
    "M190 S75",
    "M191 S0",
    "M109 S250 T1",
    "T1 ; switch extruder type to:PLASTIC",
    "MOVE_TO_BRUSH_STATION",
    "CLEAN_NOZZLE",
    "MOVE_OUT_BRUSH_STATION",
    "M104 S0 T1",
    "M141 S0",
    "M191 S0",
]

# Additional markers required in --fiber mode (P5 composite export): the
# emitter lifecycle and the tool-wrap must both be present in the output.
FIBER_MARKERS = [
    "T0 ; switch extruder type to:FIBER",
    "M1001 L",
    "; Extrude restart",
    "; Start to cut",
    "M2800",
    "M400",
    ";CUT DISTANCE 54.8",
    "; Cutting completed.",
    "; Retract",
    "M1002",
    "T1 ; switch extruder type to:PLASTIC",
]

SLICE_TIMEOUT_S = 600


def fail(msg, log_tail=""):
    print("FAIL: %s" % msg)
    if log_tail:
        print("---- orca-slicer log tail ----")
        print("\n".join(log_tail.splitlines()[-30:]))
        print("------------------------------")
    return 1


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("binary")
    ap.add_argument("--keep-dir", default=None)
    ap.add_argument("--machine", default=MACHINE, help="machine preset json")
    ap.add_argument("--fiber", action="store_true",
                    help="composite fiber mode: require fiber lifecycle markers")
    args = ap.parse_args(argv)

    # Preconditions -> SKIP(3), matching the cli/ suite convention (skip 77).
    if not os.path.isfile(args.binary):
        print("SKIP: orca-slicer binary not found: %s" % args.binary)
        return 3
    for path in (args.machine, PROCESS, FILAMENT, MODEL):
        if not os.path.isfile(path):
            print("SKIP: missing input %s" % path)
            return 3
    # The exporter needs resources/ next to the binary (or via --resources).
    res = os.path.join(os.path.dirname(os.path.abspath(args.binary)), "resources")
    if not os.path.isdir(res):
        print("SKIP: resources/ not found next to binary (%s)" % res)
        return 3

    work = tempfile.mkdtemp(prefix="fs-export-")
    try:
        datadir = os.path.join(work, "datadir")
        outdir = os.path.join(work, "out")
        os.makedirs(datadir)
        os.makedirs(outdir)
        load = ";".join([args.machine, PROCESS])
        cmd = [args.binary, "--datadir", datadir, "--load-settings", load,
               "--load-filaments", FILAMENT, "--slice", "0",
               "--outputdir", outdir, MODEL]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True,
                                  errors="replace", timeout=SLICE_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            return fail("slice timed out after %d s" % SLICE_TIMEOUT_S)
        log = (proc.stdout or "") + "\n" + (proc.stderr or "")
        if proc.returncode != 0:
            return fail("orca-slicer exited %d" % proc.returncode, log)

        gcodes = [f for f in os.listdir(outdir) if f.endswith(".gcode")]
        if not gcodes:
            return fail("no g-code exported into %s" % outdir, log)
        gpath = os.path.join(outdir, gcodes[0])
        with open(gpath, "r", encoding="utf-8", errors="replace") as fh:
            text = fh.read()
        if "EXECUTABLE_BLOCK_START" not in text or "EXECUTABLE_BLOCK_END" not in text:
            return fail("exported file has no executable block: %s" % gpath)

        missing = [m for m in REQUIRED_MARKERS if m not in text]
        if missing:
            return fail("start/end g-code markers lost in export: %s" % ", ".join(missing))
        if args.fiber:
            missing = [m for m in FIBER_MARKERS if m not in text]
            if missing:
                return fail("fiber lifecycle markers lost in export: %s" % ", ".join(missing))

        allow, tol = fs_gcode_validator.load_allowlist(
            fs_gcode_validator.default_data_path("command_allowlist.json"))
        prof = fs_gcode_validator.load_profile(
            fs_gcode_validator.default_data_path("machine_profile.json"))
        findings = fs_gcode_validator.validate_text(text, allow, tol, prof)
        errors = [f for f in findings if f.level == "ERROR"]
        warns = [f for f in findings if f.level == "WARN"]
        for f in findings:
            print(str(f))
        print("SUMMARY errors=%d warnings=%d file=%s"
              % (len(errors), len(warns), os.path.basename(gpath)))
        if errors:
            return fail("validator reported %d ERROR(s)" % len(errors))

        print("PASS: %s export slice validated (lines=%d, warns=%d)"
              % ("composite fiber" if args.fiber else "T1 plastic-only",
                 text.count("\n"), len(warns)))
        if args.keep_dir:
            os.makedirs(args.keep_dir, exist_ok=True)
            kept = os.path.join(args.keep_dir, os.path.basename(gpath))
            shutil.copyfile(gpath, kept)
            print("kept: %s" % kept)
        return 0
    finally:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
