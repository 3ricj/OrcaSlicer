#!/usr/bin/env python3
"""Freshly slice the owner S-hook project three times with the rebuilt slicer.

Writes one machine preset per variant into a throwaway datadir's user bundle so
the CLI can resolve it by file identity, then invokes the built orca-slicer.exe
once per variant and records the exact command, exit code and output hash.

Run from the repository root with the WSL python3: the argv handed to the
Windows binary must be Windows-flavoured, so paths are translated on the way
out and the record keeps the translated form (that string IS the invocation).
"""
import hashlib
import json
import os
import shutil
import subprocess
import sys

ROOT = os.path.abspath(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
OUT = os.path.join(ROOT, "out", "fs_shook")
DATADIR = os.path.join(OUT, "datadir")
BIN = os.path.join(ROOT, "build", "src", "Release", "orca-slicer.exe")
MODEL = "C:/Users/3ricj/Documents/GitHub/FibreSeeker3/Test_files/2026-09-25 S-hook/S Hook.3mf"
PROC = os.path.join(ROOT, "resources", "profiles", "FibreSeeker3", "process",
                    "0.12mm Reinforced L5 @FibreSeeker3 SK3.json")
FILA = os.path.join(ROOT, "resources", "profiles", "FibreSeeker3", "filament",
                    "FibreSeek PETG @FibreSeeker3 SK3.json")
BASE_MACHINE = os.path.join(ROOT, "resources", "profiles", "FibreSeeker3", "machine",
                            "FibreSeeker3 SK3 CF nozzle.json")

# The owner-fixed trial matrix (spec section 6).
VARIANTS = {
    "shook_A_park_wait":          {"fs_fiber_release_length_mm": "0.0", "fs_fiber_tail_margin_mm": "0.0"},
    "shook_B_forward_release":    {"fs_fiber_release_length_mm": "6.8", "fs_fiber_tail_margin_mm": "0.0"},
    "shook_C_release_margin_1mm": {"fs_fiber_release_length_mm": "6.8", "fs_fiber_tail_margin_mm": "1.0"},
}
COMMON = {
    "fs_tool_preheat_lead_s": "15.0",
    "fs_fiber_release_speed_mm_s": "10.0",
    "fs_fiber_release_anchor_mm": "8.0",
    "fs_fiber_release_length_mm": "0.0",
    "fs_fiber_tail_margin_mm": "0.0",
}

_MNT = "/mnt/c/"


def wsl_path(p):
    """Translate a C:/... path for the WSL-side filesystem check only."""
    if len(p) > 2 and p[1] == ":" and p[2] == "/":
        return "/mnt/" + p[0].lower() + p[2:]
    return p


def win_path(p):
    """The Windows binary cannot read a /mnt/c/... argv; hand it C:/... ."""
    if p.startswith(_MNT):
        return "C:/" + p[len(_MNT):]
    return p


def sha256(path):
    h = hashlib.sha256()
    with open(wsl_path(path), "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def write_machine_preset(name, overrides):
    """Materialise one variant's machine preset into the datadir user bundle.

    The CLI resolves a --load-settings file by identity against the loaded
    bundle, so a user-flavoured file placed under datadir/user/default/machine
    is resolvable; `from: User` is what the loader accepts there. Every value is
    written as a string: the loader silently ignores a bare number.
    """
    base = json.load(open(wsl_path(BASE_MACHINE), encoding="utf-8-sig"))
    preset = {"type": "machine", "name": name, "from": "User",
              "inherits": "FibreSeeker3 SK3 CF nozzle",
              # PresetCollection::load_presets drops a user preset whose version
              # does not parse as a semantic version, so it must be present.
              "version": "1.0.14.1"}
    for k, v in base.items():
        if k in ("type", "name", "from", "inherits"):
            continue
        if isinstance(v, bool):
            preset[k] = "1" if v else "0"
        elif isinstance(v, (int, float)):
            preset[k] = repr(v) if isinstance(v, float) else str(v)
        elif isinstance(v, list):
            preset[k] = [str(x) if not isinstance(x, str) else x for x in v]
        else:
            preset[k] = v
    preset.update(COMMON)
    preset.update(overrides)
    user_machine = os.path.join(wsl_path(DATADIR), "user", "default", "machine")
    os.makedirs(user_machine, exist_ok=True)
    path = os.path.join(user_machine, name + ".json")
    json.dump(preset, open(path, "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    return win_path(path)


def main():
    if not os.path.isfile(wsl_path(BIN)):
        print("FATAL: built binary missing: %s" % BIN)
        return 2
    if not os.path.isfile(wsl_path(MODEL)):
        print("FATAL: S-hook project missing: %s" % MODEL)
        return 2

    only = sys.argv[1:] if len(sys.argv) > 1 else list(VARIANTS)
    results = []
    for name in only:
        if name not in VARIANTS:
            print("FATAL: unknown variant %s" % name)
            return 2
        preset_path = write_machine_preset(name, VARIANTS[name])
        outdir = win_path(os.path.join(OUT, "gcode", name))
        if os.path.isdir(wsl_path(outdir)):
            shutil.rmtree(wsl_path(outdir))
        os.makedirs(wsl_path(outdir))
        # argv[0] must be exec-able from this shell (WSL form); every argument
        # the Windows program itself parses stays in Windows form.
        cmd = [BIN,
               "--datadir", win_path(DATADIR),
               "--load-settings", preset_path + ";" + win_path(PROC),
               "--load-filaments", win_path(FILA),
               "--slice", "0",
               "--outputdir", outdir,
               MODEL]
        print("=" * 70)
        print("VARIANT", name)
        cmd_display = [win_path(BIN)] + cmd[1:]
        print("CMD:", " ".join(cmd_display))
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              errors="replace", cwd=ROOT, timeout=1800)
        print("EXIT:", proc.returncode)
        rec = {"variant": name, "preset_path": preset_path, "outdir": outdir,
               "cmd": cmd_display, "exit": proc.returncode, "gcode": None,
               "sha256": None, "lines": None,
               "stdout_tail": (proc.stdout or "")[-400:],
               "stderr_tail": (proc.stderr or "")[-800:]}
        files = sorted(os.listdir(wsl_path(outdir))) if os.path.isdir(wsl_path(outdir)) else []
        gcodes = [f for f in files if f.endswith(".gcode")]
        if gcodes:
            gp = outdir + "/" + gcodes[0]
            rec["gcode"] = gp
            rec["sha256"] = sha256(gp)
            with open(wsl_path(gp), "r", encoding="utf-8", errors="replace") as fh:
                rec["lines"] = sum(1 for _ in fh)
            print("GCODE:", gp)
            print("SHA256:", rec["sha256"])
            print("LINES:", rec["lines"])
        else:
            print("NO GCODE EXPORTED")
            print("stderr:", rec["stderr_tail"])
        results.append(rec)

    with open(os.path.join(OUT, "export_runs.json"), "w", encoding="utf-8") as fh:
        json.dump(results, fh, indent=1)
    bad = [r["variant"] for r in results if r["exit"] != 0 or not r["gcode"]]
    print("=" * 70)
    print("FAILED VARIANTS:", bad if bad else "none")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
