#!/usr/bin/env python3
"""Negative tests for tools/verify_fs_shook.py (owner spec section 8).

Every mutation below is a deliberately broken copy of a passing export. Each one
must be caught by the SPECIFIC semantic error ID it is designed to trip, not
merely by a non-zero exit, and not by a hash mismatch that would mask a broken
detector. Each mutant therefore gets its own manifest carrying the hash of its
OWN bytes so content verification proceeds and the semantic check is what fails.
Hash-mismatch detection is tested separately by a mutant that keeps the stale
hash.

The reviewed baseline export S Hook_20261005-001021.gcode is not present in this
repository or on this machine, so case 1 reconstructs the fault profile that
review documents (zero release in all 16 windows, standby after the incoming
wait, recovery before destination travel) from the passing B export. It is
reported as a reconstruction, not as the original bytes.

Usage:
    python tools/verify_fs_shook_negatives.py --manifest out/fs_shook/manifest.json
"""

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys

VERIFIER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "verify_fs_shook.py")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def read_lines(path):
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        return fh.read().split("\n")


def write_lines(path, lines):
    with open(path, "w", encoding="utf-8", newline="") as fh:
        fh.write("\n".join(lines))


def cmd_of(lines, i):
    body = lines[i].split(";")[0].strip()
    toks = body.split()
    return (toks[0].upper() if toks else "", toks)


def has_word(toks, letter):
    return any(t[0].upper() == letter for t in toks[1:])


def word(toks, letter):
    for t in toks[1:]:
        if t[0].upper() == letter:
            try:
                return float(t[1:])
            except ValueError:
                return None
    return None


def find_windows(lines):
    opens = [i for i, l in enumerate(lines) if cmd_of(lines, i)[0] == "M1001"]
    out = []
    for o in opens:
        cl = next((i for i in range(o + 1, len(lines))
                   if cmd_of(lines, i)[0] == "M1002"), None)
        if cl is not None:
            out.append((o, cl))
    return out


def release_move_indices(lines):
    """Dry-release moves: no-extrusion G1 after V-1 and before M1002, carrying
    XY. The comment stays; only the command is ever deleted."""
    out = []
    for o, cl in find_windows(lines):
        v1 = None
        for i in range(o, cl):
            c, toks = cmd_of(lines, i)
            if c in ("G0", "G1") and (word(toks, "V") or 0) < 0:
                v1 = i
        if v1 is None:
            continue
        for i in range(v1 + 1, cl):
            c, toks = cmd_of(lines, i)
            if c not in ("G0", "G1"):
                continue
            if any(has_word(toks, x) for x in "EUV"):
                continue
            if has_word(toks, "X") or has_word(toks, "Y"):
                out.append(i)
    return out


def mut_delete_all_releases(lines):
    rel = release_move_indices(lines)
    if len(rel) < 16:
        raise RuntimeError("expected >=16 release moves, found %d" % len(rel))
    out = list(lines)
    for i in rel:
        out[i] = "; MUTANT: release move removed here"
    return out, "deleted all %d release moves (reconstructed 001021 profile)" % len(rel)


def mut_delete_one_release(lines):
    rel = release_move_indices(lines)
    if not rel:
        raise RuntimeError("no release moves found to delete")
    i = rel[len(rel) // 2]
    out = list(lines)
    out[i] = "; MUTANT: release move removed here"
    return out, "deleted the release move at line %d" % (i + 1)


def mut_recovery_early(lines):
    for i, l in enumerate(lines):
        c, toks = cmd_of(lines, i)
        if c in ("G0", "G1") and (word(toks, "E") or 0) > 5.0:
            j = i
            while j > 0:
                j -= 1
                c2, t2 = cmd_of(lines, j)
                if c2 in ("G0", "G1") and (word(t2, "X") is not None or word(t2, "Y") is not None)                         and word(t2, "E") is None:
                    out = list(lines)
                    line = out[i]
                    del out[i]
                    out.insert(j, line)
                    return out, ("moved the E recovery from line %d to before the "
                                 "destination travel at line %d" % (i + 1, j + 1))
            continue
    raise RuntimeError("no E recovery found to move")


def outgoing_standby(lines, want_tool=0):
    """A standby line for the head being PUT AWAY by a physical switch.

    S08 only judges the outgoing head's standby, so the mutation has to move
    that one: the standby must sit inside a station bracket whose matching bare
    T-word switch moves to the other head.
    """
    brackets = []
    open_at = None
    for i, l in enumerate(lines):
        c = cmd_of(lines, i)[0]
        if c == "MOVE_TO_BRUSH_STATION" and open_at is None:
            open_at = i
        elif c == "MOVE_OUT_BRUSH_STATION" and open_at is not None:
            brackets.append((open_at, i))
            open_at = None
    for a, bnd in brackets:
        # the switch this visit governs: the first bare T word after the exit
        to = None
        for j in range(bnd, min(bnd + 14, len(lines))):
            c = cmd_of(lines, j)[0]
            if len(c) > 1 and c[0] == "T" and c[1:].isdigit():
                to = int(c[1:])
                break
        if to is None or to == want_tool:
            continue                      # T0 is INCOMING here, not outgoing
        for i in range(a, bnd):
            c, toks = cmd_of(lines, i)
            if c == "M104" and "standby" in lines[i] and word(toks, "T") == want_tool:
                clean = next((k for k in range(a, bnd)
                              if cmd_of(lines, k)[0] == "CLEAN_NOZZLE"), None)
                wait = next((k for k in range(a, bnd)
                             if cmd_of(lines, k)[0] == "M109"
                             and word(cmd_of(lines, k)[1], "T") == to), None)
                return i, a, clean, wait, to
    return None


def mut_standby_after_wait(lines):
    hit = outgoing_standby(lines)
    if not hit:
        raise RuntimeError("no outgoing T0 standby inside a bracket found")
    i, a, clean, wait, to = hit
    if wait is None:
        raise RuntimeError("no incoming wait in that bracket")
    out = list(lines)
    line = out[i]
    del out[i]
    out.insert(wait, line)                # now BELOW the blocking wait
    return out, ("moved the outgoing T0 standby from line %d to after the incoming "
                 "T%d wait at line %d" % (i + 1, to, wait + 1))


def mut_standby_before_clean(lines):
    hit = outgoing_standby(lines)
    if not hit:
        raise RuntimeError("no outgoing T0 standby inside a bracket found")
    i, a, clean, wait, to = hit
    if clean is None:
        raise RuntimeError("no clean in that bracket")
    out = list(lines)
    line = out[i]
    del out[i]
    out.insert(clean, line)               # now ABOVE the clean
    return out, ("moved the outgoing T0 standby from line %d to before its clean "
                 "at line %d" % (i + 1, clean + 1))


def mut_wait_outside_station(lines):
    inside = []
    open_at = None
    for i, l in enumerate(lines):
        c = cmd_of(lines, i)[0]
        if c == "MOVE_TO_BRUSH_STATION" and open_at is None:
            open_at = i
        elif c == "MOVE_OUT_BRUSH_STATION" and open_at is not None:
            inside.extend(j for j in range(open_at, i) if cmd_of(lines, j)[0] == "M109")
            open_at = None
    if not inside:
        raise RuntimeError("no in-bracket M109 found")
    i = inside[len(inside) // 2]
    out = list(lines)
    line = out[i]
    del out[i]
    out.insert(i + 8, line)
    return out, "moved the M109 at line %d outside its station bracket" % (i + 1)


def mut_delete_preheat_command(lines):
    for i, l in enumerate(lines):
        if l.startswith("M104") and "FS_PREHEAT" in l and "T1" in l:
            out = list(lines)
            out[i] = "; FS_PREHEAT next_tool=1 lead_s=15 (comment kept, command deleted)"
            return out, "deleted the preheat command at line %d, comment retained" % (i + 1)
    raise RuntimeError("no T1 preheat command found")


def mut_c_cut_at_b(lines):
    """Move C's cut 1 mm later, i.e. back to the B position."""
    out = list(lines)
    for o, cl in find_windows(out):
        cut = next((i for i in range(o, cl) if cmd_of(out, i)[0] == "M2800"), None)
        if cut is None:
            continue
        for i in range(cut - 1, o, -1):
            c, toks = cmd_of(out, i)
            if c in ("G0", "G1") and (word(toks, "V") or 0) > 0 and word(toks, "X") is not None:
                x = word(toks, "X")
                new = re.sub(r"X-?[0-9.]+", "X%.2f" % (x + 1.0), out[i], count=1)
                out[i] = new
                return out, ("moved the first window's cut 1 mm later (line %d): "
                             "C reverted to the B cut" % (i + 1))
    raise RuntimeError("no cut position found to move")


def mut_release_across_gap(lines):
    """Translate ONE window's whole release 40 mm off the deposited material.

    Translating the entire release polyline keeps its length, feed and Z, so the
    only thing left wrong is containment: the footprint lands on bed that this
    layer never deposited. That isolates the unsupported-release check.
    """
    rel = release_move_indices(lines)
    if not rel:
        raise RuntimeError("no release move found")
    win = rel[len(rel) // 2]
    out = list(lines)
    touched = 0
    for i in rel:
        if i != win:
            continue
    # translate every release move of the window this move belongs to
    for o, cl in find_windows(out):
        if not (o < win < cl):
            continue
        for i in range(o, cl):
            if i not in rel:
                continue
            y = word(cmd_of(out, i)[1], "Y")
            if y is None:
                continue
            out[i] = re.sub(r"Y-?[0-9.]+", "Y%.2f" % (y + 40.0), out[i], count=1)
            touched += 1
    if not touched:
        raise RuntimeError("release window not found")
    return out, ("translated the release at line %d (and its window) 40 mm off the "
                 "deposited material" % (win + 1))


def mut_hash_only(lines):
    out = list(lines)
    out.insert(0, "; MUTANT: content changed, manifest hash left stale")
    return out, "changed content while leaving the manifest hash stale"


CASES = [
    ("t1_001021_profile_no_release", "shook_B_forward_release", mut_delete_all_releases,
     "FS_RELEASE_NOT_EXECUTED"),
    ("t2_delete_one_release", "shook_B_forward_release", mut_delete_one_release,
     "FS_RELEASE_NOT_EXECUTED"),
    ("t3_recovery_before_travel", "shook_B_forward_release", mut_recovery_early,
     "FS_E_RECOVERY_LOCATION"),
    ("t4a_standby_after_wait", "shook_B_forward_release", mut_standby_after_wait,
     "FS_STANDBY_AFTER_WAIT"),
    ("t4b_standby_before_clean", "shook_B_forward_release", mut_standby_before_clean,
     "FS_COOL_BEFORE_CLEAN"),
    ("t5_wait_outside_station", "shook_B_forward_release", mut_wait_outside_station,
     "FS_WAIT_OUTSIDE_STATION"),
    ("t6_preheat_command_deleted", "shook_B_forward_release", mut_delete_preheat_command,
     "FS_PREHEAT_MISSING"),
    ("t7_c_cut_at_b_position", "shook_C_release_margin_1mm", mut_c_cut_at_b,
     "FS_TAIL_DISTANCE_MISMATCH"),
    ("t8_release_across_unprinted_gap", "shook_B_forward_release", mut_release_across_gap,
     "FS_RELEASE_UNSUPPORTED"),
    ("t9_stale_manifest_hash", "shook_B_forward_release", mut_hash_only,
     "FS_HASH_MISMATCH"),
]


def run_case(root, manifest, name, src_variant, mutate, expect, workdir):
    src = next((os.path.join(root, f["gcode"]) for f in manifest["files"]
                if f["variant"] == src_variant), None)
    if src is None:
        return dict(name=name, ok=False, why="source variant missing")

    try:
        mutated, note = mutate(read_lines(src))
    except Exception as exc:
        return dict(name=name, ok=False, why="mutation failed: %s" % exc)

    d = os.path.join(workdir, name)
    gpath = os.path.join(d, "gcode", name, "plate_1.gcode")
    os.makedirs(os.path.dirname(gpath), exist_ok=True)
    write_lines(gpath, mutated)

    tm = json.loads(json.dumps(manifest))
    entry = next(f for f in tm["files"] if f["variant"] == src_variant)
    stale = next(f["sha256"] for f in manifest["files"] if f["variant"] == src_variant)
    # Keep variant as the SOURCE name so every spec lookup in the verifier
    # resolves; the mutant identity rides in its own field.
    entry["mutant"] = name
    entry["gcode"] = "gcode/%s/plate_1.gcode" % name
    entry["sha256"] = stale if expect == "FS_HASH_MISMATCH" else sha256(gpath)
    entry["negative_test"] = dict(mutation=note, expected_error=expect)
    tm["files"] = [entry]
    mpath = os.path.join(d, "manifest.json")
    json.dump(tm, open(mpath, "w", encoding="utf-8"), indent=1)

    jpath = os.path.join(d, "verification.json")
    proc = subprocess.run([sys.executable, VERIFIER, "--manifest", mpath,
                           "--json-out", jpath,
                           "--md-out", os.path.join(d, "verification.md")],
                          capture_output=True, text=True, errors="replace")

    codes, failed = [], []
    try:
        v = json.load(open(jpath, encoding="utf-8"))
        for fr in v["files"]:
            for cid, ck in fr["checks"].items():
                if ck["status"] == "FAIL":
                    failed.append(cid)
                codes.extend(ck.get("codes", []))
    except Exception as exc:
        return dict(name=name, ok=False, why="no parseable report: %s" % exc,
                    exit=proc.returncode)

    ok = proc.returncode != 0 and expect in codes
    return dict(name=name, ok=ok, exit=proc.returncode, expected=expect,
                codes=sorted(set(codes)), failed=sorted(failed), note=note)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--workdir", default=os.path.join("out", "fs_shook", "negatives"))
    args = ap.parse_args(argv)

    root = os.path.dirname(os.path.abspath(args.manifest))
    manifest = json.load(open(args.manifest, encoding="utf-8"))
    if os.path.isdir(args.workdir):
        shutil.rmtree(args.workdir)
    os.makedirs(args.workdir)

    results = []
    for name, variant, mut, expect in CASES:
        r = run_case(root, manifest, name, variant, mut, expect, args.workdir)
        results.append(r)
        print("%-34s %-4s expected=%-26s exit=%s codes=%s"
              % (r["name"], "PASS" if r.get("ok") else "FAIL", r.get("expected", "?"),
                 r.get("exit", "-"), ",".join(r.get("codes", [])) or r.get("why", "")))

    all_ok = all(r.get("ok") for r in results)
    json.dump(results, open(os.path.join(args.workdir, "negatives.json"), "w",
                            encoding="utf-8"), indent=1)
    print("")
    print("NEGATIVE_TESTS: %s (%d/%d)"
          % ("PASS" if all_ok else "FAIL",
             sum(1 for r in results if r.get("ok")), len(results)))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
