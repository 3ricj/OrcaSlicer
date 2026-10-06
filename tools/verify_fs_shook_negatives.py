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


def mut_delete_all_hotend_waits(lines):
    """Delete EVERY M109. S07 must fail on the missing waits, not pass vacuously.

    A previous revision of S07 reported "all 0 hotend waits are parked" and gave
    this file a clean bill of health, because it only ever asked whether the
    waits that existed were parked.
    """
    out = [l for i, l in enumerate(lines) if cmd_of(lines, i)[0] != "M109"]
    n = len(lines) - len(out)
    if n < 16:
        raise RuntimeError("expected >=16 M109 waits, found %d" % n)
    return out, "deleted all %d M109 hotend waits" % n


def mut_tail_v_times_30(lines):
    """Multiply the matrix V of every POST-CUT translating move by 30.

    The tail still deposits along the same path at the same feed, so length,
    release and every geometry check stay green; only the payout formula is
    violated. A comment-sorted ledger cannot see this, so S11 must judge V by
    value against the configured rate.
    """
    out = list(lines)
    touched = 0
    for o, cl in find_windows(out):
        cut = next((i for i in range(o, cl) if cmd_of(out, i)[0] == "M2800"), None)
        if cut is None:
            continue
        v1 = None
        for i in range(cut + 1, cl + 1):
            c, toks = cmd_of(out, i)
            if c in ("G0", "G1") and (word(toks, "V") or 0) < 0 \
                    and not has_word(toks, "X"):
                v1 = i
                break
        if v1 is None:
            continue
        for i in range(cut + 1, v1):
            c, toks = cmd_of(out, i)
            if c not in ("G0", "G1"):
                continue
            v = word(toks, "V")
            if v is None or v <= 0 or not has_word(toks, "X"):
                continue
            out[i] = re.sub(r"V[0-9.]+", "V%.3f" % (v * 30.0), out[i], count=1)
            touched += 1
    if touched < 100:
        raise RuntimeError("only %d post-cut moves carried matrix V" % touched)
    return out, "multiplied post-cut matrix V by 30 on %d moves" % touched


def mut_release_on_bead_edge(lines):
    """Slide ONE release 0.20 mm sideways: centreline still on the supporting
    bead, outer edge hanging over unprinted material.

    This is the case a centreline-distance test cannot see. The support bead is
    0.8 mm wide (half-width 0.4), so a 0.20 mm offset leaves the centreline
    comfortably covered while the buffered footprint's far edge reaches 0.6 mm
    from the bead axis, i.e. 0.2 mm beyond the material.
    """
    out = list(lines)
    rel = release_move_indices(out)
    if not rel:
        raise RuntimeError("no release move found")
    win = rel[len(rel) // 2]
    # the window this release belongs to
    bounds = None
    for o, cl in find_windows(out):
        if o < win < cl:
            bounds = (o, cl)
    if bounds is None:
        raise RuntimeError("release window not found")
    moves = [i for i in rel if bounds[0] < i < bounds[1]]
    # heading: from the position before the first release move to its endpoint
    c0, t0 = cmd_of(out, moves[0])
    x0, y0 = word(t0, "X"), word(t0, "Y")
    prev = None
    for i in range(0, moves[0]):
        c2, t2 = cmd_of(out, i)
        if c2 in ("G0", "G1") and has_word(t2, "X") and has_word(t2, "Y"):
            prev = (word(t2, "X"), word(t2, "Y"))
    if prev is None:
        raise RuntimeError("no position before the release")
    ux, uy = x0 - prev[0], y0 - prev[1]
    L = (ux * ux + uy * uy) ** 0.5
    if L < 1e-6:
        raise RuntimeError("degenerate release heading")
    nx, ny = -uy / L, ux / L
    OFF = 0.20
    for i in moves:
        c2, t2 = cmd_of(out, i)
        xp = word(t2, "X") + nx * OFF
        yp = word(t2, "Y") + ny * OFF
        out[i] = re.sub(r"X[-0-9.]+", "X%.3f" % xp, out[i], count=1)
        out[i] = re.sub(r"Y[-0-9.]+", "Y%.3f" % yp, out[i], count=1)
    return out, ("offset the release at line %d by %.2f mm sideways: centreline "
                 "still on the bead, outer edge 0.20 mm past the support"
                 % (moves[0] + 1, OFF))


def mut_startup_wait_duplicated(lines):
    """Move the first model T0 wait into the startup station visit.

    This is the owner's test: delete the M109 that serves the first model
    activation of T0 and re-issue it during the startup visit instead, so the
    file-wide T0 wait count is unchanged. A tally-based S07 sees 16 T0 waits and
    passes; a per-activation S07 sees that the visit immediately serving that
    activation no longer waits for T0.
    """
    out = list(lines)
    t0 = None
    for i, ln in enumerate(out):
        c, t = cmd_of(out, i)
        if c == "M109" and word(t, "T") == 0:
            t0 = i
            break
    if t0 is None:
        raise RuntimeError("no T0 M109 found")
    ent = next((i for i, ln in enumerate(out)
                if ln.strip().startswith("MOVE_TO_BRUSH_STATION")), None)
    exi = next((i for i in range(ent + 1, len(out))
                if out[i].strip().startswith("MOVE_OUT_BRUSH_STATION")), None)
    if ent is None or exi is None:
        raise RuntimeError("no startup station visit")
    moved = out[t0]
    out[t0] = "; MUTANT: T0 wait removed from its own station visit"
    out.insert(exi, moved + " ; MUTANT: duplicated at startup to keep the tally")
    return out, ("removed the T0 wait at line %d from its serving station visit "
                 "and duplicated it inside the startup visit %d..%d, preserving "
                 "the file-wide wait count" % (t0 + 1, ent + 1, exi + 1))


def mut_delete_departure_withdrawals(lines):
    """Delete every departure tool-change withdrawal (the V-4 after M1002).

    The previous S11 stationary-V ledger stopped at M1002, so these moves were
    outside the judged range entirely and deleting all of them passed.
    """
    out = list(lines)
    hits = []
    for i, ln in enumerate(out):
        c, t = cmd_of(out, i)
        if c in ("G0", "G1") and has_word(t, "V") and (not has_word(t, "X")):
            if abs(word(t, "V") + 4.0) < 1e-6:
                hits.append(i)
    if len(hits) < 10:
        raise RuntimeError("only %d departure withdrawals found" % len(hits))
    for i in hits:
        out[i] = "; MUTANT: departure tool-change withdrawal deleted"
    return out, "deleted all %d departure V -4.000 withdrawals" % len(hits)


def mut_delete_departure_lifts(lines):
    """Delete the clearance lift of every departure (the Z rise after the V-4).

    S06 used to accept ANY Z word as the lift. A departure that never rises has
    the carriage entering the brush station at printing height, so the strand
    ended in the air and the head is dragged across the part.
    """
    out = list(lines)
    hits = []
    for i, ln in enumerate(out):
        c, t = cmd_of(out, i)
        if c in ("G0", "G1") and has_word(t, "Z") and not has_word(t, "X")                 and not has_word(t, "Y") and not has_word(t, "V"):
            prev = next((k for k in range(i - 1, max(i - 4, -1), -1)
                         if cmd_of(out, k)[0] == "M1002"), None)
            if prev is not None:
                hits.append(i)
    if len(hits) < 10:
        raise RuntimeError("only %d departure lifts found" % len(hits))
    for i in hits:
        out[i] = "; MUTANT: departure clearance lift deleted"
    return out, "deleted all %d departure clearance lifts" % len(hits)


def mut_lift_before_close(lines):
    """Move one departure lift to just BEFORE its own M1002.

    The release has to run at printing height. Lifting before the window closes
    truncates it, so the strand is released in the air and the measured release
    is no longer the configured one. The lift is found as the Z-only rise that
    follows a close, then relocated above that close.
    """
    out = list(lines)
    closes = [i for i, ln in enumerate(out) if cmd_of(out, i)[0] == "M1002"]
    for cl in closes:
        for i in range(cl + 1, min(cl + 8, len(out))):
            c, t = cmd_of(out, i)
            if c not in ("G0", "G1") or not has_word(t, "Z") or has_word(t, "X")                     or has_word(t, "Y") or has_word(t, "V"):
                continue
            out.insert(cl, out[i])
            del out[i + 1]
            return out, ("moved the departure lift at line %d to just before "
                         "M1002 at line %d" % (i + 1, cl + 1))
    raise RuntimeError("no departure lift found after any window close")


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
    # The three holes the owner measured in the shipped verifier: a
    # vacuously-passing wait check, an unenforced payout formula, and a
    # containment test that only looked at the centreline.
    ("t10_all_hotend_waits_deleted", "shook_B_forward_release",
     mut_delete_all_hotend_waits, "FS_WAIT_MISSING"),
    ("t11_tail_payout_times_30", "shook_B_forward_release",
     mut_tail_v_times_30, "FS_TAIL_PAYOUT_FORMULA"),
    ("t12_release_on_bead_edge", "shook_B_forward_release",
     mut_release_on_bead_edge, "FS_RELEASE_UNSUPPORTED"),
    # The two holes the owner measured in the SECOND review round: a wait check
    # that tallied the whole file instead of judging each activation, and a
    # stationary-V ledger that stopped at M1002 and so never saw the departure
    # withdrawal.
    ("t13_startup_wait_relocated", "shook_B_forward_release",
     mut_startup_wait_duplicated, "FS_WAIT_MISSING", "S07"),
    ("t14_all_departure_withdrawals_deleted", "shook_B_forward_release",
     mut_delete_departure_withdrawals, "FS_DEPART_WITHDRAWAL_MISSING", "S11"),
    # S06 now judges the exit sequence per close KIND, and a lift is a Z
    # DISPLACEMENT rather than the presence of a Z word. These two mutants are
    # the regressions for that.
    ("t15_all_departure_lifts_deleted", "shook_B_forward_release",
     mut_delete_departure_lifts, "FS_WINDOW_SEQUENCE", "S06"),
    ("t16_lift_before_window_close", "shook_B_forward_release",
     mut_lift_before_close, "FS_WINDOW_SEQUENCE", "S06"),
]


def run_case(root, manifest, name, src_variant, mutate, expect, workdir,
           expect_check=None):
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

    # Requiring the semantic code is the point; requiring it from the intended
    # check stops a mutant passing on an unrelated failure that happens to carry
    # the same code.
    ok = (proc.returncode != 0 and expect in codes
          and (expect_check is None or expect_check in failed))
    return dict(name=name, ok=ok, exit=proc.returncode, expected=expect,
                expected_check=expect_check, codes=sorted(set(codes)),
                failed=sorted(failed), note=note)


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
    for case in CASES:
        name, variant, mut, expect = case[:4]
        exp_check = case[4] if len(case) > 4 else None
        r = run_case(root, manifest, name, variant, mut, expect, args.workdir,
                     exp_check)
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
