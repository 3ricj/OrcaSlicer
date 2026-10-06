#!/usr/bin/env python3
"""Pin S06's per-close-kind behaviour through the real per-file evaluation path.

S06 used to demand "V-1 -> M1002 -> lift -> station entry" for EVERY fibre
window. That is only true for a physical tool departure. An inter-strand close -
the next strand still runs on T0 - legitimately owes no station visit and no
departure lift, and a terminal close is followed by shutdown rather than by an
incoming head. The old check therefore could not be satisfied by a legal
multi-strand activation, and it also accepted a "lift" that was merely a Z word
repeating the height the carriage was already at.

This test drives the fixture through tools/verify_fs_shook.py exactly as the
acceptance run does (subprocess, manifest, JSON report) and asserts S06's OWN
status, so an unrelated fixture failure cannot mask a broken S06 and a vacuous
S06 cannot hide behind an otherwise-green file.

Cases:
  1 inter-strand close, no lift, no station visit      -> S06 PASS
  2 inter-strand close with a valid travel hop         -> S06 PASS
  3 actual departure, withdrawal + lift + station entry-> S06 PASS
  4a departure with the lift removed                   -> S06 FAIL
  4b departure whose lift repeats the current Z        -> S06 FAIL
  4c departure with the station entry removed          -> S06 FAIL
  5a lift moved before the release completes           -> S06 FAIL
  5b lift moved before M1002                           -> S06 FAIL
  6 terminal closure, no head change                   -> terminal rule (PASS),
      and once a head change follows it, the departure rule applies (FAIL)
  + inter-strand hop below the required clearance      -> S06 FAIL
  + spurious departure withdrawal at inter-strand close-> S06 FAIL

Exit 0 = every case behaved as specified.
"""

import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
VERIFIER = os.path.join(ROOT, "tools", "verify_fs_shook.py")
FIXTURE = os.path.join(HERE, "fixtures_s06", "s06_close_kinds.gcode")

# The close kinds the fixture is built to contain, in window order. Terminal is
# deliberately distinct from inter-strand: both owe no withdrawal, but only one
# of them is followed by another strand.
KINDS = ["interstrand", "interstrand", "departure", "terminal"]

RE_MOVE = re.compile(r"^G[01]$")
NL = chr(10)


# --------------------------------------------------------------------------
# driving the verifier through its normal entry point
# --------------------------------------------------------------------------

def run_verifier(path, workdir):
    """Return (checks dict, verifier exit code) for one G-code file."""
    # The verifier resolves a relative gcode path against the manifest directory,
    # so the bytes under test have to sit beside the manifest.
    gpath = os.path.join(workdir, "s06_fixture.gcode")
    if os.path.abspath(path) != os.path.abspath(gpath):
        shutil.copyfile(path, gpath)
    raw = open(gpath, "rb").read()
    man = {
        "provenance": {"note": "S06 close-kind fixture; not a production export"},
        "macro_contract": "UNVERIFIED",
        "bead_width_mm": 0.8,
        "preheat_lead_s": 15.0,
        "files": [{
            "variant": "s06_fixture",
            "spec_variant": "shook_A_park_wait",
            "gcode": "s06_fixture.gcode",
            "sha256": hashlib.sha256(raw).hexdigest(),
            "cmd": "fixture",
            "exit": 0,
            "effective_config": {"fs_fiber_release_length_mm": "6.8",
                                 "fs_toolchange_retract_v": "4",
                                 "fs_prime_v": "4", "fs_retract_v": "1"},
        }],
    }
    mp = os.path.join(workdir, "manifest.json")
    jp = os.path.join(workdir, "verification.json")
    md = os.path.join(workdir, "verification.md")
    open(mp, "w", encoding="utf-8", newline=NL).write(json.dumps(man, indent=1))
    proc = subprocess.run([sys.executable, VERIFIER, "--manifest", mp,
                           "--json-out", jp, "--md-out", md],
                          capture_output=True, text=True)
    if not os.path.isfile(jp):
        raise AssertionError("verifier produced no JSON; stderr:" + NL + proc.stderr)
    out = json.load(open(jp, encoding="utf-8"))
    checks = out["files"][0]["checks"]
    return checks, proc.returncode


def view(checks, cid):
    """(status, detail-with-codes) for one check, so a case can name its owner."""
    chk = checks.get(cid) or {}
    detail = " | ".join(chk.get("detail", [])) + " | codes=" + ",".join(
        chk.get("codes", []))
    return chk.get("status", "ABSENT"), detail


def classify(path):
    """The classification S06 and S11 share, read off the same code path."""
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    import verify_fs_shook as V
    lines, uns = V.load_lines(path)
    if uns:
        raise AssertionError("fixture uses unsupported commands: %s" % uns)
    aw = V.find_windows(lines)
    wins = []
    for k, (w, o, cl, cu) in enumerate(aw):
        nxt = aw[k + 1][1] if k + 1 < len(aw) else None
        V.measure_window(lines, w, o, cl, cu, bound_idx=nxt)
        wins.append(w)
    rows = V.classify_closes(lines, wins, V.find_transitions(lines), -4.0,
                             V.TOL_STATIONARY_V)
    return rows, wins


# --------------------------------------------------------------------------
# mutations
# --------------------------------------------------------------------------

def read(path):
    return open(path, encoding="utf-8").read().split(NL)


def cmd_of(text):
    toks = text.split(";")[0].split()
    return toks[0].upper() if toks else ""


def window_close(lines, nth):
    """Index of the nth M1002 (1-based) and the M1001 that opened that window."""
    opens = [i for i, t in enumerate(lines) if cmd_of(t) == "M1001"]
    closes = [i for i, t in enumerate(lines) if cmd_of(t) == "M1002"]
    o = opens[nth - 1]
    cl = next(i for i in closes if i > o)
    return o, cl


def m_drop_departure_lift(lines):
    """Case 4a: remove the departure window's clearance lift."""
    o, cl = window_close(lines, 3)
    out = list(lines)
    hit = [i for i in range(cl + 1, len(out))
           if RE_MOVE.match(cmd_of(out[i])) and "Z" in out[i].split(";")[0]]
    if not hit:
        raise AssertionError("no lift to remove")
    out[hit[0]] = "; DELETED clearance lift"
    return out


def m_repeat_z_lift(lines):
    """Case 4b: the lift repeats the current height instead of rising."""
    o, cl = window_close(lines, 3)
    out = list(lines)
    for i in range(cl + 1, len(out)):
        if RE_MOVE.match(cmd_of(out[i])) and "Z" in out[i].split(";")[0]:
            out[i] = "G1 F1200 Z0.20 ; MUTANT lift repeating the current height"
            return out
    raise AssertionError("no lift to rewrite")


def m_drop_station_entry(lines):
    """Case 4c: the departure never enters the station."""
    out = list(lines)
    hit = [i for i, t in enumerate(out) if cmd_of(t) == "MOVE_TO_BRUSH_STATION"]
    if not hit:
        raise AssertionError("no station entry to remove")
    out[hit[0]] = "; DELETED station entry"
    return out


def m_lift_before_release(lines):
    """Case 5a: lift sits immediately after V-1, before the release block."""
    o, cl = window_close(lines, 3)
    out = list(lines)
    for i in range(cl, o, -1):
        if cmd_of(out[i]).startswith("G1 F600 V-1") or "V-1" in out[i]:
            out.insert(i + 1, "G1 F1200 Z0.80 ; MUTANT lift before the release")
            return out
    raise AssertionError("no V-1 in window 3")


def m_lift_before_close(lines):
    """Case 5b: lift sits after the release but before M1002."""
    o, cl = window_close(lines, 3)
    out = list(lines)
    out.insert(cl, "G1 F1200 Z0.80 ; MUTANT lift before the window closed")
    return out


def m_weak_hop(lines):
    """Inter-strand restart hop below the required clearance."""
    out = list(lines)
    for i, t in enumerate(out):
        if "restart hop" in t:
            out[i] = "G1 F1200 Z0.40 ; MUTANT hop, only 0.20 mm of clearance"
            return out
    raise AssertionError("no restart hop in the fixture")


def m_spurious_departure(lines):
    """A departure withdrawal at an inter-strand close."""
    o, cl = window_close(lines, 1)
    out = list(lines)
    out.insert(cl + 1, "G1 F600 V-4.000 ; MUTANT spurious departure withdrawal")
    return out


def m_terminal_becomes_departure(lines):
    """Case 6 control: give the terminal close a following head change.

    With a head change after it, W4 stops being terminal and becomes a real
    departure, so S06 must start demanding withdrawal + lift + station entry for
    it. The fixture's terminal close has a Z rise but no station visit, so this
    must FAIL. That is what proves the terminal rule is kind-specific rather
    than "expected=False therefore inter-strand".
    """
    out = list(lines)
    while out and out[-1].strip() == "":
        out.pop()
    out.append("T1 ; MUTANT head change after the last window")
    return out


# Each mutant names the check that OWNS the rule it breaks. A mutant failing for
# some unrelated reason must not be scored as a pass, so the expectation is
# per-check rather than "the run went red".
CASES = [
    ("4a departure lift removed", m_drop_departure_lift, "FAIL", "S06"),
    ("4b departure lift repeats current Z", m_repeat_z_lift, "FAIL", "S06"),
    ("4c departure station entry removed", m_drop_station_entry, "FAIL", "S06"),
    ("5a lift before the release completes", m_lift_before_release, "FAIL", "S06"),
    ("5b lift before M1002", m_lift_before_close, "FAIL", "S06"),
    ("+ inter-strand hop under required clearance", m_weak_hop, "FAIL", "S06"),
    ("6 terminal close given a head change -> departure rule",
     m_terminal_becomes_departure, "FAIL", "S06"),
    ("+ spurious withdrawal at inter-strand close (S11 owns this rule)",
     m_spurious_departure, "FAIL", "S11"),
]


def main():
    for p, what in ((FIXTURE, "fixture"), (VERIFIER, "verifier")):
        if not os.path.isfile(p):
            print("FAIL  " + what + " missing: " + p)
            return 1

    tmp = tempfile.mkdtemp(prefix="fs_s06_")
    ok = True
    try:
        rows, wins = classify(FIXTURE)
        got = [r["kind"] for r in rows]
        print("classification: " + ", ".join(
            "W%d=%s" % (r["window"], r["kind"]) for r in rows))
        if got != KINDS:
            ok = False
            print("FAIL  fixture classified %s, expected %s" % (got, KINDS))
        else:
            print("  OK  all three kinds present; terminal is not folded into "
                  "inter-strand even though both owe no withdrawal")
        if rows[0]["expected"] or rows[0]["lines"]:
            ok = False
            print("FAIL  W1 inter-strand close owes a departure withdrawal")
        if wins[1].lift_kind != "hop" or wins[1].entry_ln is not None:
            ok = False
            print("FAIL  W2 restart hop not recognised (kind=%s entry=%s)"
                  % (wins[1].lift_kind, wins[1].entry_ln))
        if wins[2].lift_kind != "rise" or not wins[2].entry_after_lift:
            ok = False
            print("FAIL  W3 departure lift/entry not recognised (kind=%s after=%s)"
                  % (wins[2].lift_kind, wins[2].entry_after_lift))
        if wins[0].lift_z_repeat is not None:
            ok = False
            print("FAIL  W1 should have no Z move at all after its close")

        checks, rc = run_verifier(FIXTURE, tmp)
        st, detail = view(checks, "S06")
        print(NL + "base fixture: S06=%s (verifier exit %d)" % (st, rc))
        print("  " + detail[:420])
        if st != "PASS":
            ok = False
            print("FAIL  cases 1+2+3+6: the unmutated fixture must pass S06")
        else:
            print("  OK  cases 1, 2, 3 and the terminal rule of case 6 all pass "
                  "S06 in one file")

        for label, fn, want, owner in CASES:
            d = tempfile.mkdtemp(prefix="mut_", dir=tmp)
            open(os.path.join(d, "s06_fixture.gcode"), "w",
                 encoding="utf-8", newline=NL).write(NL.join(mutate(FIXTURE, fn)))
            checks, rc = run_verifier(os.path.join(d, "s06_fixture.gcode"), d)
            st, detail = view(checks, owner)
            good = (st == want)
            ok = ok and good
            print(NL + "%s: %s=%s (exit %d) %s"
                  % (label, owner, st, rc, "OK" if good else
                     "FAIL, expected " + want))
            print("  " + detail[:340])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print(NL + "S06_CLOSE_KINDS: " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def mutate(src, fn):
    return fn(read(src))


if __name__ == "__main__":
    sys.exit(main())
