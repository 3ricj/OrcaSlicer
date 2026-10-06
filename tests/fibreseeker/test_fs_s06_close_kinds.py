#!/usr/bin/env python3
"""Pin S06's per-close-kind behaviour through the real per-file evaluation path.

S06 used to demand "V-1 -> M1002 -> lift -> station entry" for EVERY fibre
window. That is only true for a physical tool departure. An inter-strand close -
the next strand still runs on T0 - legitimately owes no station visit and no
departure lift, and a terminal close is followed by shutdown rather than by an
incoming head. The old check therefore could not be satisfied by a legal
multi-strand activation, and it also accepted a "lift" that was merely a Z word
repeating the height the carriage was already at.

This test drives each fixture through tools/verify_fs_shook.py exactly as the
acceptance run does (subprocess, manifest, JSON report) and asserts S06's OWN
status, so an unrelated fixture failure cannot mask a broken S06 and a vacuous
S06 cannot hide behind an otherwise-green file. S11 is asserted only for the one
case whose rule S11 owns (a spurious departure withdrawal).

Close kinds come from comparing the two things that can follow a close - the
next window opening and the next physical head change - never from which head is
selected at the close.

FIXTURE s06_close_kinds.gcode
  1 inter-strand close, no lift, no station visit      -> S06 PASS
  2 inter-strand close with a valid travel hop         -> S06 PASS
  3 actual departure, withdrawal + lift + station entry-> S06 PASS
  4a departure with the lift removed                   -> S06 FAIL
  4b departure whose lift repeats the current Z        -> S06 FAIL
  4c departure with the station entry removed          -> S06 FAIL
  5a lift moved before the release completes           -> S06 FAIL
  5b lift moved before M1002                           -> S06 FAIL
  6  terminal closure, no head change                  -> terminal rule (PASS),
      and once a head change follows it, the departure rule applies (FAIL)
  +  inter-strand hop below the required clearance     -> S06 FAIL
  +  spurious departure withdrawal at inter-strand close -> S11 FAIL

FIXTURE s06_multi_strand.gcode  (several strands in BOTH a departing activation
and the final activation; W2 and W4 are both the last strand of their
activation yet get OPPOSITE kinds)
  7  classifications W1..W4 = interstrand/departure/interstrand/terminal
  7b unmutated file passes S06
  7c W1 inter-strand close needs no hop at all         -> S06 PASS
  7d W3 inter-strand restart hop weakened              -> S06 FAIL
  7e W2 departure withdrawal removed                   -> S06 FAIL
  7f W2 departure lift removed                         -> S06 FAIL
  7g W2 station entry removed                          -> S06 FAIL
  7h W2 lift placed before its release completes       -> S06 FAIL
  7i terminal W4 given a head change -> departure rule -> S06 FAIL
  7j deleting W2's head change makes W2 inter-strand, so its
     withdrawal becomes spurious                       -> S11 FAIL

FIXTURE s06_last_window_departure.gcode  (the LAST window of the file departs)
  8  classifications W1..W2 = interstrand/departure, and S06 PASSes: a last
     window with a head change coming is a departure, not a terminal close.

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
FIXDIR = os.path.join(HERE, "fixtures_s06")

# fixture basename -> the close kinds it is built to contain, in window order.
FIXTURES = {
    "s06_close_kinds": (
        os.path.join(FIXDIR, "s06_close_kinds.gcode"),
        ["interstrand", "interstrand", "departure", "terminal"]),
    "s06_multi_strand": (
        os.path.join(FIXDIR, "s06_multi_strand.gcode"),
        ["interstrand", "departure", "interstrand", "terminal"]),
    "s06_last_window_departure": (
        os.path.join(FIXDIR, "s06_last_window_departure.gcode"),
        ["interstrand", "departure"]),
}

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
# mutations. Each takes the window number it targets, so one mutator serves
# every fixture rather than hard-coding a single file's layout.
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


def m_drop_departure_lift(nth):
    """Remove a departure window's clearance lift entirely."""
    def fn(lines):
        o, cl = window_close(lines, nth)
        out = list(lines)
        hit = [i for i in range(cl + 1, len(out))
               if RE_MOVE.match(cmd_of(out[i])) and "Z" in out[i].split(";")[0]]
        if not hit:
            raise AssertionError("no lift to remove in window %d" % nth)
        out[hit[0]] = "; DELETED clearance lift"
        return out
    return fn


def m_repeat_z_lift(nth, z):
    """The lift repeats the current height instead of rising."""
    def fn(lines):
        o, cl = window_close(lines, nth)
        out = list(lines)
        for i in range(cl + 1, len(out)):
            if RE_MOVE.match(cmd_of(out[i])) and "Z" in out[i].split(";")[0]:
                out[i] = ("G1 F1200 Z%.2f ; MUTANT lift repeating the current "
                          "height" % z)
                return out
        raise AssertionError("no lift to rewrite in window %d" % nth)
    return fn


def m_drop_station_entry(nth):
    """A departure never enters the station."""
    def fn(lines):
        o, cl = window_close(lines, nth)
        out = list(lines)
        hit = [i for i in range(cl + 1, len(out))
               if cmd_of(out[i]) == "MOVE_TO_BRUSH_STATION"]
        if not hit:
            raise AssertionError("no station entry after window %d" % nth)
        out[hit[0]] = "; DELETED station entry"
        return out
    return fn


def m_drop_departure_withdrawal(nth):
    """A departure loses its stationary V withdrawal."""
    def fn(lines):
        o, cl = window_close(lines, nth)
        out = list(lines)
        hit = [i for i in range(cl + 1, len(out))
               if RE_MOVE.match(cmd_of(out[i])) and "V-4" in out[i].split(";")[0]]
        if not hit:
            raise AssertionError("no departure withdrawal in window %d" % nth)
        out[hit[0]] = "; DELETED departure withdrawal"
        return out
    return fn


def m_lift_before_release(nth):
    """Lift sits immediately after V-1, before the release block."""
    def fn(lines):
        o, cl = window_close(lines, nth)
        out = list(lines)
        for i in range(cl, o, -1):
            if "V-1" in out[i]:
                out.insert(i + 1, "G1 F1200 Z0.80 ; MUTANT lift before the release")
                return out
        raise AssertionError("no V-1 in window %d" % nth)
    return fn


def m_lift_before_close(nth):
    """Lift sits after the release but before M1002."""
    def fn(lines):
        o, cl = window_close(lines, nth)
        out = list(lines)
        out.insert(cl, "G1 F1200 Z0.80 ; MUTANT lift before the window closed")
        return out
    return fn


def m_weak_hop(marker="restart hop"):
    """Inter-strand restart hop below the required clearance."""
    def fn(lines):
        out = list(lines)
        for i, t in enumerate(out):
            if marker in t:
                out[i] = "G1 F1200 Z0.40 ; MUTANT hop, only 0.20 mm of clearance"
                return out
        raise AssertionError("no restart hop marked %r" % marker)
    return fn


def m_spurious_departure(nth):
    """A departure withdrawal at an inter-strand close."""
    def fn(lines):
        o, cl = window_close(lines, nth)
        out = list(lines)
        out.insert(cl + 1, "G1 F600 V-4.000 ; MUTANT spurious departure withdrawal")
        return out
    return fn


def m_becomes_departure():
    """Give a terminal close a following head change.

    With a head change after it, the close stops being terminal and becomes a
    real departure, so S06 must start demanding withdrawal + lift + station entry
    for it. The fixtures' terminal closes have a Z rise but no station visit, so
    this must FAIL. That is what proves the terminal rule is kind-specific rather
    than "expected=False therefore inter-strand".
    """
    def fn(lines):
        out = list(lines)
        while out and out[-1].strip() == "":
            out.pop()
        out.append("T1 ; MUTANT head change after the last window")
        return out
    return fn


def m_delete_head_change():
    """Delete the T0->T1 change, so the last window of the file has nothing
    after it and the close that used to depart becomes inter-strand. Its
    withdrawal is then spurious, which S11 owns."""
    def fn(lines):
        out = list(lines)
        hit = [i for i, t in enumerate(out)
               if cmd_of(t) == "T1" and "switch" in t.lower()]
        if not hit:
            raise AssertionError("no T1 switch to delete")
        out[hit[0]] = "; DELETED head change"
        return out
    return fn


# Each mutant names the check that OWNS the rule it breaks. A mutant failing for
# some unrelated reason must not be scored as a pass, so the expectation is
# per-check rather than "the run went red".
CASES = [
    # ---- single-file fixture: the original ten cases --------------------
    ("s06_close_kinds", "4a departure lift removed",
     m_drop_departure_lift(3), "FAIL", "S06"),
    ("s06_close_kinds", "4b departure lift repeats current Z",
     m_repeat_z_lift(3, 0.20), "FAIL", "S06"),
    ("s06_close_kinds", "4c departure station entry removed",
     m_drop_station_entry(3), "FAIL", "S06"),
    ("s06_close_kinds", "5a lift before the release completes",
     m_lift_before_release(3), "FAIL", "S06"),
    ("s06_close_kinds", "5b lift before M1002",
     m_lift_before_close(3), "FAIL", "S06"),
    ("s06_close_kinds", "+ inter-strand hop under required clearance",
     m_weak_hop(), "FAIL", "S06"),
    ("s06_close_kinds", "6 terminal close given a head change -> departure rule",
     m_becomes_departure(), "FAIL", "S06"),
    ("s06_close_kinds", "+ spurious withdrawal at inter-strand close (S11 owns)",
     m_spurious_departure(1), "FAIL", "S11"),

    # ---- multi-strand: departing activation AND final activation --------
    ("s06_multi_strand", "7e W2 departure withdrawal removed",
     m_drop_departure_withdrawal(2), "FAIL", "S06"),
    ("s06_multi_strand", "7f W2 departure lift removed",
     m_drop_departure_lift(2), "FAIL", "S06"),
    ("s06_multi_strand", "7g W2 departure station entry removed",
     m_drop_station_entry(2), "FAIL", "S06"),
    ("s06_multi_strand", "7h W2 lift before its release completes",
     m_lift_before_release(2), "FAIL", "S06"),
    ("s06_multi_strand", "7d W3 inter-strand restart hop weakened",
     m_weak_hop(), "FAIL", "S06"),
    ("s06_multi_strand", "7i terminal W4 given a head change -> departure rule",
     m_becomes_departure(), "FAIL", "S06"),
    ("s06_multi_strand", "7j W2 head change deleted -> inter-strand, so its "
                         "withdrawal is spurious (S11 owns)",
     m_delete_head_change(), "FAIL", "S11"),

    # ---- last window of the file departs --------------------------------
    ("s06_last_window_departure", "9 W2 departure lift removed",
     m_drop_departure_lift(2), "FAIL", "S06"),
    ("s06_last_window_departure", "9b W2 head change deleted -> terminal, so its "
                                  "withdrawal is spurious (S11 owns)",
     m_delete_head_change(), "FAIL", "S11"),
]


def main():
    for what, p in [("verifier", VERIFIER)] + [
            ("fixture " + k, v[0]) for k, v in sorted(FIXTURES.items())]:
        if not os.path.isfile(p):
            print("FAIL  " + what + " missing: " + p)
            return 1

    tmp = tempfile.mkdtemp(prefix="fs_s06_")
    ok = True
    try:
        # ---- every fixture classifies as designed and passes S06 --------
        for key in sorted(FIXTURES):
            path, want_kinds = FIXTURES[key]
            rows, wins = classify(path)
            got = [r["kind"] for r in rows]
            print(NL + "== %s" % key)
            print("   classification: " + ", ".join(
                "W%d=%s" % (r["window"], r["kind"]) for r in rows))
            if got != want_kinds:
                ok = False
                print("   FAIL classified %s, expected %s" % (got, want_kinds))
            for r in rows:
                print("   W%d %-11s %s" % (r["window"], r["kind"], r["basis"]))
            checks, rc = run_verifier(path, tmp)
            st, detail = view(checks, "S06")
            print("   S06=%s (verifier exit %d)" % (st, rc))
            print("   " + detail[:420])
            if st != "PASS":
                ok = False
                print("   FAIL the unmutated fixture must pass S06")

        # structural expectations that only the multi-strand file can show
        rows, wins = classify(FIXTURES["s06_multi_strand"][0])
        if [r["kind"] for r in rows] != ["interstrand", "departure",
                                         "interstrand", "terminal"]:
            ok = False
            print("FAIL  multi-strand kinds wrong")
        else:
            print(NL + "  OK  W2 and W4 are both the LAST strand of their "
                  "activation yet classify differently: W2 has a head change "
                  "coming first (departure), W4 has nothing after it (terminal)")
        if wins[0].lift_ln is not None or wins[0].entry_ln is not None:
            ok = False
            print("FAIL  W1 should have no Z move and no station visit at all")
        else:
            print("  OK  case 7c: an inter-strand close with NO hop and NO "
                  "station visit passes")
        if wins[2].lift_kind != "hop" or wins[2].entry_ln is not None:
            ok = False
            print("FAIL  W3 restart hop not recognised (kind=%s entry=%s)"
                  % (wins[2].lift_kind, wins[2].entry_ln))
        else:
            print("  OK  case 2/7: inter-strand close with a valid 0.60 mm hop "
                  "passes and is not mistaken for a departure")
        if wins[1].lift_kind != "rise" or not wins[1].entry_after_lift:
            ok = False
            print("FAIL  W2 departure lift/entry not recognised (kind=%s after=%s)"
                  % (wins[1].lift_kind, wins[1].entry_after_lift))
        else:
            print("  OK  case 3/7: departure shows withdrawal -> 0.60 mm rise -> "
                  "station entry in that order")

        # ---- mutants ---------------------------------------------------
        for key, label, fn, want, owner in CASES:
            path = FIXTURES[key][0]
            d = tempfile.mkdtemp(prefix="mut_", dir=tmp)
            open(os.path.join(d, "s06_fixture.gcode"), "w",
                 encoding="utf-8", newline=NL).write(NL.join(fn(read(path))))
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


if __name__ == "__main__":
    sys.exit(main())
