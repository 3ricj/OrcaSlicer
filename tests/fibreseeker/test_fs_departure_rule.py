"""Pin the S11 departure rule with a two-strand, single-activation fixture.

Both closes in the fixture happen while T0 is selected. Only the second is a
departure. The old rule (head at close == 0) demanded a withdrawal at the first
close too; the current rule looks forward to the next window and the next
physical head change.

Run with no arguments; exit 0 = the rule is pinned in both directions.
"""

import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))

import verify_fs_shook as V  # noqa: E402

FIXTURE = os.path.join(HERE, "fixtures_departure",
                       "good_interstrand_then_departure.gcode")
WANT_DEP = -4.0
TOL = V.TOL_STATIONARY_V


def obligations(path):
    lines, unsupported = V.load_lines(path)
    if unsupported:
        raise AssertionError("fixture uses unsupported commands: %s" % unsupported)
    trans = V.find_transitions(lines)
    wins = []
    all_wins = V.find_windows(lines)
    for k, (w, o, cl, cu) in enumerate(all_wins):
        nxt = all_wins[k + 1][1] if k + 1 < len(all_wins) else None
        V.measure_window(lines, w, o, cl, cu, bound_idx=nxt)
        wins.append(w)
    return V.departure_obligations(lines, wins, trans, WANT_DEP, TOL)


def fail(msg):
    print("FAIL  " + msg)
    return False


def main():
    if not os.path.isfile(FIXTURE):
        return fail("fixture missing: " + FIXTURE)
    rows = obligations(FIXTURE)
    ok = True
    print("fixture windows: %d" % len(rows))
    for d in rows:
        print("  W%d head=T%s next_open=%s next_change=%s expected=%s "
              "withdrawals=%s  (%s)"
              % (d["window"], d["head"], d["next_open_ln"], d["change_ln"],
                 d["expected"], d["lines"], d["basis"]))

    if len(rows) != 2:
        return fail("fixture must have exactly 2 windows, found %d" % len(rows))

    w1, w2 = rows

    # --- the inter-strand close: on T0, but NOT a departure --------------
    if w1["head"] != 0:
        ok = fail("W1 must close while T0 is selected, got T%s" % w1["head"]) and ok
    if w1["expected"]:
        ok = fail("W1 is an inter-strand close inside one T0 activation and must "
                  "NOT owe a departure withdrawal, but the rule expects one") and ok
    if w1["lines"]:
        ok = fail("W1 must carry no departure withdrawal, found lines %s"
                  % w1["lines"]) and ok

    # --- the real departure: on T0, and IS a departure -------------------
    if w2["head"] != 0:
        ok = fail("W2 must close while T0 is selected, got T%s" % w2["head"]) and ok
    if not w2["expected"]:
        ok = fail("W2 is followed by a physical T0->T1 change between windows and "
                  "MUST owe a departure withdrawal, but the rule expects none") and ok
    if len(w2["lines"]) != 1:
        ok = fail("W2 must carry exactly one departure withdrawal, found %d at %s"
                  % (len(w2["lines"]), w2["lines"])) and ok

    # --- a spurious withdrawal at the inter-strand close must be caught --
    tmp = tempfile.mkdtemp(prefix="fs_dep_")
    try:
        mutated = os.path.join(tmp, "spurious.gcode")
        src = open(FIXTURE, encoding="utf-8").read().split("\n")
        # insert a V-4 immediately after the FIRST M1002, before the next window
        out, done = [], False
        for ln in src:
            out.append(ln)
            if not done and ln.strip() == "M1002":
                out.append("M400")
                out.append("G1 F600 V-4.000 ; SPURIOUS departure at inter-strand close")
                done = True
        if not done:
            return fail("could not find the first M1002 to mutate")
        open(mutated, "w", encoding="utf-8", newline="").write("\n".join(out))
        mrows = obligations(mutated)
        if mrows[0]["expected"]:
            ok = fail("mutation: inter-strand close still not expected to depart") and ok
        if not mrows[0]["lines"]:
            ok = fail("mutation: the injected V-4 at the inter-strand close was not "
                      "collected, so a spurious withdrawal would go unnoticed") and ok
        else:
            print("  mutation: injected V-4 collected at lines %s -> the rule "
                  "reports it as spurious" % mrows[0]["lines"])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # --- deleting the real departure withdrawal must still be caught -----
    tmp2 = tempfile.mkdtemp(prefix="fs_dep2_")
    try:
        mutated = os.path.join(tmp2, "missing.gcode")
        src = open(FIXTURE, encoding="utf-8").read().split("\n")
        out, dropped = [], False
        for ln in src:
            if not dropped and "V-4.000" in ln:
                out.append("; DELETED departure withdrawal")
                dropped = True
                continue
            out.append(ln)
        if not dropped:
            return fail("fixture has no V-4.000 departure withdrawal to delete")
        open(mutated, "w", encoding="utf-8", newline="").write("\n".join(out))
        mrows = obligations(mutated)
        if mrows[1]["expected"] and not mrows[1]["lines"]:
            print("  mutation: departure withdrawal deleted at the real departure "
                  "-> expected=True, found none, so S11 fails")
        else:
            ok = fail("mutation: deleting the real departure withdrawal was not "
                      "detected (expected=%s lines=%s)"
                      % (mrows[1]["expected"], mrows[1]["lines"])) and ok
    finally:
        shutil.rmtree(tmp2, ignore_errors=True)

    if ok:
        print("DEPARTURE_RULE: PASS (inter-strand close owes none, real departure "
              "owes exactly one, both mutations detected)")
        return 0
    print("DEPARTURE_RULE: FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
