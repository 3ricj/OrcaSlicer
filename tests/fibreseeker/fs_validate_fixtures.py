#!/usr/bin/env python3
"""fs_validate_fixtures.py - fixture-suite runner for the FibreSeeker3 validator.

Each fixture in --dir (*.gcode) carries machine-readable expectations as
comment lines:

    # EXPECT: ERROR R02
    # EXPECT: WARN  R01U

The suite passes a fixture only if the SET of ERROR rule codes emitted by
fs_gcode_validator equals the set of expected ERROR rules AND the same for
WARN. Multiplicity is intentionally not compared (a rule may fire several
times); unexpected rules of either level are failures, so regressions in
either direction are caught.

Exit code 0 = all fixtures pass, 1 = at least one failure.
"""

import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fs_gcode_validator import (  # noqa: E402
    validate_text, load_allowlist, load_profile, default_data_path,
)

EXPECT_RE = re.compile(r"^#\s*EXPECT:\s*(ERROR|WARN)\s+(R\w+)\s*$", re.IGNORECASE)


def parse_fixture(path):
    """Return (expected_sets, gcode_text). expected_sets = {'ERROR': set, 'WARN': set}."""
    expected = {"ERROR": set(), "WARN": set()}
    kept = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for raw in fh.read().splitlines():
            m = EXPECT_RE.match(raw.strip())
            if m:
                expected[m.group(1).upper()].add(m.group(2).upper())
            else:
                kept.append(raw)
    return expected, "\n".join(kept)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Run FibreSeeker3 validator fixtures")
    ap.add_argument("--dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures"),
                    help="fixtures directory")
    ap.add_argument("--allowlist", default=None)
    ap.add_argument("--profile", default=None)
    ap.add_argument("-v", "--verbose", action="store_true", help="print findings for failing fixtures")
    args = ap.parse_args(argv)

    allow, tol = load_allowlist(args.allowlist or default_data_path("command_allowlist.json"))
    prof = load_profile(args.profile or default_data_path("machine_profile.json"))

    names = sorted(f for f in os.listdir(args.dir) if f.endswith(".gcode"))
    if not names:
        print("NO FIXTURES FOUND in %s" % args.dir, file=sys.stderr)
        return 1

    failures = 0
    for name in names:
        path = os.path.join(args.dir, name)
        expected, text = parse_fixture(path)
        findings = validate_text(text, allow, tol, prof)
        actual = {"ERROR": set(), "WARN": set()}
        for f in findings:
            actual[f.level].add(f.rule)

        ok = (actual["ERROR"] == expected["ERROR"]) and (actual["WARN"] == expected["WARN"])
        status = "PASS" if ok else "FAIL"
        if ok:
            print("%s  %-40s errors=%s warnings=%s" %
                  (status, name, sorted(expected["ERROR"]) or "-", sorted(expected["WARN"]) or "-"))
        else:
            failures += 1
            miss_e = expected["ERROR"] - actual["ERROR"]
            extra_e = actual["ERROR"] - expected["ERROR"]
            miss_w = expected["WARN"] - actual["WARN"]
            extra_w = actual["WARN"] - expected["WARN"]
            print("%s  %-40s missing ERROR=%s unexpected ERROR=%s missing WARN=%s unexpected WARN=%s" %
                  (status, name, sorted(miss_e) or "-", sorted(extra_e) or "-",
                   sorted(miss_w) or "-", sorted(extra_w) or "-"))
            if args.verbose:
                for f in findings:
                    print("      %s" % f)

    print("FIXTURES: %d/%d passed" % (len(names) - failures, len(names)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
