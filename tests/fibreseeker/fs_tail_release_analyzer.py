#!/usr/bin/env python3
"""fs_tail_release_analyzer.py - static export analyzer for the fibre-tail
release and thermal-transition ordering contract (owner specification v1.0).

Deliberately SEPARATE from fs_gcode_validator.py. That validator checks the fibre
DIALECT (window budgets, cut presence, V/U sign rules, allowlist). This one
checks the two things the owner spec makes non-negotiable and which dialect rules
cannot express:

  (1) ORDERING. Every long temperature wait sits INSIDE a brush-station visit;
      the outgoing head is not cooled before it releases or before it is cleaned;
      a preheated head is not clobbered back to standby before it activates.
  (2) TAIL / RELEASE. A requested release is actually executed, it carries no
      material at all, the release length matches the plan within 0.05 mm, E
      recovery does not happen at the brush station, and a post-cut direction
      change is reported rather than silently shipped.

Honesty contract, which is the point of this file:
  - Support is reported NOT_EVALUATED, never PASS. A text file does not record
    what was deposited, so an open-end release can never be cleared here.
  - No duration is ever reported as measured. The clock used here is the nominal
    commanded-motion clock: temperature waits and opaque macros contribute zero,
    and acceleration and heater behaviour are not modelled.
  - Static export verification and physical print results are separate. Print
    quality is UNTESTED until the operator supplies results.

Fixture convention (same shape as fs_validate_fixtures.py): a fixture declares
tuning and expectations as comment lines

    # OPT: release_mm=6.8
    # EXPECT: ERROR FS_WAIT_OUTSIDE_STATION
    # EXPECT: WARN  FS_TAIL_SHARP_TURN

and the suite passes only when the SET of reported codes matches at each level.
An expectation naming a code outside the contract is a fixture bug and fails.

Exit code 0 = all fixtures pass, 1 = at least one failure.
"""

import argparse
import json
import math
import os
import re
import sys

# ---------------------------------------------------------------------------
# codes, exactly as the spec names them
# ---------------------------------------------------------------------------

ERROR_CODES = {
    "FS_WAIT_OUTSIDE_STATION",
    "FS_COOL_BEFORE_RELEASE",
    "FS_COOL_BEFORE_CLEAN",
    "FS_PREHEAT_CLOBBERED",
    "FS_RELEASE_NOT_EXECUTED",
    "FS_RELEASE_EXTRUSION",
    "FS_RELEASE_UNSUPPORTED",
    "FS_TAIL_DISTANCE_MISMATCH",
    "FS_E_RECOVERY_LOCATION",
    "FS_NONBLOCKING_HEAT_UNSUPPORTED",
    "THERMAL_MANAGEMENT_DISABLED",
    "FS_PURGE_RELEASE_OUT_OF_BOUNDS",
}

WARN_CODES = {
    "FS_TAIL_SHARP_TURN",
    "FS_MACRO_CONTRACT_UNVERIFIED",
}

ALL_CODES = ERROR_CODES | WARN_CODES

# Opaque motion macros: they move the carriage but their duration is unknown, so
# they contribute zero to the nominal clock and they invalidate cached XY.
MOTION_MACROS = {"MOVE_TO_BRUSH_STATION", "CLEAN_NOZZLE", "MOVE_OUT_BRUSH_STATION", "M2800"}

STATION_ENTER = "MOVE_TO_BRUSH_STATION"
STATION_CLEAN = "CLEAN_NOZZLE"
STATION_EXIT = "MOVE_OUT_BRUSH_STATION"

PARAM_RE = re.compile(r"([A-Za-z])\s*(-?\d+(?:\.\d+)?)")
KEYVAL_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)\s*=\s*([^\s;]+)")


class Finding:
    __slots__ = ("level", "rule", "line", "msg")

    def __init__(self, level, rule, line, msg):
        self.level = level
        self.rule = rule
        self.line = line
        self.msg = msg

    def __str__(self):
        return "%s|%s|line %d|%s" % (self.level, self.rule, self.line, self.msg)


def parse_line(raw):
    """Return (cmd, params, comment). cmd is None for a blank or comment-only
    line, but the comment is still returned: the contract markers live there."""
    if ";" in raw:
        body, comment = raw.split(";", 1)
    else:
        body, comment = raw, ""
    body = body.strip()
    comment = comment.strip()
    if not body or body.startswith("#"):
        return None, {}, comment
    parts = body.split(None, 1)
    cmd = parts[0].upper()
    rest = parts[1] if len(parts) > 1 else ""
    params = {}
    for key, val in PARAM_RE.findall(rest):
        try:
            params[key.upper()] = float(val)
        except ValueError:
            pass
    return cmd, params, comment


class Analyzer:
    """One pass over a g-code export, reporting the FS_* ordering and release
    findings. Construct with the same numbers the slice used so the length check
    compares against the plan rather than against a guess."""

    def __init__(self, release_mm=0.0, tolerance_mm=0.05,
                 macro_contract="MACRO_CONTRACT_UNVERIFIED"):
        self.release_mm = float(release_mm)
        self.tol = float(tolerance_mm)
        self.macro_contract = macro_contract
        self.findings = []
        self.stats = {
            "windows": 0,
            "releases_executed": 0,
            "waits_at_station": 0,
            "waits_outside_station": 0,
            "preheats": 0,
            "post_cut_turns_gt_60": 0,
            "max_post_cut_turn_deg": 0.0,
            "support": "NOT_EVALUATED",
        }
        # Release blocks collected during the walk, for the length check.
        self._blocks = []

    def err(self, rule, line, msg):
        self.findings.append(Finding("ERROR", rule, line, msg))

    def warn(self, rule, line, msg):
        self.findings.append(Finding("WARN", rule, line, msg))

    def run(self, text):
        lines = text.splitlines()

        in_station = False
        cleaned_this_visit = False
        selected = None
        target = {}                 # tool -> last commanded S
        preheat = {}                # tool -> True once an FS_PREHEAT M104 is owed
        block = None                # open release block
        post_cut_pts = []           # deposition endpoints after the last cut
        post_cut = False
        macro_seen = False
        last_xy = None              # last commanded XY, the release start anchor

        for lineno, raw in enumerate(lines, 1):
            cmd, params, comment = parse_line(raw)

            # ---- refusal markers the exporter emits as comments ----------
            for code in ("FS_RELEASE_UNSUPPORTED", "FS_PURGE_RELEASE_OUT_OF_BOUNDS",
                         "FS_NONBLOCKING_HEAT_UNSUPPORTED", "THERMAL_MANAGEMENT_DISABLED"):
                if code in raw:
                    self.err(code, lineno, comment or code)

            # ---- release block markers (comment-only lines) --------------
            # The cut handshake marker is a comment-only line, so it must be read
            # before the blank/comment-only short-circuit below.
            if "Start to cut" in comment:
                post_cut = True
                if last_xy is not None:
                    post_cut_pts = [last_xy]

            if "FS_RELEASE_BEGIN" in raw:
                m = re.search(r"length_mm=([0-9.]+)", raw)
                block = {"begin": lineno, "expected": float(m.group(1)) if m else None,
                         "pts": [], "end": None,
                         # The release starts where the deposition ended, so a
                         # single-move release still has a measurable length.
                         "start": last_xy}
                self._blocks.append(block)
                continue
            if "FS_RELEASE_END" in raw:
                if block is None:
                    self.err("FS_RELEASE_NOT_EXECUTED", lineno,
                             "FS_RELEASE_END without a matching FS_RELEASE_BEGIN")
                else:
                    block["end"] = lineno
                    block = None
                continue

            if block is not None and cmd in ("G0", "G1"):
                # The release is dry travel. Any material word, or any Z change,
                # breaks the definition.
                for bad in ("U", "V", "E"):
                    if bad in params:
                        self.err("FS_RELEASE_EXTRUSION", lineno,
                                 "%s word inside the release block (release must be dry)" % bad)
                if "Z" in params:
                    self.err("FS_RELEASE_EXTRUSION", lineno, "Z change inside the release block")
                if "X" in params or "Y" in params:
                    block["pts"].append((params.get("X", 0.0), params.get("Y", 0.0)))

            if cmd is None:
                continue

            if cmd == "M1001":
                self.stats["windows"] += 1
                post_cut = False
                post_cut_pts = []
            elif cmd == "M1002":
                post_cut = False

            # ---- station tracking ---------------------------------------
            if cmd == STATION_ENTER:
                in_station = True
                cleaned_this_visit = False
                macro_seen = True
                continue
            if cmd == STATION_CLEAN:
                cleaned_this_visit = True
                macro_seen = True
                continue
            if cmd == STATION_EXIT:
                in_station = False
                macro_seen = True
                continue

            # ---- temperature --------------------------------------------
            if cmd in ("M104", "M109"):
                tool = int(params.get("T", -1))
                temp = params.get("S")
                if tool < 0 or temp is None:
                    continue
                prev = target.get(tool)
                if cmd == "M104":
                    if "FS_PREHEAT" in comment:
                        preheat[tool] = True
                        self.stats["preheats"] += 1
                    else:
                        # A standby drop on a head whose preheat has not yet been
                        # activated destroys the lead the scheduler paid for.
                        if preheat.get(tool) and prev is not None and temp < prev:
                            self.err("FS_PREHEAT_CLOBBERED", lineno,
                                     "M104 S%d T%d overrides the pending preheat of T%d"
                                     % (int(temp), tool, tool))
                            preheat[tool] = False
                        # Cooling the SELECTED head before it has been cleaned, or
                        # while a release is still running, leaves it below its
                        # printing target with work still queued.
                        if tool == selected and temp > 0 and prev is not None and temp < prev:
                            if block is not None:
                                self.err("FS_COOL_BEFORE_RELEASE", lineno,
                                         "T%d cooled to S%d before the release completed"
                                         % (tool, int(temp)))
                            elif in_station and not cleaned_this_visit:
                                self.err("FS_COOL_BEFORE_CLEAN", lineno,
                                         "T%d cooled to S%d before its cleaning pass"
                                         % (tool, int(temp)))
                    target[tool] = temp
                else:
                    # M109: the blocking wait. It belongs inside a station visit.
                    if temp > 0:
                        if not in_station:
                            self.err("FS_WAIT_OUTSIDE_STATION", lineno,
                                     "blocking M109 S%d T%d outside a brush-station visit"
                                     % (int(temp), tool))
                            self.stats["waits_outside_station"] += 1
                        else:
                            self.stats["waits_at_station"] += 1
                    target[tool] = temp
                    preheat[tool] = False
                continue

            if cmd in ("T0", "T1"):
                selected = 0 if cmd == "T0" else 1
                preheat[selected] = False
                continue

            # ---- E recovery location ------------------------------------
            if cmd in ("G0", "G1") and params.get("E", 0.0) > 0 and in_station:
                self.err("FS_E_RECOVERY_LOCATION", lineno,
                         "plastic recovery inside a brush-station visit (recovery belongs "
                         "at the next deposition start)")

            # ---- post-cut deposition geometry ----------------------------
            if cmd in ("G0", "G1"):
                if "X" in params or "Y" in params:
                    last_xy = (params.get("X", last_xy[0] if last_xy else 0.0),
                               params.get("Y", last_xy[1] if last_xy else 0.0))
                depositing = (params.get("U", 0.0) > 0 or params.get("V", 0.0) > 0
                              or params.get("E", 0.0) > 0)
                if depositing and "X" in params and "Y" in params and post_cut:
                    post_cut_pts.append((params["X"], params["Y"]))

            if cmd in MOTION_MACROS:
                macro_seen = True

        # ---- release reconciliation -------------------------------------
        for blk in self._blocks:
            self.stats["releases_executed"] += 1
            if blk["end"] is None:
                self.err("FS_RELEASE_NOT_EXECUTED", blk["begin"],
                         "FS_RELEASE_BEGIN with no FS_RELEASE_END")
                continue
            pts = blk["pts"]
            if not pts:
                self.err("FS_RELEASE_NOT_EXECUTED", blk["begin"],
                         "release block carries no XY move")
                continue
            if blk["expected"] is not None:
                # Measure from the deposition endpoint the release started at.
                chain = ([blk["start"]] if blk.get("start") else []) + pts
                length = 0.0
                for i in range(1, len(chain)):
                    length += math.hypot(chain[i][0] - chain[i - 1][0],
                                         chain[i][1] - chain[i - 1][1])
                if abs(length - blk["expected"]) > self.tol:
                    self.err("FS_TAIL_DISTANCE_MISMATCH", blk["begin"],
                             "release travelled %.3f mm, planned %.3f mm (tolerance %.3f)"
                             % (length, blk["expected"], self.tol))

        if self.release_mm > 0.0 and self.stats["releases_executed"] == 0:
            self.err("FS_RELEASE_NOT_EXECUTED", 1,
                     "release configured at %.3f mm but no release block was emitted"
                     % self.release_mm)

        # ---- post-cut sharp turns (WARN, reported not fatal) -------------
        turns, worst = self._sharp_turns(post_cut_pts)
        self.stats["post_cut_turns_gt_60"] = turns
        self.stats["max_post_cut_turn_deg"] = worst
        if turns:
            self.warn("FS_TAIL_SHARP_TURN", 1,
                      "%d post-cut direction change(s) over 60 degrees, max %.1f" % (turns, worst))

        # Support cannot be judged from text, ever. Say NOT_EVALUATED.
        self.stats["support"] = "NOT_EVALUATED"

        # The macro contract is only verifiable against a real macro source,
        # which a g-code export is not.
        if self.macro_contract == "MACRO_CONTRACT_UNVERIFIED" or not macro_seen:
            self.warn("FS_MACRO_CONTRACT_UNVERIFIED", 1,
                      "macro definitions unavailable; motion-macro durations are unknown "
                      "and contribute zero to the nominal clock")

        return self.findings

    @staticmethod
    def _sharp_turns(pts):
        turns = 0
        worst = 0.0
        for i in range(1, len(pts) - 1):
            ax, ay = pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]
            bx, by = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
            la, lb = math.hypot(ax, ay), math.hypot(bx, by)
            if la <= 0 or lb <= 0:
                continue
            cosv = max(-1.0, min(1.0, (ax * bx + ay * by) / (la * lb)))
            ang = math.degrees(math.acos(cosv))
            if ang > 60.0:
                turns += 1
                worst = max(worst, ang)
        return turns, worst

    def report(self, variant="unknown"):
        """JSON report. Support is NOT_EVALUATED; no duration is measured."""
        counts = {}
        for f in self.findings:
            counts[f.rule] = counts.get(f.rule, 0) + 1
        return {
            "variant": variant,
            "findings": [str(f) for f in self.findings],
            "code_counts": counts,
            "stats": self.stats,
            "clock_definition": ("nominal commanded motion; temperature waits and opaque macros "
                                 "contribute zero; acceleration and heater behaviour not modelled"),
            "durations_measured": False,
            "support": "NOT_EVALUATED",
            "physical_tail_clearance": "unmeasured",
            "macro_contract": self.macro_contract,
            "print_quality": "UNTESTED",
        }


def analyze_text(text, **kw):
    a = Analyzer(**kw)
    a.run(text)
    return a


# ---------------------------------------------------------------------------
# fixture suite
# ---------------------------------------------------------------------------

EXPECT_RE = re.compile(r"^#\s*EXPECT:\s*(ERROR|WARN)\s+([A-Z_0-9]+)\s*$", re.IGNORECASE)
OPT_RE = re.compile(r"^#\s*OPT:\s*(\w+)=([^\s]+)")


def parse_fixture(path):
    expected = {"ERROR": set(), "WARN": set()}
    opts = {}
    kept = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for raw in fh.read().splitlines():
            s = raw.strip()
            m = EXPECT_RE.match(s)
            if m:
                expected[m.group(1).upper()].add(m.group(2).upper())
                continue
            om = OPT_RE.match(s)
            if om:
                try:
                    opts[om.group(1)] = float(om.group(2))
                except ValueError:
                    opts[om.group(1)] = om.group(2)
                continue
            kept.append(raw)
    return expected, "\n".join(kept), opts


def main(argv=None):
    ap = argparse.ArgumentParser(description="Analyze fibre-tail release / ordering fixtures")
    ap.add_argument("--dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                  "fixtures_tail_release"))
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--report", default=None, help="write a JSON report to this path")
    args = ap.parse_args(argv)

    if not os.path.isdir(args.dir):
        print("NO FIXTURE DIRECTORY at %s" % args.dir, file=sys.stderr)
        return 1

    names = sorted(f for f in os.listdir(args.dir) if f.endswith(".gcode"))
    if not names:
        print("NO FIXTURES FOUND in %s" % args.dir, file=sys.stderr)
        return 1

    failures = 0
    reports = []
    for name in names:
        path = os.path.join(args.dir, name)
        expected, text, opts = parse_fixture(path)
        unknown = (expected["ERROR"] | expected["WARN"]) - ALL_CODES
        if unknown:
            print("FAIL  %-46s fixture names unknown codes %s" % (name, sorted(unknown)))
            failures += 1
            continue
        a = analyze_text(text, **opts)
        actual = {"ERROR": set(), "WARN": set()}
        for f in a.findings:
            actual[f.level].add(f.rule)
        ok = actual["ERROR"] == expected["ERROR"] and actual["WARN"] == expected["WARN"]
        if ok:
            print("PASS  %-46s errors=%s warnings=%s"
                  % (name, sorted(expected["ERROR"]) or "-", sorted(expected["WARN"]) or "-"))
        else:
            failures += 1
            print("FAIL  %-46s missing ERROR=%s unexpected ERROR=%s missing WARN=%s unexpected WARN=%s"
                  % (name, sorted(expected["ERROR"] - actual["ERROR"]) or "-",
                     sorted(actual["ERROR"] - expected["ERROR"]) or "-",
                     sorted(expected["WARN"] - actual["WARN"]) or "-",
                     sorted(actual["WARN"] - expected["WARN"]) or "-"))
        if args.verbose:
            for f in a.findings:
                print("      %s" % f)
        reports.append(a.report(name))

    print("TAIL-RELEASE FIXTURES: %d/%d passed" % (len(names) - failures, len(names)))
    if args.report:
        with open(args.report, "w", encoding="utf-8") as fh:
            json.dump({"fixtures": reports,
                       "clock_definition": ("nominal commanded motion; temperature waits and "
                                            "opaque macros contribute zero"),
                       "durations_measured": False,
                       "support": "NOT_EVALUATED",
                       "print_quality": "UNTESTED"}, fh, indent=2)
        print("REPORT: %s" % args.report)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
