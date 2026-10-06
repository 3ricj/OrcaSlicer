#!/usr/bin/env python3
"""fs_verify_release_geometry.py - measure the fibre release from COORDINATES.

Why this file exists. The owner ruling of 2026-10-05 is that a release is only
verified by the coordinates it actually moved through and by where the first Z
lift lands relative to them, and that "comments or configuration values alone
are insufficient". Every other check in this repo reads markers: the analyzer
reads `; FS_RELEASE_BEGIN length_mm=...`, and that string is written by the
planner, so agreeing with it only proves the planner is self-consistent. This
script never consults a comment. It reconstructs the carriage path from G0/G1
words and measures it.

What it proves per window, from coordinates alone:

  1. RELEASE EXECUTED. After the cut handshake there is at least one XY move
     carrying no U, V or E word. A release that was only configured, and only
     mentioned in a comment, scores zero here.
  2. RELEASE DISTANCE. The XY path length of those moves, chained from the
     position each move actually started from. Note where that is: the blade
     fires at S-(T+M) and the printer keeps laying the severed tail until it
     reaches the strand end S, so the release starts at S, NOT at the cut. The
     measured length is compared against the number the planner recorded in the
     JSON evidence manifest, so a disagreement is plan-vs-motion, not a tautology.
  3. RELEASE PRECEDES THE FIRST Z LIFT. The first Z command after the cut is
     located by line number and the whole release must sit before it. A release
     run after the nozzle has lifted is dry travel over the part and worth nothing.
  4. RELEASE IS DRY AND FLAT. No U, V or E word and no Z word in any release move.
  5. TRANSITION ORDERING. Every blocking M109 is paid inside a brush-station visit.

Exit code 0 only when every check passes on every export.

Honesty contract: distances are computed from commanded coordinates, which is
all a static export records. No duration is measured, physical tail clearance is
unmeasured, and print quality is UNTESTED until the operator supplies results.
"""

import argparse
import glob
import json
import math
import os
import re
import sys

PARAM_RE = re.compile(r"([A-Za-z])\s*(-?\d+(?:\.\d+)?)")

# Words that mean "material is being pushed". A release may carry none of them.
MATERIAL_WORDS = ("U", "V", "E")

TOL_MM = 0.05


def parse_body(raw):
    """Return (cmd, params) with every comment stripped. Comments are NEVER
    consulted by this script; that is the entire point."""
    body = raw.split(";", 1)[0].strip()
    if not body or body.startswith("#"):
        return None, {}
    parts = body.split(None, 1)
    cmd = parts[0].upper()
    params = {}
    if len(parts) > 1:
        for key, val in PARAM_RE.findall(parts[1]):
            try:
                params[key.upper()] = float(val)
            except ValueError:
                pass
    return cmd, params


class Window:
    """One M1001..M1002 fibre window, reconstructed from motion."""

    def __init__(self, index):
        self.index = index
        self.begin_line = None
        self.end_line = None
        self.cut_line = None
        # Last XY commanded before the cut, i.e. where the blade fired. The
        # release does NOT start here: the blade cuts at S-(T+M) and the printer
        # keeps depositing the severed tail until it reaches the strand end S.
        # Recorded because the cut position is a required evidence field.
        self.cut_xy = None
        # Moves after the cut, in order:
        #   (lineno, x, y, material, has_z, prev_xy)
        # prev_xy is where the move started from, which is what a release length
        # has to be chained from.
        self.post_cut = []
        self.first_z_line = None
        self.first_z_target = None


def scan(text):
    """Walk the export once. Returns (windows, waits, station_depth)."""
    windows = []
    cur = None
    last_xy = None
    waits = []            # (lineno, tool, target, in_station)
    in_station = False
    station_depth = 0

    for lineno, raw in enumerate(text.splitlines(), 1):
        cmd, p = parse_body(raw)
        if cmd is None:
            continue

        if cmd == "MOVE_TO_BRUSH_STATION":
            in_station = True
            station_depth += 1
        elif cmd == "MOVE_OUT_BRUSH_STATION":
            in_station = False
        elif cmd == "M109":
            waits.append((lineno, int(p.get("T", -1)), p.get("S"), in_station))
        elif cmd in ("G0", "G1"):
            if "X" in p or "Y" in p:
                x = p.get("X", last_xy[0] if last_xy else 0.0)
                y = p.get("Y", last_xy[1] if last_xy else 0.0)
                material = any(p.get(w, 0.0) > 0.0 for w in MATERIAL_WORDS)
                if cur is not None and cur.cut_line is not None:
                    cur.post_cut.append((lineno, x, y, material, "Z" in p, last_xy))
                last_xy = (x, y)
            # The first Z command after the cut is the first Z lift, whatever its
            # direction. Direction is not what was asked; POSITION in the sequence
            # is, and any Z word after the cut means the nozzle is being moved in
            # Z, which ends the window's in-plane work.
            if "Z" in p and cur is not None and cur.cut_line is not None \
                    and cur.first_z_line is None:
                cur.first_z_line = lineno
                cur.first_z_target = p["Z"]

        if cmd == "M1001":
            cur = Window(len(windows) + 1)
            cur.begin_line = lineno
            cur.cut_xy = last_xy
            windows.append(cur)
        elif cmd == "M1002":
            if cur is not None:
                cur.end_line = lineno
                cur = None
        elif cmd == "M2800" and cur is not None and cur.cut_line is None:
            # The cut handshake. The position is the last commanded XY, which is
            # the blade position by definition.
            cur.cut_line = lineno
            cur.cut_xy = last_xy

    return windows, waits, station_depth


def release_moves(win):
    """The release: post-cut XY moves carrying no material and no Z, from the
    first such move up to the first material move or the window close.

    Deliberately NOT bounded by any comment. The tail deposition is skipped
    because it carries material words; the V-only tail retract because it carries
    no XY."""
    out = []
    for (lineno, x, y, material, has_z, prev_xy) in win.post_cut:
        if material or has_z:
            # A material or Z move after a run of dry moves ends the release.
            if out:
                break
            continue
        out.append((lineno, x, y, prev_xy))
    return out


def path_length(moves):
    """XY path length of `moves`, chained from each move's own start position."""
    total = 0.0
    for (_lineno, x, y, prev_xy) in moves:
        if prev_xy is None:
            continue
        total += math.hypot(x - prev_xy[0], y - prev_xy[1])
    return total


def verify_export(path, planned=None):
    """Measure one export. Returns a result dict; problems listed inside it.

    `planned` is an optional list indexed by window number, holding the release
    length the planner recorded for that window, or None where no release was
    planned."""
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        text = fh.read()
    windows, waits, _depth = scan(text)

    problems = []
    win_results = []

    for win in windows:
        moves = release_moves(win)
        length = path_length(moves) if moves else 0.0
        has_release = len(moves) > 0

        res = {
            "window": win.index,
            "window_begin_line": win.begin_line,
            "window_end_line": win.end_line,
            "cut_line": win.cut_line,
            "cut_xy": list(win.cut_xy) if win.cut_xy else None,
            "release_executed": has_release,
            "release_move_count": len(moves),
            "release_start_xy": (list(moves[0][3]) if moves and moves[0][3] else None),
            "release_moves": [[ln, x, y] for (ln, x, y, _p) in moves],
            "release_end_xy": [moves[-1][1], moves[-1][2]] if moves else None,
            "release_measured_xy_mm": round(length, 6),
            "first_z_move_line": win.first_z_line,
            "first_z_target": win.first_z_target,
        }

        if has_release:
            res["release_before_first_z_lift"] = (
                win.first_z_line is not None and moves[-1][0] < win.first_z_line)
            if win.first_z_line is None:
                problems.append("window %d: release executed but no Z move found "
                                "before the window closed, so the release cannot be "
                                "shown to precede the lift" % win.index)
            elif not res["release_before_first_z_lift"]:
                problems.append(
                    "window %d: release ends on line %d, at or after the first Z "
                    "lift on line %d - the nozzle lifted before the release ran"
                    % (win.index, moves[-1][0], win.first_z_line))

            # Dry and flat holds by construction of release_moves(), but it is
            # re-checked against the raw list so a parser bug cannot hide it.
            for (ln, _x, _y, _p) in moves:
                for (pl, _px, _py, material, has_z, _prev) in win.post_cut:
                    if pl == ln and (material or has_z):
                        problems.append("window %d line %d: release move carries "
                                        "material or a Z word" % (win.index, ln))

            want = planned[win.index] if planned and win.index < len(planned) else None
            res["planned_release_mm"] = want
            if want is not None:
                res["distance_matches_plan"] = abs(length - want) <= TOL_MM
                if not res["distance_matches_plan"]:
                    problems.append(
                        "window %d: release travelled %.3f mm, the planner recorded "
                        "%.3f mm (tolerance %.3f)"
                        % (win.index, length, want, TOL_MM))
        else:
            res["release_before_first_z_lift"] = None
            res["planned_release_mm"] = (
                planned[win.index] if planned and win.index < len(planned) else None)
            # A planned release that never moved the carriage is the exact defect
            # this script exists to catch, so it is a failure, not a pass.
            if res["planned_release_mm"]:
                problems.append("window %d: planner recorded a %.3f mm release but no "
                                "dry XY move was executed"
                                % (win.index, res["planned_release_mm"]))

        win_results.append(res)

    outside = [w for w in waits if not w[3]]
    for (lineno, tool, target, _ins) in outside:
        problems.append("line %d: blocking M109 S%s T%d paid outside a brush-station "
                        "visit" % (lineno, int(target) if target is not None else "?", tool))

    return {
        "file": os.path.basename(path),
        "windows": win_results,
        "window_count": len(windows),
        "releases_executed": sum(1 for r in win_results if r["release_executed"]),
        "blocking_waits": len(waits),
        "waits_outside_station": len(outside),
        "problems": problems,
    }


def planned_for_file(manifest_path, base):
    """Release lengths the planner recorded for one export, indexed by window.

    The manifest carries one record per release-bearing window, keyed by
    window_id, which the generator sets to the export's own file stem. Windows
    with no planned release get None at index 0 so indexing is by window number.
    """
    if not manifest_path or not os.path.exists(manifest_path):
        return None
    with open(manifest_path, "r", encoding="utf-8") as fh:
        manifest = json.load(fh)
    recs = manifest.get("records", [])
    mine = [r for r in recs if r.get("window_id") == base]
    if not mine:
        return None
    return [None] + [r.get("actual_release_mm") for r in mine]


def _default_evidence_dir():
    """Locate docs/evidence/tail_release by searching upward from this file.

    Deriving it by counting ".." components broke: this file sits at
    tests/fibreseeker, so the previous default resolved to tests/docs/... and
    the suite reported "no exports" unless --dir was passed explicitly.
    """
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        cand = os.path.join(d, "docs", "evidence", "tail_release")
        if os.path.isdir(cand):
            return cand
        parent = os.path.dirname(d)
        if parent == d:
            return cand
        d = parent


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Measure fibre release geometry from coordinates, not comments")
    ap.add_argument("--dir", default=_default_evidence_dir(),
                    help="directory of tail-release exports to measure")
    ap.add_argument("--json", default=None, help="evidence manifest to compare against")
    ap.add_argument("--report", default=None, help="write a JSON report to this path")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)

    d = os.path.normpath(args.dir)
    names = sorted(glob.glob(os.path.join(d, "*.gcode")))
    if not names:
        print("NO EXPORTS in %s" % d, file=sys.stderr)
        return 1

    manifest = args.json or os.path.join(d, "tail_release_evidence.json")

    results = []
    failures = 0
    for name in names:
        base = os.path.splitext(os.path.basename(name))[0]
        r = verify_export(name, planned_for_file(manifest, base))
        results.append(r)
        ok = not r["problems"]
        if not ok:
            failures += 1
        print("%-6s %-46s windows=%d releases=%d waits=%d outside_station=%d"
              % ("PASS" if ok else "FAIL", r["file"], r["window_count"],
                 r["releases_executed"], r["blocking_waits"],
                 r["waits_outside_station"]))
        if args.verbose or not ok:
            for w in r["windows"]:
                if w["release_executed"]:
                    print("       win %d: %.3f mm  %s -> %s  line(s) %s  first Z lift line %s"
                          % (w["window"], w["release_measured_xy_mm"],
                             w["release_start_xy"], w["release_end_xy"],
                             [m[0] for m in w["release_moves"]],
                             w["first_z_move_line"]))
            for p in r["problems"]:
                print("       ! %s" % p)

    print("GEOMETRY: %d/%d exports verified" % (len(results) - failures, len(results)))
    if args.report:
        with open(args.report, "w", encoding="utf-8") as fh:
            json.dump({
                "exports": results,
                "method": ("distances computed from commanded G0/G1 XY coordinates, "
                           "chained from the position each move started from; comments "
                           "and configuration values are not consulted"),
                "tolerance_mm": TOL_MM,
                "durations_measured": False,
                "physical_tail_clearance": "unmeasured",
                "print_quality": "UNTESTED",
            }, fh, indent=2)
        print("REPORT: %s" % args.report)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
