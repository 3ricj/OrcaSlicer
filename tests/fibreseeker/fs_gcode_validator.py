#!/usr/bin/env python3
"""fs_gcode_validator.py - independent final-G-code validator for FibreSeeker3 / SK3.

Standalone Python 3 (stdlib only). This validator is intentionally INDEPENDENT of
OrcaSlicer's own C++ gcode processing: it re-parses the final g-code from scratch
and enforces the machine contract (docs/superpowers/fibreseeker3/machine_contract.md).

Rule families (code -> contract clause):
  R01  ERROR unknown command not in allowlist                       (12.7)
  R01U WARN  tolerated-unknown pair M1001/M1002 (no fw handler)      (4)
  R02  ERROR E mixed with U/V on one move (firmware error)           (1)
  R03  ERROR negative U (fiber feed is forward-only)                 (12.4)
  R04  ERROR U/V movement while T1 active (plastic head is E-only)   (2)
  R05  ERROR E movement while T0 active (composite head has no E)    (2)
  R06  ERROR fiber deposit with no open M1001 window                 (8.1)
  R06C ERROR M1002 without an open window                             (8.1)
  R06N ERROR nested M1001                                             (8.1)
  R06T ERROR tool switch while window open                            (8.5)
  R06X ERROR window still open at end of file                         (8.1)
  R07  ERROR budget violation: not L <= sum(U window) < L+1           (4/8.1)
  R07W WARN  M1001 without L, or M82 mode (budget unverifiable)       (4)
  R08  ERROR first material move in window not a U-only restart      (8.2)
  R08P ERROR second material move in window not a V-only prime        (8.2)
  R09  ERROR window ended without M2800 (closed by M1002 / at EOF)    (8.4)
  R09D ERROR duplicate M2800 in window                                (8.4)
  R09M ERROR M2800 not immediately followed by M400                   (8.4)
  R09X ERROR M2800 outside any window                                 (8.4)
  R10  ERROR deposit move carries U>0 without V>0 (post-prime)        (8.3)
  R11  ERROR |V - U*P| > tol on joint deposit move                    (3)
  R12  ERROR M106 P outside 0..5                                      (6)
  R12W WARN  M106 without explicit P                                  (6)
  R13  WARN  fiber deposit before any '; LAYER:' marker (priming)     (11)
  R13W WARN  layer number decreased                                    (11)
  R14  ERROR travel bound violation (X/Y, from machine profile)        (1)
  R15  WARN  first move before G21/G90                                 (1)
  R16  WARN  restart feed below calibrated tail length                 (8.2, U04)

Layer marker: only `; LAYER:<n>` (case-sensitive, colon immediately after LAYER).
Decoys deliberately NOT matched: `; MACROLAYER:`, `; LAYER_COUNT:`,
SET_PRINT_STATS_INFO CURRENT/TOTAL_LAYER=... .

Material mode note: the validator assumes relative material units (M83). Under
M82 the U/V words are absolute positions and the budget check is skipped (WARN
R07W), because the reference dialect and this project's emitter use M83.

Exit codes: 0 = no ERRORs, 1 = at least one ERROR, 2 = usage/IO problem.
"""

import argparse
import json
import os
import re
import sys

# ---------------------------------------------------------------------------
# allowlist + profile
# ---------------------------------------------------------------------------

# Built-in fallback of commands this project's emitter and fixtures produce.
# Used only when data/command_allowlist.json is missing.
FALLBACK_ALLOWLIST = [
    "G0", "G1", "G4", "G21", "G90", "G91", "G92",
    "M82", "M83", "M104", "M105", "M106", "M107", "M109",
    "M114", "M140", "M141", "M190", "M191", "M204", "M220", "M221",
    "M280", "M400",
    "T0", "T1",
    "ACTIVATE_EXTRUDER", "BED_MESH_OFFSET", "SAVE_GCODE_STATE", "RESTORE_GCODE_STATE",
    "SET_GCODE_OFFSET", "SET_EXTRUDER_MODE", "RESTORE_EXTRUDER2",
    "SET_PRINT_STATS_INFO", "SET_PRESSURE_ADVANCE", "SET_VELOCITY_LIMIT",
    "SET_SERVO", "GET_POSITION",
    "MOVE_TO_BRUSH_STATION", "MOVE_OUT_BRUSH_STATION", "CLEAN_NOZZLE",
    "M2800",
]

FALLBACK_TOLERATED_UNKNOWN = ["M1001", "M1002"]

DEFAULT_PROFILE = {
    "travel_mm": {"x_min": -45.0, "x_max": 325.0, "y_min": -4.2, "y_max": 339.0},
    "tail_length_mm": 54.8,
    "restart_feed_mm": 55.0,
}


def load_allowlist(path):
    """Return (commands:set, tolerated_unknown:set). Missing file -> fallback."""
    if path and os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
        cmds = {c.upper() for c in data.get("commands", [])}
        tol = {c.upper() for c in data.get("tolerated_unknown", FALLBACK_TOLERATED_UNKNOWN)}
        return cmds, tol
    return set(FALLBACK_ALLOWLIST), set(FALLBACK_TOLERATED_UNKNOWN)


def load_profile(path):
    prof = {k: (dict(v) if isinstance(v, dict) else v) for k, v in DEFAULT_PROFILE.items()}
    if path and os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
        prof["travel_mm"].update(data.get("travel_mm", {}))
        for k in ("tail_length_mm", "restart_feed_mm"):
            if k in data:
                prof[k] = float(data[k])
    return prof


# ---------------------------------------------------------------------------
# tokenizer
# ---------------------------------------------------------------------------

LAYER_RE = re.compile(r"^;\s*LAYER:(\d+)")  # case-sensitive; cannot match MACROLAYER


class Stmt:
    __slots__ = ("line", "cmd", "params", "raw")

    def __init__(self, line, cmd, params, raw):
        self.line = line
        self.cmd = cmd
        self.params = params
        self.raw = raw


def tokenize(lines):
    """Return (statements, layer_events). Comments stripped from statements;
    layer markers detected on raw comment lines."""
    statements = []
    layers = []  # (line, layer_number)
    for lineno, raw in enumerate(lines, 1):
        m = LAYER_RE.match(raw)
        if m:
            layers.append((lineno, int(m.group(1))))
        body = raw.split(";", 1)[0].strip()
        # Klipper accepts '#' as a comment character too; fixtures use
        # '# EXPECT:' lines to carry their expectations.
        if not body or body.startswith("#"):
            continue
        tokens = body.split()
        cmd = tokens[0].upper()
        params = {}
        for tok in tokens[1:]:
            # Klipper/RepRap params are letter-prefixed (X110.0, U2.0, L80);
            # the vendor dialect also accepts KEY=VALUE (e.g. CURRENT_LAYER=1).
            if "=" in tok:
                k, _, v = tok.partition("=")
                try:
                    params[k.upper()] = float(v)
                except ValueError:
                    params[k.upper()] = v
            elif len(tok) > 1 and tok[0].isalpha():
                try:
                    params[tok[0].upper()] = float(tok[1:])
                except ValueError:
                    params[tok.upper()] = True
            else:
                params[tok.upper()] = True
        statements.append(Stmt(lineno, cmd, params, raw))
    return statements, layers


def num(st, key):
    v = st.params.get(key)
    if isinstance(v, bool) or v is None:
        return None
    return float(v)


# ---------------------------------------------------------------------------
# validator
# ---------------------------------------------------------------------------

class Finding:
    __slots__ = ("level", "rule", "line", "msg")

    def __init__(self, level, rule, line, msg):
        self.level = level
        self.rule = rule
        self.line = line
        self.msg = msg

    def __str__(self):
        return "%s|%s|line %d|%s" % (self.level, self.rule, self.line, self.msg)


class Validator:
    def __init__(self, allowlist, tolerated_unknown, profile):
        self.allow = allowlist
        self.tolerated = tolerated_unknown
        self.profile = profile
        self.findings = []

    def err(self, rule, line, msg):
        self.findings.append(Finding("ERROR", rule, line, msg))

    def warn(self, rule, line, msg):
        self.findings.append(Finding("WARN", rule, line, msg))

    def run(self, statements, layers):
        allow, tolerated, prof = self.allow, self.tolerated, self.profile

        tool = None             # None | 0 | 1, set only by bare T0/T1
        layer = None
        layer_ev = 0
        window = None           # {"line","L","u_sum","cut","state","skip_budget"}
        pos = {"X": 0.0, "Y": 0.0, "Z": 0.0}
        abs_mode = True         # G90 default
        mat_abs = False         # M83 default -> False
        saw_g21 = False
        saw_g90 = False
        moved = False

        def eat_layers(upto_line):
            nonlocal layer, layer_ev
            while layer_ev < len(layers) and layers[layer_ev][0] < upto_line:
                ln, lv = layers[layer_ev]
                if layer is not None and lv < layer:
                    self.warn("R13W", ln, "layer decreased (%d -> %d)" % (layer, lv))
                layer = lv
                layer_ev += 1

        for si, st in enumerate(statements):
            cmd = st.cmd

            # ---- allowlist ---------------------------------------------------
            if cmd not in allow:
                if cmd in tolerated:
                    self.warn("R01U", st.line, "%s has no firmware handler (tolerated no-op)" % cmd)
                else:
                    self.err("R01", st.line, "unknown command %r not in machine allowlist" % cmd)

            eat_layers(st.line)

            # ---- mode setters -------------------------------------------------
            if cmd == "G21":
                saw_g21 = True
                continue
            if cmd == "G90":
                abs_mode = True
                saw_g90 = True
                continue
            if cmd == "G91":
                abs_mode = False
                continue
            if cmd == "M82":
                mat_abs = True
                continue
            if cmd == "M83":
                mat_abs = False
                continue

            # ---- tool switch ----------------------------------------------------
            if cmd in ("T0", "T1"):
                if window is not None:
                    self.err("R06T", st.line, "tool switch to %s with fiber window still open" % cmd)
                tool = 0 if cmd == "T0" else 1
                continue

            # ---- moves -----------------------------------------------------------
            if cmd in ("G0", "G1"):
                if not moved:
                    moved = True
                    if not saw_g21:
                        self.warn("R15", st.line, "first move before G21")
                    if not saw_g90:
                        self.warn("R15", st.line, "first move before G90")
                u = num(st, "U") or 0.0
                v = num(st, "V") or 0.0
                e = num(st, "E") or 0.0
                x, y, z = num(st, "X"), num(st, "Y"), num(st, "Z")
                if x is not None:
                    pos["X"] = x if abs_mode else pos["X"] + x
                if y is not None:
                    pos["Y"] = y if abs_mode else pos["Y"] + y
                if z is not None:
                    pos["Z"] = z if abs_mode else pos["Z"] + z

                tv = prof["travel_mm"]
                if x is not None and not (tv["x_min"] - 1e-9 <= pos["X"] <= tv["x_max"] + 1e-9):
                    self.err("R14", st.line, "X %.3f outside travel [%.1f, %.1f]" % (pos["X"], tv["x_min"], tv["x_max"]))
                if y is not None and not (tv["y_min"] - 1e-9 <= pos["Y"] <= tv["y_max"] + 1e-9):
                    self.err("R14", st.line, "Y %.3f outside travel [%.1f, %.1f]" % (pos["Y"], tv["y_min"], tv["y_max"]))

                if e != 0.0 and (u != 0.0 or v != 0.0):
                    self.err("R02", st.line, "E mixed with U/V on one move (firmware rejects this)")
                if u < 0.0:
                    self.err("R03", st.line, "negative U on move (fiber feed is forward-only)")
                if tool == 1 and (u != 0.0 or v != 0.0):
                    self.err("R04", st.line, "U/V movement while T1 active (plastic head is E-only)")
                if tool == 0 and e != 0.0:
                    self.err("R05", st.line, "E movement while T0 active (composite head has no E channel)")

                deposit = (u > 0.0) or (v > 0.0 and tool == 0)
                if deposit:
                    if layer is None:
                        self.warn("R13", st.line, "fiber deposit before any '; LAYER:' marker")
                    if window is None:
                        self.err("R06", st.line, "fiber deposit with no open M1001 window")
                    else:
                        self._window_move(window, st, u, v)
                elif u == 0.0 and v < 0.0 and window is not None:
                    pass  # matrix retract inside window: allowed (V-only retract)
                continue

            # ---- fiber window ------------------------------------------------------
            if cmd == "M1001":
                if window is not None:
                    self.err("R06N", st.line, "nested M1001 (window open since line %d)" % window["line"])
                lval = num(st, "L")
                window = {"line": st.line, "L": int(lval) if lval is not None else None,
                          "u_sum": 0.0, "cut": None, "state": 0, "skip_budget": mat_abs}
                if mat_abs:
                    self.warn("R07W", st.line, "M82 active: budget check skipped (dialect uses M83)")
                if lval is None:
                    self.warn("R07W", st.line, "M1001 without L (budget unverifiable)")
                continue
            if cmd == "M1002":
                if window is None:
                    self.err("R06C", st.line, "M1002 without an open window")
                else:
                    if window["cut"] is None:
                        self.err("R09", st.line, "window closed by M1002 without M2800 cut")
                    lv, us = window["L"], window["u_sum"]
                    # Absorb binary-float error of summing 3-decimal values (~1e-13
                    # per term); without the epsilon a compliant budget on an exact
                    # integer boundary reads as a violation.
                    eps = 1e-6
                    if lv is not None and not window["skip_budget"] and not (lv - eps <= us < lv + 1.0 + eps):
                        self.err("R07", st.line, "budget violation: L=%d, sum(U)=%.5f" % (lv, us))
                    window = None
                continue
            if cmd == "M2800":
                if window is None:
                    self.err("R09X", st.line, "M2800 outside any fiber window")
                elif window["cut"] is not None:
                    self.err("R09D", st.line, "duplicate M2800 in window (opened line %d)" % window["line"])
                else:
                    window["cut"] = st.line
                    nxt = statements[si + 1] if si + 1 < len(statements) else None
                    if nxt is None or nxt.cmd != "M400":
                        self.err("R09M", st.line, "M2800 not immediately followed by M400")
                continue

            # ---- fans ---------------------------------------------------------------
            if cmd == "M106":
                p = st.params.get("P")
                if p is None:
                    self.warn("R12W", st.line, "M106 without P (both part-cool fans; prefer explicit P)")
                elif isinstance(p, float) and not (0.0 <= p <= 5.0):
                    self.err("R12", st.line, "M106 P=%s outside 0..5" % p)
                continue

        if window is not None:
            if window["cut"] is None:
                self.err("R09", window["line"], "window never cut (no M2800) before end of file")
            self.err("R06X", window["line"], "fiber window still open at end of file")

        return self.findings

    def _window_move(self, window, st, u, v):
        """Deposit-class material move inside an open window."""
        if window["state"] == 0:
            if u > 0.0 and v <= 0.0:
                restart = self.profile["restart_feed_mm"]
                if u < restart - 1e-9:
                    self.warn("R16", st.line, "restart feed %.3f below calibrated tail %.3f" % (u, restart))
                window["state"] = 1
            else:
                self.err("R08", st.line, "first material move in window must be a U-only restart, got U=%.5f V=%.5f" % (u, v))
                window["state"] = 1
        elif window["state"] == 1:
            if v > 0.0 and u <= 0.0:
                window["state"] = 2
            else:
                self.err("R08P", st.line, "second material move in window must be a V-only prime, got U=%.5f V=%.5f" % (u, v))
                window["state"] = 2
        elif window["state"] == 2 and u > 0.0 and v <= 0.0:
            self.err("R10", st.line, "deposit move carries U>0 without V>0 (fiber without matrix)")
        if u > 0.0 and v > 0.0:
            p = num(st, "P")
            if p is not None:
                tol = max(0.001, u * 0.0006)
                if abs(v - u * p) > tol:
                    self.err("R11", st.line, "V %.5f != U %.5f * P %.5f (tol %.4f)" % (v, u, p, tol))
        if u > 0.0:
            window["u_sum"] += u


def validate_text(text, allowlist, tolerated, profile):
    statements, layers = tokenize(text.splitlines())
    v = Validator(allowlist, tolerated, profile)
    return v.run(statements, layers)


def default_data_path(name):
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", name)


def main(argv=None):
    ap = argparse.ArgumentParser(description="FibreSeeker3 final-G-code validator")
    ap.add_argument("file", help="g-code file to validate")
    ap.add_argument("--allowlist", default=None, help="command allowlist JSON (generated)")
    ap.add_argument("--profile", default=None, help="machine profile JSON")
    ap.add_argument("--json", action="store_true", help="emit findings as JSON")
    args = ap.parse_args(argv)

    try:
        allow, tol = load_allowlist(args.allowlist or default_data_path("command_allowlist.json"))
        prof = load_profile(args.profile or default_data_path("machine_profile.json"))
        with open(args.file, "r", encoding="utf-8", errors="replace") as fh:
            text = fh.read()
    except OSError as exc:
        print("IO error: %s" % exc, file=sys.stderr)
        return 2

    findings = validate_text(text, allow, tol, prof)
    errors = [f for f in findings if f.level == "ERROR"]
    warns = [f for f in findings if f.level == "WARN"]

    if args.json:
        print(json.dumps([{"level": f.level, "rule": f.rule, "line": f.line, "msg": f.msg}
                          for f in findings], indent=1))
    else:
        for f in findings:
            print(str(f))
        print("SUMMARY errors=%d warnings=%d file=%s" % (len(errors), len(warns), os.path.basename(args.file)))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
