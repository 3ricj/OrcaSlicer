#!/usr/bin/env python3
"""verify_fs_shook.py - acceptance verifier for the FibreSeeker3 S-hook exports.

Reads a manifest naming three freshly-sliced S-hook G-code files plus their
provenance, re-parses the emitted bytes from scratch, recomputes every SHA-256,
and evaluates the mandatory checks S01..S13 of
FibreSeeker3_Agent_Spec_Toolchange_and_Tail_Release sections 5-7 and 11.

Rules that are not negotiable:

  * Every distance is measured from EMITTED COORDINATES. A comment may help the
    report NAME a phase; it never substitutes for a command. A sidecar that
    disagrees with the G-code loses.
  * The deterministic clock is the one the spec pins: translating blocks are
    60 * XYZ-distance / modal F; stationary extrusion blocks are
    60 * max(|E|,|U|,|V|) / F; an explicit dwell contributes its P word;
    temperature waits and opaque macros contribute ZERO. A motion block with no
    resolvable feed is an ERROR, not zero time.
  * A check that cannot be evaluated is NOT_EVALUATED, and a NOT_EVALUATED
    mandatory check FAILS the run. Silence is not a pass.
  * Software acceptance and physical print results are reported separately and
    never conflated.

Exit status: 0 only when every mandatory check is evaluated and passes for all
three files. Nonzero for a failed/missing export, unsupported parse, missing
evidence, NOT_EVALUATED mandatory check, or violated invariant.

    python tools/verify_fs_shook.py \
        --manifest out/fs_shook/manifest.json \
        --json-out out/fs_shook/verification.json \
        --md-out   out/fs_shook/verification.md
"""

import hashlib
import json
import math
import os
import re
import sys

# --------------------------------------------------------------------------
# Constants the spec pins
# --------------------------------------------------------------------------

TOL_PATH = 0.05          # mm, path-length rounding tolerance
TOL_RELEASE_Z = 0.001    # mm
TOL_TOTAL = 0.10         # mm, cut-through-release-end cross-check
TOL_LEAD = 0.10          # s, preheat lead tolerance
TOL_CONTAIN = 0.02       # mm, release containment tolerance
CONTAIN_PITCH = 0.20     # mm, along-path sampling pitch for the footprint test
CONTAIN_LAT = 8          # lateral samples across the buffered footprint
TOL_TAIL_V = 0.05        # mm of matrix V per window (shipped max deviation 0.009)
TOL_BODY_RATE = 0.02     # relative, body matrix payout rate (shipped max 0.005)
TOL_STATIONARY_V = 0.002 # mm, per-window stationary V amounts
TOL_LIFT_Z = 0.005       # mm, slack when judging a clearance lift against spec
# The departure Z clearance. FiberEmitter.hpp pins lift_z_mm = 0.6 and GCode.cpp
# feeds strand.z + lift_z_mm to the paired tool-change block, so 0.6 mm above
# the strand is the clearance a departure lift has to reach. Overridable from
# the effective config (fs_toolchange_lift_z) when a build exposes it.
DEPARTURE_CLEARANCE_MM = 0.6

NOMINAL_TAIL = 54.8      # T
EXPECTED_LAYERS = 32
EXPECTED_Z_FIRST = 0.24
EXPECTED_Z_STEP = 0.12
EXPECTED_Z_LAST = 3.96
EXPECTED_WINDOWS = 16    # 1 purge + 15 model strands
EXPECTED_CUTS = 16
EXPECTED_CLOSES = 16

VARIANTS = ("shook_A_park_wait", "shook_B_forward_release",
            "shook_C_release_margin_1mm")

# variant -> (post-cut deposition, release, cut-through-end total, M, R)
VSPEC = {
    "shook_A_park_wait":          dict(dep=54.8, rel=0.0, total=54.8, margin=0.0, release=0.0),
    "shook_B_forward_release":    dict(dep=54.8, rel=6.8, total=61.6, margin=0.0, release=6.8),
    "shook_C_release_margin_1mm": dict(dep=55.8, rel=6.8, total=62.6, margin=1.0, release=6.8),
}

MANDATORY = ("S01", "S02", "S03", "S04", "S05", "S06", "S07", "S08", "S09",
             "S10", "S11", "S12", "S13")

PASS, FAIL, NE = "PASS", "FAIL", "NOT_EVALUATED"

# Opaque motion macros: after one, the position is UNKNOWN until the contract or
# an explicit positioning command resolves it.
OPAQUE_MOTION = {"MOVE_TO_BRUSH_STATION", "MOVE_OUT_BRUSH_STATION", "CLEAN_NOZZLE",
                 "PARK", "UNPARK", "PARK_ON", "PARK_OFF", "WIPE", "WIPE_POS"}
STATION_IN = {"MOVE_TO_BRUSH_STATION"}
STATION_OUT = {"MOVE_OUT_BRUSH_STATION"}

FENCE = chr(96) * 3

RE_WORD = re.compile(r"([A-Za-z])([-+0-9.eE]+)")
RE_MOVE = re.compile(r"^G[01]$")
RE_ARC = re.compile(r"^G[23]$")


def near(a, b, tol):
    return abs(a - b) <= tol


def seg(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


class ParseError(Exception):
    pass


# --------------------------------------------------------------------------
# Line model
# --------------------------------------------------------------------------

def strip_comment(line):
    i = line.find(";")
    return (line[:i] if i >= 0 else line), (line[i + 1:] if i >= 0 else "")


class L(object):
    """One line: its command, its words, and the modal state AFTER it."""
    __slots__ = ("n", "raw", "cmd", "w", "comment", "x", "y", "z", "f", "tool",
                 "station", "e", "u", "v", "pos_known")

    def __init__(self, n, raw):
        self.n = n
        self.raw = raw
        self.cmd = ""
        self.w = {}
        self.comment = ""
        self.x = self.y = self.z = float("nan")
        self.f = float("nan")
        self.tool = None
        self.station = None
        self.e = self.u = self.v = 0.0
        self.pos_known = False


def tokenize(line):
    body, comment = strip_comment(line)
    toks = body.split()
    cmd = toks[0].upper() if toks else ""
    words = {}
    for t in toks[1:]:
        m = RE_WORD.match(t)
        if m:
            try:
                words[m.group(1).upper()] = float(m.group(2))
            except ValueError:
                pass
    return cmd, words, comment.strip()


def load_lines(path):
    """Parse the file into lines carrying resolved modal state."""
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        raw = fh.read().split("\n")

    out = []
    x = y = z = f = float("nan")
    tool = None
    station = False
    pos_known = False
    unsupported = []

    for i, r in enumerate(raw, start=1):
        cmd, w, cmt = tokenize(r)
        L_ = L(i, r)
        L_.cmd = cmd
        L_.w = w
        L_.comment = cmt

        if RE_MOVE.match(cmd):
            if "F" in w:
                f = w["F"]
            if "X" in w:
                x = w["X"]
            if "Y" in w:
                y = w["Y"]
            if "Z" in w:
                z = w["Z"]
            pos_known = True
            L_.e = w.get("E", 0.0)
            L_.u = w.get("U", 0.0)
            L_.v = w.get("V", 0.0)
        elif RE_ARC.match(cmd):
            unsupported.append((i, cmd + " (arc dialect: this build emits none; "
                                   "an arc here means the parser is incomplete)"))
        elif cmd in STATION_IN:
            station = True
            pos_known = False
        elif cmd in STATION_OUT:
            station = False
            pos_known = False
        elif cmd in OPAQUE_MOTION:
            pos_known = False
        elif len(cmd) > 1 and cmd[0] == "T" and cmd[1:].isdigit():
            tool = int(cmd[1:])
        elif cmd in ("G92",):
            if "X" in w and "Y" in w:
                x, y = w["X"], w["Y"]
                pos_known = True

        if cmd and cmd[0] in "GM" and not (
                RE_MOVE.match(cmd) or RE_ARC.match(cmd) or
                cmd in ("G4", "G10", "G21", "G90", "G91", "G92", "G92.1",
                        "M400", "M104", "M105", "M106", "M107", "M109", "M140",
                        "M141", "M190", "M191", "M73", "M82", "M83", "M1001",
                        "M1002", "M2800", "M2801", "M106") or
                cmd in OPAQUE_MOTION or cmd.startswith("SET_") or
                (cmd[0] == "T" and cmd[1:].isdigit())):
            # Anything the model does not cover is reported, not silently zeroed.
            if cmd not in ("M106", "M107"):
                unsupported.append((i, "unmodelled command " + cmd))

        L_.x, L_.y, L_.z, L_.f = x, y, z, f
        L_.tool = tool
        L_.station = station
        L_.pos_known = pos_known
        out.append(L_)

    return out, unsupported


# --------------------------------------------------------------------------
# The deterministic clock
# --------------------------------------------------------------------------

def block_seconds(lines, a, b):
    """Nominal seconds over lines [a, b) using the spec's clock.

    Translating: 60 * XYZ distance / modal F.
    Stationary extrusion: 60 * max(|E|,|U|,|V|) / F.
    Dwell: its P word. Temperature waits and macros: zero.
    A translating block with no resolvable feed is an error.
    """
    t = 0.0
    # Seed from the modal position the line BEFORE the range established. A
    # temperature command does not move the carriage, so the first motion after
    # it travels real distance from where the machine already was: the spec's
    # clock charges 60*dXYZ/F to every translating block, including the first.
    prev = None
    if a > 0 and math.isfinite(lines[a - 1].x):
        prev = (lines[a - 1].x, lines[a - 1].y, lines[a - 1].z)
    errors = []
    for i in range(a, b):
        c = lines[i]
        if not RE_MOVE.match(c.cmd) and c.cmd != "G4":
            continue
        if c.cmd == "G4":
            t += c.w.get("P", c.w.get("S", 0.0))
            continue
        f = c.f
        if not math.isfinite(f) or f <= 0.0:
            errors.append("line %d: motion block with no resolvable feed" % c.n)
            continue
        cur = (c.x, c.y, c.z)
        if prev is None:
            prev = cur
            continue
        d = math.sqrt((cur[0] - prev[0]) ** 2 + (cur[1] - prev[1]) ** 2 +
                      (cur[2] - prev[2]) ** 2)
        mat = max(abs(c.e), abs(c.u), abs(c.v))
        if d > 1e-9:
            t += 60.0 * d / f
        elif mat > 1e-9:
            t += 60.0 * mat / f
        prev = cur
    return t, errors


# --------------------------------------------------------------------------
# Fibre window reconstruction
# --------------------------------------------------------------------------

class Win(object):
    def __init__(self):
        self.open_ln = self.cut_ln = self.close_ln = None
        self.open_idx = self.cut_idx = self.close_idx = None
        self.v1_ln = self.rel_begin_ln = self.rel_end_ln = None
        self.lift_ln = self.entry_ln = None
        # How this close was classified: "departure" (T0 physically leaves),
        # "interstrand" (next strand still runs on T0) or "terminal" (nothing
        # follows). Set by classify_closes(); None until it runs.
        self.kind = None
        # The first REAL clearance lift after the close: a positive Z rise.
        # A Z word that repeats the current Z is recorded separately, because
        # re-issuing the current height moves nothing and clears nothing.
        self.lift_kind = None        # "rise" | "hop" | None
        self.lift_z_from = self.lift_z_to = self.lift_z_rise = None
        self.lift_z_repeat = None    # line of a Z word that did not rise
        self.entry_after_lift = False
        self.v1_idx = None
        self.next_open_idx = None
        self.budget_L = None
        self.cut_xy = self.dep_end_xy = self.rel_end_xy = (float("nan"),) * 2
        self.dep_len = self.rel_len = self.total_len = 0.0
        self.rel_f = float("nan")
        self.rel_z_dev = 0.0
        self.rel_words = set()
        self.post_cut_u = 0.0
        self.z = float("nan")
        self.is_purge = False
        self.errors = []
        self.deposits = []
        # Payout accounting, measured from emitted coordinates (S11).
        self.tail_v = 0.0
        self.tail_len = 0.0
        self.body_v = 0.0
        self.body_len = 0.0
        self.stationary_v = []   # (line, value) for every V-only move
        # Stationary V issued AFTER the window closes and up to the first lift:
        # this is where the departure tool-change withdrawal lives, so a ledger
        # that stops at M1002 never sees it (S11).
        self.depart_v = []       # (line, value)
        self.leaves_t0 = None    # True when this close is followed by a real T0 exit
        self.footprint_worst = None
        self.footprint_worst_at = None


def find_windows(lines):
    opens = [i for i, c in enumerate(lines) if c.cmd == "M1001"]
    closes = [i for i, c in enumerate(lines) if c.cmd == "M1002"]
    cuts = [i for i, c in enumerate(lines) if c.cmd == "M2800"]
    wins = []
    for k, o in enumerate(opens):
        cl = next((i for i in closes if i > o), None)
        if cl is None:
            raise ParseError("M1001 at line %d is never closed" % lines[o].n)
        cu = next((i for i in cuts if o < i < cl), None)
        w = Win()
        w.open_ln = lines[o].n
        w.close_ln = lines[cl].n
        w.cut_ln = lines[cu].n if cu is not None else None
        m = re.search(r"\bL(\d+)", strip_comment(lines[o].raw)[0])
        w.budget_L = int(m.group(1)) if m else None
        w.z = lines[cu].z if cu is not None else lines[o].z
        # The purge window is the startup sacrificial strand: the FIRST M1001
        # window, laid before any object feature. (No window ever contains a
        # ;TYPE: line, so a comment heuristic cannot distinguish them.)
        w.is_purge = (k == 0)
        wins.append((w, o, cl, cu))
    return wins


def measure_window(lines, w, o, cl, cu, bound_idx=None):
    """Measure one window from emitted coordinates."""
    # Line indices the thermal checks need, recorded before any early return so a
    # malformed window still has an identity in the report.
    w.open_idx = o
    w.close_idx = cl
    w.cut_idx = cu
    w.next_open_idx = bound_idx
    if cu is None:
        w.errors.append("FS_WINDOW_NO_CUT: window opened at line %d has no M2800" % w.open_ln)
        return

    # Position AT the cut: the endpoint of the last joint (U-bearing) deposit
    # before M2800. The blade fires mid-segment, so the first tail move's own
    # length only counts if the cut position is seeded from that joint.
    cut_xy = None
    for i in range(cu - 1, o, -1):
        c = lines[i]
        if RE_MOVE.match(c.cmd) and c.u > 0 and math.isfinite(c.x) and math.isfinite(c.y):
            cut_xy = (c.x, c.y)
            break
    if cut_xy is None:
        w.errors.append("FS_CUT_POSITION_UNKNOWN: no joint deposit before M2800 at line %d" % w.cut_ln)
        return
    w.cut_xy = cut_xy

    # V-1: the first stationary negative-V move after the cut.
    v1 = None
    for i in range(cu + 1, cl + 1):
        c = lines[i]
        if RE_MOVE.match(c.cmd) and c.v < 0 and not (math.isfinite(c.x) and "X" in c.w):
            v1 = i
            break
        if RE_MOVE.match(c.cmd) and c.v < 0 and "V" in c.w and "X" not in c.w and "Y" not in c.w:
            v1 = i
            break
    if v1 is None:
        w.errors.append("FS_NO_TAIL_RETRACT: no V-1 between M2800 (line %d) and M1002 (line %d)"
                        % (w.cut_ln, w.close_ln))
        return
    w.v1_ln = lines[v1].n
    w.v1_idx = v1

    # Deposition phase: cut -> V-1, every translating block, including the
    # rounded zero-V blocks the tail carries.
    pos = cut_xy
    dep = 0.0
    for i in range(cu + 1, v1 + 1):
        c = lines[i]
        if not RE_MOVE.match(c.cmd):
            continue
        if "X" in c.w and "Y" in c.w and math.isfinite(c.x) and math.isfinite(c.y):
            nxt = (c.x, c.y)
            dep += seg(pos, nxt)
            pos = nxt
        w.deposits.append((pos[0], pos[1], c.z))
    w.dep_len = dep
    w.dep_end_xy = pos

    # Tail matrix payout: the positive V the post-cut moves carry, over the path
    # they travel. This is the quantity the payout formula governs, so it is
    # measured once here and judged by S11 rather than re-derived there.
    tv = 0.0
    tl = 0.0
    tpos = cut_xy
    for i in range(cu + 1, v1):
        c = lines[i]
        if not RE_MOVE.match(c.cmd):
            continue
        if "X" in c.w and "Y" in c.w and math.isfinite(c.x) and math.isfinite(c.y):
            nxt = (c.x, c.y)
            tl += seg(tpos, nxt)
            tpos = nxt
            tv += max(0.0, c.v)
    w.tail_v = tv
    w.tail_len = tl

    # Body payout: the matrix the pre-cut joint deposits carry over the same kind
    # of path. The tail factor is a fraction of THIS rate, so S11 needs both.
    bv = 0.0
    bl = 0.0
    bpos = None
    for i in range(o + 1, cu):
        c = lines[i]
        if not RE_MOVE.match(c.cmd):
            continue
        if "X" in c.w and "Y" in c.w and math.isfinite(c.x) and math.isfinite(c.y):
            nxt = (c.x, c.y)
            if bpos is not None:
                bl += seg(bpos, nxt)
            bpos = nxt
            bv += max(0.0, c.v)
    w.body_v = bv
    w.body_len = bl

    # Stationary V inventory for this window: every V-only move, whatever it is
    # commented. S11 classifies by VALUE, so a comment cannot launder a payout.
    for i in range(o, cl + 1):
        c = lines[i]
        if RE_MOVE.match(c.cmd) and c.v != 0.0 and "X" not in c.w and "Y" not in c.w:
            w.stationary_v.append((c.n, c.v))

    # Release: the no-extrusion path AFTER V-1 and BEFORE M1002 / any lift.
    rb = next((i for i in range(v1, cl + 1) if lines[i].comment.startswith("FS_RELEASE_BEGIN")), None)
    re_ = next((i for i in range(v1, cl + 1) if lines[i].comment.startswith("FS_RELEASE_END")), None)
    w.rel_begin_ln = lines[rb].n if rb is not None else None
    w.rel_end_ln = lines[re_].n if re_ is not None else None

    # Measure the release from COMMANDS, not from the comment. Scan V-1..M1002
    # for translating blocks that carry no E/U/V; stop at the first Z move.
    rel = 0.0
    pos = w.dep_end_xy
    rel_z = []
    words_seen = set()
    rel_f = float("nan")
    started = False
    for i in range(v1 + 1, cl):
        c = lines[i]
        if not RE_MOVE.match(c.cmd):
            continue
        if "Z" in c.w:
            break                      # the lift ends the release window
        if "X" not in c.w or "Y" not in c.w:
            continue                   # stationary move: not part of the path
        for ax in ("E", "U", "V"):
            if ax in c.w:
                words_seen.add(ax)
        nxt = (c.x, c.y)
        rel += seg(pos, nxt)
        pos = nxt
        started = True
        if "F" in c.w:
            rel_f = c.w["F"]
        rel_z.append(c.z)
    w.rel_len = rel
    w.rel_words = words_seen
    w.rel_f = rel_f
    w.rel_end_xy = pos
    if rel_z:
        w.rel_z_dev = max(abs(rz - w.z) for rz in rel_z)

    # Post-cut U drive: any fibre payout from the cut through the close.
    su = 0.0
    for i in range(cu + 1, cl + 1):
        su += lines[i].u
    w.post_cut_u = su

    # First lift after the close, and the first station entry after that.
    # Between the close and that lift sits the departure withdrawal, so collect
    # the stationary V moves on the way. Stopping at the lift is the point: the
    # previous revision of the S11 ledger ended at M1002 and therefore never
    # judged the departure amount at all.
    #
    # The scan also stops at the next window opening (bound_idx). A close that
    # does not depart is followed by another window rather than by a station
    # visit, so an unbounded scan would read that window's own recovery/prime
    # stationary V, and even its later departure withdrawal, as if they belonged
    # to this close.
    scan_end = bound_idx if bound_idx is not None else len(lines)
    z_at = lines[cl].z
    lift_i = None
    for i in range(cl, scan_end):
        c = lines[i]
        if lift_i is None and RE_MOVE.match(c.cmd) and "Z" in c.w:
            to = c.w["Z"]
            # A lift is a DISPLACEMENT, not the presence of a Z word. Re-issuing
            # the height the carriage is already at satisfies nothing, so it is
            # recorded and skipped rather than accepted as the clearance move.
            if math.isfinite(z_at) and math.isfinite(to) and to <= z_at + 1e-9:
                if w.lift_z_repeat is None:
                    w.lift_z_repeat = c.n
                continue
            lift_i = i
            w.lift_ln = c.n
            w.lift_kind = "rise"
            w.lift_z_from = z_at
            w.lift_z_to = to
            w.lift_z_rise = to - z_at
        if w.entry_ln is None and c.cmd in STATION_IN:
            w.entry_ln = c.n
        if lift_i is not None and w.entry_ln is not None:
            break
    w.entry_after_lift = (w.entry_ln is not None and w.lift_ln is not None
                          and w.entry_ln > w.lift_ln)
    if w.lift_kind == "rise":
        w.lift_kind = "hop" if w.entry_ln is None else "rise"
    # Stationary V between the close and the departure lift, or, when this close
    # does not lift at all (an inter-strand close is followed by the next window,
    # not by a station visit), up to that next window opening. Collecting only
    # after a lift was found would leave a spurious withdrawal at an
    # inter-strand close invisible.
    stop_i = lift_i if lift_i is not None else scan_end
    for j in range(cl + 1, stop_i):
        cc = lines[j]
        if (RE_MOVE.match(cc.cmd) and cc.v != 0.0
                and "X" not in cc.w and "Y" not in cc.w):
            w.depart_v.append((cc.n, cc.v))

    w.total_len = w.dep_len + w.rel_len
    return w


# --------------------------------------------------------------------------
# Deposited-material mask (S13): independent reconstruction from coordinates
# --------------------------------------------------------------------------

def layer_deposits(lines, z, fiber_width=0.7, plastic_width=0.4):
    """Every deposited segment at physical Z, in file order, with its half-width.

    A deposit is a translating move that carries material on either channel:
    V>0 (matrix, the composite head) or E>0 (plastic). Fibre payout U is not
    itself a deposition axis - matrix V is what forms the bead - but a joint
    move carries both, so V>0 is the fibre-side test.

    Each segment carries the half-width of the bead that ACTUALLY formed it, not
    one inherited width for the whole layer. Using the wider fibre width for
    plastic segments too would invent support that was never deposited, which is
    the failure mode this whole function exists to catch.
    """
    out = []
    prev = None
    for c in lines:
        if not RE_MOVE.match(c.cmd):
            continue
        if not (math.isfinite(c.x) and math.isfinite(c.y)):
            prev = None
            continue
        cur = (c.x, c.y)
        if prev is not None and near(c.z, z, 1e-6) and seg(prev, cur) > 1e-9:
            if c.v > 0.0:
                out.append((prev, cur, c.n, fiber_width / 2.0))
            elif c.e > 0.0:
                out.append((prev, cur, c.n, plastic_width / 2.0))
        prev = cur
    return out


def dist_pt_seg(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def release_supported(lines, w, half_width, before_ln, plastic_width=0.4):
    """Is the release FOOTPRINT covered by material deposited earlier in the
    SAME physical layer? Reconstructed from emitted coordinates, not from an
    emitter boolean.

    The footprint is the release centreline buffered by half the actual composite
    bead width, so containment is judged on the whole swept strip. Judging the
    centreline alone would pass a release sitting on the edge of a supporting
    bead with half its width over nothing.
    """
    deps = [d for d in layer_deposits(lines, w.z, half_width * 2.0, plastic_width)
            if d[2] < before_ln]
    if not deps:
        return False, "no deposited material at Z %.2f before line %d" % (w.z, before_ln)
    return _contain_check(lines, w, deps, half_width)


def _contain_check(lines, w, deps, half_width):
    # Rebuild the release polyline from the emitted commands, not the comments.
    pts = []
    pos = w.dep_end_xy
    for c in lines:
        if c.n <= w.v1_ln or c.n >= w.close_ln:
            continue
        if not RE_MOVE.match(c.cmd):
            continue
        if "Z" in c.w:
            break
        if "X" not in c.w or "Y" not in c.w:
            continue
        pts.append(((pos[0], pos[1]), (c.x, c.y)))
        pos = (c.x, c.y)
    if not pts:
        return True, "no release to check"
    # Sample the swept strip: along the path at CONTAIN_PITCH, and across it at
    # CONTAIN_LAT samples spanning -half_width..+half_width. Every sample must
    # lie within TOL_CONTAIN of some deposited segment inflated by that
    # segment's own half-width.
    worst = 0.0
    worst_at = None
    for a, b in pts:
        L = seg(a, b)
        if L <= 1e-9:
            continue
        ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
        nx, ny = -uy, ux
        n_along = max(2, int(L / CONTAIN_PITCH) + 1)
        for si in range(n_along + 1):
            t = si / float(n_along)
            px, py = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
            for li in range(CONTAIN_LAT + 1):
                off = -half_width + (2.0 * half_width) * (li / float(CONTAIN_LAT))
                q = (px + nx * off, py + ny * off)
                esc = min(dist_pt_seg(q, s[0], s[1]) - s[3] for s in deps)
                if esc > worst:
                    worst = esc
                    worst_at = (q, esc)
    w.footprint_worst = worst
    w.footprint_worst_at = worst_at[0] if worst_at else None
    if worst > TOL_CONTAIN:
        where = ""
        if worst_at:
            where = " at (%.2f, %.2f)" % (worst_at[0][0], worst_at[0][1])
        return False, ("release footprint escapes material already deposited in "
                       "this layer by %.3f mm%s (bead half-width %.3f mm, "
                       "tolerance %.2f mm)" % (worst, where, half_width, TOL_CONTAIN))
    return True, ("full release footprint (bead width %.2f mm) inside material "
                  "deposited earlier in the same layer; worst escape %.3f mm"
                  % (half_width * 2.0, worst))



def purge_corridor_ok(w, spec, half_width):
    """The startup purge is the spec's ONE exception to same-layer containment:
    it is sacrificial material on bare bed, so what legitimises its forward
    extension is a validated bed-contact corridor, not earlier deposition."""
    # Corridor: the band the purge line sweeps, extended forward by the release.
    # Emitted purge deposits along Y10 from X(from) to X90.00 and releases to
    # X96.800; the spec pins those numbers.
    if not (near(w.dep_end_xy[0], 90.0, 0.01) and near(w.dep_end_xy[1], 10.0, 0.01)):
        return False, "purge deposition ends at %s, expected X90.00 Y10.00" % (
            "%.2f,%.2f" % w.dep_end_xy,)
    if spec["release"] > 0.0:
        if not near(w.rel_end_xy[0], 90.0 + spec["release"], 0.05):
            return False, "purge release ends at X%.3f, expected X%.3f" % (
                w.rel_end_xy[0], 90.0 + spec["release"])
    if not near(w.rel_end_xy[1], 10.0, 0.01):
        return False, "purge release leaves the corridor in Y (%.3f)" % w.rel_end_xy[1]
    return True, "purge release inside the validated bed-contact corridor"


# --------------------------------------------------------------------------
# Thermal / transition extraction
# --------------------------------------------------------------------------

def find_transitions(lines):
    """Every physical head change, with the line numbers the report needs."""
    out = []
    n = len(lines)
    for i, c in enumerate(lines):
        if not (len(c.cmd) > 1 and c.cmd[0] == "T" and c.cmd[1:].isdigit()):
            continue
        to = int(c.cmd[1:])
        frm = None
        for j in range(i - 1, -1, -1):
            cc = lines[j]
            if len(cc.cmd) > 1 and cc.cmd[0] == "T" and cc.cmd[1:].isdigit():
                frm = int(cc.cmd[1:])
                break
        if frm is None or frm == to:
            continue
        out.append(dict(idx=i, frm=frm, to=to, switch_ln=c.n))
    return out


def classify_closes(lines, wins, trans, want_dep, tol):
    """Classify every fibre-window close as departure / interstrand / terminal.

    Being on T0 when a window closes does not establish that T0 is departing.
    An inter-strand close -- the next strand still runs on T0 -- also happens
    while T0 is selected, so a rule of the form "head at close == 0 therefore
    owe a withdrawal" wrongly demands one there. The rule looks forward
    instead: to the next fibre window and to the next physical head change.

The three kinds are decided by comparing the two things that can follow a close
-- the next window opening and the next physical head change -- not by looking
at which head is selected:

      - closes on a head other than T0: T0 is not the outgoing head, nothing owed
      - NEITHER a later window NOR a later head change: terminal. Shutdown
        follows; there is no incoming head to prepare for.
      - the next head change comes first, or there is no later window but there
        IS a later head change: departure. T0 hands over before the next strand
        (or before a non-depositing tail) and owes exactly one withdrawal.
      - a later window exists and no head change precedes it: inter-strand. T0
        stays active across the gap even if the head changes again much later,
        so nothing is owed here.

Comparing the two forward markers is what makes "no later head change" mean
terminal rather than inter-strand: the old rule treated a missing head change as
proof that T0 never leaves, which mislabelled the last strand of a file that
ends on T0 with no hand-off, and mislabelled a departing activation that has no
following window.

    Deriving this from physical head changes rather than from the window list is
    also what makes several strands inside one T0 activation correct: they are
    one activation, so only the last of them departs.

    The same classification serves S06 and S11. S06 must NOT infer "inter-strand"
    from "no withdrawal owed", because a terminal close also owes nothing yet is
    not an inter-strand move: it is followed by shutdown, not by another strand.

    Returns one row per window, in window order.
    """
    rows = []
    n = len(wins)
    for k, w in enumerate(wins):
        head = head_at(lines, w.close_idx)
        nxt_idx = wins[k + 1].open_idx if k + 1 < n else None
        nxt_ln = lines[nxt_idx].n if nxt_idx is not None else None
        chg = next((t for t in trans if t["idx"] > w.close_idx), None)
        if head != 0:
            # T0 is not the outgoing head at all, so it owes nothing here.
            kind = "interstrand"
            basis = "closes while T%s is selected, so T0 is not the outgoing head" % head
            nxt_chg_ln, nxt_chg = nxt_ln, ""
        elif nxt_idx is None and chg is None:
            # Nothing follows this close: neither another strand nor a hand-off.
            kind = "terminal"
            basis = ("no later window opening and no later physical head change, "
                     "so this close is followed by shutdown rather than by another "
                     "strand or an incoming head")
            nxt_chg_ln, nxt_chg = None, ""
        elif chg is not None and (nxt_idx is None or chg["idx"] < nxt_idx):
            # The head change comes first -- including when there is no later
            # window at all, which is a departure into a non-depositing tail.
            kind = "departure"
            nxt_chg_ln, nxt_chg = chg["switch_ln"], (
                " (next change T%d->T%d)" % (chg["frm"], chg["to"]))
            basis = ("next physical head change T%d->T%d at line %d precedes the "
                     "next window opening%s, so T0 hands over before the next strand"
                     % (chg["frm"], chg["to"], chg["switch_ln"],
                        "" if nxt_ln is None else " at line %d" % nxt_ln))
        else:
            # A later window exists and no head change precedes it, so T0 stays
            # active across the gap even if the head changes again much later.
            kind = "interstrand"
            nxt_chg_ln, nxt_chg = chg["switch_ln"] if chg else None, (
                "" if chg is None else
                " (next change T%d->T%d at line %d)"
                % (chg["frm"], chg["to"], chg["switch_ln"]))
            basis = ("next window opens at line %d before any head change%s, so T0 "
                     "stays active between strands" % (nxt_ln, nxt_chg))
        hits = [ln for ln, val in w.depart_v if near(val, want_dep, tol)]
        w.kind = kind
        rows.append(dict(window=k + 1, head=head, kind=kind, basis=basis,
                         expected=(kind == "departure"), lines=hits,
                         next_open_ln=nxt_ln,
                         # next_change_ln is the head change the decision was
                         # actually made against: the one that comes first.
                         change_ln=nxt_chg_ln, next_change_ln=nxt_chg_ln,
                         next_change=nxt_chg,
                         values=[round(v, 3) for _, v in w.depart_v]))
    return rows


def clearance_mm(spec):
    """Required departure Z clearance, from the effective config when present."""
    v = (spec or {}).get("fs_toolchange_lift_z")
    try:
        v = float(v)
    except (TypeError, ValueError):
        return DEPARTURE_CLEARANCE_MM
    return v if v > 0.0 else DEPARTURE_CLEARANCE_MM


def head_at(lines, idx):
    """Tool selected at a line index, from the last T command at or before it."""
    for j in range(idx, -1, -1):
        c = lines[j]
        if len(c.cmd) > 1 and c.cmd[0] == "T" and c.cmd[1:].isdigit():
            return int(c.cmd[1:])
    return None


def station_brackets(lines):
    """(entry_line, exit_line) pairs from the actual station macros."""
    pairs = []
    open_at = None
    for c in lines:
        if c.cmd in STATION_IN:
            if open_at is None:
                open_at = c.n
        elif c.cmd in STATION_OUT:
            if open_at is not None:
                pairs.append((open_at, c.n))
                open_at = None
    return pairs


def wait_obligations(lines, brackets, spec):
    """Blocking-wait obligations, one per managed activation.

    Derived from PHYSICAL head changes, never from a file-wide tally. Each head
    selection - startup included - must be preceded by a station visit that
    waits for the incoming head at its active temperature, and the visit that
    satisfies an obligation must be the one immediately before the activation.
    Two waits at startup therefore cannot discharge a missing wait later, and
    several fibre windows inside one T0 activation correctly share a single
    obligation because there is only one physical change into T0.
    """
    want = {0: spec.get("fs_t0_temp"), 1: spec.get("fs_t1_temp")}
    sw = find_transitions(lines)
    # The startup head is the FIRST T command in the file. find_transitions()
    # skips it because it has no predecessor to compare against, so taking the
    # startup head from sw[0] would describe the first CHANGE rather than the
    # first ACTIVATION and leave the vendor startup wait with no obligation at
    # all - i.e. deleting it would pass.
    first = None
    for c in lines:
        if len(c.cmd) > 1 and c.cmd[0] == "T" and c.cmd[1:].isdigit():
            first = dict(head=int(c.cmd[1:]), ln=c.n)
            break
    acts = []
    if first is not None:
        acts.append(dict(kind="startup", head=first["head"], ln=first["ln"]))
    # Every recorded change is its own obligation, including sw[0]: the startup
    # activation above is the first T command, not the first change, so sw[0]
    # (the first physical head change) is a separate activation needing its own
    # wait.
    for k in range(len(sw)):
        acts.append(dict(kind="switch", head=sw[k]["to"], ln=sw[k]["switch_ln"],
                         frm=(sw[k - 1]["to"] if k > 0 else first["head"])))
    hot = [c for c in lines if c.cmd == "M109"]

    def serving_bracket(ln, head, want_s):
        """The station visit that must carry this activation's wait.

        Preference order: the last visit that CLOSES before the activation, then
        the first visit that OPENS after it. Backward-first is the normal case -
        the head is waited for at the station and only then selected. Forward is
        needed only for the startup head, which the vendor selects before any
        station visit exists. A bracket serves at most one obligation, so a wait
        cannot be reused across activations.
        """
        back = [b for b in brackets if b[1] < ln]
        fwd = [b for b in brackets if b[0] > ln]
        cands = ([back[-1]] if back else []) + ([fwd[0]] if fwd else [])
        for b in cands:
            if b in claimed:
                continue
            for c in hot:
                if (b[0] <= c.n <= b[1] and want_s is not None
                        and int(c.w.get("T", -1)) == head
                        and c.w.get("S") is not None
                        and abs(c.w["S"] - want_s) < 0.5):
                    return b, c
        return (cands[0] if cands else None), None

    out = []
    claimed = set()
    for a in acts:
        o = dict(a)
        o["want_s"] = want.get(a["head"])
        b, c = serving_bracket(a["ln"], a["head"], o["want_s"])
        o["bracket"] = b
        o["wait"] = c
        if b is not None and c is not None:
            claimed.add(b)
        out.append(o)
    return out


def thermal_events(lines, window_start, window_end):
    """Standby / preheat / wait lines for a slice of the file."""
    ev = dict(standby={}, preheat={}, wait={}, clean_ln=None, entry_ln=None,
              exit_ln=None)
    for i in range(window_start, window_end):
        c = lines[i]
        if c.cmd in STATION_IN:
            ev["entry_ln"] = c.n
        elif c.cmd in STATION_OUT:
            ev["exit_ln"] = c.n
        elif c.cmd == "CLEAN_NOZZLE":
            ev["clean_ln"] = c.n
        elif c.cmd == "M104":
            t = int(c.w.get("T", -1))
            s = c.w.get("S", None)
            if s is None:
                continue
            if "FS_PREHEAT" in c.comment:
                ev["preheat"][t] = (c.n, s)
            else:
                ev["standby"].setdefault(t, []).append((c.n, s))
        elif c.cmd == "M109":
            t = int(c.w.get("T", -1))
            ev["wait"][t] = (c.n, c.w.get("S", None))
    return ev


# --------------------------------------------------------------------------
# Check result plumbing
# --------------------------------------------------------------------------

class Check(object):
    def __init__(self, cid, title):
        self.id = cid
        self.title = title
        self.status = NE
        self.detail = []
        self.codes = []
        self.measured = None
        self.expected = None

    def ok(self, msg):
        self.status = PASS
        self.detail.append(msg)

    def bad(self, msg, code=None):
        self.status = FAIL
        self.detail.append(msg)
        if code:
            self.codes.append(code)

    def note(self, msg):
        self.detail.append(msg)

    def to_json(self):
        return dict(id=self.id, title=self.title, status=self.status,
                    measured=self.measured, expected=self.expected,
                    codes=sorted(set(self.codes)),
                    detail=self.detail[:40])


def deposition_zs(lines):
    """Physical Z values at which material is actually deposited.

    Counted from extrusion, not from Z-hops or layer comments: a Z-hop over a
    gap carries no material and must not be counted as a layer.
    """
    zs = {}
    prev = None
    for c in lines:
        if not RE_MOVE.match(c.cmd):
            continue
        if not (math.isfinite(c.x) and math.isfinite(c.y)):
            prev = None
            continue
        cur = (c.x, c.y)
        if prev is not None and seg(prev, cur) > 1e-9:
            if c.v > 0.0 or c.e > 0.0:
                zs[round(c.z, 3)] = zs.get(round(c.z, 3), 0) + 1
        prev = cur
    return sorted(zs)


# --------------------------------------------------------------------------
# Per-file evaluation
# --------------------------------------------------------------------------

def recovered_at_dest(i, lines):
    """True when the modal position at the recovery line i equals the position
    the next deposition move starts from. A recovery emitted BEFORE destination
    travel fails this: the travel between them moves the nozzle, so the recovery
    happened somewhere other than the deposition start."""
    c = lines[i]
    if not (math.isfinite(c.x) and math.isfinite(c.y)):
        return False
    for j in range(i + 1, min(i + 60, len(lines))):
        cc = lines[j]
        if cc.cmd in STATION_IN:
            return False
        if RE_MOVE.match(cc.cmd) and cc.e > 1e-6 and "X" in cc.w and "Y" in cc.w:
            # Position just before this deposition is the previous line's modal state.
            prev = lines[j - 1]
            if not (math.isfinite(prev.x) and math.isfinite(prev.y)):
                return False
            return seg((c.x, c.y), (prev.x, prev.y)) <= 0.05
    return False


def visit_for_switch(switch_ln, brackets, slack=12):
    """The station visit that governs a tool switch.

    The emitted order is MOVE_TO_BRUSH_STATION ... MOVE_OUT_BRUSH_STATION, then
    the fan clause and the mode resets, then the bare T word. So the switch line
    sits just AFTER the bracket, not inside it: match the last bracket that ends
    before the switch, within a small slack for those intervening lines. A switch
    with no such visit is a real transition with no station visit, which the
    caller must not silently skip.
    """
    cands = [(a, b) for a, b in brackets if a < switch_ln <= b + slack]
    if not cands:
        return None
    return max(cands, key=lambda ab: ab[1])


def scheduled_preheat(lines, in_head, prev_switch_ln, switch_ln):
    """The most recent marked nonblocking target command for the incoming head
    inside the current activation, i.e. after the previous physical switch and
    at or before this one. Returns (line_number, S) or None.

    The scheduler splices this line into the outgoing material, so it sits
    BEFORE the station visit; scanning only the visit would never find it.
    """
    found = None
    for i in range(max(0, prev_switch_ln), min(len(lines), switch_ln + 1)):
        c = lines[i]
        if c.cmd != "M104" or "FS_PREHEAT" not in c.comment:
            continue
        if int(c.w.get("T", -1)) != in_head:
            continue
        found = (c.n, c.w.get("S"))
    return found


def line_index(lines, ln):
    """Index of the 1-based line number ln (lines are dense, so ln-1)."""
    return ln - 1


def evaluate_file(variant, path, spec, evidence, macro_status):
    """Run every per-file check against one emitted G-code file."""
    res = dict(variant=variant, gcode=path, windows=[], transitions=[],
               checks={}, errors=[])
    if not os.path.isfile(path):
        c = Check("S00", "export present")
        c.bad("file missing: " + path)
        res["checks"]["S00"] = c
        for m in MANDATORY:
            res["checks"].setdefault(m, Check(m, "not evaluated: export missing"))
        return res

    try:
        lines, unsupported = load_lines(path)
    except ParseError as e:
        c = Check("S00", "parse")
        c.bad("unsupported parse: %s" % e)
        res["checks"]["S00"] = c
        for m in MANDATORY:
            res["checks"].setdefault(m, Check(m, "not evaluated: parse failed"))
        return res

    res["lines"] = len(lines)
    res["unsupported"] = unsupported[:20]
    raw = open(path, "rb").read()
    res["sha256"] = hashlib.sha256(raw).hexdigest()

    raw_lines = raw.decode("utf-8", "replace").split("\n")
    wins = []
    _all_wins = find_windows(lines)
    for k, (w, o, cl, cu) in enumerate(_all_wins):
        nxt = _all_wins[k + 1][1] if k + 1 < len(_all_wins) else None
        measure_window(lines, w, o, cl, cu, bound_idx=nxt)
        wins.append(w)
    res["windows"] = wins

    # One forward-looking classification of every close, computed ONCE and
    # shared by S06 (what the exit sequence owes) and S11 (what the payout owes).
    # Two independent classifications would eventually disagree about whether a
    # given close was a departure.
    trans = find_transitions(lines)
    # The configured departure withdrawal. Absent from the effective config, the
    # registered default 4.0 mm is what the emitter emits and therefore what is
    # required; S11 records that substitution rather than silently trusting it.
    tcv = spec.get("fs_toolchange_retract_v")
    want_dep = -float(tcv if tcv not in (None, "") else 4.0)
    res["close_kinds"] = classify_closes(lines, wins, trans, want_dep,
                                         TOL_STATIONARY_V)

    half_width = spec.get("bead_width_mm", 0.7) / 2.0

    # ---- S01 same-project structure ------------------------------------
    s01 = Check("S01", "same-project structure")
    zs = deposition_zs(lines)
    model_zs = [z for z in zs if z > EXPECTED_Z_FIRST + 1e-9]
    n_layers = len(model_zs) + (1 if zs and near(zs[0], EXPECTED_Z_FIRST, 1e-6) else 0)
    ncuts = sum(1 for c in lines if c.cmd == "M2800")
    ncloses = sum(1 for c in lines if c.cmd == "M1002")
    nopens = sum(1 for c in lines if c.cmd == "M1001")
    npurge = sum(1 for w in wins if w.is_purge)
    nmodel = len(wins) - npurge
    s01.note("deposition layers=%d (expected %d); Z %.2f..%.2f step %.2f"
             % (n_layers, EXPECTED_LAYERS, zs[0] if zs else float("nan"),
                zs[-1] if zs else float("nan"), EXPECTED_Z_STEP))
    s01.note("windows=%d (purge %d + model %d); cuts=%d closes=%d"
             % (len(wins), npurge, nmodel, ncuts, ncloses))
    zstep_ok = all(near(model_zs[i] - model_zs[i - 1], EXPECTED_Z_STEP, 0.001)
                   for i in range(1, len(model_zs)))
    if (n_layers == EXPECTED_LAYERS and len(wins) == EXPECTED_WINDOWS and
            npurge == 1 and nmodel == 15 and ncuts == EXPECTED_CUTS and
            ncloses == EXPECTED_CLOSES and nopens == EXPECTED_CLOSES and
            zs and near(zs[0], EXPECTED_Z_FIRST, 1e-6) and
            near(zs[-1], EXPECTED_Z_LAST, 1e-6) and zstep_ok):
        s01.ok("structure matches the S-hook baseline")
    else:
        s01.bad("structure differs: layers %d/%d, windows %d/%d, cuts %d/%d, "
                "closes %d/%d, first Z %.2f, last Z %.2f, uniform %.2f step: %s"
                % (n_layers, EXPECTED_LAYERS, len(wins), EXPECTED_WINDOWS,
                   ncuts, EXPECTED_CUTS, ncloses, EXPECTED_CLOSES,
                   zs[0] if zs else -1, zs[-1] if zs else -1, EXPECTED_Z_STEP,
                   zstep_ok))
    res["checks"]["S01"] = s01

    # ---- S02 tail deposition ------------------------------------------
    s02 = Check("S02", "tail deposition (post-cut)")
    bad = []
    for k, w in enumerate(wins, start=1):
        if not near(w.dep_len, spec["dep"], TOL_PATH):
            bad.append("W%d L%d measured %.3f mm, expected %.1f +-%.2f"
                       % (k, w.cut_ln or 0, w.dep_len, spec["dep"], TOL_PATH))
    s02.measured = [round(w.dep_len, 3) for w in wins]
    s02.expected = spec["dep"]
    if bad:
        s02.bad("%d of %d windows off spec: %s" % (len(bad), len(wins), bad[0]),
                code="FS_TAIL_DISTANCE_MISMATCH")
        for b in bad[1:6]:
            s02.note(b)
    else:
        s02.ok("all %d windows within %.1f +-%.2f mm (min %.3f max %.3f)"
               % (len(wins), spec["dep"], TOL_PATH,
                  min(w.dep_len for w in wins), max(w.dep_len for w in wins)))
    res["checks"]["S02"] = s02

    # ---- S03 actual release -------------------------------------------
    s03 = Check("S03", "actual dry release")
    bad = []
    codes3 = []
    for k, w in enumerate(wins, start=1):
        if not near(w.rel_len, spec["rel"], TOL_PATH):
            bad.append("W%d L%d release %.3f mm, expected %.1f +-%.2f"
                       % (k, w.v1_ln or 0, w.rel_len, spec["rel"], TOL_PATH))
            if spec["rel"] > 0.0 and w.rel_len < 0.05:
                codes3.append("FS_RELEASE_NOT_EXECUTED")
            else:
                codes3.append("FS_TAIL_DISTANCE_MISMATCH")
    s03.measured = [round(w.rel_len, 3) for w in wins]
    s03.expected = spec["rel"]
    if bad:
        s03.bad("%d of %d windows off spec: %s" % (len(bad), len(wins), bad[0]),
                code=codes3[0])
        for cd in sorted(set(codes3[1:])):
            if cd not in s03.codes:
                s03.codes.append(cd)
        for b in bad[1:6]:
            s03.note(b)
    elif spec["rel"] == 0.0:
        s03.ok("all %d windows carry zero release (variant A: intentional)" % len(wins))
    else:
        s03.ok("all %d windows carry a measured %.1f mm dry release between V-1 "
               "and M1002/lift (min %.3f max %.3f)"
               % (len(wins), spec["rel"],
                  min(w.rel_len for w in wins), max(w.rel_len for w in wins)))
    res["checks"]["S03"] = s03

    # ---- S04 release motion -------------------------------------------
    s04 = Check("S04", "release motion legality")
    if spec["rel"] == 0.0:
        s04.ok("no release in variant A; nothing to legalise")
    else:
        bad = []
        codes4 = []
        for k, w in enumerate(wins, start=1):
            if w.rel_words:
                bad.append("W%d release carries %s" % (k, sorted(w.rel_words)))
                codes4.append("FS_RELEASE_EXTRUSION")
            if not near(w.rel_f, 600.0, 1.0):
                bad.append("W%d release F%.0f, expected F600" % (k, w.rel_f))
            if w.rel_z_dev > TOL_RELEASE_Z:
                bad.append("W%d release Z deviates %.4f mm" % (k, w.rel_z_dev))
            if w.rel_len > 0 and not near(w.rel_z_dev, 0.0, TOL_RELEASE_Z):
                bad.append("W%d release left strand Z by %.4f mm" % (k, w.rel_z_dev))
        if bad:
            s04.bad("; ".join(bad[:4]),
                    code=(codes4[0] if codes4 else "FS_RELEASE_EXTRUSION"))
            for cd in sorted(set(codes4)):
                if cd not in s04.codes:
                    s04.codes.append(cd)
        else:
            s04.ok("all releases use F600, hold strand Z within %.3f mm, and carry "
                   "no E/U/V word" % TOL_RELEASE_Z)
    res["checks"]["S04"] = s04

    # ---- S05 no post-cut fibre drive ----------------------------------
    s05 = Check("S05", "no post-cut fibre drive")
    bad = [w for w in wins if abs(w.post_cut_u) > 1e-6]
    if bad:
        s05.bad("%d windows drive U after the cut: %s"
                % (len(bad), ["W? +%.3f" % w.post_cut_u for w in bad[:4]]),
                code="FS_POSTCUT_FIBRE_DRIVE")
    else:
        s05.ok("no U feed or withdrawal from cut through window close in any window")
    res["checks"]["S05"] = s05

    # ---- S06 end sequence ---------------------------------------------
    # One rule for every strand, then a rule per close kind. The kind comes from
    # the SAME forward-looking classification S11 uses (res["close_kinds"]), so
    # the two checks can never disagree about what a close was.
    s06 = Check("S06", "window exit sequence, per close kind")
    want_clear = clearance_mm(spec)
    bad = []
    n_dep = n_int = n_term = 0
    for k, w in enumerate(wins, start=1):
        kind = w.kind or "departure"
        n_dep += kind == "departure"
        n_int += kind == "interstrand"
        n_term += kind == "terminal"

        # --- every strand, regardless of kind --------------------------
        # Final deposition -> V-1 -> configured release AT PRINTING Z -> M1002,
        # with no lift and no departure in the middle of it.
        if w.v1_ln is None or w.close_ln is None:
            bad.append("W%d (%s) missing V-1 or M1002" % (k, kind))
            continue
        if not (w.v1_ln < w.close_ln):
            bad.append("W%d (%s) order V-1@%s M1002@%s"
                       % (k, kind, w.v1_ln, w.close_ln))
            continue
        if w.rel_len > 0.0:
            if w.rel_begin_ln is None or w.rel_end_ln is None:
                bad.append("W%d (%s) releases %.3f mm but has no release block "
                           "between V-1@%d and M1002@%d"
                           % (k, kind, w.rel_len, w.v1_ln, w.close_ln))
                continue
            if not (w.v1_ln < w.rel_begin_ln < w.rel_end_ln < w.close_ln):
                bad.append("W%d (%s) release lines %s..%s not between V-1@%d and "
                           "M1002@%d"
                           % (k, kind, w.rel_begin_ln, w.rel_end_ln, w.v1_ln,
                              w.close_ln))
                continue
        # Nothing may leave the printing height before the window closes. A Z
        # move inside V-1..M1002 truncates the release scan, so a release that
        # was asked for would silently not have been executed.
        early = [c.n for c in lines[w.v1_idx:w.close_idx]
                 if RE_MOVE.match(c.cmd) and "Z" in c.w]
        if early:
            bad.append("W%d (%s) lifts at line %d BEFORE the window closes at line "
                       "%d, so the strand ended in the air%s"
                       % (k, kind, early[0], w.close_ln,
                          "" if w.rel_len <= 0.0 else
                          " and the %.3f mm release was truncated" % w.rel_len))
            continue

        # --- kind-specific ---------------------------------------------
        if kind == "departure":
            # M1002 -> departure withdrawal -> clearance lift -> station entry.
            dep = [ln for ln, val in w.depart_v if near(val, want_dep, TOL_STATIONARY_V)]
            if len(dep) != 1:
                bad.append("W%d (departure) requires exactly 1 stationary V %+.3f "
                           "between M1002 at line %d and the lift, found %d "
                           "(post-close stationary V: %s)"
                           % (k, want_dep, w.close_ln, len(dep),
                              ", ".join("%+.3f@%d" % (v, ln) for ln, v in
                                        w.depart_v) or "none"))
                continue
            if w.lift_ln is None:
                why = ("but its Z word repeats the current height, moving nothing"
                       if w.lift_z_repeat else "and no Z move follows at all")
                bad.append("W%d (departure) has no clearance lift after M1002 at "
                           "line %d %s" % (k, w.close_ln, why))
                continue
            if w.lift_z_rise is None or w.lift_z_rise + TOL_LIFT_Z < want_clear:
                bad.append("W%d (departure) lift at line %d rises %s mm from Z%s, "
                           "below the %.2f mm departure clearance"
                           % (k, w.lift_ln,
                              "%.3f" % w.lift_z_rise if w.lift_z_rise else "0.000",
                              "%.2f" % w.lift_z_from if w.lift_z_from is not None
                              else "?", want_clear))
                continue
            if w.entry_ln is None:
                bad.append("W%d (departure) lifts at line %d but never enters the "
                           "station, so the outgoing head is never cleaned"
                           % (k, w.lift_ln))
                continue
            if not w.entry_after_lift:
                bad.append("W%d (departure) enters the station at line %d before "
                           "the clearance lift at line %d"
                           % (k, w.entry_ln, w.lift_ln))
                continue
        elif kind == "interstrand":
            # T0 stays active: no station visit and no departure lift are OWED.
            # What IS owed is that the strand was released and closed before the
            # carriage repositioned, and that the repositioning keeps the
            # existing restart clearance. A Z rise here is therefore judged as
            # the restart hop the emitter legitimately performs, not rejected
            # for existing: redundancy is not a defect, an inadequate hop is.
            if w.lift_kind in ("hop", "rise") and w.lift_ln is not None:
                if w.lift_z_rise is None or w.lift_z_rise + TOL_LIFT_Z < want_clear:
                    bad.append("W%d (inter-strand) restart move at line %d rises "
                               "%s mm from Z%s, below the %.2f mm clearance the "
                               "restart hop policy requires"
                               % (k, w.lift_ln,
                                  "%.3f" % w.lift_z_rise if w.lift_z_rise
                                  is not None else "0.000",
                                  "%.2f" % w.lift_z_from if w.lift_z_from is not
                                  None else "?", want_clear))
                    continue
        # terminal: closure is the whole duty. Shutdown is judged separately
        # (S07 for waits, S11 for payout); no incoming-head transition is
        # invented here.
    if bad:
        s06.bad("; ".join(bad[:4]), code="FS_WINDOW_SEQUENCE")
    else:
        s06.ok("%d windows: %d physical departure(s) verified V-1 -> release -> "
               "M1002 -> withdrawal -> clearance lift -> station entry; %d "
               "inter-strand close(s) verified released and closed before "
               "repositioning with no station visit owed; %d terminal close(s) "
               "verified released and closed before shutdown"
               % (len(wins), n_dep, n_int, n_term))
    res["checks"]["S06"] = s06

    # ---- S07 station waits --------------------------------------------
    # Three independent duties. (a) Every wait that exists must be PARKED. (b)
    # Every managed ACTIVATION must be preceded by its own parked wait for the
    # incoming head at its active temperature. (c) That wait must belong to the
    # station visit immediately before the activation it serves.
    #
    # A previous revision judged only a file-wide tally, so deleting every M109
    # passed vacuously ("all 0 hotend waits are parked") and relocating a wait
    # passed on count alone. Judging per activation closes both: duplicate waits
    # at startup cannot discharge a later obligation, and several fibre windows
    # inside one T0 activation correctly need one wait, not one per window,
    # because there is one physical change into T0.
    s07 = Check("S07", "hotend waits parked, one per managed activation")
    brackets = station_brackets(lines)
    waits = [c for c in lines if c.cmd in ("M109", "M190", "M191")]
    hot_waits = [c for c in waits if c.cmd == "M109"]
    off = []
    for c in hot_waits:
        if not any(a <= c.n <= b for a, b in brackets):
            off.append("line %d: %s" % (c.n, c.raw.strip()))
    s07.measured = len(off)
    s07.expected = 0

    ob = wait_obligations(lines, brackets, spec)
    res["wait_obligations"] = []
    unmet = []
    codes07 = []
    for o in ob:
        row = dict(kind=o["kind"], head=o["head"], activation_ln=o["ln"],
                   required_s=o["want_s"],
                   station=("%d..%d" % o["bracket"]) if o["bracket"] else None,
                   wait_ln=None, wait_s=None, ok=False, why=None)
        if o["want_s"] is None:
            row["why"] = "FS_WAIT_TEMP_UNKNOWN"
            unmet.append("%s activation of T%d at line %d: the manifest records no "
                         "active temperature for that head, so its required wait "
                         "cannot be evaluated"
                         % (o["kind"], o["head"], o["ln"]))
            codes07.append("FS_WAIT_TEMP_UNKNOWN")
            res["wait_obligations"].append(row)
            continue
        if o["bracket"] is None:
            row["why"] = "FS_WAIT_NO_STATION"
            unmet.append("%s activation of T%d at line %d has no station visit "
                         "around it at all, so its M109 S%d T%d cannot be parked"
                         % (o["kind"], o["head"], o["ln"], o["want_s"], o["head"]))
            codes07.append("FS_WAIT_NO_STATION")
            res["wait_obligations"].append(row)
            continue
        if o["wait"] is None:
            row["why"] = "FS_WAIT_MISSING"
            lo, hi = o["bracket"]
            inb = [c for c in lines if c.cmd == "M109" and lo <= c.n <= hi]
            wrong = [c for c in inb if int(c.w.get("T", -1)) == o["head"]]
            if wrong:
                row["wait_ln"] = wrong[0].n
                row["wait_s"] = wrong[0].w.get("S")
                row["why"] = "FS_WAIT_WRONG_TEMP"
                codes07.append("FS_WAIT_WRONG_TEMP")
                unmet.append("%s activation of T%d at line %d: the serving station "
                             "visit at %d..%d waits for T%d at S%s but the active "
                             "temperature is %s; required M109 S%d T%d"
                             % (o["kind"], o["head"], o["ln"], lo, hi, o["head"],
                                wrong[0].w.get("S"), o["want_s"], o["want_s"],
                                o["head"]))
            else:
                codes07.append("FS_WAIT_MISSING")
                unmet.append("%s activation of T%d at line %d: the serving station "
                             "visit at %d..%d carries no M109 for T%d; required "
                             "M109 S%d T%d, found [%s]"
                             % (o["kind"], o["head"], o["ln"], lo, hi, o["head"],
                                o["want_s"], o["head"],
                                ", ".join("T%d@%d" % (int(c.w.get("T", -1)), c.n)
                                          for c in inb) or "no waits"))
            res["wait_obligations"].append(row)
            continue
        row["ok"] = True
        row["wait_ln"] = o["wait"].n
        row["wait_s"] = o["wait"].w.get("S")
        res["wait_obligations"].append(row)

    n_ok = sum(1 for r in res["wait_obligations"] if r["ok"])
    s07.note("%d managed activations (1 startup + %d physical head changes), each "
             "requiring its own parked wait; %d satisfied; %d hotend waits present; "
             "%d bed/chamber waits out of scope"
             % (len(ob), len(ob) - 1, n_ok, len(hot_waits),
                len(waits) - len(hot_waits)))
    if unmet:
        s07.bad("%d of %d managed activations lack a parked wait for the incoming "
                "head at its active temperature: %s"
                % (len(unmet), len(ob), unmet[0]), code=codes07[0])
        for cd in sorted(set(codes07)):
            if cd not in s07.codes:
                s07.codes.append(cd)
        for u in unmet[1:5]:
            s07.note(u)
    if off:
        s07.bad("%d of %d hotend waits outside a station bracket: %s"
                % (len(off), len(hot_waits), off[0]),
                code="FS_WAIT_OUTSIDE_STATION")
        for o in off[1:5]:
            s07.note(o)
    if not off and not unmet:
        s07.ok("all %d hotend waits execute inside an explicit station bracket, and "
               "each of the %d managed activations (startup plus %d physical head "
               "changes) is served by its own parked wait for the incoming head at "
               "its active temperature"
               % (len(hot_waits), len(ob), len(ob) - 1))
    res["checks"]["S07"] = s07

    # ---- S08 outgoing thermal order -----------------------------------
    s08 = Check("S08", "outgoing thermal order")
    trans = find_transitions(lines)
    res["transitions"] = trans
    bad = []
    codes8 = []
    for t in trans:
        # The visit bracket that contains the switch.
        br = visit_for_switch(t["switch_ln"], brackets)
        if br is None:
            bad.append("switch at line %d: T%d->T%d has no station visit; every "
                       "physical transition must clean and wait at the station"
                       % (t["switch_ln"], t["frm"], t["to"]))
            codes8.append("FS_WAIT_OUTSIDE_STATION")
            continue
        ev = thermal_events(lines, br[0] - 1, br[1])
        out_head = t["frm"]
        in_head = t["to"]
        sb = ev["standby"].get(out_head, [])
        if not sb:
            bad.append("switch at line %d: outgoing T%d never parked in the visit"
                       % (t["switch_ln"], out_head))
            continue
        sln = sb[0][0]
        if ev["clean_ln"] is not None and sln < ev["clean_ln"]:
            bad.append("switch at line %d: T%d standby at line %d precedes its clean "
                       "at line %d" % (t["switch_ln"], out_head, sln, ev["clean_ln"]))
            codes8.append("FS_COOL_BEFORE_CLEAN")
        wt = ev["wait"].get(in_head)
        if wt and sln > wt[0]:
            bad.append("switch at line %d: T%d standby at line %d comes after the "
                       "incoming wait at line %d" % (t["switch_ln"], out_head, sln, wt[0]))
            codes8.append("FS_STANDBY_AFTER_WAIT")
    if bad:
        s08.bad("; ".join(bad[:4]), code=(codes8[0] if codes8 else None))
        for cd in sorted(set(codes8)):
            if cd not in s08.codes:
                s08.codes.append(cd)
    else:
        s08.ok("in every transition the outgoing head stays at its active target "
               "through deposition, release and cleaning; standby follows cleaning "
               "and precedes the incoming wait")
    res["checks"]["S08"] = s08

    # ---- S09 preheat schedule -----------------------------------------
    s09 = Check("S09", "preheat schedule, both directions")
    lead_req = spec.get("preheat_lead_s", 15.0)
    rows = []
    bad = []
    codes9 = []
    for t in trans:
        br = visit_for_switch(t["switch_ln"], brackets)
        if br is None:
            bad.append("transition T%d->T%d at line %d: no station visit to pay its "
                       "wait in" % (t["frm"], t["to"], t["switch_ln"]))
            codes9.append("FS_WAIT_OUTSIDE_STATION")
            continue
        ev = thermal_events(lines, br[0] - 1, br[1])
        in_head = t["to"]
        k = trans.index(t)
        if k == 0:
            continue  # startup: no outgoing activation; S07 owns the startup wait
        prev_sw = trans[k - 1]["switch_ln"]
        ph = scheduled_preheat(lines, in_head, prev_sw, t["switch_ln"])
        # Spec section 4: the timing endpoint is the end of the OUTGOING head's
        # deposition plus release, before departure/brush travel. Available time
        # runs back from that endpoint to the earliest insertion point after the
        # outgoing head's own switch setup - the previous physical switch.
        if t["frm"] == 0:
            # Fibre outgoing: the window that closed before this visit; its
            # endpoint is M1002, i.e. after deposition and the dry release.
            prior = [w for w in wins
                     if w.close_idx is not None and w.close_idx < br[0] - 1]
            if not prior:
                bad.append("transition T0->T1 at line %d: no fibre window before "
                           "the visit" % t["switch_ln"])
                codes9.append("FS_PREHEAT_SCHEDULE")
                continue
            end = prior[-1].close_idx
        else:
            # Plastic outgoing: the last extruding move before the visit.
            entry_idx = br[0] - 1
            end = None
            for i in range(entry_idx, max(entry_idx - 400, 0), -1):
                c = lines[i]
                if RE_MOVE.match(c.cmd) and c.e > 1e-6:
                    end = i
                    break
            if end is None:
                bad.append("transition T1->T0 at line %d: no plastic deposition "
                           "found before the visit" % t["switch_ln"])
                codes9.append("FS_PREHEAT_SCHEDULE")
                continue
        start = line_index(lines, prev_sw) + 1
        avail, cerr = block_seconds(lines, start, end)
        required = min(lead_req, avail)
        if ph is None:
            bad.append("transition T%d->T%d at line %d: no nonblocking preheat "
                       "command for the incoming head" % (t["frm"], t["to"], t["switch_ln"]))
            codes9.append("FS_PREHEAT_MISSING")
            continue
        ph_ln = ph[0]
        meas, merr = block_seconds(lines, ph_ln - 1, end)
        # Evidence the handoff table asks for beyond the timing: the station
        # visit that pays this switch, the thermal lines inside it, where the
        # head is positioned and where its pending E is finally paid.
        out_head = t["frm"]
        sb = ev["standby"].get(out_head, [])
        wt_in = ev["wait"].get(in_head, (None, None))
        pos_ln = dep_ln = None
        for i in range(t["idx"], min(t["idx"] + 60, len(lines))):
            c2 = lines[i]
            if pos_ln is None and RE_MOVE.match(c2.cmd) and ("X" in c2.w or "Y" in c2.w)                     and abs(c2.e) < 1e-9:
                pos_ln = c2.n
            if dep_ln is None and RE_MOVE.match(c2.cmd) and c2.e > 1e-6                     and ("X" in c2.w or "Y" in c2.w):
                dep_ln = c2.n
            if pos_ln is not None and dep_ln is not None:
                break
        rows.append(dict(switch=t["switch_ln"], frm=t["frm"], to=t["to"],
                         preheat_ln=ph_ln, avail=avail, required=required,
                         measured=meas, target=ph[1],
                         clean_ln=ev["clean_ln"], entry_ln=ev["entry_ln"],
                         exit_ln=ev["exit_ln"],
                         standby_ln=(sb[0][0] if sb else None),
                         standby_c=(sb[0][1] if sb else None),
                         wait_ln=wt_in[0], wait_c=wt_in[1],
                         positioning_ln=pos_ln, first_deposition_ln=dep_ln,
                         station_state="bracketed"))
        if merr or cerr:
            bad.append("transition at line %d: clock error %s"
                       % (t["switch_ln"], (merr or cerr)[0]))
        elif abs(meas - required) > TOL_LEAD:
            bad.append("transition T%d->T%d at line %d: measured lead %.2f s vs "
                       "required %.2f s (available %.2f s)"
                       % (t["frm"], t["to"], t["switch_ln"], meas, required, avail))
            codes9.append("FS_PREHEAT_SCHEDULE")
        # No standby drop below the preheat target between the scheduled preheat
        # and the wait. An idempotent re-assert at the same target is allowed.
        wt = ev["wait"].get(in_head)
        if wt:
            for i in range(ph_ln, wt[0]):
                c2 = lines[i - 1]
                if c2.cmd == "M104" and int(c2.w.get("T", -1)) == in_head                         and c2.w.get("S", 1e9) < ph[1] - 1e-9:
                    bad.append("transition at line %d: T%d target drop to %.0f at line "
                               "%d clobbers the preheat at line %d"
                               % (t["switch_ln"], in_head, c2.w.get("S", 0), c2.n, ph_ln))
                    codes9.append("FS_PREHEAT_CLOBBERED")
    res["preheat_rows"] = rows
    s09.measured = [round(r["measured"], 3) for r in rows]
    s09.expected = "min(%.1f, available) per transition" % lead_req
    if not rows and not bad:
        s09.bad("no physical transition could be evaluated; S09 is NOT_EVALUATED "
                "and cannot satisfy the software gate")
    if bad:
        s09.bad("; ".join(bad[:4]), code=(codes9[0] if codes9 else None))
        for cd in sorted(set(codes9)):
            if cd not in s09.codes:
                s09.codes.append(cd)
    else:
        s09.ok("%d transitions in both directions meet the lead within %.2f s; "
               "no incoming standby clobbers a scheduled preheat"
               % (len(rows), TOL_LEAD))
    res["checks"]["S09"] = s09

    # ---- S10 E recovery ------------------------------------------------
    # The spec's ledger, rebuilt from emitted coordinates: one pending total
    # shared by tool-change withdrawals and ordinary travel retractions. Every
    # negative E adds to it; every positive E must be paid AT the next
    # deposition start, after any travel, and must match the pending total.
    s10 = Check("S10", "pending E recovery at the deposition start")
    led = 0.0
    bad = []
    codes10 = []
    rec_rows = []
    for i, c in enumerate(lines):
        if not RE_MOVE.match(c.cmd):
            continue
        if c.e < -1e-9:
            led += c.e
            continue
        if c.e <= 1e-9:
            continue
        # Positive E with no pending withdrawal is ordinary deposition, not a
        # recovery: the ledger only tracks recovery obligations it was charged.
        if led >= -1e-6:
            continue
        amt = c.e
        moved = math.isfinite(c.x) and math.isfinite(c.y) and \
                ("X" in c.w or "Y" in c.w)
        nxt_dep = None
        for j in range(i + 1, min(i + 60, len(lines))):
            cc = lines[j]
            if cc.cmd in STATION_IN:
                break
            if RE_MOVE.match(cc.cmd) and cc.e > 1e-6 and "X" in cc.w and "Y" in cc.w:
                nxt_dep = cc.n
                break
        if c.station:
            bad.append("line %d: E %+.2f recovered inside the station bracket"
                       % (c.n, amt))
            codes10.append("FS_E_RECOVERY_LOCATION")
        elif nxt_dep is None:
            bad.append("line %d: E %+.2f recovery not followed by deposition"
                       % (c.n, amt))
            codes10.append("FS_E_RECOVERY_LOCATION")
        elif moved or not recovered_at_dest(i, lines):
            bad.append("line %d: E %+.2f recovery is not at the deposition start "
                       "(moved=%s, at-destination=%s)"
                       % (c.n, amt, moved, recovered_at_dest(i, lines)))
            codes10.append("FS_E_RECOVERY_LOCATION")
        elif led < -1e-6 and abs(amt - (-led)) > 0.05:
            bad.append("line %d: recovered %.3f mm but the ledger holds %.3f mm"
                       % (c.n, amt, -led))
            codes10.append("FS_E_RECOVERY_AMOUNT")
        else:
            rec_rows.append(dict(ln=c.n, amount=amt, ledger=-led, moved=moved))
        led = 0.0
    s10.measured = [round(r["amount"], 3) for r in rec_rows]
    if bad:
        s10.bad("%d recovery faults: %s" % (len(bad), bad[0]),
                code=(codes10[0] if codes10 else None))
        for cd in sorted(set(codes10)):
            if cd not in s10.codes:
                s10.codes.append(cd)
        for b in bad[1:5]:
            s10.note(b)
    else:
        s10.ok("%d recoveries, each at a deposition start after destination travel, "
               "each matching the pending ledger; no positive E at a brush exit"
               % len(rec_rows))
    res["e_recoveries"] = rec_rows
    res["checks"]["S10"] = s10

    # ---- S11 payout / budgets -----------------------------------------
    s11 = Check("S11", "payout formula and budgets")
    bad = []
    codes11 = []
    for k, w in enumerate(wins, start=1):
        if w.budget_L is None:
            bad.append("W%d M1001 carries no L word" % k)
            continue
        su = 0.0
        for i, c in enumerate(lines):
            if w.open_ln <= c.n <= w.close_ln and RE_MOVE.match(c.cmd):
                su += c.u
        if not (w.budget_L <= su < w.budget_L + 1.0):
            bad.append("W%d L=%d but window U sums %.3f" % (k, w.budget_L, su))
    # --- body matrix payout rate ---------------------------------------
    # The body deposits matrix at fiber_rate x matrix_ratio mm of V per mm of
    # path. Judged per window, on windows long enough for the ratio to mean
    # something, so a wrong rate cannot hide behind a short move.
    fr = spec.get("fs_fiber_rate")
    mr = spec.get("fs_matrix_ratio")
    tvc = spec.get("fs_tail_v_factor")
    if fr is None or mr is None:
        bad.append("body payout rate cannot be evaluated: the manifest records no "
                   "fs_fiber_rate / fs_matrix_ratio for this export")
        codes11.append("FS_PAYOUT_EVIDENCE_MISSING")
    else:
        body_rate = fr * mr
        worst_b = 0.0
        worst_b_w = None
        for k, w in enumerate(wins, start=1):
            if w.body_len < 20.0:
                continue
            obs = w.body_v / w.body_len
            rel = abs(obs - body_rate) / body_rate
            if rel > worst_b:
                worst_b, worst_b_w = rel, (k, obs)
        if worst_b_w is None:
            bad.append("no window is long enough to evaluate the body payout rate")
            codes11.append("FS_PAYOUT_EVIDENCE_MISSING")
        elif worst_b > TOL_BODY_RATE:
            bad.append("body matrix payout rate: W%d observed %.5f V/mm against the "
                       "configured %.5f (fs_fiber_rate %.3f x fs_matrix_ratio %.4f), "
                       "%.2f percent off, tolerance %.1f percent"
                       % (worst_b_w[0], worst_b_w[1], body_rate, fr, mr,
                          worst_b * 100.0, TOL_BODY_RATE * 100.0))
            codes11.append("FS_TAIL_PAYOUT_FORMULA")
        s11.note("body payout rate worst relative deviation %.3f percent "
                 "(tolerance %.1f percent)" % (worst_b * 100.0,
                                               TOL_BODY_RATE * 100.0))

        # --- tail matrix payout ----------------------------------------
        # tail V = tail path x fs_tail_v_factor x body rate. The startup purge is
        # sacrificial and pays at the FULL body rate (factor 1.0); that is a
        # documented emitter choice, not a discount to be enforced on it.
        if tvc is None:
            bad.append("tail payout cannot be evaluated: the manifest records no "
                       "fs_tail_v_factor for this export")
            codes11.append("FS_PAYOUT_EVIDENCE_MISSING")
        else:
            worst_t = 0.0
            first_bad = None
            for k, w in enumerate(wins, start=1):
                fac = 1.0 if w.is_purge else tvc
                exp = w.tail_len * fac * body_rate
                dev = w.tail_v - exp
                if abs(dev) > worst_t:
                    worst_t = abs(dev)
                if abs(dev) > TOL_TAIL_V and first_bad is None:
                    first_bad = (k, w.tail_v, exp, w.tail_len, fac)
            if first_bad is not None:
                k, obs, exp, tl, fac = first_bad
                bad.append("tail matrix payout: W%d carries %.3f mm of V over "
                           "%.3f mm of tail path but the formula (fs_tail_v_factor "
                           "%.2f x fs_fiber_rate %.3f x fs_matrix_ratio %.4f) "
                           "requires %.3f mm; observed payout is %.1fx the required "
                           "amount"
                           % (k, obs, tl, fac, fr, mr, exp,
                              obs / exp if exp > 0 else float("inf")))
                codes11.append("FS_TAIL_PAYOUT_FORMULA")
            s11.note("tail payout worst absolute deviation %.4f mm (tolerance "
                     "%.2f mm)" % (worst_t, TOL_TAIL_V))

    # --- departure tool-change withdrawal --------------------------------
    # The stationary-V ledger above stops at M1002, which is exactly where the
    # departure withdrawal begins, so it never judged that amount. Spec section
    # 3 step 4 requires the additional V withdrawal ONCE for an actual T0
    # departure, between the window close and the lift, and NONE for a close
    # that keeps T0 active. Deleting all 16 of them used to pass S11. Whether a
    # close is a departure is decided by looking FORWARD (see
    # departure_obligations), not by the head selected at the close, because an
    # inter-strand close is also on T0 and owes nothing.
    if tcv is None:
        s11.note("fs_toolchange_retract_v absent from the effective config; the "
                 "registered default 4.0 mm is used as the required departure "
                 "withdrawal")
    if want_dep < 0.0:
        dep_bad = []
        dep_missing = []
        dep_spurious = []
        dep_rows = res["close_kinds"]
        for d, w in zip(dep_rows, wins):
            k = d["window"]
            if d["expected"] and len(d["lines"]) != 1:
                dep_missing.append(k)
                dep_bad.append(
                    "W%d is an actual T0 departure (%s): requires exactly 1 "
                    "stationary V %+.3f between M1002 at line %d and the first lift "
                    "at line %d, found %d (window carries stationary V %s after the "
                    "close)"
                    % (k, d["basis"], want_dep, w.close_ln, w.lift_ln or 0,
                       len(d["lines"]),
                       ", ".join("%+.3f@%d" % (v, ln) for ln, v in
                                 w.depart_v) or "none"))
            elif (not d["expected"]) and d["lines"]:
                dep_spurious.append(k)
                dep_bad.append(
                    "W%d is not a T0 departure (%s): must carry NO departure "
                    "withdrawal, found %d at lines %s"
                    % (k, d["basis"], len(d["lines"]), d["lines"]))
        res["departure_withdrawals"] = dep_rows
        n_exp = sum(1 for d in dep_rows if d["expected"])
        s11.note("close kinds shared with S06: %s; departure withdrawal: %d of %d "
                 "windows are an actual T0 departure "
                 "and each requires exactly one stationary V %+.3f after M1002 and "
                 "before the lift; %d verified; %d close(s) require none"
                 % ("/".join("%s:%d" % (kk, sum(1 for d in dep_rows
                                                if d["kind"] == kk))
                             for kk in ("departure", "interstrand", "terminal")),
                    n_exp, len(wins), want_dep, n_exp - len(dep_missing),
                    len(wins) - n_exp))
        if dep_missing:
            codes11.append("FS_DEPART_WITHDRAWAL_MISSING")
        if dep_spurious:
            codes11.append("FS_DEPART_WITHDRAWAL_SPURIOUS")
        if dep_bad:
            bad.extend(dep_bad[:4])
            for b in dep_bad[4:8]:
                s11.note(b)

    # --- stationary V amounts ------------------------------------------
    # Classified by VALUE, never by comment: a re-labelled payout is still a
    # payout, and a comment-sorted ledger is exactly what let a 30x tail payout
    # through this check before.
    if "fs_prime_v" in spec and "fs_retract_v" in spec:
        # The emitter splits the window-start stationary V into two accounted
        # lines (FiberEmitter.cpp step 3): recover the previous run's retract,
        # then any EXTRA configured as an anchor prime, where extra is
        # fs_prime_v minus fs_retract_v. The prime line is absent entirely when
        # the two are equal. Expecting fs_prime_v on the prime line is the check
        # being wrong, not the export.
        prime_extra = spec["fs_prime_v"] - spec["fs_retract_v"]
        want_sv = dict(recover=spec["fs_retract_v"],
                       prime=prime_extra,
                       retract=-spec["fs_retract_v"],
                       tc=-(spec.get("fs_toolchange_retract_v", 4.0)))
        required_keys = ["recover", "retract"] + (["prime"] if prime_extra > 0 else [])
        sv_bad = []
        for k, w in enumerate(wins, start=1):
            got = {}
            for ln, val in w.stationary_v:
                for key, want in want_sv.items():
                    if near(val, want, TOL_STATIONARY_V):
                        got.setdefault(key, []).append(ln)
                        break
                else:
                    sv_bad.append("W%d line %d: stationary V %+.3f matches none of "
                                  "the contracted amounts [%s]"
                                  % (k, ln, val,
                                     ", ".join("%+.3f" % v for v in
                                               sorted(set(want_sv.values())))))
            for key in required_keys:
                if len(got.get(key, [])) != 1:
                    sv_bad.append("W%d has %d stationary V %+.3f moves, required "
                                  "exactly 1 (lines %s)"
                                  % (k, len(got.get(key, [])), want_sv[key],
                                     got.get(key, [])))
        if sv_bad:
            bad.extend(sv_bad[:6])
            codes11.append("FS_TAIL_PAYOUT_FORMULA")
        s11.note("stationary V per window: recover %+.3f, prime extra %+.3f, "
                 "retract %+.3f, tool-change %+.3f"
                 % (want_sv["recover"], want_sv["prime"], want_sv["retract"],
                    want_sv["tc"]))
    else:
        bad.append("stationary V amounts cannot be evaluated: the manifest records "
                   "no fs_prime_v / fs_retract_v")
        codes11.append("FS_PAYOUT_EVIDENCE_MISSING")
    u55 = sum(1 for c in lines if RE_MOVE.match(c.cmd) and near(c.u, 55.0, 1e-6))
    s11.note("U55 reload present on %d of %d windows" % (u55, len(wins)))
    if bad or u55 != len(wins):
        s11.bad(("; ".join(bad[:4]) if bad else "") +
                ("U55 reload on %d/%d windows" % (u55, len(wins))
                 if u55 != len(wins) else ""),
                code=(codes11[0] if codes11 else None))
        for cd in sorted(set(codes11)):
            if cd not in s11.codes:
                s11.codes.append(cd)
    else:
        s11.ok("M1001 L agrees with the recalculated window U under the established "
               "floor contract in all %d windows; U55 reload preserved on every "
               "window; body and tail matrix payouts match the configured formula; "
               "every real T0 departure carries exactly one withdrawal between "
               "M1002 and the lift and no inter-strand close carries one"
               % len(wins))
    res["checks"]["S11"] = s11

    # ---- S13 support evidence -----------------------------------------
    s13 = Check("S13", "release support evidence")
    if not evidence:
        s13.status = NE
        s13.note("no geometry evidence supplied for this export; the containment "
                 "verdict cannot be evaluated from the G-code alone")
    else:
        bad = []
        for k, w in enumerate(wins, start=1):
            if w.rel_len <= 0.0:
                continue
            if w.is_purge:
                ok, why = purge_corridor_ok(w, spec, half_width)
            else:
                ok, why = release_supported(lines, w, half_width, w.v1_ln,
                                            spec.get("plastic_width_mm", 0.4))
            w.support = "SUPPORTED" if ok else "UNSUPPORTED"
            w.support_why = why
            if not ok:
                bad.append("W%d (Z%.2f): %s" % (k, w.z, why))
        if bad:
            s13.bad("%d releases fail independent containment: %s" % (len(bad), bad[0]),
                    code="FS_RELEASE_UNSUPPORTED")
            for b in bad[1:5]:
                s13.note(b)
        else:
            s13.ok("every model release footprint lies inside material deposited "
                   "earlier in the same physical layer (buffer %.3f mm, tolerance "
                   "%.2f mm); the purge release lies inside its validated corridor"
                   % (half_width, TOL_CONTAIN))
    res["checks"]["S13"] = s13

    return res


# --------------------------------------------------------------------------
# Cross-file: S12 comparison isolation
# --------------------------------------------------------------------------

def deposited_path(path):
    """Reconstructed deposited centreline: the ordered list of (x,y) vertices of
    every fibre deposition move, at emitted precision.

    Compared between B and C this answers "same geometry?" without demanding
    literal line equality, which C's 1 mm earlier cut legitimately breaks (the
    tail subdivision differs even when the centreline does not).
    """
    lines, _ = load_lines(path)
    pts = []
    for w, o, cl, cu in find_windows(lines):
        if cu is None or w.is_purge:
            continue  # the purge cut legitimately differs B vs C (spec table)
        for i in range(o, cl + 1):
            c = lines[i]
            if RE_MOVE.match(c.cmd) and c.v > 0.0 and "X" in c.w and "Y" in c.w:
                pts.append((round(c.x, 2), round(c.y, 2)))
    return pts


def seam_points(path):
    """Window start vertices: where each strand opens, i.e. its seam."""
    lines, _ = load_lines(path)
    out = []
    for w, o, cl, cu in find_windows(lines):
        if cu is None or w.is_purge:
            continue  # purge has no rotated seam; excluded from B/C equality
        for i in range(o, cu):
            c = lines[i]
            if RE_MOVE.match(c.cmd) and c.v > 0.0 and "X" in c.w and "Y" in c.w:
                out.append((round(c.x, 2), round(c.y, 2)))
                break
    return out


def cut_positions(path):
    lines, _ = load_lines(path)
    out = []
    for w, o, cl, cu in find_windows(lines):
        if cu is None or w.is_purge:
            continue  # model cuts only; purge cut table is checked in S13
        measure_window(lines, w, o, cl, cu)
        out.append((w.cut_xy, w.dep_len))
    return out


def polyline_equiv(pa, pb, tol=0.02):
    """Two reconstructed deposited centrelines are equivalent when every vertex
    of each lies on the other's polyline (within tol) and their total path
    lengths agree within TOL_PATH. Vertex lists may legitimately differ: the
    spec allows C's 1 mm earlier cut to change segment subdivision while the
    geometry stays identical."""
    def on_poly(pt, poly):
        for a, b2 in zip(poly, poly[1:]):
            ax, ay = a; bx, by = b2; px, py = pt
            dx, dy = bx - ax, by - ay
            l2 = dx * dx + dy * dy
            if l2 < 1e-12:
                if abs(px - ax) <= tol and abs(py - ay) <= tol:
                    return True
                continue
            t = ((px - ax) * dx + (py - ay) * dy) / l2
            t = max(0.0, min(1.0, t))
            if abs(px - (ax + t * dx)) <= tol and abs(py - (ay + t * dy)) <= tol:
                return True
        return False
    if not pa or not pb:
        return False, 0, 0, float("nan")
    off_a = sum(1 for q in pa if not on_poly(q, pb))
    off_b = sum(1 for q in pb if not on_poly(q, pa))
    la = sum(seg(x, y) for x, y in zip(pa, pa[1:]))
    lb = sum(seg(x, y) for x, y in zip(pb, pb[1:]))
    return (off_a == 0 and off_b == 0 and abs(la - lb) <= TOL_PATH), off_a, off_b, la - lb


def check_s12(results, manifest):
    s12 = Check("S12", "comparison isolation")
    by = dict((r["variant"], r) for r in results)
    if len(by) < 3:
        s12.status = NE
        s12.note("fewer than three exports present")
        return s12

    cfgs = {}
    for v in VARIANTS:
        r = by.get(v)
        cfg = (r or {}).get("effective_config") or {}
        cfgs[v] = cfg

    # Configs differ only in the listed keys.
    allowed = {"fs_fiber_release_length_mm", "fs_fiber_tail_margin_mm", "name"}
    diffs = []
    base = cfgs.get(VARIANTS[0], {})
    for v in VARIANTS[1:]:
        for k in set(list(base.keys()) + list(cfgs.get(v, {}).keys())):
            if k in allowed:
                continue
            if base.get(k) != cfgs[v].get(k):
                diffs.append("%s: %s differs (%r vs %r)" % (VARIANTS[0], v, k,
                                                            base.get(k), cfgs[v].get(k)))
    # B and C: identical deposited centreline and seam placement.
    pb = deposited_path(by["shook_B_forward_release"]["gcode"])
    pc = deposited_path(by["shook_C_release_margin_1mm"]["gcode"])
    sb = seam_points(by["shook_B_forward_release"]["gcode"])
    sc = seam_points(by["shook_C_release_margin_1mm"]["gcode"])
    geo_ok, off_b, off_c, len_delta = polyline_equiv(pb, pc)
    seam_ok = (len(sb) == len(sc) and all(a == b for a, b in zip(sb, sc)))

    # C's cut sits 1 mm earlier along the same path than B's.
    cb = cut_positions(by["shook_B_forward_release"]["gcode"])
    cc = cut_positions(by["shook_C_release_margin_1mm"]["gcode"])
    cut_ok = len(cb) == len(cc)
    worst = 0.0
    if cut_ok:
        for (xb, _), (xc, _) in zip(cb, cc):
            worst = max(worst, seg(xb, xc))
        # 1 mm earlier along the path means the cut vertex moved; the deposited
        # tail then measures 1 mm longer, which S02 already pins.
        dep_delta = [ccx[1] - cbx[1] for cbx, ccx in zip(cb, cc)]
        cut_ok = cut_ok and all(near(d, 1.0, TOL_PATH) for d in dep_delta)

    if not cfgs.get(VARIANTS[1]) or not cfgs.get(VARIANTS[2]):
        s12.status = NE
        s12.note("effective configurations not recorded in the manifest; the runs "
                 "cannot be proven to differ only as listed")
        return s12

    problems = []
    if diffs:
        problems.append("configs differ outside the allowed keys: " + "; ".join(diffs[:3]))
    if not geo_ok:
        problems.append("B and C deposited centrelines are not polyline-equivalent "
                        "(%d B-vertices off C, %d C-vertices off B, length delta "
                        "%.3f mm; %d vs %d vertices)"
                        % (off_b, off_c, len_delta, len(pb), len(pc)))
    if not seam_ok:
        problems.append("B and C seam placement differs")
    if not cut_ok:
        problems.append("C's cut is not 1.00 mm earlier along the shared path "
                        "(max vertex shift %.3f mm)" % worst)
    s12.measured = dict(centrelines_equal=geo_ok, seams_equal=seam_ok,
                        b_off_c=off_b, c_off_b=off_c,
                        length_delta_mm=round(len_delta, 4),
                        vertices=(len(pb), len(pc)),
                        max_cut_shift_mm=round(worst, 3),
                        config_diffs=len(diffs))
    if problems:
        s12.bad("; ".join(problems))
    else:
        s12.ok("configs differ only in release length and tail margin; B and C share "
               "a polyline-equivalent deposited centreline (%d vs %d vertices, all "
               "mutually on-polyline, length delta %.4f mm) and seam placement; "
               "C's cut is 1.00 mm earlier along it (max vertex shift %.3f mm)"
               % (len(pb), len(pc), len_delta, worst))
    s12.note("A-to-B changes the seam policy as well as adding the release; that is "
             "the intended experimental distinction, not a control violation")
    return s12


def check_cross_totals(results):
    """The spec's cross-check: total XY from cut through release end."""
    out = []
    for r in results:
        # The spec row follows spec_variant, exactly as evaluate_file resolves
        # it, so a mutant or fixture that carries its own variant name is still
        # judged against the export it was derived from.
        v = r.get("spec_variant", r["variant"])
        spec = VSPEC[v]
        bad = []
        for k, w in enumerate(r["windows"], start=1):
            if not near(w.total_len, spec["total"], TOL_TOTAL):
                bad.append("W%d total %.3f mm, expected %.1f +-%.2f "
                           "(deposition %.3f + release %.3f)"
                           % (k, w.total_len, spec["total"], TOL_TOTAL,
                              w.dep_len, w.rel_len))
        out.append(dict(variant=r["variant"], spec_variant=v, bad=bad))
    return out


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------

def window_rows(res):
    rows = []
    for k, w in enumerate(res["windows"], start=1):
        rows.append(dict(
            file=os.path.basename(res["gcode"]), sha256=res.get("sha256", ""),
            variant=res["variant"], window=k,
            kind="purge" if w.is_purge else "model",
            # How the close was classified, and whether the Z move after it was a
            # real displacement. S06/S11 decide by this, so the report has to show
            # it: "no withdrawal owed" is true of both inter-strand and terminal.
            close_kind=w.kind,
            lift_kind=w.lift_kind,
            lift_rise_mm=(None if w.lift_z_rise is None else round(w.lift_z_rise, 3)),
            z=round(w.z, 3),
            open_ln=w.open_ln, cut_ln=w.cut_ln, v1_ln=w.v1_ln,
            rel_begin_ln=w.rel_begin_ln, rel_end_ln=w.rel_end_ln,
            close_ln=w.close_ln, lift_ln=w.lift_ln, entry_ln=w.entry_ln,
            cut_xy=[round(v, 3) for v in w.cut_xy],
            dep_end_xy=[round(v, 3) for v in w.dep_end_xy],
            rel_end_xy=[round(v, 3) for v in w.rel_end_xy],
            post_cut_dep_mm=round(w.dep_len, 3),
            release_mm=round(w.rel_len, 3),
            total_mm=round(w.total_len, 3),
            release_f=(None if not math.isfinite(w.rel_f) else w.rel_f),
            release_z_dev_mm=round(w.rel_z_dev, 4),
            post_cut_u_mm=round(w.post_cut_u, 4),
            budget_L=w.budget_L,
            support=getattr(w, "support", "NOT_EVALUATED"),
            support_detail=getattr(w, "support_why", ""),
            # Worst measured escape of the buffered release footprint past the
            # material already deposited in this layer, in mm. None for a window
            # with no release or for the purge corridor.
            footprint_escape_mm=(None if getattr(w, "footprint_worst", None) is None
                                 else round(w.footprint_worst, 4)),
            footprint_worst_xy=(None if getattr(w, "footprint_worst_at", None) is None
                                else [round(v, 2) for v in w.footprint_worst_at]),
            errors=list(w.errors),
        ))
    return rows


def transition_rows(res):
    """One row per physical transition: outgoing/incoming tools, the line of
    every thermal and positioning event, the E the transition owed and where it
    was paid, and the preheat timing. Absent evidence is None, never a guess."""
    recs = res.get("e_recoveries", [])
    rows = []
    for r in res.get("preheat_rows", []):
        rec = next((x for x in recs
                    if r["switch"] <= x["ln"] <= (r.get("first_deposition_ln") or (1 << 30))),
                   None)
        rows.append(dict(variant=res["variant"], switch_ln=r["switch"],
                         outgoing="T%d" % r["frm"], incoming="T%d" % r["to"],
                         preheat_ln=r["preheat_ln"], preheat_target_c=r["target"],
                         station_entry_ln=r.get("entry_ln"),
                         clean_ln=r.get("clean_ln"),
                         standby_ln=r.get("standby_ln"),
                         standby_target_c=r.get("standby_c"),
                         wait_ln=r.get("wait_ln"), wait_target_c=r.get("wait_c"),
                         station_exit_ln=r.get("exit_ln"),
                         switch_ln_note=r["switch"],
                         positioning_ln=r.get("positioning_ln"),
                         recovery_ln=(rec or {}).get("ln"),
                         e_pending_mm=(round(-rec["ledger"], 3) if rec else None),
                         e_recovered_mm=(round(rec["amount"], 3) if rec else None),
                         first_deposition_ln=r.get("first_deposition_ln"),
                         station_state=r.get("station_state", "UNBRACKETED"),
                         available_s=round(r["avail"], 3),
                         required_s=round(r["required"], 3),
                         measured_s=round(r["measured"], 3),
                         delta_s=round(r["measured"] - r["required"], 3)))
    return rows


def render_md(payload):
    L = []
    A = L.append
    A("# FibreSeeker3 S-hook export verification")
    A("")
    A("Generated by `tools/verify_fs_shook.py`. Distances are measured from emitted")
    A("coordinates; comments are used to name phases, never to measure them.")
    A(FENCE + "text")
    A("SOFTWARE_EXPORT_ACCEPTANCE: %s" % payload["software_acceptance"])
    A("MACRO_CONTRACT: %s" % payload["macro_contract"])
    A("PHYSICAL_PRINT_RESULT: %s" % payload["physical_print_result"])
    A(FENCE)
    A("")
    if payload["macro_contract"] != "VERIFIED":
        A("**The macro contract is %s.** Software acceptance below is therefore" % payload["macro_contract"])
        A("**conditional**: it proves what this slicer commands against a documented")
        A("adapter contract, not that the firmware macros behave that way on hardware.")
        A("")
    A("## Provenance")
    A("")
    A("| item | value |")
    A("|---|---|")
    for k, v in payload["provenance"].items():
        A("| %s | `%s` |" % (k, v))
    A("")
    A("## Mandatory checks")
    A("")
    A("| ID | check | " + " | ".join(r["variant"] for r in payload["files"]) + " |")
    A("|---|---|" + "---|" * len(payload["files"]))
    for cid in MANDATORY:
        title = ""
        cells = []
        for r in payload["files"]:
            c = r["checks"].get(cid)
            title = c["title"] if c and not title else title
            cells.append(c["status"] if c else NE)
        A("| %s | %s | %s |" % (cid, title or "-", " | ".join(cells)))
    A("")
    for r in payload["files"]:
        A("## %s" % r["variant"])
        A("")
        A("- file: `%s`" % os.path.basename(r["gcode"]))
        A("- SHA-256: `%s`" % r.get("sha256", "n/a"))
        A("- lines: %s" % r.get("lines", "n/a"))
        A("")
        A("### Check detail")
        A("")
        for cid in MANDATORY:
            c = r["checks"].get(cid)
            if not c:
                continue
            A("- **%s %s - %s**" % (c["id"], c["title"], c["status"]))
            for d in c["detail"]:
                A("  - %s" % d)
        A("")
        A("### Per-window measurements")
        A("")
        A("footprint escape is the worst measured distance the buffered release")
        A("strip reaches past material already deposited in the same layer;")
        A("negative or zero means the whole strip is covered.")
        A("")
        A("| W | kind | close | Z | open | cut | V-1 | rel beg | rel end | close ln | "
          "lift | entry | dep mm | rel mm | total mm | rel F | rel Z dev | post-cut U | "
          "L | support | footprint escape mm |")
        A("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
        for w in r["window_rows"]:
            esc = w.get("footprint_escape_mm")
            # close column: the classification plus what the Z move after the
            # close actually did, so a reader can see "departure/rise+0.60"
            # versus "interstrand/noZ" without cross-referencing the JSON.
            rise = w.get("lift_rise_mm")
            close_cell = "%s/%s%s" % (
                w.get("close_kind") or "-",
                w.get("lift_kind") or "noZ",
                "+%.2f" % rise if rise is not None else "")
            A("| %d | %s | %s | %.2f | %s | %s | %s | %s | %s | %s | %s | %s | %.3f | "
              "%.3f | %.3f | %s | %.4f | %.3f | %s | %s | %s |"
              % (w["window"], w["kind"], close_cell, w["z"], w["open_ln"],
                 w["cut_ln"], w["v1_ln"],
                 w["rel_begin_ln"] or "-", w["rel_end_ln"] or "-", w["close_ln"],
                 w["lift_ln"], w["entry_ln"], w["post_cut_dep_mm"], w["release_mm"],
                 w["total_mm"], w["release_f"] if w["release_f"] is not None else "-",
                 w["release_z_dev_mm"], w["post_cut_u_mm"], w["budget_L"], w["support"],
                 ("%.4f" % esc) if esc is not None else "-"))
        A("")
        A("")
        A("### Per-transition measurements")
        A("")
        A("Line numbers are 1-based in the emitted file. E pend is the pending")
        A("withdrawal the transition owed; E rec is where it was actually paid.")
        A("")
        A("| out | in | entry | clean | standby | wait | exit | switch | preheat "
          "| tgt C | pos | E rec | E pend | first dep | avail s | req s | meas s "
          "| delta s |")
        A("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")

        def cell(v, fmt="%s"):
            if v is None:
                return "-"
            return fmt % v

        for t in r["transition_rows"]:
            A("| %s | %s | %s | %s | %s | %s | %s | %d | %d | %s | %s | %s | %s "
              "| %s | %.3f | %.3f | %.3f | %+.3f |"
              % (t["outgoing"], t["incoming"], cell(t.get("station_entry_ln")),
                 cell(t.get("clean_ln")), cell(t.get("standby_ln")),
                 cell(t.get("wait_ln")), cell(t.get("station_exit_ln")),
                 t["switch_ln"], t["preheat_ln"], cell(t["preheat_target_c"], "%.0f"),
                 cell(t.get("positioning_ln")), cell(t.get("recovery_ln")),
                 cell(t.get("e_pending_mm"), "%.1f"),
                 cell(t.get("first_deposition_ln")),
                 t["available_s"], t["required_s"], t["measured_s"], t["delta_s"]))
        A("")

    # Per-activation wait obligations. The point of this table is that the row
    # count equals the activation count: a tally that hides an unwaited
    # activation cannot survive a row per activation.
    if r.get("wait_obligations"):
        A("### Wait obligations, one row per managed activation")
        A("")
        A("Each activation must be served by its own station visit waiting for "
          "the incoming head at its active temperature. A station visit appears "
          "at most once.")
        A("")
        A("| kind | head | activation line | serving station visit | wait line | "
          "wait S | required S | verdict |")
        A("|---|---|---|---|---|---|---|---|")
        for o in r["wait_obligations"]:
            A("| %s | T%d | %d | %s | %s | %s | %s | %s |"
              % (o["kind"], o["head"], o["activation_ln"],
                 o["station"] or "-", cell(o["wait_ln"]),
                 cell(o["wait_s"], "%.0f"), cell(o["required_s"], "%.0f"),
                 "served" if o["ok"] else (o["why"] or "unmet")))
        A("")

    # Departure withdrawals: one row per fibre window, so a missing withdrawal
    # is a visible row rather than an absent line in a count.
    if r.get("departure_withdrawals"):
        A("### Departure tool-change withdrawals, one row per fibre window")
        A("")
        A("Whether a close is a departure is decided by looking forward to the "
          "next fibre window and the next physical head change, not by the head "
          "selected at the close: an inter-strand close is also on T0 and owes "
          "nothing. A real T0 departure owes exactly one stationary V withdrawal "
          "between M1002 and the lift.")
        A("")
        A("| window | head at close | next window | next change | departure owed "
          "| basis | withdrawal lines | stationary V after close | verdict |")
        A("|---|---|---|---|---|---|---|---|---|")
        for d in r["departure_withdrawals"]:
            ok = (len(d["lines"]) == 1) if d["expected"] else (not d["lines"])
            A("| %d | T%s | %s | %s | %s | %s | %s | %s | %s |"
              % (d["window"], d["head"],
                 "L%s" % d["next_open_ln"] if d.get("next_open_ln") else "none",
                 "L%s" % d["change_ln"] if d.get("change_ln") else "none",
                 "yes" if d["expected"] else "no", d["basis"],
                 ", ".join(str(x) for x in d["lines"]) or "-",
                 ", ".join("%+.3f" % v for v in d["values"]) or "-",
                 "ok" if ok else "FAIL"))
        A("")
    A("## Cross-check: cut through release end")
    A("")
    A("| variant | expected total | deposition | release | verdict |")
    A("|---|---|---|---|---|")
    for c in payload["cross_totals"]:
        A("| %s | %.1f | %.1f | %.1f | %s |"
          % (c["variant"], c["expected"], c["dep"], c["rel"],
             "PASS" if not c["bad"] else "FAIL (%d windows)" % len(c["bad"])))
    A("")
    A("## Outstanding, and not claimed by this report")
    A("")
    for n in payload["limitations"]:
        A("- %s" % n)
    A("")
    return "\n".join(L)


# --------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------

def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--json-out", required=True)
    ap.add_argument("--md-out", required=True)
    ap.add_argument("--allow-missing", action="store_true",
                    help="evaluate only the files present (negative-test mode)")
    args = ap.parse_args(argv)

    with open(args.manifest, "r", encoding="utf-8") as fh:
        man = json.load(fh)

    macro = man.get("macro_contract", "MACRO_CONTRACT_UNVERIFIED")
    macro_status = {"VERIFIED": "VERIFIED", "UNVERIFIED": "UNVERIFIED",
                    "INCOMPATIBLE": "INCOMPATIBLE"}.get(
        macro.replace("MACRO_CONTRACT_", ""), "UNVERIFIED")

    results = []
    for entry in man["files"]:
        variant = entry["variant"]
        path = entry["gcode"]
        if not os.path.isabs(path):
            path = os.path.join(os.path.dirname(os.path.abspath(args.manifest)), path)
            path = os.path.normpath(path)
        # A negative-test mutant carries its own variant name; spec_variant
        # names the export it was mutated from, so the expectations come from
        # the real run rather than needing a table entry per mutant.
        spec = dict(VSPEC[entry.get("spec_variant", variant)])
        spec["bead_width_mm"] = entry.get("bead_width_mm",
                                        man.get("bead_width_mm", 0.7))
        spec["preheat_lead_s"] = entry.get("preheat_lead_s",
                                           man.get("preheat_lead_s", 15.0))
        # Payout formula inputs come from the EFFECTIVE merged config the run
        # used, so the verifier judges the emitted bytes against the numbers the
        # slicer actually sliced with rather than a re-read of one preset file.
        eff = entry.get("effective_config") or {}
        for fk in ("fs_fiber_rate", "fs_matrix_ratio", "fs_tail_v_factor",
                   "fs_prime_v", "fs_retract_v", "fs_toolchange_retract_v",
                   "fs_toolchange_lift_z", "fs_t0_temp", "fs_t1_temp"):
            if eff.get(fk) is not None:
                try:
                    spec[fk] = float(eff[fk])
                except (TypeError, ValueError):
                    pass
        # T1 is the plastic head; its active temperature is the filament
        # nozzle target, which is what its M109 must wait for.
        if eff.get("fs_t1_temp") is None:
            nt = eff.get("nozzle_temperature")
            if isinstance(nt, list) and nt:
                try:
                    spec["fs_t1_temp"] = float(nt[0])
                except (TypeError, ValueError):
                    pass
        nd = eff.get("nozzle_diameter")
        if isinstance(nd, list) and nd:
            try:
                spec["plastic_width_mm"] = float(nd[0])
            except (TypeError, ValueError):
                pass
        r = evaluate_file(variant, path, spec, entry.get("evidence"), macro_status)
        r["spec_variant"] = entry.get("spec_variant", variant)
        r["effective_config"] = entry.get("effective_config")
        r["command"] = entry.get("cmd")
        r["exit"] = entry.get("exit")
        results.append(r)

        # Hash verification: the manifest's claim against the bytes on disk.
        hv = Check("S00", "content hash")
        want = entry.get("sha256")
        if want is None:
            hv.status = NE
            hv.note("manifest records no hash")
        elif r.get("sha256") is None:
            hv.bad("file unreadable, cannot recompute")
        elif r["sha256"] != want:
            hv.bad("manifest says %s, file hashes to %s" % (want, r["sha256"]),
                   code="FS_HASH_MISMATCH")
        else:
            hv.ok("recomputed SHA-256 matches the manifest")
        r["checks"]["S00"] = hv

    # S12 across the three.
    s12 = check_s12(results, man)
    for r in results:
        r["checks"]["S12"] = s12

    cross = check_cross_totals(results)
    cross_bad = sum(len(c["bad"]) for c in cross)
    cross_summary = []
    for r in results:
        spec = VSPEC[r.get("spec_variant", r["variant"])]
        cb = next(c["bad"] for c in cross if c["variant"] == r["variant"])
        cross_summary.append(dict(variant=r["variant"], expected=spec["total"],
                                  dep=spec["dep"], rel=spec["rel"], bad=cb))
    if cross_bad:
        for r in results:
            cs = next(c for c in cross_summary if c["variant"] == r["variant"])
            if cs["bad"]:
                r["checks"]["S02"].bad("cross-check total: " + cs["bad"][0])
                r["checks"]["S03"].bad("cross-check total: " + cs["bad"][0])

    # Unsupported dialect is a parser failure, not a pass.
    for r in results:
        if r.get("unsupported"):
            c = Check("S00", "dialect coverage")
            c.bad("%d commands outside the modelled dialect, e.g. line %d: %s"
                  % (len(r["unsupported"]), r["unsupported"][0][0],
                     r["unsupported"][0][1]))
            r["checks"]["S00"] = c

    for r in results:
        r["window_rows"] = window_rows(r)
        r["transition_rows"] = transition_rows(r)

    # Serialise the Check objects before anything reads them as data, so the
    # JSON, the Markdown and the verdict all see the same representation.
    for r in results:
        r["checks"] = dict((k, c.to_json()) for k, c in r["checks"].items())

    # ---- verdict -------------------------------------------------------
    def status_of(r, cid):
        c = r["checks"].get(cid)
        return c["status"] if c else NE

    all_pass = True
    verdict_rows = []
    for r in results:
        row = dict(variant=r["variant"], per_check={})
        for cid in MANDATORY:
            st = status_of(r, cid)
            row["per_check"][cid] = st
            if st != PASS:
                all_pass = False
        s00 = r["checks"].get("S00")
        if s00 and s00["status"] != PASS:
            all_pass = False
        verdict_rows.append(row)

    if macro_status == "INCOMPATIBLE":
        all_pass = False

    software = "PASS" if all_pass else "FAIL"

    payload = dict(
        software_acceptance=software,
        macro_contract=macro_status,
        physical_print_result=man.get("physical_print_result", "NOT_TESTED"),
        provenance=man.get("provenance", {}),
        clock=man.get("clock", {}),
        verdicts=verdict_rows,
        cross_totals=cross_summary,
        limitations=man.get("limitations", []) + [
            "Physical tail clearance and print quality are UNMEASURED until operator trials.",
            "A nominal software PASS does not establish that spikes or stringing are solved.",
            "Tight-turn behaviour and Rocket motion-setting differences remain outstanding.",
            "The separate missing-intermediate-solid-layer investigation is not addressed here; "
            "a layer-count check does not clear a partially missing-infill defect.",
        ],
        files=[dict((k, v) for k, v in r.items() if k != "windows") for r in results],
    )

    def clean(o):
        if isinstance(o, dict):
            return dict((k, clean(v)) for k, v in o.items())
        if isinstance(o, (list, tuple)):
            return [clean(v) for v in o]
        if isinstance(o, float) and not math.isfinite(o):
            return None
        return o

    payload = clean(payload)
    d = os.path.dirname(os.path.abspath(args.json_out))
    if d and not os.path.isdir(d):
        os.makedirs(d)
    with open(args.json_out, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=1)
    with open(args.md_out, "w", encoding="utf-8") as fh:
        fh.write(render_md(payload))

    # ---- console summary ----------------------------------------------
    print("=" * 78)
    for r in payload["files"]:
        print("%-28s %s" % (r["variant"], " ".join(
            "%s=%s" % (c, r["checks"][c]["status"][0]) for c in MANDATORY
            if c in r["checks"])))
    print("=" * 78)
    print("SOFTWARE_EXPORT_ACCEPTANCE: %s" % software)
    print("MACRO_CONTRACT: %s" % macro_status)
    print("PHYSICAL_PRINT_RESULT: %s" % payload["physical_print_result"])
    fails = [(r["variant"], cid, r["checks"][cid])
             for r in payload["files"] for cid in MANDATORY
             if cid in r["checks"] and r["checks"][cid]["status"] != PASS]
    for v, cid, c in fails:
        # The first detail line is often an informational note appended before
        # the failure, so reporting it made a real failure look like a passing
        # note. Report the recorded error codes and the first line that actually
        # states a measured-vs-required mismatch.
        msg = ""
        for d in c["detail"]:
            low = d.lower()
            if ("required" in low or "expected" in low or "missing" in low
                    or "matches none" in low or "percent off" in low
                    or "escapes" in low or "outside" in low):
                msg = d
                break
        if not msg:
            msg = c["detail"][-1] if c["detail"] else "no detail"
        print("  FAIL %s %s [%s]: %s"
              % (v, cid, ",".join(c["codes"]) or "no code", msg))
    return 0 if software == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
