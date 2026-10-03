// License: GNU AGPLv3 or higher

#include "FiberStrand.hpp"

#include <cmath>

namespace Slic3r {
namespace Fiber {

double FiberStrand::path_length() const
{
    double len = 0.0;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const double dx = pts[i + 1].x - pts[i].x;
        const double dy = pts[i + 1].y - pts[i].y;
        len += std::sqrt(dx * dx + dy * dy);
    }
    return len;
}

long FiberStrand::budget_L(double restart_feed) const
{
    // Same convention as FiberRun: L = floor(restart_printed + sum(u_printed)).
    // Tail moves carry no U (the blade is upstream of every drive that can act
    // on the severed tail), so they never enter the budget.
    const double total = round3(restart_feed) + total_u_feed;
    return static_cast<long>(std::floor(total + 1e-9));
}

bool FiberStrand::finalize(std::string* error)
{
    auto fail = [this, error](const char* msg) {
        if (error) {
            *error = msg;
        }
        body_pts.clear();
        body_u.clear();
        tail_pts.clear();
        tail_v.clear();
        return false;
    };

    if (!std::isfinite(z) || layer_id == 0) {
        return fail("FiberStrand: layer_id must be >= 1 and z finite");
    }
    if (pts.size() < 2) {
        return fail("FiberStrand: path needs at least two points");
    }
    if (!(ratio_p > 0.0) || !std::isfinite(ratio_p)) {
        return fail("FiberStrand: matrix:fiber ratio P must be finite and > 0");
    }
    if (!(fiber_rate > 0.0) || !std::isfinite(fiber_rate)) {
        return fail("FiberStrand: fiber_rate must be finite and > 0");
    }
    if (!(feed_mm_min > 0.0) || !std::isfinite(feed_mm_min)) {
        return fail("FiberStrand: feed_mm_min must be finite and > 0");
    }
    if (!(tail_length_mm > 0.0) || !std::isfinite(tail_length_mm)) {
        return fail("FiberStrand: calibrated tail_length_mm must be finite and > 0 (a strand without a reserved tail suffix is not a strand)");
    }
    if (!(tail_v_factor > 0.0) || tail_v_factor > 1.0 || !std::isfinite(tail_v_factor)) {
        return fail("FiberStrand: tail_v_factor must be finite and in (0,1]");
    }

    // Validate geometry and collect segment lengths up front.
    std::vector<double> seg_len(pts.size() - 1, 0.0);
    double total = 0.0;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        if (!std::isfinite(pts[i].x) || !std::isfinite(pts[i].y) || !std::isfinite(pts[i + 1].x) || !std::isfinite(pts[i + 1].y)) {
            return fail("FiberStrand: non-finite path coordinate");
        }
        const double dx = pts[i + 1].x - pts[i].x;
        const double dy = pts[i + 1].y - pts[i].y;
        const double seg = std::sqrt(dx * dx + dy * dy);
        if (seg < 1e-6) {
            // Dropping or merging a zero-length segment would silently simplify
            // a finalized strand, which the model forbids. Reject instead.
            return fail("FiberStrand: zero-length segment (paths must not repeat points)");
        }
        seg_len[i] = seg;
        total += seg;
    }
    total_path = total;

    // Early-cut schedule (operator ruling 2026-10-01): the blade fires at
    // total_path - tail_length so the severed tail becomes the strand's final
    // deposited section. The path must carry BOTH a body and the suffix.
    const double s_cut = total - tail_length_mm;
    if (!(s_cut > 0.0)) {
        return fail("FiberStrand: path shorter than the calibrated tail (isolated feature: report it or apply the plastic-only policy)");
    }
    if (!(round3(s_cut * fiber_rate) > 0.0)) {
        return fail("FiberStrand: body feed rounds to zero before the cut (fiber without matrix would be emitted)");
    }
    // tail_v_factor is a fraction of the BODY matrix payout: the tail deposits
    // matrix at (tail_v_factor * fiber_rate * ratio_p) mm of V per mm of tail
    // path, so the post-cut V/XY ratio matches the joint U/V deposit ratio
    // instead of jumping to ~1.0 (G-code review finding #1, 2026-10-01).
    if (!(round3(tail_length_mm * tail_v_factor * fiber_rate * ratio_p) > 0.0)) {
        return fail("FiberStrand: tail matrix feed rounds to zero under the calibrated factor");
    }

    auto point_at = [&](size_t seg_idx, double t) {
        const FiberPoint& a = pts[seg_idx];
        const FiberPoint& b = pts[seg_idx + 1];
        FiberPoint p;
        p.x = a.x + (b.x - a.x) * t;
        p.y = a.y + (b.y - a.y) * t;
        return p;
    };

    body_pts.clear();
    body_u.clear();
    tail_pts.clear();
    tail_v.clear();

    // Walk the polyline, splitting it at s_cut into a body prefix (joint U/V
    // deposits) and a tail suffix (V-only deposits after the cut). Sub-moves
    // whose printed feed would round to zero are merged forward (geometry is
    // never dropped - only the feed accumulates into the next emitted move).
    double cum = 0.0;
    double u_acc = 0.0;
    double v_acc = 0.0;
    bool   in_tail   = false;
    FiberPoint cut_pt = pts.front();

    // The emitter derives each body V from the PRINTED U at the same 3-decimal
    // precision, so a body move is only printable when BOTH feeds survive the
    // rounding: a move whose matrix feed rounds to zero would deposit fiber
    // without matrix. Mirrors the emitter's rounding of the ratio exactly.
    const double p_round = round3(ratio_p);
    auto body_move_printable = [&](double u_printed) {
        return u_printed > 0.0 && round3(u_printed * p_round) > 0.0;
    };

    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const double len = seg_len[i];
        if (!in_tail && cum + len <= s_cut + 1e-9) {
            // Entirely before the cut.
            u_acc += len * fiber_rate;
            const double printed = round3(u_acc);
            if (body_move_printable(printed)) {
                body_pts.push_back(pts[i + 1]);
                body_u.push_back(printed);
                u_acc = 0.0;
            }
            cum += len;
            continue;
        }
        if (!in_tail) {
            // This segment contains the cut point: split it.
            const double t = (s_cut - cum) / len; // 0..1 within this segment
            const double body_part = len * t;
            const double tail_part = len - body_part;
            cut_pt = point_at(i, t);
            u_acc += body_part * fiber_rate;
            const double printed = round3(u_acc);
            if (body_move_printable(printed)) {
                body_pts.push_back(cut_pt);
                body_u.push_back(printed);
                u_acc = 0.0;
            } else if (!body_u.empty()) {
                // Sub-tick remainder at the cut boundary: fold into the last
                // body move (same endpoint, geometry unchanged).
                body_u.back() = round3(body_u.back() + u_acc);
                u_acc = 0.0;
            }
            v_acc += tail_part * tail_v_factor * fiber_rate * ratio_p;
            // Flush the post-cut remainder at the vertex closing this segment
            // so the turn at that vertex stays in the deposited path. A sub-
            // tick remainder (path < 0.001 mm, below print precision) stays in
            // v_acc and carries into the next vertex move instead.
            const double v_printed = round3(v_acc);
            if (v_printed > 0.0) {
                tail_pts.push_back(pts[i + 1]);
                tail_v.push_back(v_printed);
                v_acc = 0.0;
            }
            in_tail = true;
            cum += len;
            continue;
        }
        // Wholly inside the tail suffix (or the post-split remainder handled by
        // the next iterations from tail_from).
        v_acc += len * tail_v_factor * fiber_rate * ratio_p;
        const double printed = round3(v_acc);
        if (printed > 0.0) {
            tail_pts.push_back(pts[i + 1]);
            tail_v.push_back(printed);
            v_acc = 0.0;
        }
        cum += len;
    }

    // The tail suffix must end exactly at the strand endpoint. If trailing
    // accumulation never crossed the printed tick, flush it at the endpoint.
    if (in_tail) {
        if (v_acc > 0.0) {
            const double printed = round3(v_acc);
            if (printed > 0.0) {
                tail_pts.push_back(pts.back());
                tail_v.push_back(printed);
            } else if (!tail_v.empty()) {
                tail_v.back() = round3(tail_v.back() + v_acc);
            }
        } else if (tail_pts.empty() || tail_pts.back().x != pts.back().x || tail_pts.back().y != pts.back().y) {
            // Tail accumulated to zero printed feed but the path continued:
            // a V-free post-cut XY move would be attached travel, which the
            // ruling forbids. Emit the endpoint move with the last tail value
            // only if that keeps the move material-bearing; otherwise reject.
            return fail("FiberStrand: tail suffix produced no printed matrix feed (tail would deposit as inert travel)");
        }
    } else {
        // Cut fell at/after the last vertex within tolerance: no tail suffix
        // exists. A strand without a tail suffix is the legacy per-run shape,
        // not a strand - reject rather than silently degrade.
        return fail("FiberStrand: cut position at path end (no tail suffix to reserve)");
    }

    if (body_u.empty()) {
        return fail("FiberStrand: no body moves before the cut");
    }
    if (tail_v.empty()) {
        return fail("FiberStrand: no tail moves after the cut");
    }

    double u_total = 0.0;
    for (double u : body_u) {
        u_total += u;
    }
    double v_total = 0.0;
    for (double v : tail_v) {
        v_total += v;
    }
    total_u_feed = round3(u_total);
    total_v_tail = round3(v_total);
    finalized    = true;
    return true;
}

} // namespace Fiber
} // namespace Slic3r
