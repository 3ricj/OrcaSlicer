// License: GNU AGPLv3 or higher

#include "FiberStrandPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

#include "../ClipperUtils.hpp"

namespace Slic3r {
namespace Fiber {

namespace {

// Closed-loop perimeter in mm (input Points are a ring without closing duplicate).
double ring_length_mm(const Points& pts)
{
    double len = 0.0;
    const size_t n = pts.size();
    for (size_t i = 0; i < n; ++i) {
        const Point& a = pts[i];
        const Point& b = pts[(i + 1) % n];
        const double dx = unscale<double>(b.x() - a.x());
        const double dy = unscale<double>(b.y() - a.y());
        len += std::sqrt(dx * dx + dy * dy);
    }
    return len;
}

// Length of an mm point run as planned (a closed loop carries its closing
// duplicate, so this is its full perimeter).
double path_length_mm(const std::vector<FiberPoint>& pts)
{
    double len = 0.0;
    for (size_t i = 0; i + 1 < pts.size(); ++i)
        len += std::hypot(pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y);
    return len;
}

// Signed shoelace area of a scaled ring; positive when counter-clockwise.
// Sign only: the orientation source of truth, independent of any library
// convention.
double ring_signed_area(const Points& pts)
{
    double a = 0.0;
    const size_t n = pts.size();
    for (size_t i = 0; i < n; ++i) {
        const Point& p = pts[i];
        const Point& q = pts[(i + 1) % n];
        a += double(p.x()) * double(q.y()) - double(q.x()) * double(p.y());
    }
    return 0.5 * a;
}

// Strict even-odd containment of a scaled point in a scaled ring. A point on
// the boundary (or coincident with a vertex) is NOT inside: perimeters that
// merely touch must not be mistaken for nesting; only proper containment counts.
bool point_strictly_inside(const Points& poly, const Point& pt)
{
    const size_t n = poly.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const Point& a = poly[i];
        const Point& b = poly[j];
        if (pt.x() == a.x() && pt.y() == a.y())
            return false; // coincident vertex: touching, not containment
        const int64_t cr = int64_t(b.x() - a.x()) * int64_t(pt.y() - a.y()) -
                           int64_t(b.y() - a.y()) * int64_t(pt.x() - a.x());
        if (cr == 0 &&
            std::min(a.x(), b.x()) <= pt.x() && pt.x() <= std::max(a.x(), b.x()) &&
            std::min(a.y(), b.y()) <= pt.y() && pt.y() <= std::max(a.y(), b.y()))
            return false; // on an edge
    }
    bool in = false;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const Point& a = poly[i];
        const Point& b = poly[j];
        if ((a.y() > pt.y()) != (b.y() > pt.y())) {
            const double xint = double(a.x()) + (double(pt.y()) - double(a.y())) *
                (double(b.x()) - double(a.x())) / double(b.y() - a.y());
            if (double(pt.x()) < xint)
                in = !in;
        }
    }
    return in;
}

// Lexicographic minimum vertex of a ring: deterministic sort/rotation key.
std::pair<int64_t, int64_t> ring_lexmin(const Points& pts)
{
    std::pair<int64_t, int64_t> k = {std::numeric_limits<int64_t>::max(), std::numeric_limits<int64_t>::max()};
    for (const Point& p : pts)
        k = std::min(k, {p.x(), p.y()});
    return k;
}

// Ring -> closed mm point list (closing duplicate of the first vertex appended,
// so the strand is a full deposition loop). Adjacent identical points (possible
// from clipper joins) are collapsed: dropping a zero-length duplicate is not
// a geometric simplification, it is the same point.
std::vector<FiberPoint> ring_to_mm_closed(const Points& pts)
{
    std::vector<FiberPoint> out;
    out.reserve(pts.size() + 1);
    for (const Point& p : pts) {
        const Vec2d v = unscale(p);
        if (!out.empty() && out.back().x == v.x() && out.back().y == v.y())
            continue;
        out.push_back({v.x(), v.y()});
    }
    if (out.size() >= 2 && out.front().x == out.back().x && out.front().y == out.back().y)
        out.pop_back();
    out.push_back(out.front());
    return out;
}

// Rotate the closed ring so it starts at its lexicographically smallest vertex:
// the same geometric loop always yields the same point list (determinism).
// The rotation only permutes the open vertex run; the appended closing
// duplicate must be re-anchored to the new first vertex, or the loop would
// close on a chord instead of its boundary edge.
void canonical_rotate(std::vector<FiberPoint>& closed)
{
    const size_t n = closed.size() - 1; // exclude the closing duplicate
    if (n < 2)
        return;
    size_t best = 0;
    for (size_t i = 1; i < n; ++i) {
        if (closed[i].x < closed[best].x || (closed[i].x == closed[best].x && closed[i].y < closed[best].y))
            best = i;
    }
    std::rotate(closed.begin(), closed.begin() + best, closed.begin() + n);
    closed.back() = closed.front();
}

// Move the start of a closed ring to the point `target` mm along it, measured
// from the current start. A target that falls mid-edge gets a vertex inserted
// there: the inserted point is on the edge, so the ring and its length are
// unchanged and only the seam moves.
void rotate_to_arc_position(std::vector<FiberPoint>& closed, double target)
{
    const size_t n = closed.size() - 1; // exclude the closing duplicate
    if (n < 3 || target <= 0.0)
        return;
    double acc = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const FiberPoint& a = closed[i];
        const FiberPoint& b = closed[i + 1];
        const double seg = std::hypot(b.x - a.x, b.y - a.y);
        if (acc + seg <= target) {
            acc += seg;
            continue;
        }
        const double along = target - acc; // mm past vertex i
        size_t start = i;
        if (along > EPSILON) {
            const double t = along / seg;
            closed.insert(closed.begin() + (i + 1), {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)});
            start = i + 1;
        }
        std::rotate(closed.begin(), closed.begin() + start, closed.end() - 1);
        closed.back() = closed.front();
        return;
    }
}

// Deterministic scatter offset for one loop: a hash of where the loop sits in
// the layer, never a random draw, so re-slicing the same model reproduces the
// same seams while consecutive layers break in different places.
double scatter_offset(double total, size_t layer_id, size_t island, size_t level, size_t loop)
{
    uint64_t h = 0x9E3779B97F4A7C15ull;
    for (const uint64_t v : {uint64_t(layer_id), uint64_t(island), uint64_t(level), uint64_t(loop)}) {
        h ^= v + 0x9E3779B97F4A7C15ull + (h << 6) + (h >> 2);
        h *= 0xBF58476D1CE4E5B9ull;
        h ^= h >> 31;
    }
    return total * double(h % 1000000ull) / 1000000.0;
}

// Arc position of the midpoint of the ring's longest edge, so the seam lands as
// far from a corner as that ring allows.
double longest_edge_midpoint(const std::vector<FiberPoint>& closed)
{
    const size_t n = closed.size() - 1;
    double acc = 0.0, best_len = -1.0, best_mid = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double seg = std::hypot(closed[i + 1].x - closed[i].x, closed[i + 1].y - closed[i].y);
        if (seg > best_len) {
            best_len = seg;
            best_mid = acc + 0.5 * seg;
        }
        acc += seg;
    }
    return best_mid;
}

// Place the loop's seam per the operator's policy. aligned leaves the canonical
// rotation alone, which is why an unset profile is byte-identical.
void place_seam(std::vector<FiberPoint>& closed, const StrandLayerParams& params,
                size_t island, size_t level, size_t loop)
{
    switch (params.seam_position) {
    case FiberSeamPosition::fspScattered:
        rotate_to_arc_position(closed, scatter_offset(path_length_mm(closed), params.layer_id, island, level, loop));
        break;
    case FiberSeamPosition::fspLongestEdge:
        rotate_to_arc_position(closed, longest_edge_midpoint(closed));
        break;
    case FiberSeamPosition::fspAligned:
        break;
    }
}

// Reverse the deposition direction of a closed ring, start vertex preserved.
void reverse_direction(std::vector<FiberPoint>& closed)
{
    const size_t n = closed.size() - 1;
    if (n < 3)
        return;
    std::vector<FiberPoint> out(closed.size());
    for (size_t i = 0; i < closed.size(); ++i)
        out[i] = closed[(n - i) % n];
    closed.swap(out);
}

// ---------------------------------------------------------------------------
// Serpentine interior fill helpers (operator ruling 2026-10-01, fill build).
// ---------------------------------------------------------------------------

constexpr double FILL_PI = 3.14159265358979323846;

// Boundary-touching counts as INSIDE (closed region test). Used for connector
// containment: a connector grazing a hole edge is treated as leaving material
// (conservative: the strand breaks instead of risking deposition over void).
bool point_closed_inside(const Points& poly, const Point& pt)
{
    const size_t n = poly.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const Point& a = poly[i];
        const Point& b = poly[j];
        if (pt.x() == a.x() && pt.y() == a.y())
            return true; // coincident vertex: on the region, counts as inside
        const int64_t cr = int64_t(b.x() - a.x()) * int64_t(pt.y() - a.y()) -
                           int64_t(b.y() - a.y()) * int64_t(pt.x() - a.x());
        if (cr == 0 &&
            std::min(a.x(), b.x()) <= pt.x() && pt.x() <= std::max(a.x(), b.x()) &&
            std::min(a.y(), b.y()) <= pt.y() && pt.y() <= std::max(a.y(), b.y()))
            return true; // on an edge
    }
    bool in = false;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const Point& a = poly[i];
        const Point& b = poly[j];
        if ((a.y() > pt.y()) != (b.y() > pt.y())) {
            const double xint = double(a.x()) + (double(pt.y()) - double(a.y())) *
                (double(b.x()) - double(a.x())) / double(b.y() - a.y());
            if (double(pt.x()) < xint)
                in = !in;
        }
    }
    return in;
}

bool inside_material(const ExPolygon& isl, const Point& pt)
{
    if (!point_closed_inside(isl.contour.points, pt))
        return false;
    for (const Polygon& h : isl.holes)
        if (point_closed_inside(h.points, pt))
            return false;
    return true;
}

// A straight connector may be deposited only if it stays inside material along
// its whole length. Sampled conservatively: at least one test per half-pitch,
// every sample (endpoints included) must be inside the contour and outside
// every hole (closed test: grazing counts as outside).
bool connector_inside(const ExPolygon& isl, const Point& from, const Point& to, double pitch_mm)
{
    if (!inside_material(isl, from) || !inside_material(isl, to))
        return false;
    const double len_scaled = std::hypot(double(to.x() - from.x()), double(to.y() - from.y()));
    const double step = std::max(scale_(0.5 * pitch_mm), scale_(0.25)); // stride floor 0.25 mm
    int n = int(std::ceil(len_scaled / step));
    n = std::min(std::max(n, 2), 256);
    for (int i = 1; i < n; ++i) { // interior samples
        const double t = double(i) / double(n);
        Point s;
        s.x() = std::llround(double(from.x()) + t * double(to.x() - from.x()));
        s.y() = std::llround(double(from.y()) + t * double(to.y() - from.y()));
        if (!inside_material(isl, s))
            return false;
    }
    return true;
}

double dist_scaled(const Point& a, const Point& b)
{
    return std::hypot(double(b.x() - a.x()), double(b.y() - a.y()));
}

// One admitted fill chord (scaled bed units), oriented with the scan direction:
// a is the smaller along-angle endpoint.
struct FillChord {
    Point    a;
    Point    b;
    long     family; // rib family index (isogrid: 0/1/2; single-pass: always 0)
    long     scan;   // scanline index (deterministic ordering key)
    int      sub;    // pass within one rib (isogrid twin pass: 0/1)
    double   along_a; // scaled along-angle coordinate of a (within-scan order)
};

bool chord_before(const FillChord& x, const FillChord& y)
{
    if (x.family != y.family)
        return x.family < y.family;
    if (x.scan != y.scan)
        return x.scan < y.scan;
    if (x.sub != y.sub)
        return x.sub < y.sub;
    return x.along_a < y.along_a;
}

// Deterministic island / lane order: area descending, then bbox min, then
// vertex count. Shared by the island sort and the fiber-lane sort so both are
// reproducible for identical input.
bool expoly_before(const ExPolygon& a, const ExPolygon& b)
{
    const double aa = a.area();
    const double ab = b.area();
    if (aa != ab)
        return aa > ab;
    const BoundingBox ba = a.contour.bounding_box();
    const BoundingBox bb = b.contour.bounding_box();
    if (ba.min.x() != bb.min.x())
        return ba.min.x() < bb.min.x();
    if (ba.min.y() != bb.min.y())
        return ba.min.y() < bb.min.y();
    return a.contour.size() < b.contour.size();
}

// A joint counts as a TURN once the heading changes by more than this. Below it
// the vertex is clipper/geometry noise on a straight run, not a corner the
// roving has to be bent around.
constexpr double TURN_EPS_RAD = 5.0 * FILL_PI / 180.0;

// Joints of a planned deposition path whose corner is tighter than
// min_radius_mm, in ascending order (min_radius_mm 0 = off, empty result). The
// largest circular fillet that fits in the corner at a vertex while consuming
// at most half of each adjacent segment is
// min(len_in, len_out) / (2 * tan(|dtheta| / 2)): a straight joint is unbounded,
// a right-angle corner on legs of L comes out at L/2, and a serpentine reversal
// over one pitch of lateral offset comes out at pitch/2 - which is what makes a
// fine serpentine illegal on a machine with a real minimum bend radius.
// The planner never widens or straightens a path to make a corner legal and
// reducing the feedrate cannot legalize a bend, so the caller either deposits
// the corner anyway or cuts the fiber there (fs_fiber_tight_turn_policy).
//
// `closed` means pts carries a duplicated closing vertex and wraps.
std::vector<size_t> tight_turn_joints(const std::vector<FiberPoint>& pts, bool closed, double min_radius_mm)
{
    std::vector<size_t> out;
    if (!(min_radius_mm > 0.0))
        return out;
    const size_t n = closed && pts.size() >= 2 ? pts.size() - 1 : pts.size();
    if (n < 3)
        return out;
    const auto seg_len = [&pts, n](size_t i) {
        const FiberPoint& a = pts[i % n];
        const FiberPoint& b = pts[(i + 1) % n];
        return std::hypot(b.x - a.x, b.y - a.y);
    };
    const auto heading = [&pts, n](size_t i) {
        const FiberPoint& a = pts[i % n];
        const FiberPoint& b = pts[(i + 1) % n];
        return std::atan2(b.y - a.y, b.x - a.x);
    };
    // Joint i joins segment i-1 and segment i. A closed loop has all n joints;
    // an open polyline has the interior ones only.
    const size_t first = closed ? 0 : 1;
    const size_t last  = closed ? n - 1 : n - 2;
    for (size_t i = first; i <= last; ++i) {
        const size_t in = (i + n - 1) % n;
        double d = heading(i) - heading(in);
        while (d >  FILL_PI)
            d -= 2.0 * FILL_PI;
        while (d < -FILL_PI)
            d += 2.0 * FILL_PI;
        const double ad = std::abs(d);
        if (ad <= TURN_EPS_RAD)
            continue;
        const double shortest = std::min(seg_len(in), seg_len(i));
        // tan is clamped short of pi/2 so an exact reversal yields a finite,
        // effectively zero radius instead of a NaN.
        const double t = std::tan(0.5 * std::min(ad, FILL_PI - 1e-9));
        const double r = t > 0.0 ? shortest / (2.0 * t) : std::numeric_limits<double>::max();
        if (r < min_radius_mm)
            out.push_back(i);
    }
    return out;
}

// Replace each corner tighter than min_radius with a circular fillet of the
// largest radius that fits in the corner (capped at min_radius). The arc sits
// on the inside of the turn, so a convex outer loop moves inboard and stays
// in material. Hairpins that cannot reach min_radius are still rounded as far
// as the legs allow instead of being deposited as two abrupt vertices.
void fillet_path_turns(std::vector<FiberPoint>& pts, bool closed, double min_radius_mm)
{
    if (!(min_radius_mm > 0.0) || pts.size() < 3)
        return;
    const size_t n = closed && pts.size() >= 2 ? pts.size() - 1 : pts.size();
    if (n < 3)
        return;
    const size_t first = closed ? 0 : 1;
    const size_t last  = closed ? n - 1 : n - 2;
    std::vector<FiberPoint> out;
    out.reserve(pts.size() * 2);
    if (!closed)
        out.push_back(pts.front());
    for (size_t i = first; i <= last; ++i) {
        const FiberPoint& a = pts[(i + n - 1) % n];
        const FiberPoint& b = pts[i % n];
        const FiberPoint& c = pts[(i + 1) % n];
        const double lin = std::hypot(b.x - a.x, b.y - a.y);
        const double lout = std::hypot(c.x - b.x, c.y - b.y);
        const double hin = std::atan2(b.y - a.y, b.x - a.x);
        const double hout = std::atan2(c.y - b.y, c.x - b.x);
        double d = hout - hin;
        while (d >  FILL_PI) d -= 2.0 * FILL_PI;
        while (d < -FILL_PI) d += 2.0 * FILL_PI;
        const double ad = std::abs(d);
        const double t = std::tan(0.5 * std::min(std::max(ad, 1e-9), FILL_PI - 1e-9));
        const double rmax = (ad <= TURN_EPS_RAD || !(t > 0.0)) ? 0.0 : std::min(lin, lout) / (2.0 * t);
        const double r = std::min(min_radius_mm, rmax);
        if (ad <= TURN_EPS_RAD || r < 0.15) {
            out.push_back(b);
            continue;
        }
        const double trim = r * t;
        const FiberPoint p0{ b.x - std::cos(hin) * trim, b.y - std::sin(hin) * trim };
        const FiberPoint p1{ b.x + std::cos(hout) * trim, b.y + std::sin(hout) * trim };
        const double side = (d >= 0.0) ? 1.0 : -1.0;
        const double cang = hin + side * FILL_PI / 2.0;
        const FiberPoint ctr{ p0.x + std::cos(cang) * r, p0.y + std::sin(cang) * r };
        double a0 = std::atan2(p0.y - ctr.y, p0.x - ctr.x);
        double a1 = std::atan2(p1.y - ctr.y, p1.x - ctr.x);
        double sweep = a1 - a0;
        while (sweep >  FILL_PI) sweep -= 2.0 * FILL_PI;
        while (sweep < -FILL_PI) sweep += 2.0 * FILL_PI;
        if (side > 0.0 && sweep < 0.0) sweep += 2.0 * FILL_PI;
        if (side < 0.0 && sweep > 0.0) sweep -= 2.0 * FILL_PI;
        const int steps = std::max(2, int(std::ceil(std::abs(sweep) * r / 1.0)));
        auto push_pt = [&out](const FiberPoint& q) {
            if (out.empty() || std::hypot(q.x - out.back().x, q.y - out.back().y) >= 1e-6)
                out.push_back(q);
        };
        push_pt(p0);
        for (int k = 1; k < steps; ++k) {
            const double ang = a0 + sweep * (double(k) / double(steps));
            push_pt(FiberPoint{ ctr.x + std::cos(ang) * r, ctr.y + std::sin(ang) * r });
        }
        push_pt(p1);
    }
    if (!closed) {
        if (out.empty() || std::hypot(pts.back().x - out.back().x, pts.back().y - out.back().y) >= 1e-6)
            out.push_back(pts.back());
    } else if (!out.empty()) {
        // A fully consumed side makes the last fillet's p1 coincide with the
        // first fillet's p0; that already closes the ring. A second copy of
        // the start vertex is a zero-length segment that finalize rejects.
        if (std::hypot(out.back().x - out.front().x, out.back().y - out.front().y) >= 1e-6)
            out.push_back(out.front());
    }
    if (out.size() >= 3)
        pts.swap(out);
}

// Rotate a closed loop so the last tail_length_mm of deposition contains as
// few remaining tight joints as possible (the tail is where a reversal at a
// hook tip is most expensive). Tie-break: furthest from the first tight joint.
void rotate_tail_off_tight(std::vector<FiberPoint>& closed, double tail_length_mm, double min_radius_mm)
{
    if (closed.size() < 4 || !(min_radius_mm > 0.0) || !(tail_length_mm > 0.0))
        return;
    const size_t n = closed.size() - 1;
    const std::vector<size_t> tight = tight_turn_joints(closed, true, min_radius_mm);
    if (tight.empty())
        return;
    const double total = path_length_mm(closed);
    if (!(total > tail_length_mm))
        return;
    size_t best_i = 0;
    size_t best_hits = tight.size() + 1;
    for (size_t s = 0; s < n; ++s) {
        double acc = 0.0;
        size_t hits = 0;
        for (size_t k = 0; k < n; ++k) {
            const size_t a = (s + k) % n;
            const size_t b = (s + k + 1) % n;
            acc += std::hypot(closed[b].x - closed[a].x, closed[b].y - closed[a].y);
            const double from_end = total - acc;
            if (from_end > tail_length_mm)
                continue;
            for (size_t j : tight) {
                if (j == b % n || j == a)
                    ++hits;
            }
        }
        if (hits < best_hits) {
            best_hits = hits;
            best_i = s;
        }
    }
    if (best_i == 0)
        return;
    std::vector<FiberPoint> rot;
    rot.reserve(closed.size());
    for (size_t k = 0; k < n; ++k)
        rot.push_back(closed[(best_i + k) % n]);
    rot.push_back(rot.front());
    closed.swap(rot);
}

// Cut a planned path at the given joints so no deposited piece turns tighter
// than the limit. The offending vertex becomes an ENDPOINT of both neighboring
// pieces: the fiber is cut there and restarted, so the bend itself is never
// deposited. A closed ring opens at its first offending joint and is cut at the
// rest, yielding one piece per joint; an open path yields one more piece than
// joints. Pieces shorter than two points are dropped by the caller's viability
// check like any other short feature.
std::vector<std::vector<FiberPoint>> split_at_joints(const std::vector<FiberPoint>& pts, bool closed,
                                                     const std::vector<size_t>& joints)
{
    std::vector<std::vector<FiberPoint>> out;
    if (joints.empty())
        return out;
    const size_t n = closed && pts.size() >= 2 ? pts.size() - 1 : pts.size();
    const auto at = [&pts, n](size_t i) { return pts[i % n]; };
    if (closed) {
        for (size_t k = 0; k < joints.size(); ++k) {
            const size_t from = joints[k];
            // Walk forward to the next cut, wrapping; a lone cut walks the
            // whole ring back to itself and simply opens it there.
            const size_t to   = joints[(k + 1) % joints.size()];
            const size_t span = (to + n - from) % n;
            std::vector<FiberPoint> piece;
            piece.reserve(span + 1);
            for (size_t s = 0; s <= (span == 0 ? n : span); ++s)
                piece.push_back(at(from + s));
            out.push_back(std::move(piece));
        }
    }
    else {
        size_t from = 0;
        for (const size_t j : joints) {
            if (j > from)
                out.emplace_back(pts.begin() + from, pts.begin() + j + 1);
            from = j;
        }
        if (from + 1 < n)
            out.emplace_back(pts.begin() + from, pts.end());
    }
    return out;
}

// Deposition resolution bound (fs_fiber_max_arc_seg, reference machine
// FiberMaxArcSegmentLength 3 to 4 mm). That machine reports its fiber windows at
// a far finer resolution than the planned geometry needs - a 241 mm loop comes
// out as ~195 moves, median segment 0.96 mm - so the fiber and matrix feeds stay
// in step with XY along the whole path and a turn is never commanded as one long
// chord. Splitting a segment inserts COLLINEAR vertices: the deposited geometry
// and its length are identical, only the reporting resolution changes, so this
// is a resampling and not a simplification. 0 leaves the path exactly as
// planned, which is the legacy byte stream.
void resample_max_segment(std::vector<FiberPoint>& pts, double max_seg_mm)
{
    if (!(max_seg_mm > 0.0) || pts.size() < 2)
        return;
    std::vector<FiberPoint> out;
    out.reserve(pts.size());
    out.push_back(pts.front());
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const FiberPoint& a = pts[i];
        const FiberPoint& b = pts[i + 1];
        const double len = std::hypot(b.x - a.x, b.y - a.y);
        // The epsilon keeps a segment that is an exact multiple of the bound
        // from picking up one extra division through floating-point slack.
        const long n = std::max(1L, long(std::ceil(len / max_seg_mm - 1e-9)));
        for (long k = 1; k < n; ++k) {
            const double t = double(k) / double(n);
            out.push_back({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)});
        }
        out.push_back(b);
    }
    pts.swap(out);
}

// Side-effect-free twin of the fill_chords admission gate (area pre-filter,
// morphological opening, pitch erosion). The concert decision in
// build_layer_strands needs to know BEFORE choosing producers whether the
// gate will refuse this island; re-running the gate here (only when wall
// loops are enabled) keeps the accounting in fill_chords honest (no double
// counting) at the cost of a few microseconds of clipper work.
bool chords_admitted(const ExPolygon& isl, double clearance_mm, double min_wall_mm, double min_area_mm2,
                     double inset_mm)
{
    if (min_area_mm2 > 0.0) {
        const double area_mm2 = unscale<double>(unscale<double>(isl.area()));
        if (area_mm2 < min_area_mm2)
            return false;
    }
    ExPolygons region;
    if (min_wall_mm > 0.0) {
        const float r = float(scale_(0.5 * min_wall_mm));
        const ExPolygons admitted = offset2_ex(ExPolygons{isl}, -r, r);
        if (admitted.empty())
            return false;
        region = offset_ex(admitted, float(scale_(-clearance_mm)));
    } else {
        region = offset_ex(isl, float(scale_(-clearance_mm)));
    }
    // Mirror of the fill_chords outer inset (fiber mode parity, M2): the
    // concert decision must see exactly the region the producer will clip to,
    // or wall/chord ownership would disagree between twin and producer.
    if (inset_mm > 0.0)
        region = offset_ex(region, float(scale_(-inset_mm)));
    return !region.empty();
}

// Parallel chords at `angle_deg` from +X, spaced pitch_mm, each scanline either
// a single pass or - when twin_offset_mm is positive - a pair of passes that
// distance apart (one isogrid rib). Admission (operator
// ruling 2026-10-01, fill/trace concert): thin bands are the serpentine's job
// alone only where the wall is thick enough; refused islands hand the wall to
// the concentric wall loops (see wall_loops_enabled). Before anything else the
// island is morphologically OPENED by radius min_wall/2 (offset -r then +r).
// If the opening is empty the island is thinner than min_wall everywhere and
// produces NO chords at all. A cheap area pre-filter skips dust islands up
// front. Chords that survive then behave exactly as before: clip region =
// admitted region eroded by clearance_mm, scanlines struck (k+0.5)*pitch
// inside that region's boundary, so chords ride exact multiples of pitch
// inside the admitted region, every chord keeps its clearance from the
// material edge, and line-to-line pitch stays uniform. The clearance is half a
// pitch for the single-pass patterns (the legacy value) and half a bead for
// isogrid, whose rib pitch is several beads wide and must not push the ribs
// that far off the wall. Chords shorter than min_seg
// are dropped and counted (backstop); nothing is silently merged or extended
// to reach a minimum.
//
// Returns whether the island was ADMITTED (a fillable region existed). A false
// return (area gate, empty opening, or region empty after pitch erosion) is
// the refusal signal the caller uses to switch this island to concentric wall
// loops - the concert. An admitted island whose chords were all dropped by
// min-seg returns true: the fill path still owns it (steering limit, not a
// thin wall).
bool fill_chords(const ExPolygon& isl, double pitch_mm, double angle_deg, double min_seg_mm,
                 double min_wall_mm, double min_area_mm2, double inset_mm, double clearance_mm,
                 double twin_offset_mm, long family,
                 StrandLayerResult& res, std::vector<FillChord>& out)
{
    if (min_area_mm2 > 0.0) {
        const double area_mm2 = unscale<double>(unscale<double>(isl.area()));
        if (area_mm2 < min_area_mm2)
            return false;
    }
    const double delta_scaled = scale_(-clearance_mm);
    ExPolygons region;
    if (min_wall_mm > 0.0) {
        const float r = float(scale_(0.5 * min_wall_mm));
        const ExPolygons admitted = offset2_ex(ExPolygons{isl}, -r, r);
        if (admitted.empty())
            return false; // thin wall: refused, the concert partner takes over
        region = offset_ex(admitted, float(delta_scaled));
    } else {
        region = offset_ex(isl, float(delta_scaled));
    }
    if (region.empty())
        return false;
    // Fiber mode mapping: extra erosion AFTER the pitch erosion, so
    // chords stop inset_mm short of the walls and the outer skin stays plastic
    // (the vendor's ExtendIntoPerimeters inverted: CF inside, FFF outside). If
    // the inset consumes the fillable region the island is refused and hands
    // over to the wall loop producer; the chords_admitted twin mirrors this.
    if (inset_mm > 0.0) {
        region = offset_ex(region, float(scale_(-inset_mm)));
        if (region.empty())
            return false;
    }

    BoundingBox bb = region.front().contour.bounding_box();
    for (const ExPolygon& e : region)
        bb.merge(e.contour.bounding_box());

    const double th = angle_deg * FILL_PI / 180.0;
    const Vec2d d(std::cos(th), std::sin(th));              // along chord
    const Vec2d nvec(-std::sin(th), std::cos(th));          // scan axis
    const Point corners[4] = {bb.min, {bb.min.x(), bb.max.y()}, {bb.max.x(), bb.min.y()}, bb.max};
    double ulo = std::numeric_limits<double>::max(), uhi = ulo * -1.0;
    double vlo = ulo, vhi = ulo * -1.0;
    for (const Point& c : corners) {
        const double u = double(c.x()) * d.x() + double(c.y()) * d.y();
        const double v = double(c.x()) * nvec.x() + double(c.y()) * nvec.y();
        ulo = std::min(ulo, u); uhi = std::max(uhi, u);
        vlo = std::min(vlo, v); vhi = std::max(vhi, v);
    }
    const double pitch_scaled = scale_(pitch_mm);
    const double u_lo = ulo - pitch_scaled;
    const double u_hi = uhi + pitch_scaled;

    // Clip against the eroded region as an even-odd polygon set (contours +
    // grown holes): chords never touch a void, endpoints keep half-a-pitch
    // clearance from every hole edge, and the intersection subtracts hole
    // interiors directly (no second pass needed).
    Polygons clip_region;
    for (const ExPolygon& e : region) {
        clip_region.push_back(e.contour);
        for (const Polygon& h : e.holes)
            clip_region.push_back(h);
    }

    const double twin_scaled = twin_offset_mm > 0.0 ? scale_(twin_offset_mm) : 0.0;
    const int    passes      = twin_scaled > 0.0 ? 2 : 1;
    for (long k = 0; k < 100000; ++k) { // hard guard against pathological pitch
        const double v0 = vlo + (double(k) + 0.5) * pitch_scaled;
        if (v0 > vhi)
            break;
        for (int sub = 0; sub < passes; ++sub) {
            const double v = v0 + double(sub) * twin_scaled;
            Point pa, pb;
            pa.x() = std::llround(u_lo * d.x() + v * nvec.x());
            pa.y() = std::llround(u_lo * d.y() + v * nvec.y());
            pb.x() = std::llround(u_hi * d.x() + v * nvec.x());
            pb.y() = std::llround(u_hi * d.y() + v * nvec.y());
            Lines clipped = intersection_ln(Line(pa, pb), clip_region);
            for (const Line& ln : clipped) {
                const Point& p1 = ln.a;
                const Point& p2 = ln.b;
                const double len_mm = unscale<double>(dist_scaled(p1, p2));
                if (len_mm < min_seg_mm || (p1.x() == p2.x() && p1.y() == p2.y())) {
                    ++res.fill_chords_dropped;
                    continue;
                }
                FillChord ch;
                const double a1 = double(p1.x()) * d.x() + double(p1.y()) * d.y();
                const double a2 = double(p2.x()) * d.x() + double(p2.y()) * d.y();
                ch.a = a1 <= a2 ? p1 : p2;
                ch.b = a1 <= a2 ? p2 : p1;
                ch.along_a = std::min(a1, a2);
                ch.family = family;
                ch.scan = k;
                ch.sub = sub;
                out.push_back(ch);
                ++res.fill_chords;
            }
        }
    }
    std::sort(out.begin(), out.end(), chord_before);
    return true;
}

// Order boundary loops of one erosion level by lexicographic minimum vertex
// (shared determinism rule for concentric modes).
void sort_loops_lexmin(std::vector<const Points*>& loops)
{
    std::sort(loops.begin(), loops.end(), [](const Points* a, const Points* b) {
        const std::pair<int64_t, int64_t> ka = ring_lexmin(*a);
        const std::pair<int64_t, int64_t> kb = ring_lexmin(*b);
        if (ka.first != kb.first)
            return ka.first < kb.first;
        return ka.second < kb.second;
    });
}

double polyline_length_mm(const std::vector<Point>& pts)
{
    double len = 0.0;
    for (size_t i = 0; i + 1 < pts.size(); ++i)
        len += dist_scaled(pts[i], pts[i + 1]);
    return unscale<double>(len);
}

} // namespace

StrandLayerResult build_layer_strands(const std::vector<std::vector<FiberPoint>>& rings,
                                      const StrandLayerParams& params,
                                      std::string* first_error)
{
    StrandLayerResult res;
    auto note = [first_error](const std::string& msg) {
        if (first_error && first_error->empty())
            *first_error = msg;
    };

    if (params.layer_id == 0 || !std::isfinite(params.z)) {
        note("build_layer_strands: layer_id must be >= 1 and z finite");
        return res;
    }
    if (!(params.ratio_p > 0.0) || !(params.fiber_rate > 0.0) || !(params.feed_mm_min > 0.0) ||
        !(params.tail_length_mm > 0.0) || !(params.tail_v_factor > 0.0) || params.tail_v_factor > 1.0 ||
        !(params.pitch_mm > 0.0) || !std::isfinite(params.ratio_p) || !std::isfinite(params.fiber_rate) ||
        !std::isfinite(params.feed_mm_min) || !std::isfinite(params.tail_length_mm) ||
        !std::isfinite(params.pitch_mm)) {
        note("build_layer_strands: invalid strand layer parameters");
        return res;
    }
    if (params.max_levels < 1) {
        note("build_layer_strands: max_levels must be >= 1");
        return res;
    }
    if (params.fill_enabled && (!std::isfinite(params.fill_angle_deg) || !(params.fill_min_seg_mm >= 0.0) ||
        !std::isfinite(params.fill_min_wall_width) || !(params.fill_min_wall_width >= 0.0) ||
        !std::isfinite(params.fill_min_area_mm2) || !(params.fill_min_area_mm2 >= 0.0) ||
        !std::isfinite(params.fill_outer_inset) || !(params.fill_outer_inset >= 0.0))) {
        note("build_layer_strands: fill enabled with non-finite angle or negative fill gate parameters");
        return res;
    }
    if (params.wall_loops_enabled && !(params.wall_pitch_mm > 0.0)) {
        note("build_layer_strands: wall loops enabled with non-positive wall pitch");
        return res;
    }
    if (!std::isfinite(params.boundary_inset_mm) || !(params.boundary_inset_mm >= 0.0)) {
        note("build_layer_strands: boundary inset must be finite and non-negative");
        return res;
    }
    if (!std::isfinite(params.min_turn_radius_mm) || !(params.min_turn_radius_mm >= 0.0) ||
        !std::isfinite(params.max_arc_seg_mm) || !(params.max_arc_seg_mm >= 0.0)) {
        note("build_layer_strands: feasibility limits must be finite and non-negative");
        return res;
    }
    if (params.fill_enabled && params.infill_pattern == FiberInfillPattern::fipIsogrid &&
        !(params.fiber_width_mm > 0.0)) {
        note("build_layer_strands: isogrid fill needs a positive fiber bead width");
        return res;
    }

    // Chord clearance from the material edge: half a pitch for the single-pass
    // patterns (legacy), half a bead for isogrid, whose rib pitch spans several
    // beads and would otherwise push the ribs far off the wall.
    const bool   isogrid = params.infill_pattern == FiberInfillPattern::fipIsogrid;
    const double fill_clearance_mm = 0.5 * (isogrid ? params.fiber_width_mm : params.pitch_mm);

    // Serpentine interior fill for one island: chords clipped to the eroded
    // region (see fill_chords), walked in scanline order and chained into
    // continuous OPEN strands by greedy nearest-endpoint selection. A connector
    // is deposited only when the straight turn stays inside material along its
    // whole sampled length; when no turn is geometrically possible the strand
    // is CUT there (a real lifecycle break, counted) and a new strand starts at
    // the next unused chord - turns are never faked and chords are never joined
    // through air or across a hole. The whole path of a fill strand lies inside
    // material, so its tail suffix (last tail_length mm of the path itself) is
    // automatically deposited on-part. Viability is checked at strand level:
    // a chained group too short to carry body + tail is rejected whole.
    auto emit_island_fill = [&](const ExPolygon& isl, bool seed_with_loops, size_t island_idx) {
        // This lane's own level-0 boundary loops, given the same deterministic
        // order, canonical rotation, seam and direction treatment they get as
        // standalone traces: chaining changes only where the strand is CUT, not
        // where the fiber goes.
        std::vector<std::vector<Point>> seeds;
        if (seed_with_loops) {
            std::vector<const Points*> loops;
            loops.push_back(&isl.contour.points);
            for (const Polygon& h : isl.holes)
                loops.push_back(&h.points);
            sort_loops_lexmin(loops);
            size_t parity = 0;
            for (const Points* loop : loops) {
                if (loop->size() < 3)
                    continue;
                if (ring_length_mm(*loop) >= params.tail_length_mm + params.min_viable_margin)
                    ++res.viable_rings; // enforce input: a reinforceable boundary existed
                std::vector<FiberPoint> closed = ring_to_mm_closed(*loop);
                if (closed.size() < 3)
                    continue;
                canonical_rotate(closed);
                place_seam(closed, params, island_idx, 0, parity);
                if ((island_idx + parity) % 2 != 0)
                    reverse_direction(closed);
                ++parity;
                std::vector<Point> scaled;
                scaled.reserve(closed.size());
                for (const FiberPoint& p : closed)
                    scaled.push_back(Point::new_scale(p.x, p.y));
                seeds.emplace_back(std::move(scaled));
            }
        }

        std::vector<FillChord> chords;
        if (isogrid) {
            // Isogrid: three rib families 60 degrees
            // apart, offset 30 degrees from the laydown angle, each rib a twin
            // pass one bead wide. The families share the clip region and the
            // rib pitch, and all their chords go into one chaining walk so ribs
            // still join into long strands with one cut each.
            for (long fam = 0; fam < 3; ++fam)
                fill_chords(isl, params.pitch_mm, params.fill_angle_deg + 30.0 + 60.0 * double(fam),
                            params.fill_min_seg_mm, params.fill_min_wall_width, params.fill_min_area_mm2,
                            params.fill_outer_inset, fill_clearance_mm, params.fiber_width_mm, fam,
                            res, chords);
        } else {
            fill_chords(isl, params.pitch_mm, params.fill_angle_deg, params.fill_min_seg_mm,
                        params.fill_min_wall_width, params.fill_min_area_mm2, params.fill_outer_inset,
                        fill_clearance_mm, 0.0, 0, res, chords);
        }
        std::vector<char> used(chords.size(), 0);
        std::vector<Point> path; // scaled bed units, open polyline under construction

        // One open fill path -> one strand, with the viability and turn-radius
        // rulings applied. Reused per piece when the split policy cuts a path.
        auto emit_fill_path = [&](std::vector<FiberPoint> pts) {
            if (pts.size() < 2)
                return;
            if (!(path_length_mm(pts) >= params.tail_length_mm + params.min_viable_margin)) {
                ++res.rejected_short; // cannot carry body + tail
                return;
            }
            resample_max_segment(pts, params.max_arc_seg_mm);
            FiberStrand strand;
            strand.layer_id       = params.layer_id;
            strand.z              = params.z;
            strand.pts            = std::move(pts);
            strand.ratio_p        = params.ratio_p;
            strand.fiber_rate     = params.fiber_rate;
            strand.feed_mm_min    = params.feed_mm_min;
            strand.tail_length_mm = params.tail_length_mm;
            strand.tail_v_factor  = params.tail_v_factor;
            std::string err;
            if (strand.finalize(&err)) {
                res.strands.emplace_back(std::move(strand));
            } else if (err.find("path shorter") != std::string::npos) {
                ++res.rejected_short; // defensive: pre-checked above
            } else {
                ++res.rejected_other;
                note(err);
            }
        };

        auto flush = [&]() {
            if (path.size() >= 2) {
                const double len = polyline_length_mm(path);
                if (!(len >= params.tail_length_mm + params.min_viable_margin)) {
                    ++res.rejected_short; // chained group cannot carry body + tail
                } else {
                    std::vector<FiberPoint> pts;
                    pts.reserve(path.size());
                    for (const Point& p : path) {
                        const Vec2d v = unscale(p);
                        pts.push_back({v.x(), v.y()});
                    }
                    const std::vector<size_t> tight0 =
                        tight_turn_joints(pts, false, params.min_turn_radius_mm);
                    if (!tight0.empty() && params.tight_turn_policy == FiberTightTurnPolicy::fttKeep)
                        fillet_path_turns(pts, false, params.min_turn_radius_mm);
                    // Count the joints the policy had to deal with, measured BEFORE the
                    // fillet. A fillet replaces one corner with an arc, and the arc's own
                    // vertices are not turns: re-measuring the filleted path reports the
                    // tessellation of the fillet rather than the joint that prompted it,
                    // inflating the diagnostic by the arc's segment count (measured on the
                    // 60 x 40 rect at fs_fiber_min_radius 25: four trace corners reported
                    // as 54 joints, so 90 where the joints actually number 40). Under the
                    // split policy no fillet is applied, so this is the same set the split
                    // decision below uses.
                    const std::vector<size_t>& tight = tight0;
                    res.tight_turns += tight.size();
                    if (!tight.empty() && params.tight_turn_policy == FiberTightTurnPolicy::fttSplit) {
                        res.tight_turn_splits += tight.size();
                        note("build_layer_strands: fill strand split at a turn below fs_fiber_min_radius");
                        for (std::vector<FiberPoint>& piece : split_at_joints(pts, false, tight))
                            emit_fill_path(std::move(piece));
                    }
                    else {
                        emit_fill_path(std::move(pts));
                    }
                }
            }
            path.clear();
        };

        // The walk's "sources" are the boundary seeds (each a closed loop that
        // may open into a fill) followed by the unused chords. Seeds go first
        // so a reachable fill is attached to its surrounding loop instead of
        // being started as its own strand and leaving the loop as a cut.
        size_t remaining_chords = chords.size();
        size_t next_seed = 0;
        while (next_seed < seeds.size() || remaining_chords > 0) {
            // Prefer a leftover boundary seed: attaching fill to it saves a cut.
            // Fall back to the next unused chord when every seed is already used.
            size_t seed_size = 0; // 0 means this strand did not start on a seed
            if (next_seed < seeds.size()) {
                path = seeds[next_seed++];
                seed_size = path.size();
            } else {
                size_t s = chords.size();
                for (size_t i = 0; i < chords.size(); ++i)
                    if (!used[i]) { s = i; break; }
                if (s == chords.size())
                    break;
                path.push_back(chords[s].a);
                path.push_back(chords[s].b);
                used[s] = 1;
                --remaining_chords;
            }
            // Extend greedily onto unused chords. Seeds themselves are never
            // targets: a loop is a closed perimeter and chaining two of them
            // would leave a connector that is also a fiber perimeter, which is
            // a different geometry change.
            for (;;) {
                const Point cur = path.back();
                long best = -1;
                int best_rev = 0;
                double best_d = 0.0;
                for (size_t i = 0; i < chords.size(); ++i) {
                    if (used[i])
                        continue;
                    for (int rev = 0; rev < 2; ++rev) {
                        const Point& entry = rev == 0 ? chords[i].a : chords[i].b;
                        if (!connector_inside(isl, cur, entry, params.pitch_mm))
                            continue;
                        const double dd = dist_scaled(cur, entry);
                        if (best < 0 || dd < best_d - 1.0 ||
                            (std::abs(dd - best_d) <= 1.0 &&
                             (i < size_t(best) || (i == size_t(best) && rev < best_rev)))) {
                            best = long(i);
                            best_rev = rev;
                            best_d = dd;
                        }
                    }
                }
                if (best < 0)
                    break; // no turn reachable from here: strand break
                const Point& entry = best_rev == 0 ? chords[best].a : chords[best].b;
                const Point& exit_ = best_rev == 0 ? chords[best].b : chords[best].a;
                if (!(entry.x() == path.back().x() && entry.y() == path.back().y()))
                    path.push_back(entry); // coincident endpoints are the same point, not a segment
                path.push_back(exit_);
                used[best] = 1;
                --remaining_chords;
            }
            // A seed that absorbed at least one chord was a cut we did not spend.
            if (seed_size > 0 && path.size() > seed_size)
                ++res.chained_loops;
            flush();
            if (remaining_chords > 0 && next_seed >= seeds.size())
                ++res.fill_breaks; // the walk stopped with chords left over: a real cut
        }
    };

    // Group the perimeter rings into islands with holes. Overlapping or nested
    // rings (perimeters of touching regions) merge; nesting depth makes a ring
    // a hole, so nothing is ever deposited across a hole interior.
    Polygons polys;
    polys.reserve(rings.size());
    for (const std::vector<FiberPoint>& ring : rings) {
        bool finite = ring.size() >= 3;
        for (const FiberPoint& fp : ring)
            finite = finite && std::isfinite(fp.x) && std::isfinite(fp.y);
        if (!finite) {
            // A malformed candidate ring means the producer could not hand over
            // a reconstructable path: account it, never silently drop it.
            ++res.rejected_other;
            note("build_layer_strands: malformed ring (non-finite vertex or fewer than 3 points)");
            continue;
        }
        Points pts;
        pts.reserve(ring.size());
        for (const FiberPoint& fp : ring)
            pts.push_back(Point::new_scale(fp.x, fp.y));
        polys.emplace_back(std::move(pts));
    }
    if (polys.empty())
        return res;

    // Orientation normalisation: Clipper fills with the NonZero rule, so a ring
    // that encloses a hole must be wound opposite its container, or the hole
    // interior counts as material and the annulus becomes a plate. Classify
    // every input ring by its containment depth among the other input rings
    // (even-odd): even depth is material (make CCW), odd depth is a hole (make
    // CW). The planner then accepts rings in whatever orientation the producer
    // emits, and the union reproduces the intended islands with holes.
    for (size_t i = 0; i < polys.size(); ++i) {
        size_t depth = 0;
        const Point probe = polys[i].points.front();
        for (size_t j = 0; j < polys.size(); ++j)
            if (i != j && point_strictly_inside(polys[j].points, probe))
                ++depth;
        const bool ccw = ring_signed_area(polys[i].points) > 0.0;
        if (ccw == (depth % 2 != 0))
            polys[i].reverse();
    }

    ExPolygons islands = union_ex(polys);

    // Deterministic emission order: area descending, then bbox min, then vertex
    // count. Identical input always yields an identical strand list.
    std::sort(islands.begin(), islands.end(), expoly_before);

    size_t island_idx = 0;
    for (const ExPolygon& isl : islands) {
        const double area_mm2 = unscale<double>(unscale<double>(isl.area()));
        if (area_mm2 < params.min_island_area_mm2) {
            // Dust island (support stubs, slivers): plastic-only fallback. Not
            // a reinforcement failure - physically meaningless to reinforce.
            ++res.skipped_tiny;
            continue;
        }
        ++res.islands;

        // The FIBER LANE: the input rings are the plastic outer-wall centerline,
        // so with plastic walls planned outboard of the fiber the whole fiber
        // region has to move inboard by boundary_inset_mm, or the roving is laid
        // on the very bead it is supposed to hide behind. Level 0 rides the
        // inset region, the deeper levels erode from the same island, and the
        // interior fill clips inside each lane. An island the inset consumes
        // entirely is physically too thin to carry fiber behind its walls:
        // plastic-only fallback, counted.
        ExPolygons lanes;
        if (params.boundary_inset_mm > 0.0) {
            lanes = offset_ex(isl, float(scale_(-params.boundary_inset_mm)));
            if (lanes.empty()) {
                ++res.rejected_thin;
                ++island_idx;
                continue;
            }
            std::sort(lanes.begin(), lanes.end(), expoly_before);
        } else {
            lanes.push_back(isl);
        }

        // Boundary trace and interior levels over this island. Level 0 is the
        // boundary itself: the profile trace the operator demanded (fiber
        // follows the part outline, through its real corners and turns). With
        // fill off, level k >= 1 rides the island eroded by k * pitch (concentric
        // mode). With fill on, the deeper concentric levels are REPLACED by the
        // serpentine interior fill: the interior carries fiber at uniform pitch
        // instead of collapsing to empty air when erosion runs out. Erosion
        // shrinks outer contours and grows hole contours, so every loop stays
        // inside material; the fill clips to the eroded region for the same
        // reason.
        //
        // Concert (operator ruling 2026-10-01, wall-loop build): with fill on,
        // an island the admission gate REFUSES (thinner than the opening
        // diameter everywhere, or below the area floor) is not left to a single
        // trace line. When wall loops are enabled it switches to WALL mode: the
        // same concentric erosion levels, spaced wall_pitch instead of the
        // serpentine pitch, so the wall thickness is carried by multiple loops
        // hugging the shape. Admitted islands keep the trace + chords concert
        // byte-identical.
        //
        // Fiber mode mapping: with a loop clamp (wall_outer_loops +
        // wall_inner_loops > 0) the wall loop producer runs on ADMITTED islands
        // too - the vendor Reinforced/Fortified carry F. outer/inner loops
        // alongside the chord fill - and emits exactly the clamped loop count
        // (outer boundary first, inner erosion levels as the continuation). The
        // chord fill still owns the admitted interior. Clamp 0 reproduces the
        // legacy concert above byte-for-byte.
        const size_t loop_clamp = params.wall_outer_loops + params.wall_inner_loops;
        const bool refused = params.fill_enabled && params.wall_loops_enabled &&
            std::none_of(lanes.begin(), lanes.end(), [&](const ExPolygon& lane) {
                return chords_admitted(lane, fill_clearance_mm, params.fill_min_wall_width,
                                       params.fill_min_area_mm2, params.fill_outer_inset);
            });
        const bool wall_mode = refused || (params.fill_enabled && params.wall_loops_enabled && loop_clamp > 0);
        // Chain the boundary into the fill only on the concert path: fill on,
        // wall mode off, so there is a fill to attach to and the boundary is
        // not already carrying wall thickness. Off for every other case so
        // existing callers stay byte-identical.
        const bool chain_into_fill = params.chain_loops_into_fill && params.fill_enabled && !wall_mode;
        const size_t level_end = params.fill_enabled && !wall_mode ? size_t(0)
            : (loop_clamp > 0 && (wall_mode || !params.fill_enabled) ? loop_clamp - 1 : params.max_levels);
        const double level_pitch = wall_mode ? params.wall_pitch_mm : params.pitch_mm;
        size_t loop_parity = 0;
        for (size_t level = 0; level <= level_end; ++level) {
            // Level 0 is handed to the fill walk as seed loops when chaining:
            // emitting it here would deposit the boundary twice.
            if (level == 0 && chain_into_fill)
                continue;
            ExPolygons lvl;
            if (level == 0)
                lvl = lanes;
            else {
                // offset_ex takes its delta in SCALED units, like every other
                // Clipper coordinate: convert mm explicitly. Deeper levels are
                // measured from the lane, so they compose with the inset.
                const double delta_scaled =
                    scale_(-(params.boundary_inset_mm + double(level) * level_pitch));
                lvl = offset_ex(isl, float(delta_scaled));
                if (lvl.empty())
                    break; // the island is exhausted; deeper levels are subsets
            }

            // Collect every loop at this level (outer contours and hole
            // contours) and order them by lexicographic minimum vertex.
            std::vector<const Points*> loops;
            for (const ExPolygon& e : lvl) {
                loops.push_back(&e.contour.points);
                for (const Polygon& h : e.holes)
                    loops.push_back(&h.points);
            }
            sort_loops_lexmin(loops);

            for (const Points* loop : loops) {
                if (loop->size() < 3)
                    continue;
                const double len = ring_length_mm(*loop);
                const bool viable_len = len >= params.tail_length_mm + params.min_viable_margin;
                if (level == 0 && viable_len)
                    ++res.viable_rings; // enforce input: a reinforceable boundary existed
                if (!viable_len) {
                    // Loop cannot physically carry body + calibrated tail: an
                    // isolated short feature. Plastic-only fallback, counted.
                    ++res.rejected_short;
                    continue;
                }

                std::vector<FiberPoint> closed = ring_to_mm_closed(*loop);
                if (closed.size() < 3) {
                    ++res.rejected_other;
                    continue;
                }
                canonical_rotate(closed);
                place_seam(closed, params, island_idx, level, loop_parity);
                if ((island_idx + loop_parity) % 2 != 0)
                    reverse_direction(closed); // alternate direction: wear and reaction symmetry
                ++loop_parity;

                // One planned path (the ring itself, or an open piece of it when
                // the split policy cut a tight corner) -> one strand.
                const auto emit_loop_path = [&](std::vector<FiberPoint> pts, bool is_piece) {
                    if (pts.size() < 2)
                        return;
                    if (is_piece &&
                        !(path_length_mm(pts) >= params.tail_length_mm + params.min_viable_margin)) {
                        ++res.rejected_short; // piece cannot carry body + tail
                        return;
                    }
                    resample_max_segment(pts, params.max_arc_seg_mm);

                    FiberStrand strand;
                    strand.layer_id       = params.layer_id;
                    strand.z              = params.z;
                    strand.pts            = std::move(pts);
                    strand.ratio_p        = params.ratio_p;
                    strand.fiber_rate     = params.fiber_rate;
                    strand.feed_mm_min    = params.feed_mm_min;
                    strand.tail_length_mm = params.tail_length_mm;
                    strand.tail_v_factor  = params.tail_v_factor;

                    std::string err;
                    if (strand.finalize(&err)) {
                        if (wall_mode && level >= 1)
                            ++res.wall_loops; // concentric wall loop hugging the shape
                        res.strands.emplace_back(std::move(strand));
                    } else if (err.find("path shorter") != std::string::npos) {
                        ++res.rejected_short; // defensive: pre-checked above
                    } else {
                        ++res.rejected_other;
                        note(err);
                    }
                };

                const std::vector<size_t> tight0 =
                    tight_turn_joints(closed, true, params.min_turn_radius_mm);
                if (params.min_turn_radius_mm > 0.0 &&
                    params.tight_turn_policy == FiberTightTurnPolicy::fttKeep) {
                    fillet_path_turns(closed, true, params.min_turn_radius_mm);
                    rotate_tail_off_tight(closed, params.tail_length_mm, params.min_turn_radius_mm);
                }
                // Pre-fillet count; see the fill-path site above for why.
                const std::vector<size_t>& tight = tight0;
                res.tight_turns += tight.size();
                if (!tight.empty() && params.tight_turn_policy == FiberTightTurnPolicy::fttSplit) {
                    res.tight_turn_splits += tight.size();
                    note("build_layer_strands: loop split at a turn below fs_fiber_min_radius");
                    for (std::vector<FiberPoint>& piece : split_at_joints(closed, true, tight))
                        emit_loop_path(std::move(piece), true);
                }
                else {
                    emit_loop_path(std::move(closed), false);
                }
            }
        }
        // Concert: an island refused by the fill gate has nothing for the
        // serpentine to contribute, skip the empty fill pass. A clamped wall
        // loop run on an ADMITTED island does not suppress its fill.
        if (params.fill_enabled && !refused)
            for (const ExPolygon& lane : lanes)
                emit_island_fill(lane, chain_into_fill, island_idx);
        ++island_idx;
    }

    return res;
}

} // namespace Fiber
} // namespace Slic3r
