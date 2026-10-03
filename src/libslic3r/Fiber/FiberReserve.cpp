// License: GNU AGPLv3 or higher
//
// FibreSeeker3 plastic reservation for continuous fiber (R2.1) - implementation.
// See FiberReserve.hpp for the contract: exclusion band d(w) =
// composite/2 + w/2 - bond_overlap around the final accepted strand geometry
// (body + post-cut tail), evaluated per planned plastic bead width, applied in
// place to the already-sliced collection tree so every downstream consumer
// (export order, cooling, material and time accounting) sees the reserved
// program.

#include "FiberReserve.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#include "../BoundingBox.hpp"
#include "../ClipperUtils.hpp"
#include "../ExtrusionEntity.hpp"
#include "../ExtrusionEntityCollection.hpp"
#include "FiberStrand.hpp"

namespace Slic3r {
namespace Fiber {

namespace {

// A merged exclusion-band set for one plastic bead width, with its bounding
// box for cheap per-entity pre-checks.
struct BandSet {
    ExPolygons  bands;
    BoundingBox bbox;   // undefined when bands is empty
};

// Strand geometry (bed mm) -> scaled collection frame: collections store
// scale_(bed_mm - origin), the exact inverse of the harvest at GCode.cpp
// (unscale(p) + instance shift).
Point fiber_pt_to_scaled(const FiberPoint& p, const Vec2d& origin)
{
    return Point(coord_t(scale_(p.x - origin.x())), coord_t(scale_(p.y - origin.y())));
}

// One Polyline per strand covering every millimetre it deposits. body_pts and
// tail_pts hold move DESTINATIONS only (FiberStrand: "joint deposits ending at
// body_pts[i]"), so the first body move runs from pts.front() and that opening
// segment belongs to the band too - omitting it left the strand's first segment
// unreserved, and plastic survived right on top of it. Body and tail are
// contiguous along the same planned path, so they chain into a single polyline.
// Consecutive duplicated vertices collapse; a closing duplicate (closed ring
// strands carry one by planner contract) is preserved so the round-cap open
// offset yields the uniform tube/annulus.
Polylines collect_fiber_polylines(const std::vector<FiberStrand>& strands, const Vec2d& origin)
{
    Polylines out;
    for (const FiberStrand& s : strands) {
        Points pts;
        pts.reserve(1 + s.body_pts.size() + s.tail_pts.size());
        const auto append = [&pts, &origin](const FiberPoint& fp) {
            const Point q = fiber_pt_to_scaled(fp, origin);
            if (pts.empty() || pts.back() != q)
                pts.push_back(q);
        };
        if (! s.pts.empty())
            append(s.pts.front());
        for (const FiberPoint& fp : s.body_pts)
            append(fp);
        for (const FiberPoint& fp : s.tail_pts)
            append(fp);
        if (pts.size() >= 2)
            out.emplace_back(std::move(pts));
    }
    return out;
}

// Capsule/annulus bands for one plastic bead width, lazily built and cached
// per width (key = width in microns). d(w) <= 0 caches an empty set: that bead
// width is allowed to touch the fiber.
struct BandBuilder {
    Polylines fiber;
    double    composite_mm      = 0.4;
    double    overlap_mm        = 0.1;
    double    fallback_width_mm = 0.4;   // used when an entity carries no usable width
    std::map<int, BandSet> cache;

    const BandSet& bands_for(double w_mm)
    {
        if (!(w_mm > 0.0))
            w_mm = fallback_width_mm;
        const int key = int(std::lround(w_mm * 1000.0));
        if (auto it = cache.find(key); it != cache.end())
            return it->second;
        BandSet bs;
        const double d = composite_mm / 2.0 + w_mm / 2.0 - overlap_mm;
        if (d > 0.0 && !fiber.empty()) {
            // Round-closed open offset of the polylines: capsule per segment,
            // round miter joins at vertices, round caps at the ends; the
            // union heals every overlap into one exclusion region.
            Polygons rings = offset(fiber, float(scale_(d)), ClipperLib::JoinType::jtRound,
                                    double(scale_(0.1)), ClipperLib::EndType::etOpenRound);
            bs.bands = union_ex(rings);
            for (const ExPolygon& x : bs.bands) {
                bs.bbox.merge(x.contour.points);
                for (const Polygon& h : x.holes)
                    bs.bbox.merge(h.points);
            }
        }
        return cache.emplace(key, std::move(bs)).first->second;
    }

    // Width selector honoring the planned-width ruling: the entity's own bead
    // width, else the composite width (the widest, most conservative band).
    double width_of(const ExtrusionPath& p) const
    {
        return p.width > 0.f ? double(p.width) : fallback_width_mm;
    }
};

bool bbox_overlap(const BoundingBox& a, const BoundingBox& b)
{
    if (!a.defined || !b.defined)
        return false;
    return !(a.max.x() < b.min.x() || b.max.x() < a.min.x() ||
             a.max.y() < b.min.y() || b.max.y() < a.min.y());
}

// Filter out clipper dust: < 2 points or shorter than the fragment floor.
void filter_fragments(Polylines& frags, double min_len)
{
    frags.erase(std::remove_if(frags.begin(), frags.end(),
                               [&min_len](const Polyline& pl) {
                                   return pl.points.size() < 2 || pl.length() <= min_len;
                               }),
                frags.end());
}

enum class ClipResult { Intact, Clipped, Removed };

// Subtract the bands from one plain (non contoured / non sloped) leaf path.
// On ClipResult::Clipped the surviving fragment clones (full metadata via the
// (Polyline3, ExtrusionPath) copy constructor) are appended to `out`; the
// caller owns `src` and deletes it on Clipped / Removed.
ClipResult clip_single_path(const ExtrusionPath& src, const BandSet& bs, double intact_tol,
                            double min_frag, ExtrusionEntitiesPtr& out)
{
    if (!bs.bbox.defined)
        return ClipResult::Intact;
    const Polyline src2d = src.polyline.to_polyline();
    // Point-wise merge on purpose: BoundingBox::merge(const Points&) routes via
    // construct<false>, which leaves a ZERO-AREA box undefined, so straight
    // axis-aligned paths would produce an undefined box and silently skip the
    // overlap pre-check. merge(const Point&) marks the box defined from the
    // first point unconditionally.
    BoundingBox bb;
    for (const Point& q : src2d.points)
        bb.merge(q);
    if (!bbox_overlap(bb, bs.bbox))
        return ClipResult::Intact;
    Polylines frags = diff_pl(Polylines{ src2d }, bs.bands);
    double len1 = 0.0;
    for (const Polyline& pl : frags)
        len1 += pl.length();
    filter_fragments(frags, min_frag);
    // Nothing of substance removed (disjoint hit or tangential touch): keep
    // the original pointer so unhit layers stay byte-identical.
    const double len0 = src2d.length();
    if (std::fabs(len1 - len0) <= intact_tol && !frags.empty())
        return ClipResult::Intact;
    if (frags.empty())
        return ClipResult::Removed;
    const coord_t z = src.polyline.points.front().z();
    for (Polyline& pl : frags)
        out.push_back(new ExtrusionPath(Polyline3(pl, z), src));
    return ClipResult::Clipped;
}

// A path-list entity (loop / multipath) is decomposed into leaf paths when the
// bands actually remove length from it (total-length comparison, so a bbox
// false positive never fragments anything). On false nothing was touched; on
// true the survivors / fragment clones were appended to `out` and the caller
// deletes the original entity.
bool decompose_paths(const ExtrusionPaths& paths, double len0, BandBuilder& bb, double width_mm,
                     double intact_tol, double min_frag, ExtrusionEntitiesPtr& out)
{
    if (paths.empty())
        return false;
    const BandSet& bs = bb.bands_for(width_mm);
    if (!bs.bbox.defined)
        return false;
    // Point-wise merge: see clip_single_path (merge(const Points&) leaves a
    // zero-area box undefined, straight paths would skip the pre-check).
    BoundingBox bbx;
    for (const ExtrusionPath& p : paths)
        for (const Point3& q : p.polyline.points)
            bbx.merge(Point(q.x(), q.y()));
    if (!bbox_overlap(bbx, bs.bbox))
        return false;
    std::vector<Polylines> per(paths.size());
    double len1 = 0.0;
    for (size_t i = 0; i < paths.size(); ++i) {
        Polylines fr = diff_pl(Polylines{ paths[i].polyline.to_polyline() }, bs.bands);
        for (const Polyline& pl : fr)
            len1 += pl.length();
        filter_fragments(fr, min_frag);
        per[i] = std::move(fr);
    }
    if (std::fabs(len1 - len0) <= intact_tol)
        return false;   // no material actually removed: keep the entity whole
    for (size_t i = 0; i < paths.size(); ++i) {
        if (per[i].empty())
            continue;
        const coord_t z = paths[i].polyline.points.front().z();
        for (Polyline& pl : per[i])
            out.push_back(new ExtrusionPath(Polyline3(pl, z), paths[i]));
    }
    return true;
}

// Recursive in-place band subtraction over a collection tree. Returns the
// number of leaf path entities removed or fragmented (a fragmented loop /
// multipath counts once). Untouched entities keep their exact original
// pointers.
size_t process_collection(ExtrusionEntityCollection& col, BandBuilder& bb, double intact_tol,
                          double min_frag)
{
    size_t changed = 0;
    ExtrusionEntitiesPtr src;
    src.swap(col.entities);
    ExtrusionEntitiesPtr keep;
    keep.reserve(src.size());
    for (ExtrusionEntity* e : src) {
        if (e->is_collection()) {
            ExtrusionEntityCollection* sub = static_cast<ExtrusionEntityCollection*>(e);
            changed += process_collection(*sub, bb, intact_tol, min_frag);
            if (sub->empty())
                delete sub;
            else
                keep.push_back(sub);
            continue;
        }
        if (e->is_loop()) {
            // Sloped loops carry writer seam metadata the fragment clone
            // cannot preserve: conservative skip (never counted).
            if (dynamic_cast<ExtrusionLoopSloped*>(e) != nullptr) {
                keep.push_back(e);
                continue;
            }
            ExtrusionLoop* loop = static_cast<ExtrusionLoop*>(e);
            const double w = loop->paths.empty() ? bb.fallback_width_mm : bb.width_of(loop->paths.front());
            if (decompose_paths(loop->paths, loop->length(), bb, w, intact_tol, min_frag, keep)) {
                delete loop;
                ++changed;
            }
            else
                keep.push_back(loop);
            continue;
        }
        // A multipath is neither a loop nor a collection, so it has to be
        // matched before the leaf cast below: it holds an ExtrusionPaths
        // vector, not a polyline, and reading it as a path yields a garbage
        // width that blows up the band offset.
        if (ExtrusionMultiPath* mp = dynamic_cast<ExtrusionMultiPath*>(e); mp != nullptr) {
            const double w = mp->paths.empty() ? bb.fallback_width_mm : bb.width_of(mp->paths.front());
            if (decompose_paths(mp->paths, mp->length(), bb, w, intact_tol, min_frag, keep)) {
                delete mp;
                ++changed;
            }
            else
                keep.push_back(mp);
            continue;
        }
        ExtrusionPath* p = dynamic_cast<ExtrusionPath*>(e);
        // Any other entity shape is unknown to this pass: keep it untouched
        // rather than reinterpret its layout.
        if (p == nullptr) {
            keep.push_back(e);
            continue;
        }
        // Contoured / sloped paths likewise keep per-Z or per-endpoint writer
        // metadata a Polyline3-from-2D clone would silently drop.
        if (p->z_contoured || dynamic_cast<const ExtrusionPathSloped*>(p) != nullptr ||
            dynamic_cast<const ExtrusionPathContoured*>(p) != nullptr) {
            keep.push_back(p);
            continue;
        }
        const ClipResult r = clip_single_path(*p, bb.bands_for(bb.width_of(*p)), intact_tol, min_frag, keep);
        if (r == ClipResult::Intact)
            keep.push_back(p);
        else {
            delete p;
            ++changed;
        }
    }
    col.append(std::move(keep));
    return changed;
}

// Recursive external-perimeter drop (outer_wall diagnostic mode).
size_t drop_external_recursive(ExtrusionEntityCollection& col)
{
    size_t removed = 0;
    ExtrusionEntitiesPtr src;
    src.swap(col.entities);
    ExtrusionEntitiesPtr keep;
    keep.reserve(src.size());
    for (ExtrusionEntity* e : src) {
        if (e->is_collection()) {
            ExtrusionEntityCollection* sub = static_cast<ExtrusionEntityCollection*>(e);
            removed += drop_external_recursive(*sub);
            if (sub->empty())
                delete sub;
            else
                keep.push_back(sub);
        }
        else if (e->role() == erExternalPerimeter) {
            delete e;
            ++removed;
        }
        else
            keep.push_back(e);
    }
    col.append(std::move(keep));
    return removed;
}

} // anonymous namespace

size_t subtract_reserve_bands(ExtrusionEntityCollection& collection,
                              const std::vector<FiberStrand>& strands,
                              const ReserveParams& params)
{
    BandBuilder bb;
    bb.fiber = collect_fiber_polylines(strands, params.origin_offset_mm);
    if (bb.fiber.empty())
        return 0;
    bb.composite_mm      = params.composite_width_mm;
    bb.overlap_mm        = params.bond_overlap_mm;
    bb.fallback_width_mm = params.composite_width_mm > 0.0 ? params.composite_width_mm : 0.4;
    // Sub-10-micron length change is treated as untouched so unhit paths keep
    // their original pointers. Surviving clipped fragments shorter than 1 mm
    // are dropped: they cannot form a controlled bead and only add retract /
    // hop / restart noise (G-code review, 2026-10-02).
    const double intact_tol = scale_(0.01);
    const double min_frag   = scale_(1.0);
    return process_collection(collection, bb, intact_tol, min_frag);
}

size_t drop_external_perimeters(ExtrusionEntityCollection& collection)
{
    return drop_external_recursive(collection);
}

} // namespace Fiber
} // namespace Slic3r
