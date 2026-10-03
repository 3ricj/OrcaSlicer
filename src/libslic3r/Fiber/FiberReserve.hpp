// License: GNU AGPLv3 or higher
//
// FibreSeeker3 plastic reservation for continuous fiber (R2.1).
//
// Wherever a composite strand will be deposited, the plastic body must not
// also be deposited there: the two materials cannot share the same space, and
// a plastic path retracing the fiber channel leaves the composite riding on
// loose retrace strands instead of bonding to the layer below. The reservation
// pass removes plastic from an exclusion band around the FINAL accepted strand
// paths (planned body AND the post-cut tail suffix, per operator requirement
// R2.1-2): forbidden half-width for a plastic bead of planned width w is
//
//     d(w) = composite_width / 2 + w / 2 - bonding_overlap
//
// with composite_width the planned composite bead width and bonding_overlap
// the deliberate intermaterial bond area (planned widths, never measured
// actuals). d(w) <= 0 means that bead width is allowed to touch the fiber -
// nothing is subtracted for it. Bands are capsules (round-closed offset of the
// strand polylines); a closed ring strand yields the expected annulus tube.
//
// The pass mutates an already-sliced collection tree IN PLACE so every
// downstream consumer (export order, cooling buffer, material and time
// accounting) sees the reserved program. Surviving fragments keep the source
// path's role / width / height / mm3_per_mm. Entities whose geometry the bands
// cannot reach are kept untouched; contoured / sloped entities and sloped
// loops carry writer metadata the fragment clone cannot preserve and are
// conservatively left in place (never counted as reserved).

#pragma once

#include <cstddef>
#include <vector>

#include "../Point.hpp" // Vec2d

namespace Slic3r {

class ExtrusionEntityCollection;

namespace Fiber {

class FiberStrand;

struct ReserveParams
{
    // Planned composite bead width (fs_fiber_nozzle_diameter), mm.
    double composite_width_mm = 0.4;
    // Intentional bonding overlap subtracted from the exclusion distance, mm
    // (fs_fiber_bond_overlap).
    double bond_overlap_mm = 0.1;
    // Print origin: collection coordinates are scaled (bed_mm - origin).
    Vec2d origin_offset_mm{0.0, 0.0};
};

// Subtract the exclusion bands of `strands` from `collection` in place.
// Returns the number of leaf path entities clipped away or removed entirely
// (a fragmented loop or multipath counts once). Deterministic: identical input
// collections and strands yield identical results.
size_t subtract_reserve_bands(ExtrusionEntityCollection& collection,
                              const std::vector<FiberStrand>& strands,
                              const ReserveParams& params);

// outer_wall diagnostic mode: drop every external-perimeter leaf, regardless
// of the strand geometry (it shows what a full wall reservation would cost).
// Never a substitute for the band pass; reported as diagnostic only.
size_t drop_external_perimeters(ExtrusionEntityCollection& collection);

} // namespace Fiber
} // namespace Slic3r
