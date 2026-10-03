// License: GNU AGPLv3 or higher
//
// FibreSeeker3 fiber mode resolution.
//
// Pure mapping from the fs_fiber_mode selector and its pattern keys onto the
// strand planner's per-layer parameters. Walls mode carries an interior-only
// fiber fill whose density is the coverage percent; solid is 100 percent
// coverage inside the outer plastic shell. Both keep the fill inset short of
// the walls (fill_outer_inset) while the outer skin stays plastic.
// plastic_only is suppressed upstream (the G-code exporter never calls the
// fiber block in that mode); off is the perimeter-following path and never
// reaches these functions.
//
// The spacing law belongs to the PATTERN, not to the mode: isogrid ribs are
// three beads apart at full density, a solid fill is one bead apart, and the
// legacy rectilinear serpentine is one bead divided by the density. The fiber
// perimeter itself is one loop per island per fiber layer under every mode -
// the plastic wall counts position that loop (see StrandLayerParams::
// boundary_inset_mm) rather than multiplying it.
//
// Deterministic, stateless and side-effect free: the layer counter is passed
// in, so the same inputs always resolve to the same parameters (unit-testable
// without a print).

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "../PrintConfig.hpp" // FiberMode, FiberInfillPattern

namespace Slic3r {
namespace Fiber {

// One resolved fiber layer under a fiber mode. The caller overlays
// these onto the legacy StrandLayerParams wiring; everything not listed
// (gates, tail, rates, wall pitch) keeps its fs_* key value.
struct FiberModeLayer
{
    bool   fill_enabled     = true;    // false only when coverage == 0 (fiber perimeter only)
    FiberInfillPattern pattern = FiberInfillPattern::fipRectilinear; // interior fill producer
    double pitch_mm         = 0.7;     // fiber line / rib spacing for that pattern, mm
    double fill_angle_deg   = 0.0;     // this layer's laydown angle, from the cycled angle list
    double fill_outer_inset = 0.0;     // fill clip region erosion short of the walls, mm
};

// Parses the slash-separated laydown angle list (fs_fiber_fill_angles, e.g.
// "0/90/0"): tokens split on '/' and ',', whitespace and empty tokens skipped,
// values kept as written. Returns empty when no parseable angle exists.
std::vector<double> parse_fiber_angle_list(const std::string& spec);

// Legacy rectilinear spacing: pitch = fiber_width / (coverage/100), clamped to
// [0.5 * width, 20 mm]. coverage <= 0 (or a non-finite width) returns the width
// fallback: the fill is switched off by the caller, and a valid positive pitch
// must still reach the planner.
double fiber_pitch_for_coverage(double coverage_percent, double fiber_width_mm);

// Isogrid rib spacing: pitch = 3 * fiber_width / (density/100), clamped to
// [fiber_width, 200 mm]. Measured on the reference machine's Benchy Reinforced
// levels - a 0.8 mm bead gives 3.00 mm ribs at 80 percent and 4.00 mm at 60
// percent - and the factor 3 is what makes an isogrid of three rib families
// come out at the density the key asks for.
double fiber_isogrid_pitch(double density_percent, double fiber_width_mm);

// Resolves one fiber-emitted layer of a reinforced/fortified print.
// mode            - fmWalls or fmSolid (fmSolid forces 100 percent
//                   coverage; the coverage key is ignored there).
// pattern         - fs_fiber_infill_pattern, which owns the spacing law.
// angles_deg      - parsed fs_fiber_fill_angles; empty falls back to {0}.
// fiber_width_mm  - deposited composite bead width (fs_fiber_bead_width, or the
//                   fiber nozzle diameter when that key is unset).
// fiber_layer_idx - counts fiber-EMITTED layers (0-based): the angle cycles as
//                   angles[idx % size], independent of skipped plastic layers,
//                   so band schedules still alternate the laydown.
FiberModeLayer resolve_fiber_mode_layer(FiberMode mode, FiberInfillPattern pattern,
                                        const std::vector<double>& angles_deg,
                                        double coverage_percent, double fiber_width_mm,
                                        double fill_inset_mm,
                                        size_t fiber_layer_idx);

} // namespace Fiber
} // namespace Slic3r
