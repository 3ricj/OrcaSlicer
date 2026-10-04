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

#include "../PrintConfig.hpp" // FiberMode, FiberInfillPattern, FiberSchedule

namespace Slic3r {

// Named by the schedule queries below. Forward declarations have to sit in
// Slic3r, not in Slic3r::Fiber: an elaborated-type-specifier inside the Fiber
// namespace would declare new classes there instead of referring to these.
class Layer;
class PrintObject;
class Print;

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

// The fs_* keys the layer schedule decision consumes, so the schedule can be
// evaluated outside the exporter without dragging a PrintConfig along. Filled
// from m_config at the call site; the defaults are the key defaults.
struct FiberScheduleParams
{
    FiberSchedule schedule = FiberSchedule::fsEveryLayer;
    FiberMode     mode     = FiberMode::fmOff;
    // fs_fiber_z_step: height of one fiber macro layer.
    double        macro_layer_height_mm = 0.24;
    // fs_fiber_band_z_min / fs_fiber_band_z_max, mm from the print bottom.
    double        band_z_min_mm = 0.2;
    double        band_z_max_mm = 3.8;
};

// How many plastic layers make up one fiber macro layer: the plastic-layer
// count of a bead `macro_layer_height_mm` tall laid on `plastic_layer_height_mm`
// layers, at least 1. Counted rather than matched on Z, so the schedule stays
// deterministic and exactly periodic when the bead height is not a multiple of
// the layer height. Degenerate inputs (zero or non-finite either side) give 1,
// i.e. every layer closes a macro layer.
size_t fiber_macro_layer_plastic_layers(double macro_layer_height_mm, double plastic_layer_height_mm);

// Whether ONE layer receives fiber under the schedule. The same rules the
// exporter applies per layer, lifted out of the export loop so the decision is
// available before the first layer is written (composite-head priming has to
// know whether the plate carries fiber at all) and testable without a print.
//
// z_bottom is the print-Z of the object's own layer 0: a band schedule measures
// from the object bottom, not from the bed. `height` is the layer height, used
// both as the band's step tolerance and to size a macro layer. `is_last_layer`
// carries the plastic-top-skin rule.
bool fiber_layer_scheduled(const FiberScheduleParams& sched, size_t layer_id, double print_z,
                           double height, bool is_last_layer, double z_bottom);

// The first layer of `object` that the schedule selects, or nullptr when the
// object lays no fiber at all. Thin adapter over fiber_layer_scheduled.
const Layer* first_fiber_layer(const PrintObject& object, const FiberScheduleParams& sched);

// True when any object of the print lays fiber under the schedule. plastic_only
// suppresses fiber even with the capability on, so it never counts as a print
// with CF features.
bool print_carries_fiber(const Print& print, const FiberScheduleParams& sched);

// Distance the whole fiber lane is held inboard of the external-perimeter
// centerline, mm. The harvested rings are that centerline, so this is what keeps
// roving off the exterior surface: the exterior belongs to plastic.
//
// The lane sits behind the requested plastic wall pack - past its walls, past
// half of each bead, less the deliberate bonding overlap - and the wall count is
// clamped to at least ONE: no fiber mode, and no legacy perimeter-following
// schedule, may put roving on the outer wall. A profile asking for zero walls
// outside the fiber still gets one, because the alternative is exposed roving on
// the visible surface. An island too thin to hold the lane behind that wall pack
// is refused by the planner (plastic-only fallback), never squeezed onto the skin.
//
// Non-finite or non-positive inputs fall back to the bead widths so the caller
// always receives a finite non-negative inset.
double fiber_lane_inset_mm(int plastic_walls_outer, double wall_pitch_mm,
                           double plastic_width_mm, double fiber_width_mm,
                           double bond_overlap_mm);

} // namespace Fiber
} // namespace Slic3r
