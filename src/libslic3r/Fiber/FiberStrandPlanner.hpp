// License: GNU AGPLv3 or higher
//
// FibreSeeker3 strand PLANNER (operator ruling 2026-10-01, strand build).
//
// Turns one layer's external-perimeter rings into finalized FiberStrands that
// follow the PART PROFILE: for each island formed by the rings, the planner
// always emits the level-0 boundary trace (closed loops through the real
// corners, held boundary_inset_mm inboard of the ring so the roving sits behind
// the plastic walls rather than on them), and either concentric deeper levels
// (area eroded by k*pitch) or -
// when fill is enabled - a SERPENTINE INTERIOR FILL: parallel chords clipped
// to the island eroded by one pitch, walked in scanline order and chained
// into continuous open strands wherever the turn can be taken inside material
// (a turn that would leave the part or cross a hole CUTS the strand there,
// never fakes a move). The two producers work in concert per island: when the
// chord admission REFUSES an island (thin wall) and wall loops are enabled,
// the wall thickness is instead carried by CONCENTRIC WALL LOOPS hugging the
// shape at wall_pitch (operator ruling 2026-10-01: fiber is a fill that hugs
// the part shape on narrow walls, not a single trace line). Either way every
// strand is one continuous deposition (long strands, real turn geometry, no
// chord fragments, and - with fill on - no empty interior). The strand model
// (FiberStrand: one early cut, tail deposited along the remaining path INSIDE
// the part) makes each strand exactly one lifecycle event; for open fill
// strands the tail suffix is the last tail_length mm of the path itself,
// automatically on-part.
//
// The interior fill has three patterns. Rectilinear and solid share the
// single-pass serpentine producer (solid simply runs it at one bead of pitch);
// isogrid strikes three rib families 60 degrees apart, each rib a twin pass one
// bead wide, and chains the ribs into the same long strands.
//
// Reject, never simplify: a loop that cannot carry body + calibrated tail is
// rejected and accounted (finalize() is the sole authority), and so is a strand
// with a corner tighter than min_turn_radius_mm. Islands too small
// to hold any strand are plastic-only fallback and reported separately, so an
// enforced job fails only when a REINFORCEABLE island produced no fiber, not on
// features physically shorter than the tail.
//
// Determinism: islands ordered by area desc then bbox min; loops canonical-
// rotated to their lexicographically smallest vertex and then to the seam
// position, whose scatter offset is a hash of the loop's own address in the
// layer rather than a random draw; deposition direction
// alternates with (island + level) parity; fill chords ordered by scanline
// index then along-angle coordinate, walked greedily nearest-endpoint-first.
// Identical input always yields an identical strand list in an identical order.
//
// All lengths mm, bed coordinates mm (same frame as the producer call site).

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "FiberStrand.hpp"
#include "../PrintConfig.hpp" // FiberInfillPattern
namespace Slic3r {
namespace Fiber {

struct StrandLayerParams
{
    size_t layer_id = 0;              // 1-based, for strand layer markers
    double z = 0.0;                   // deposition Z
    double ratio_p = 0.0;             // matrix:fiber ratio (V = U * P)
    double fiber_rate = 0.0;          // fiber (U) mm per mm of path
    double feed_mm_min = 0.0;         // deposit feedrate
    double tail_length_mm = 0.0;      // calibrated tail (fs_tail_length), must be > 0
    double tail_v_factor = 1.0;       // tail matrix payout as a fraction of the body matrix rate
                                      // (tail V/mm = factor * fiber_rate * ratio_p); 1.0 = auto, (0,1]
    double pitch_mm = 2.0;            // spacing between adjacent fiber lines (concentric or fill)
    size_t max_levels = 16;           // erosion guard for very thick islands (concentric mode)
    double min_island_area_mm2 = 0.04;// dust islands below this are plastic-only
    double min_viable_margin = 1.0;   // extra mm over tail required to call an island reinforceable
    bool   fill_enabled = false;      // serpentine interior fill replaces deeper concentric levels
    // Lay an island's boundary trace and its interior fill as ONE strand where
    // the connector between them stays inside material, instead of cutting
    // between them. The material between the two lanes is exactly what makes
    // the connector legal, so the cut was never physically required; it costs a
    // restart purge and a tail. Default off: existing callers and tests keep
    // byte-identical one-cut-per-loop output.
    bool   chain_loops_into_fill = false;
    double fill_angle_deg = 0.0;      // chord direction, degrees from +X (laydown angle)
    double fill_min_seg_mm = 3.0;     // clipped chords shorter than this are dropped, not emitted
    // Fill admission gate (operator ruling: fill is for large areas, never thin
    // walls; trace and fill must work in concert). Both default 0 = gate open,
    // reproducing pre-gate behavior byte-for-byte for existing callers/tests.
    double fill_min_wall_width = 0.0; // chords admitted only inside the shape opened to this diameter, mm
    double fill_min_area_mm2 = 0.0;   // islands below this area never receive chords, mm^2
    // Concert partner of the gate (operator ruling 2026-10-01): where the gate
    // refuses an island, concentric wall loops fill the wall thickness instead
    // of leaving the level-0 trace alone. Default OFF: existing callers and
    // tests keep byte-identical trace-only behavior on refused islands.
    bool   wall_loops_enabled = false; // chord-refused islands get concentric wall loops, not trace only
    double wall_pitch_mm = 0.7;        // spacing between concentric wall loops, mm (> 0 when enabled)
    // Fiber mode mapping. All three default to the
    // legacy semantics, so existing callers and tests stay byte-identical.
    // Extra erosion of the chord clip region applied AFTER the pitch erosion:
    // chords stop this many mm short of the walls (vendor ExtendIntoPerimeters
    // inverted) - the FFF outer skin of Reinforced/Fortified.
    double fill_outer_inset = 0.0;
    // Clamp on concentric wall loops when wall loops run: exactly
    // wall_outer_loops + wall_inner_loops levels (outer boundary first, inner
    // continuation), and loops additionally run on islands the fill gate
    // ADMITS. 0 = legacy unbounded erosion on chord-REFUSED islands only.
    size_t wall_outer_loops = 0;
    size_t wall_inner_loops = 0;
    // Distance the whole fiber region is held inboard of the input rings, mm.
    // The input rings are the plastic outer-wall centerline, so with plastic
    // walls outboard of the fiber the fiber lane must sit this far inside them
    // or the roving is deposited on top of the plastic bead it is supposed to
    // hide behind. Level 0 rides the inset region and the deeper levels and the
    // interior fill compose from it. 0 = legacy (fiber ON the input ring).
    double boundary_inset_mm = 0.0;
    // Interior fill pattern. Rectilinear (the default) is the legacy
    // single-pass serpentine and keeps existing callers byte-identical; solid
    // is the same producer driven at one bead of pitch; isogrid strikes three
    // rib families 60 degrees apart, each rib a twin pass one bead wide.
    FiberInfillPattern infill_pattern = FiberInfillPattern::fipRectilinear;
    // Deposited composite bead width, mm. Required by isogrid (twin-pass offset
    // and wall clearance); unused by the other patterns.
    double fiber_width_mm = 0.0;
    // Smallest corner fillet radius the roving can be laid around, mm.
    // 0 = off (legacy). The planner never widens or straightens a path to make
    // it legal and reducing the feedrate cannot legalize a tight bend, so the
    // only choices are to deposit the corner anyway or to cut the fiber there;
    // tight_turn_policy picks between them and both outcomes are counted in
    // tight_turns.
    double min_turn_radius_mm = 0.0;
    // What to do at a corner tighter than min_turn_radius_mm. keep (default)
    // deposits the strand as planned, which is byte-identical to the limit
    // being off; split cuts the strand at the offending joint and lays each
    // piece separately.
    FiberTightTurnPolicy tight_turn_policy = FiberTightTurnPolicy::fttKeep;
    // Where the seam of a closed loop sits. aligned (default) is the canonical
    // start vertex and is byte-identical to the legacy behavior; the other two
    // move the start along the ring, inserting a collinear vertex where the new
    // start falls mid-edge, so the geometry and its length never change.
    FiberSeamPosition seam_position = FiberSeamPosition::fspAligned;
    // Forward dry release (fs_fiber_release_length_mm > 0): when set, a closed
    // loop's seam is placed by the deterministic release-seam rule instead of by
    // seam_position, because the dry move has to retrace material the strand
    // itself deposited, which is only true where the release lies on the same
    // qualifying straight run as the seam. Null (release off) leaves the seam
    // policy byte-identical to the legacy behaviour.
    //
    // The rotation happens HERE, before finalize(), because a finalized strand is
    // immutable. The emission side still re-validates the opening run and refuses
    // the release (FS_RELEASE_UNSUPPORTED) when a loop has no qualifying run, so
    // a refused release never silently becomes R = 0.
    const class TailReleaseParams* release_seam_params = nullptr;
    // Longest segment the strand may be reported as, mm. Segments above it are
    // split at collinear points, so the geometry and its length are unchanged
    // and only the deposition resolution rises (the reference machine reports
    // its fiber windows at a ~1 mm median segment). 0 = off (legacy).
    double max_arc_seg_mm = 0.0;
};

struct StrandLayerResult
{
    std::vector<FiberStrand> strands; // finalized, emission order
    size_t islands = 0;               // islands above the dust threshold
    size_t viable_rings = 0;          // level-0 loops on reinforceable islands (enforce input)
    size_t skipped_tiny = 0;          // dust islands -> plastic-only fallback
    size_t rejected_short = 0;        // loops/strands shorter than tail + margin -> plastic-only fallback
    size_t rejected_other = 0;        // finalize failures that are NOT a length fallback (enforce input)
    // Fill accounting (zero while fill_enabled is off):
    size_t fill_chords = 0;           // chords admitted to the walk (>= min seg)
    size_t fill_chords_dropped = 0;   // clipped chords below min seg: dropped and counted
    size_t fill_breaks = 0;           // strand breaks where a turn was geometrically impossible
    size_t wall_loops = 0;            // wall-loop strands on chord-refused islands (zero while wall_loops_enabled is off)
    size_t rejected_thin = 0;         // islands the boundary inset consumes entirely -> plastic-only fallback
    // Turn-radius accounting (zero while min_turn_radius_mm is 0). Counted under
    // BOTH policies, so an operator can size the cost of splitting before
    // switching: tight_turns is the number of offending joints found,
    // tight_turn_splits the number of cuts the split policy actually made.
    size_t tight_turns = 0;
    size_t tight_turn_splits = 0;
    // Boundary traces chained into an interior fill instead of being cut away
    // from it (zero while chain_loops_into_fill is off). Each one is a cut, a
    // restart purge and a tail that the layer did not have to spend.
    size_t chained_loops = 0;
};

// Builds the layer's strands from closed perimeter rings (bed mm; each ring is
// an ordered vertex list without a duplicated closing vertex). Rings are unioned
// into islands with holes (nesting depth decides outer vs hole), then either
// eroded level by level or boundary-traced + serpentine-filled per
// params.fill_enabled. Returns an empty result (with first_error set) on invalid
// parameters. `first_error` may be null.
StrandLayerResult build_layer_strands(const std::vector<std::vector<FiberPoint>>& rings,
                                      const StrandLayerParams& params,
                                      std::string* first_error = nullptr);

} // namespace Fiber
} // namespace Slic3r
