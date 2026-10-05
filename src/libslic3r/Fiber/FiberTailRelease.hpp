// License: GNU AGPLv3 or higher
//
// FibreSeeker3 fibre-tail cut planning, forward dry release, and the
// deterministic preheat scheduler (owner specification v1.0).
//
// This module is the PLANNING half of two separable changes:
//
//   (1) thermal / transition ordering  -> the preheat scheduler and the
//       station-only-wait ordering contract below;
//   (2) tail and release planning      -> the cut position, the tail margin and
//       the forward dry release.
//
// It is deliberately pure and dependency-free (C++ standard library only, like
// every other TU in Fiber/), so the shipped code compiles and runs with no
// CMake tree and no Boost. Everything here returns data; nothing here touches
// printer state, reads PrintConfig, or emits a tool change.
//
// ---------------------------------------------------------------------------
// The three distances, which the spec fixes and which must not be conflated
// ---------------------------------------------------------------------------
//
//   S = planned deposition length of the strand, EXCLUDING any release
//   T = nominal cut-to-nozzle tail length (fs_tail_length, 54.8 mm)
//   M = tail margin (fs_fiber_tail_margin_mm, 0..3 mm)
//   R = forward dry release length (fs_fiber_release_length_mm, 0..10 mm)
//
//   cut distance            = S - (T + M)
//   post-cut deposition     = T + M
//   total post-cut XY       = T + M + R
//
// The margin moves the cut EARLIER. It never extends the deposited footprint
// past the planned strand end: the endpoint stays the endpoint, and a margin
// deliberately permits a matrix-only terminal section (the tail is V-only by
// construction, see FiberStrand).
//
// R is not deposition, not the restart feed U, and not extra prime. During the
// release there is no U, no V and no E at all, no Z change, no reversal and no
// new sharp turn. Because it deposits nothing it reserves nothing, and because
// it is dry it is never described as sensor-confirmed fibre clearance.
//
// ---------------------------------------------------------------------------
// Seam policy (closed strands)
// ---------------------------------------------------------------------------
//
// A forward release over an open end would lay dry travel into the air. So a
// closed strand is cyclically rotated to a seam placed at the forward end of a
// qualifying straight run, and the release retraces the first R mm of that same
// path forward over already deposited material. Deterministic, never random,
// never a reversal, never a second deposited loop. The margin must not influence
// the ranking, so that a comparison export changing M keeps the same seam.
//
// ---------------------------------------------------------------------------
// The preheat clock is nominal, and that is stated, not hidden
// ---------------------------------------------------------------------------
//
// The scheduler models commanded motion only. Temperature waits and opaque
// macro calls contribute ZERO seconds; arcs use arc length, never the chord;
// acceleration and heater behaviour are not modelled. The number it produces is
// a scheduling position, not a measurement, and every consumer must say so.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "FiberRun.hpp" // FiberPoint + the shared printed-precision rounding

namespace Slic3r {
namespace Fiber {

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
//
// The spec names these codes and the analyzer (tests/fibreseeker) reports them.
// They are declared here as one enum so the planner, the analyzer and the tests
// cannot drift into three spellings of the same finding.

enum class FsCode
{
    WaitOutsideStation,         // FS_WAIT_OUTSIDE_STATION
    CoolBeforeRelease,          // FS_COOL_BEFORE_RELEASE
    CoolBeforeClean,            // FS_COOL_BEFORE_CLEAN
    PreheatClobbered,           // FS_PREHEAT_CLOBBERED
    ReleaseNotExecuted,         // FS_RELEASE_NOT_EXECUTED
    ReleaseExtrusion,           // FS_RELEASE_EXTRUSION
    ReleaseUnsupported,         // FS_RELEASE_UNSUPPORTED
    TailDistanceMismatch,       // FS_TAIL_DISTANCE_MISMATCH
    ERecoveryLocation,          // FS_E_RECOVERY_LOCATION
    TailSharpTurn,              // FS_TAIL_SHARP_TURN              (WARN)
    MacroContractUnverified,    // FS_MACRO_CONTRACT_UNVERIFIED    (WARN)
    NonblockingHeatUnsupported, // FS_NONBLOCKING_HEAT_UNSUPPORTED
    ThermalManagementDisabled,  // THERMAL_MANAGEMENT_DISABLED
    PurgeReleaseOutOfBounds,    // FS_PURGE_RELEASE_OUT_OF_BOUNDS
};

// The literal wire spelling of a code, e.g. "FS_RELEASE_UNSUPPORTED".
const char* fs_code_name(FsCode code);

// True for the two codes the spec classifies as warnings, false for the errors.
bool fs_code_is_warning(FsCode code);

// One diagnostic instance. `detail` carries the object/layer/path/reason text
// the spec requires on FS_RELEASE_UNSUPPORTED and friends.
struct FsFinding
{
    FsCode      code;
    std::string detail;
};

// ---------------------------------------------------------------------------
// Tail / cut / release geometry
// ---------------------------------------------------------------------------

struct TailReleaseParams
{
    // Nominal cut-to-nozzle tail length T, mm (fs_tail_length). Retained value;
    // 54.8 in the shipped profile. Deliberately NOT rounded to 55 to match the
    // reload feed, which is a separate quantity.
    double nominal_tail_mm = 54.8;
    // Tail margin M, mm (fs_fiber_tail_margin_mm), 0..3.
    double margin_mm = 0.0;
    // Forward dry release R, mm (fs_fiber_release_length_mm), 0..10. 0 disables.
    double release_mm = 0.0;
    // Feed for the dry move, mm/s (fs_fiber_release_speed_mm_s), 1..20.
    double release_speed_mm_s = 10.0;
    // Straight supported deposition required before a release may start, mm
    // (fs_fiber_release_anchor_mm), 2..20.
    double anchor_mm = 8.0;
    // ACTUAL composite bead width, mm, resolved by the caller from the fibre
    // configuration. The support test buffers the release footprint by half of
    // THIS value and never by an inherited "T1 WIDTH" comment, which describes
    // the plastic nozzle rather than the roving. 0 means the caller could not
    // resolve it, which the spec makes an explicit configuration error rather
    // than something to guess at.
    double bead_width_mm = 0.7;
};

// The derived distances for one strand. Pure arithmetic on the four numbers
// above plus the strand's own planned length; no geometry needed.
struct TailReleaseDistances
{
    double planned_length_mm   = 0.0; // S
    double cut_distance_mm     = 0.0; // S - (T + M)
    double post_cut_deposit_mm = 0.0; // T + M
    double post_cut_xy_mm      = 0.0; // T + M + R
    double release_mm          = 0.0; // R as requested
    double release_f           = 0.0; // F word for the dry move, mm/min
};

// Validates the parameter set itself (ranges, finiteness). Returns false with a
// human-readable `error` when a value is out of the range the spec fixes; the
// caller surfaces that as a configuration error rather than a slice failure.
bool validate_tail_release_params(const TailReleaseParams& p, std::string* error);

// Derives the four distances for a strand of `planned_length_mm`. Rejects
// S <= T + M explicitly: such a strand would have a zero or negative body, and
// silently shortening the tail would hide an unprintable feature. On failure
// `error` names the reason and `out` is zeroed.
bool plan_tail_release_distances(double planned_length_mm,
                                 const TailReleaseParams& p,
                                 TailReleaseDistances& out,
                                 std::string* error);

// ---------------------------------------------------------------------------
// Straight-run qualification and seam selection
// ---------------------------------------------------------------------------

// One maximal straight run of a polyline: the inclusive segment index range, its
// length, and the measured straightness that qualified it.
struct StraightRun
{
    size_t first_seg = 0;      // first segment index in the run
    size_t last_seg  = 0;      // last segment index in the run (inclusive)
    double length_mm = 0.0;    // cumulative length of the run
    double heading_spread_deg = 0.0;   // cumulative |turn| inside the run
    double max_deviation_mm   = 0.0;   // max perpendicular deviation from the chord
    double start_distance_mm  = 0.0;   // cumulative path distance at the run start
};

// Every maximal run of consecutive segments whose cumulative heading spread is
// <= `max_spread_deg` and whose perpendicular deviation from the run chord is
// <= `max_dev_mm`. Runs are returned in path order and never overlap. A closed
// path is treated as closed: a run that wraps the closure point is emitted once,
// starting at the wrap, so the seam policy sees the same candidates a printer
// sees. `closed` is the planner's real topology, not a proximity guess.
std::vector<StraightRun> find_straight_runs(const std::vector<FiberPoint>& pts,
                                            bool closed,
                                            double max_spread_deg = 2.0,
                                            double max_dev_mm     = 0.05);

// The seam chosen for a closed strand, plus everything the evidence record has
// to carry about the choice.
struct ReleaseSeam
{
    bool   valid = false;
    size_t run_first_seg = 0;
    size_t run_last_seg  = 0;
    double run_length_mm = 0.0;
    double seam_distance_mm = 0.0;   // cumulative path distance of the seam
    double rotation_mm = 0.0;       // == seam_distance_mm, kept separate for the report
    double release_start_distance_mm = 0.0;
    std::vector<FsFinding> rejected; // why each candidate run did not qualify
};

// Selects the deterministic supported forward-release seam for a CLOSED strand.
//
// A candidate run must be at least anchor + R + 0.02 mm long. The seam sits
// R + 0.01 mm before the run's forward end, so both the anchor deposition and
// the release lie inside the same straight run. Among qualifying runs the
// LONGEST wins; ties break on original path index, then on original cumulative
// distance. The margin is not an input to the ranking, which is what makes the
// B/C comparison exports differ only in M.
//
// Returns false with FS_RELEASE_UNSUPPORTED when no run qualifies. The caller
// must fail that export: never shorten R, never fall back to R = 0.
bool select_release_seam(const std::vector<FiberPoint>& pts,
                         bool closed,
                         const TailReleaseParams& p,
                         ReleaseSeam& out,
                         std::string* error);

// Cyclically rotates a closed path so it starts at `seam_distance_mm` along the
// original path, splitting the containing segment if needed. Traversal direction
// is preserved and the centreline locus is unchanged: the returned path visits
// the same points in the same order, from a different start.
std::vector<FiberPoint> rotate_closed_path(const std::vector<FiberPoint>& pts,
                                           double seam_distance_mm);

// ---------------------------------------------------------------------------
// Release footprint support (open strands, and the audit of closed ones)
// ---------------------------------------------------------------------------

// A release is only legal where the buffered footprint of the dry move lies
// inside material ALREADY DEPOSITED IN THAT PHYSICAL LAYER. The solid model
// silhouette is not a substitute and sparse-infill voids are not support, so
// support is answered by a caller-supplied mask rather than by geometry alone.
//
// This is the seam between the pure planner and the exporter's real deposited
// footprint. A G-code-only analyzer cannot answer it at all and must report
// NOT_EVALUATED, never PASS.
struct ReleaseSupportQuery
{
    std::vector<FiberPoint> path;  // the dry path, in order, first point = release start
    double half_width_mm = 0.0;    // half the ACTUAL composite bead width
    double z = 0.0;                // physical layer Z of the deposition being released
    double tolerance_mm = 0.02;    // containment tolerance
};

enum class SupportVerdict
{
    Supported,      // footprint lies inside deposited material of that layer
    Unsupported,    // it does not
    NotEvaluated,   // no deposited-footprint geometry available (analyzer case)
};

// Abstract support oracle. The exporter implements this over the real per-layer
// deposition footprints; the tests implement it over a rectangle.
class ReleaseSupportModel
{
public:
    virtual ~ReleaseSupportModel() = default;
    virtual SupportVerdict query(const ReleaseSupportQuery& q) const = 0;
};

// Builds the buffered footprint polyline of a dry release path: the centreline
// advanced by half the composite bead width at each end, so containment can be
// tested against deposited material. Exposed because the evidence record names
// the footprint it checked.
std::vector<FiberPoint> release_footprint(const std::vector<FiberPoint>& path, double half_width_mm);

// ---------------------------------------------------------------------------
// Release geometry for one strand
// ---------------------------------------------------------------------------

struct ReleasePlan
{
    bool   valid = false;
    bool   closed = false;
    // Where the release starts and ends, absolute bed mm at the deposition Z.
    FiberPoint start;
    FiberPoint end;
    double z = 0.0;
    double requested_mm = 0.0;
    double planned_mm   = 0.0;   // length of `path` on printed-rounded geometry
    double f_mm_min     = 0.0;   // F word for the dry move
    std::vector<FiberPoint> path; // >= 2 points; multi-segment only when legal
    ReleaseSeam seam;            // meaningful when closed
    SupportVerdict support = SupportVerdict::NotEvaluated;
    std::vector<FsFinding> findings;
};

// Plans the forward dry release for one strand.
//
// Closed strand: `pts` must already be rotated to `seam` (call
// select_release_seam then rotate_closed_path first). The release is then the
// first R mm of the rotated path, retraced forward over deposited material.
//
// Open strand: the endpoint is preserved, the last anchor_mm of deposition must
// qualify as straight, and the release is exactly R mm along the final forward
// tangent, accepted only if `support` answers Supported.
//
// R == 0 is not a failure: it means the feature is disabled and the plan comes
// back valid with an empty path, so variant A stays independently exportable.
// Returns false only for a request that cannot be honoured (no qualifying
// candidate, unsupported open-air extension), with FS_RELEASE_UNSUPPORTED.
bool plan_fiber_release(const std::vector<FiberPoint>& pts,
                        bool closed,
                        double z,
                        const TailReleaseParams& p,
                        const ReleaseSupportModel* support,
                        ReleasePlan& out,
                        std::string* error);

// ---------------------------------------------------------------------------
// Emission of the dry release moves
// ---------------------------------------------------------------------------

// Renders the release block for a planned release: the FS_RELEASE_BEGIN comment
// carrying the planned length and speed, one G1 per path segment with XY and F
// and NO U, V or E, then FS_RELEASE_END carrying physical_clearance=unmeasured.
// The comments describe planned commands; they never claim sensing.
std::string emit_fiber_release(const ReleasePlan& plan);

// ---------------------------------------------------------------------------
// Deterministic preheat scheduler
// ---------------------------------------------------------------------------

// One scheduled block inside one head activation. The kinds are exhaustive for
// the nominal clock: anything the clock cannot model is a Macro or a TempWait
// and contributes zero seconds by contract, not by omission.
struct MotionBlock
{
    enum class Kind
    {
        Linear,     // XY(Z) move with no material: duration = path / F
        Deposit,    // material-bearing move: duration = path / F
        Stationary, // extruder-only move: duration = max|E|,|U|,|V| / F
        Dwell,      // G4: duration = dwell seconds as interpreted by the caller
        TempWait,   // M109/M190/M191: contributes ZERO
        Macro,      // opaque (MOVE_TO_BRUSH_STATION, CLEAN_NOZZLE, M2800): ZERO
    };
    Kind   kind = Kind::Linear;
    double path_mm = 0.0;        // arc length for arcs, never the chord
    double feed_mm_min = 0.0;    // modal F of the block
    double max_material_mm = 0.0;// max(|E|,|U|,|V|) for a stationary block
    double dwell_s = 0.0;
};

// Nominal duration of one block in seconds, per the clock definition the spec
// fixes. Temperature waits and macros return 0.0. Non-positive or non-finite
// inputs return 0.0 rather than inventing a duration.
double motion_block_nominal_seconds(const MotionBlock& b);

// Nominal duration of a whole activation, seconds.
double nominal_activation_seconds(const std::vector<MotionBlock>& blocks);

struct PreheatPlan
{
    bool   valid = false;
    bool   inserted = false;
    int    tool = -1;              // the head being preheated
    int    target_c = 0;           // its working temperature
    size_t insert_index = 0;       // block index the M104 goes BEFORE
    double requested_lead_s = 0.0; // what was asked (fs_tool_preheat_lead_s)
    double nominal_lead_s = 0.0;   // what the clock can actually deliver
    double endpoint_s = 0.0;       // nominal end of the outgoing activation
    bool   clamped_to_activation = false;
    bool   clobbered_before_activation = false;
    std::vector<FsFinding> findings;
};

// Schedules the nonblocking M104 for the incoming head of a scheduled physical
// tool change A -> B.
//
// The target insertion point is `lead_s` before the nominal end of A's planned
// deposition plus release. If that precedes A's activation the plan reports
// insert_index 0 (right after A's entry setup, which the caller owns); if it
// lands inside a block the caller splits that block, preserving geometry and
// proportional material payout. The result is clamped to A's activation
// interval, and `clamped_to_activation` records that it was.
//
// lead_s == 0 is NOT "no temperature protection": it means no predictive lead,
// and the plan still reports the head's target at the start of the activation.
// The blocking M109 at the station stays mandatory either way.
PreheatPlan plan_tool_preheat(const std::vector<MotionBlock>& outgoing_activation,
                              int incoming_tool,
                              int incoming_target_c,
                              double lead_s);

// Renders the preheat line. The comment carries the tool and the requested lead
// so the analyzer can pair it with the M109 that must follow at the station.
std::string emit_tool_preheat(const PreheatPlan& plan);

// ---------------------------------------------------------------------------
// Plastic E withdrawal ledger
// ---------------------------------------------------------------------------
//
// One authoritative ledger, shared by the tool-change emitter and ordinary
// travel retraction, so a redundant 1 mm travel withdrawal is never stacked on
// top of a pending tool-change withdrawal and recovery happens exactly once at
// the next real deposition start. It tracks E only: the matrix V sequence is
// separate bookkeeping and is never inferred from it.

class EWithdrawalLedger
{
public:
    // Record an actual negative E withdrawal of `mm` (positive magnitude).
    void withdraw(double mm);
    // True when a travel retraction of `mm` would be redundant on top of what
    // is already pending, i.e. the pending amount already covers it.
    bool covers(double travel_retract_mm) const;
    // The pending recovery amount, printed precision.
    double pending_mm() const { return m_pending; }
    // Consume the whole pending amount; returns what was recovered (0 if none).
    double recover();
    // True once a withdrawal is outstanding, i.e. recovery is owed somewhere.
    bool owed() const { return m_pending > 0.0; }
    // Line number of the last recovery, for the evidence record. -1 = never.
    long recovered_at_line() const { return m_recovered_at_line; }
    void note_recovered_at_line(long line) { m_recovered_at_line = line; }

private:
    double m_pending = 0.0;
    long   m_recovered_at_line = -1;
};

// ---------------------------------------------------------------------------
// Evidence record
// ---------------------------------------------------------------------------

// The JSON sidecar the spec requires. Built as a string here rather than through
// nlohmann so this TU stays dependency-free and the analyzer can read it back.
struct TailReleaseEvidence
{
    // Identity
    std::string object_id;
    size_t      copy_id = 0;
    size_t      layer_index = 0;
    double      physical_z = 0.0;
    std::string window_id;
    // Seam
    std::string original_seam;    // "none" for an open strand
    std::string selected_seam;
    double      rotation_mm = 0.0;
    // Distances
    double      nominal_tail_mm = 0.0;
    double      margin_mm = 0.0;
    double      requested_release_mm = 0.0;
    double      actual_release_mm = 0.0;
    double      post_cut_deposit_mm = 0.0;
    // Endpoints
    FiberPoint  cut_xyz{0.0, 0.0};
    FiberPoint  deposit_end_xyz{0.0, 0.0};
    FiberPoint  release_end_xyz{0.0, 0.0};
    double      release_speed_mm_s = 0.0;
    double      anchor_mm = 0.0;
    double      bead_width_mm = 0.0;
    std::string support_result = "NOT_EVALUATED";
    // Ledger
    double      stationary_v_mm = 0.0;
    double      deposited_v_mm = 0.0;
    long        budget_L = 0;
    // Geometry audit
    size_t      post_cut_turns_gt_60 = 0;
    double      max_post_cut_turn_deg = 0.0;
    // Honesty fields the spec pins
    std::string physical_tail_clearance = "unmeasured";
    std::string macro_contract = "MACRO_CONTRACT_UNVERIFIED";
};

// Serialises one evidence record as a JSON object, one key per line.
std::string evidence_to_json(const TailReleaseEvidence& e);

// Serialises a list of records as a JSON array with the manifest header the
// spec requires (clock definition, its limits, and the separation of static
// export verification from physical print results).
std::string evidence_manifest_to_json(const std::vector<TailReleaseEvidence>& records,
                                      const std::string& variant_name);

} // namespace Fiber
} // namespace Slic3r
