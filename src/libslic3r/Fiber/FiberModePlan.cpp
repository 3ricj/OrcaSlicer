// License: GNU AGPLv3 or higher
//
// Implementation of the fiber mode -> strand planner mapping. See
// FiberModePlan.hpp for the contract.

#include "FiberModePlan.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace Slic3r {
namespace Fiber {

std::vector<double> parse_fiber_angle_list(const std::string& spec)
{
    std::vector<double> out;
    // Slash-separated canonical form ("0/90/0"); commas accepted as a
    // tolerant alias. strtod-based so "45", " 45.5", "-30" parse; a token
    // with trailing junk keeps its numeric prefix, matching strtod behavior.
    std::string tok;
    const auto flush = [&out, &tok]() {
        if (tok.empty())
            return;
        const char* begin = tok.c_str();
        char* end = nullptr;
        const double v = std::strtod(begin, &end);
        if (end != begin && std::isfinite(v))
            out.push_back(v);
        tok.clear();
    };
    for (const char c : spec) {
        if (c == '/' || c == ',')
            flush();
        else if (c != ' ' && c != '\t')
            tok.push_back(c);
        else
            flush(); // whitespace also terminates a token
    }
    flush();
    return out;
}

double fiber_pitch_for_coverage(double coverage_percent, double fiber_width_mm)
{
    const double d = fiber_width_mm;
    if (!(d > 0.0) || !std::isfinite(d))
        return 0.7; // planner default; caller guards the key, defensive only
    if (!(coverage_percent > 0.0) || !std::isfinite(coverage_percent))
        return d; // coverage 0: fill off, this value only has to stay positive
    const double pitch = d / (coverage_percent / 100.0);
    if (!std::isfinite(pitch))
        return d;
    return std::min(std::max(pitch, 0.5 * d), 20.0);
}

double fiber_isogrid_pitch(double density_percent, double fiber_width_mm)
{
    const double d = fiber_width_mm;
    if (!(d > 0.0) || !std::isfinite(d))
        return 0.7;
    if (!(density_percent > 0.0) || !std::isfinite(density_percent))
        return d; // density 0: ribs off, this value only has to stay positive
    const double pitch = 3.0 * d / (density_percent / 100.0);
    if (!std::isfinite(pitch))
        return d;
    return std::min(std::max(pitch, d), 200.0);
}

FiberModeLayer resolve_fiber_mode_layer(FiberMode mode, FiberInfillPattern pattern,
                                        const std::vector<double>& angles_deg,
                                        double coverage_percent, double fiber_width_mm,
                                        double fill_inset_mm,
                                        size_t fiber_layer_idx)
{
    FiberModeLayer out;
    // Solid forces 100 percent coverage; the coverage key is inert there.
    // Walls mode takes its density from the coverage key.
    const double cov = mode == FiberMode::fmSolid ? 100.0 : coverage_percent;
    out.pattern = pattern;
    // The spacing law belongs to the pattern. Solid lays adjacent passes one
    // bead apart and ignores the density entirely; isogrid and the legacy
    // rectilinear serpentine both scale with it, by different laws.
    switch (pattern) {
    case FiberInfillPattern::fipSolid:
        out.pitch_mm = fiber_width_mm > 0.0 && std::isfinite(fiber_width_mm) ? fiber_width_mm : 0.7;
        break;
    case FiberInfillPattern::fipIsogrid:
        out.pitch_mm = fiber_isogrid_pitch(cov, fiber_width_mm);
        break;
    default:
        out.pitch_mm = fiber_pitch_for_coverage(cov, fiber_width_mm);
        break;
    }
    out.fill_enabled  = cov > 0.0;
    out.fill_outer_inset = std::max(0.0, fill_inset_mm);
    const std::vector<double> empty;
    const std::vector<double>& angles = angles_deg.empty() ? empty : angles_deg;
    if (!angles.empty()) {
        const double a = angles[fiber_layer_idx % angles.size()];
        out.fill_angle_deg = std::isfinite(a) ? a : 0.0;
    }
    return out;
}

double fiber_lane_inset_mm(int plastic_walls_outer, double wall_pitch_mm,
                           double plastic_width_mm, double fiber_width_mm,
                           double bond_overlap_mm)
{
    // The exterior belongs to plastic: a fiber lane with NOTHING outboard of it
    // would be roving laid on the visible surface, so the wall pack is at least
    // one wall deep whatever the key asks for.
    const int walls = std::max(1, plastic_walls_outer);
    // Non-finite or non-positive geometry cannot be reasoned about; fall back to
    // the widths the caller always has (bead widths) rather than emit a lane at
    // the surface. A negative overlap is treated as no overlap.
    if (!std::isfinite(plastic_width_mm) || !(plastic_width_mm > 0.0))
        plastic_width_mm = fiber_width_mm;
    if (!std::isfinite(fiber_width_mm) || !(fiber_width_mm > 0.0))
        return 0.0; // nothing to place: the caller's bead width is unusable
    if (!std::isfinite(wall_pitch_mm) || !(wall_pitch_mm > 0.0))
        wall_pitch_mm = plastic_width_mm;
    if (!std::isfinite(bond_overlap_mm) || bond_overlap_mm < 0.0)
        bond_overlap_mm = 0.0;
    // Past the wall pack, past half of each bead, less the deliberate bond.
    const double inset = double(walls) * wall_pitch_mm + 0.5 * (plastic_width_mm + fiber_width_mm) -
        bond_overlap_mm;
    return std::isfinite(inset) && inset > 0.0 ? inset : 0.0;
}

} // namespace Fiber
} // namespace Slic3r
