// License: GNU AGPLv3 or higher
//
// Measured non-CF byte-identity proof for the paired composite tool-change
// sequence.
//
// The owner's ruling is that everything added for the paired sequence stays
// behind fs_fiber_enabled / fs_t0_wrap, so a print that lays no fibre must come
// out of the exporter byte-for-byte as it did before the sequence existed. That
// claim is cheap to argue from the diff and expensive to fake, so this test
// measures it: it slices a plastic-only model through the real exporter and
// writes the .gcode next to the test binary. Two builds of the same source
// (pristine base vs modified) then produce files that are compared byte for
// byte by tmp/fs_diff_proof.js, with only the exporter's own generation
// timestamp line normalised.
//
// The case is deliberately free of any assertion about the fibre path: it only
// has to be deterministic and to reach psSlicingFinished, so it also compiles
// and passes on a tree that has no fibre tool-change code at all.

#include <catch2/catch_all.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <boost/filesystem.hpp>

#include <fstream>
#include <string>

using namespace Slic3r;

namespace {

// Machine / process / filament presets for the FS3 single-extruder plastic
// configuration, shared with the Python export gate so both halves of the
// FibreSeeker3 test surface slice the same thing.
const char* const FS_PRESET_FILES[] = {
    "fs3_t1_plastic_machine.json",
    "fs3_t1_plastic_process.json",
    "fs3_t1_plastic_filament.json",
};

void load_preset(DynamicPrintConfig& onto, const boost::filesystem::path& file)
{
    REQUIRE(boost::filesystem::exists(file));
    DynamicPrintConfig                 cfg;
    std::map<std::string, std::string> key_values;
    std::string                        reason;
    cfg.load_from_json(file.string(), ForwardCompatibilitySubstitutionRule::Disable, key_values, reason);
    onto.apply(std::move(cfg));
}

// Pin the two settings that make a byte comparison meaningful. See the call
// site for what each one is doing there.
void make_bytes_comparable(DynamicPrintConfig& config)
{
    config.set_key_value("gcode_label_objects", new ConfigOptionBool(false));
}

} // namespace

TEST_CASE("Plastic-only export is deterministic and reaches the exporter", "[Fiber][FiberExport]")
{
    const boost::filesystem::path presets = boost::filesystem::path(PROFILES_DIR).parent_path().parent_path() / "tests" / "fibreseeker" / "presets";

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    for (const char* f : FS_PRESET_FILES)
        load_preset(config, presets / f);

    // Explicit rather than left at the registered default, so the proof does
    // not silently become a fibre test if that default ever changes.
    config.set_key_value("fs_fiber_enabled", new ConfigOptionBool(false));
    make_bytes_comparable(config);

    Model model;
    ModelObject* obj = model.add_object("cube", "", make_cube(20., 20., 20.));
    ModelInstance* inst = obj->add_instance();
    inst->set_offset(Vec3d(150., 150., 0.));

    Print print;
    print.apply(model, config);
    print.process();
    REQUIRE(print.is_step_done(psSlicingFinished));

    const boost::filesystem::path out = boost::filesystem::current_path() / "fs_export_plastic.gcode";
    print.export_gcode(out.string(), nullptr, nullptr);
    REQUIRE(boost::filesystem::exists(out));

    // Non-empty and carrying the machine start block, so a zero-length or
    // aborted export cannot pass the byte comparison by being empty twice.
    std::ifstream is(out.string(), std::ios::binary);
    const std::string body{std::istreambuf_iterator<char>(is), std::istreambuf_iterator<char>()};
    CHECK(body.size() > 1000);
    CHECK(body.find("MOVE_TO_BRUSH_STATION") != std::string::npos);
    // The cooling path is genuinely in the compared bytes. This fork does not
    // register enable_fan / fan_mode / min_fan_speed at all (the shipped preset
    // carries a dead enable_fan key), so CoolingBuffer drives the fan from
    // fan_cooling_layer_time / fan_min_speed / fan_max_speed and emits varying
    // speeds. Asserting a non-zero count is what makes the byte comparison a
    // fan-output proof rather than a claim: if the cooling path ever stopped
    // emitting, this fails instead of quietly narrowing the comparison.
    {
        size_t n = 0;
        for (size_t p = body.find("M106"); p != std::string::npos; p = body.find("M106", p + 4))
            ++n;
        CHECK(n > 0);
    }
}

// The same slice with the composite head capability on, so the exporter emits a
// real fibre window and therefore real tool-change blocks. This is the case that
// produces the before/after evidence: run it against the pristine base and
// against the modified tree and diff the exported files, and the tool-change
// blocks are the only thing that can differ.
//
// The schedule is forced to every_layer / solid so a plain cube carries fibre on
// its solid layers under either build; the shipped CF-nozzle profile supplies the
// V-axis numbers (fs_prime_v 4, fs_retract_v 1) and fs_t0_temp 270.
TEST_CASE("Composite export reaches the fibre window and emits the tool-change wrap", "[Fiber][FiberExport]")
{
    const boost::filesystem::path presets = boost::filesystem::path(PROFILES_DIR).parent_path().parent_path() / "tests" / "fibreseeker" / "presets";

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    for (const char* f : FS_PRESET_FILES)
        load_preset(config, presets / f);

    config.set_key_value("fs_fiber_enabled", new ConfigOptionBool(true));
    make_bytes_comparable(config);
    config.set_key_value("fs_t0_wrap", new ConfigOptionBool(true));
    config.set_key_value("fs_fiber_schedule", new ConfigOptionEnum<FiberSchedule>(FiberSchedule::fsEveryLayer));
    config.set_key_value("fs_fiber_mode", new ConfigOptionEnum<FiberMode>(FiberMode::fmSolid));
    config.set_key_value("fs_fiber_band_z_min", new ConfigOptionFloat(0.0));
    config.set_key_value("fs_fiber_band_z_max", new ConfigOptionFloat(1000.0));
    config.set_key_value("fs_t0_temp", new ConfigOptionInt(270));
    config.set_key_value("fs_prime_v", new ConfigOptionFloat(4.0));
    config.set_key_value("fs_retract_v", new ConfigOptionFloat(1.0));
    // No fs_fiber_prime line: the key postdates the pristine base this test also
    // compiles against, and its registered default is "never", which is what is
    // wanted here - the priming line is a separate feature and it consumes the
    // first window's blocking wait.
    Model model;
    ModelObject* obj = model.add_object("cube", "", make_cube(20., 20., 20.));
    ModelInstance* inst = obj->add_instance();
    inst->set_offset(Vec3d(150., 150., 0.));

    Print print;
    print.apply(model, config);
    print.process();
    REQUIRE(print.is_step_done(psSlicingFinished));

    const boost::filesystem::path out = boost::filesystem::current_path() / "fs_export_fiber.gcode";
    print.export_gcode(out.string(), nullptr, nullptr);
    REQUIRE(boost::filesystem::exists(out));

    std::ifstream is(out.string(), std::ios::binary);
    const std::string body{std::istreambuf_iterator<char>(is), std::istreambuf_iterator<char>()};
    CHECK(body.size() > 1000);
    // A fibre window ran: the wrap markers the emitter owns are in the output.
    CHECK(body.find("M1001") != std::string::npos);
    CHECK(body.find("M1002") != std::string::npos);

    // The tool-change sequence is PAIRED AND PER-WINDOW, not merely present.
    //
    // Asserting only that a marker appears somewhere is not a guard: this case
    // passed unchanged against the pristine base, where the whole sequence fired
    // exactly once per plate. What the objective requires is that every switch
    // carries its own block, so the assertion has to be about counts and their
    // relationships. Measured on this model: 100 plastic->fibre switches and 100
    // fibre->plastic switches, i.e. 200 tool changes, and the pristine base emits
    // one of each per-plate item and no standby pair and no tool-change V retract
    // at all.
    //
    // Deliberately text-only: no new fs_* key is named here, so this file still
    // compiles against a base that does not register them, which is what lets the
    // same source run against a control build.
    // Count emitted lines, not text. The export carries a config-dump block whose
    // comments echo machine_start_gcode verbatim, so a plain substring search
    // counts a command that was never sent: CLEAN_NOZZLE and M106 P1 S0 each read
    // one high that way (measured: 202 and 102 by substring, 201 and 101 by line).
    // Only lines that are not comments are G-code.
    const auto count_of = [&body](const char* needle) {
        size_t n = 0;
        size_t line_start = 0;
        for (;;) {
            const size_t nl = body.find('\n', line_start);
            const size_t line_end = nl == std::string::npos ? body.size() : nl;
            size_t s2 = line_start;
            while (s2 < line_end && (body[s2] == ' ' || body[s2] == '\t')) ++s2;
            if (s2 < line_end && body[s2] != ';') {
                const std::string line = body.substr(s2, line_end - s2);
                if (line.find(needle) != std::string::npos) ++n;
            }
            if (nl == std::string::npos) break;
            line_start = nl + 1;
        }
        return n;
    };
    const size_t n_windows   = count_of("M1002");
    const size_t n_withdraw  = count_of("G1 E-10");
    const size_t n_standby_t1 = count_of("M104 S150 T1");
    const size_t n_standby_t0 = count_of("M104 S180 T0");
    const size_t n_wait      = count_of("M109 S270 T0");
    const size_t n_brush     = count_of("CLEAN_NOZZLE");
    const size_t n_v_retract = count_of("V-4.000");
    const size_t n_fan_off   = count_of("M106 P1 S0");
    const size_t n_aux_on    = count_of("M106 P3 S255");
    const size_t n_aux_off   = count_of("M106 P3 S0");

    // Several windows, so "once per plate" cannot satisfy a floor.
    REQUIRE(n_windows >= 20);
    // Every plastic->fibre switch: outgoing withdrawal, standby for the head being
    // put away, blocking readiness wait, and the second matrix withdrawal on the
    // way back out. These four are one-per-switch, so they must agree with each
    // other, and the switch count is half the window count.
    CHECK(n_withdraw == n_windows / 2);
    CHECK(n_standby_t1 == n_windows / 2);
    CHECK(n_wait == n_windows / 2);
    CHECK(n_v_retract == n_windows / 2);
    // Both heads carry a standby target at both switch directions, so twice.
    CHECK(n_standby_t0 == n_windows);
    // Brush at every tool change (both directions) plus the start-gcode clean.
    CHECK(n_brush == n_windows + 1);
    // Part-cooling is explicitly zeroed while the fibre head is depositing, once
    // per switch plus the start-gcode zero.
    CHECK(n_fan_off == n_windows / 2 + 1);
    // The auxiliary ports are raised for every fibre pass and zeroed for every
    // plastic pass, so both counts track the switch count. The zero side also
    // appears once in the start gcode, which is what left them stranded at 0
    // before this clause existed.
    CHECK(n_aux_on == n_windows / 2);
    CHECK(n_aux_off == n_windows / 2 + 1);
}