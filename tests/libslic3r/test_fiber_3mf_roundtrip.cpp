// FibreSeeker3 P7: every registered fs_* option must survive BBS-3MF project
// save/load exactly. Persistence is asserted at the libslic3r level, mirroring
// test_precise_seam_3mf.cpp, because the headless CLI --export-3mf path is not
// reliable in this environment. Also asserts the migration direction: a project
// written without any fs_* key loads with the declared defaults, i.e. legacy
// files keep FFF behavior (fs_fiber_enabled false).

#include <catch2/catch_all.hpp>

#include "test_utils.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/Semver.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/miniz_extension.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;

namespace {

// Every registered fs_* key with the exact value carried by the FibreSeeker3
// CF machine profile and its Reinforced L5 process preset (fs_fiber_enforce
// exercised at its opt-in value 1), deliberately different from several config
// defaults, so an accidental fallback to defaults during the round trip would
// fail.
const std::vector<std::pair<std::string, std::string>> fs_vendor_expectations = {
    {"fs_fiber_enabled", "1"},      {"fs_fiber_enforce", "1"},    {"fs_restart_feed", "55"},
    {"fs_tail_length", "54.8"},     {"fs_restart_z_hop", "1.2"},  {"fs_prime_v", "4"},
    {"fs_retract_v", "1"},          {"fs_fiber_rate", "0.98"},    {"fs_matrix_ratio", "0.034"},
    {"fs_deposit_feed", "1200"},    {"fs_rectify_enabled", "1"},  {"fs_rectify_angle", "0"},
    {"fs_rectify_spacing", "2"},    {"fs_rectify_min_seg", "10"}, 
    {"fs_t0_wrap", "1"},            {"fs_fill_min_wall_width", "0"}, {"fs_fill_min_area", "0"},
    {"fs_fiber_wall_loops", "1"},   {"fs_fiber_wall_pitch", "0.7"},
    {"fs_fiber_schedule", "every_layer"}, {"fs_fiber_band_z_min", "0.2"}, {"fs_fiber_band_z_max", "3.8"},
    {"fs_fiber_z_step", "0.24"},    {"fs_tail_v_factor", "1"},    {"fs_t0_temp", "270"},
    {"fs_fiber_mode", "walls"}, {"fs_fiber_coverage_percent", "80"},
    {"fs_fiber_infill_pattern", "isogrid"}, {"fs_fiber_bead_width", "0.8"},
    {"fs_fiber_plastic_walls_outer", "1"}, 
    {"fs_fiber_fill_inset", "0.3"}, {"fs_fiber_fill_angles", "0/90/0"},
    {"fs_fiber_min_radius", "12"},  {"fs_fiber_max_arc_seg", "3"},
    {"fs_fiber_reserve", "band"},   {"fs_fiber_bond_overlap", "0.1"},
    {"fs_fiber_nozzle_diameter", "0.7"},
    {"fs_fiber_prime", "when_fiber"}, {"fs_fiber_prime_length", "90"},
    // Paired composite tool-change sequence. The shipped CF profile does not
    // override these, so the vendor values ARE the declared defaults, picked to
    // match the reference machine: standby 180/150, V-4 @ 600, brush on.
    {"fs_t0_standby_temp", "180"}, {"fs_t1_standby_temp", "150"},
    {"fs_toolchange_retract_v", "4"}, {"fs_toolchange_retract_v_speed", "600"},
    {"fs_brush_on_toolchange", "1"},
};

// Declared defaults from PrintConfig.cpp (must stay in sync with the defs).
// Every key a legacy project can be missing has to come back at the value that
// reproduces FFF behavior, which for the fiber pattern keys means the legacy
// rectilinear producer with both feasibility checks off.
const std::vector<std::pair<std::string, std::string>> fs_default_expectations = {
    {"fs_fiber_enabled", "0"},      {"fs_fiber_enforce", "0"},    {"fs_restart_feed", "55"},
    {"fs_tail_length", "54.8"},     {"fs_restart_z_hop", "1.2"},  {"fs_prime_v", "4"},
    {"fs_retract_v", "1"},          {"fs_fiber_rate", "0.98"},    {"fs_matrix_ratio", "0.034"},
    {"fs_deposit_feed", "1200"},    {"fs_rectify_enabled", "0"},  {"fs_rectify_angle", "0"},
    {"fs_rectify_spacing", "4"},    {"fs_rectify_min_seg", "3"},  
    {"fs_t0_wrap", "0"},            {"fs_fill_min_wall_width", "4"}, {"fs_fill_min_area", "100"},
    {"fs_fiber_wall_loops", "0"},   {"fs_fiber_wall_pitch", "0.7"},
    {"fs_fiber_schedule", "every_layer"}, {"fs_fiber_band_z_min", "0.2"}, {"fs_fiber_band_z_max", "3.8"},
    {"fs_fiber_z_step", "0.24"},    {"fs_tail_v_factor", "1"},    {"fs_t0_temp", "0"},
    {"fs_fiber_mode", "off"},       {"fs_fiber_coverage_percent", "80"},
    {"fs_fiber_infill_pattern", "rectilinear"}, {"fs_fiber_bead_width", "0"},
    {"fs_fiber_plastic_walls_outer", "0"}, 
    {"fs_fiber_fill_inset", "0.3"}, {"fs_fiber_fill_angles", "0/90/0"},
    {"fs_fiber_min_radius", "0"},   {"fs_fiber_max_arc_seg", "0"},
    {"fs_fiber_reserve", "off"},    {"fs_fiber_bond_overlap", "0.1"},
    {"fs_fiber_nozzle_diameter", "0.7"},
    // Migration invariant: a legacy project must not gain a priming line.
    {"fs_fiber_prime", "never"}, {"fs_fiber_prime_length", "80"},
    // And must not gain a paired tool-change sequence it never asked for: the
    // defaults are inert unless fs_fiber_enabled and fs_t0_wrap are both on.
    {"fs_t0_standby_temp", "180"}, {"fs_t1_standby_temp", "150"},
    {"fs_toolchange_retract_v", "4"}, {"fs_toolchange_retract_v_speed", "600"},
    {"fs_brush_on_toolchange", "1"},
};

struct Scene {
    ScopedTemporaryDir backup{"orca_fiber_3mf"};
    Model model;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();

    Scene()
    {
        model.set_backup_path(backup.string());
        auto *object = model.add_object();
        object->name = "fiber round trip";
        object->add_volume(make_cube(20, 20, 2));
        object->add_instance();
    }
};

void save(const std::string &path, Scene &scene)
{
    StoreParams params;
    params.path = path.c_str();
    params.model = &scene.model;
    params.config = &scene.config;
    params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    REQUIRE(store_bbs_3mf(params));
}

void load(const std::string &path, DynamicPrintConfig &config, Model &model)
{
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    struct ImportedResources {
        PlateDataPtrs plates;
        std::vector<Preset *> presets;
        ~ImportedResources() { release_PlateData_list(plates); for (auto *preset : presets) delete preset; }
    } resources;
    bool bbs = false, orca = false;
    Semver version;
    REQUIRE(load_bbs_3mf(path.c_str(), &config, &substitutions, &model, &resources.plates,
                         &resources.presets, &bbs, &orca, &version, nullptr,
                         LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
}

// Raw text of Metadata/project_settings.config, to assert on persistence itself
// rather than trusting only the importer's view of it.
std::string read_project_settings(const std::string &path)
{
    struct Reader {
        mz_zip_archive zip{};
        ~Reader() { if (zip.m_pState) close_zip_reader(&zip); }
    } reader;
    REQUIRE(open_zip_reader(&reader.zip, path));
    for (mz_uint i = 0; i < mz_zip_reader_get_num_files(&reader.zip); ++i) {
        mz_zip_archive_file_stat stat;
        REQUIRE(mz_zip_reader_file_stat(&reader.zip, i, &stat));
        if (stat.m_is_directory) continue;
        std::string name(stat.m_filename);
        std::replace(name.begin(), name.end(), '\\', '/');
        if (name != "Metadata/project_settings.config") continue;
        std::string data(size_t(stat.m_uncomp_size), '\0');
        REQUIRE(mz_zip_reader_extract_to_mem(&reader.zip, i, data.data(), data.size(), 0));
        return data;
    }
    FAIL("project_settings.config missing from exported 3MF");
    return {};
}

} // namespace

TEST_CASE("All fs_* options survive a BBS 3MF project round trip", "[Fiber]")
{
    Scene source;
    for (const auto &[key, value] : fs_vendor_expectations)
        source.config.set_deserialize_strict(key, value);

    // The source must genuinely carry the vendor values before writing.
    for (const auto &[key, value] : fs_vendor_expectations) {
        CAPTURE(key, value);
        REQUIRE(source.config.has(key));
        REQUIRE(source.config.opt_serialize(key) == value);
    }

    ScopedTemporaryFile file(".3mf");
    save(file.string(), source);

    // Independent of the importer: the values must literally be inside the project file.
    const std::string project = read_project_settings(file.string());
    for (const auto &[key, value] : fs_vendor_expectations) {
        CAPTURE(key, value);
        CHECK(project.find("\"" + key + "\"") != std::string::npos);
    }

    DynamicPrintConfig loaded = DynamicPrintConfig::full_print_config();
    Model loaded_model;
    load(file.string(), loaded, loaded_model);

    for (const auto &[key, value] : fs_vendor_expectations) {
        CAPTURE(key, value);
        REQUIRE(loaded.has(key));
        CHECK(loaded.opt_serialize(key) == value);
    }
    // The model payload must survive unchanged alongside the config switch.
    CHECK(loaded_model.objects.size() == 1);
}

TEST_CASE("Legacy project without fs_* keys loads with FFF-safe defaults", "[Fiber]")
{
    Scene source;
    for (const auto &[key, value] : fs_default_expectations)
        source.config.erase(key);

    ScopedTemporaryFile file(".3mf");
    save(file.string(), source);

    // The written project must genuinely lack the keys, otherwise this test
    // would silently verify nothing about legacy migration.
    const std::string project = read_project_settings(file.string());
    CHECK(project.find("fs_fiber_enabled") == std::string::npos);

    // A current build loading that legacy file must see the declared defaults.
    DynamicPrintConfig loaded = DynamicPrintConfig::full_print_config();
    Model loaded_model;
    load(file.string(), loaded, loaded_model);

    for (const auto &[key, value] : fs_default_expectations) {
        CAPTURE(key, value);
        REQUIRE(loaded.has(key));
        CHECK(loaded.opt_serialize(key) == value);
    }
    // Migration invariant: the capability stays off for legacy projects.
    CHECK(loaded.opt_bool("fs_fiber_enabled") == false);
    CHECK(loaded_model.objects.size() == 1);
}
