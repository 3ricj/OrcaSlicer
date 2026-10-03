// Regression test for the "option in def + UI but missing from preset key list"
// crash class.
//
// The print preset's DynamicPrintConfig is seeded with only the keys returned by
// Preset::print_options() (PresetBundle.cpp). A field added to PrintRegionConfig
// or PrintObjectConfig and registered via print_config_def plus a TabPrint
// optgroup, but left out of print_options(), still gets its control built; on tab
// activation reload_config -> get_config_value dispatches to opt_bool/opt_int on a
// DynamicPrintConfig with no entry for the key, and the accessor null-derefs the
// result of option<T>(key).
//
// The invariant asserted here is the inverse: every key declared on
// PrintRegionConfig and PrintObjectConfig appears in Preset::print_options() or
// Preset::filament_options(), the two preset key lists that seed a print preset's
// DynamicConfig.

#include <catch2/catch_all.hpp>

#include "libslic3r/Preset.hpp"
#include "libslic3r/PrintConfig.hpp"

#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>
#include <nlohmann/json.hpp>

#include <map>
#include <set>
#include <sstream>

using namespace Slic3r;

namespace {

// Deprecated keys renamed in handle_legacy() (ironing_direction ->
// ironing_angle, wall_infill_order -> wall_sequence); neither is in a
// preset list. Register new options in a preset list, not here.
const std::set<std::string> kDeprecatedRegionFields = {
    "ironing_direction",
    "wall_infill_order",
};

void check_keys_are_in_a_preset(const t_config_option_keys& keys, const std::string& class_name)
{
    REQUIRE_FALSE(keys.empty());
    const auto& print_options    = Preset::print_options();
    const auto& filament_options = Preset::filament_options();
    const std::set<std::string> in_print(print_options.begin(), print_options.end());
    const std::set<std::string> in_filament(filament_options.begin(), filament_options.end());
    for (const std::string& key : keys) {
        DYNAMIC_SECTION(class_name << "::" << key)
        {
            INFO("'" << key << "' on " << class_name
                     << " is missing from "
                        "Preset::print_options()/filament_options(); add it to "
                        "s_Preset_print_options (or s_Preset_filament_options) in Preset.cpp.");
            const bool registered = in_print.count(key) || in_filament.count(key) || kDeprecatedRegionFields.count(key);
            REQUIRE(registered);
        }
    }
}

// The keys a preset JSON may carry that are preset metadata rather than config
// options. ConfigBase::load_from_json() consumes these into key_values and never
// looks them up in print_config_def; everything else it hands to set_deserialize.
const std::set<std::string> kPresetMetadataKeys = {
    BBL_JSON_KEY_VERSION,    BBL_JSON_KEY_IS_CUSTOM,  BBL_JSON_KEY_URL,
    BBL_JSON_KEY_NAME,       BBL_JSON_KEY_DESCRIPTION, BBL_JSON_KEY_TYPE,
    BBL_JSON_KEY_FROM,       BBL_JSON_KEY_SETTING_ID, BBL_JSON_KEY_BASE_ID,
    BBL_JSON_KEY_USER_ID,    BBL_JSON_KEY_FILAMENT_ID, BBL_JSON_KEY_INHERITS,
    BBL_JSON_KEY_INCLUDES,   BBL_JSON_KEY_INSTANTIATION,
    ORCA_JSON_KEY_UPDATE_TIME, ORCA_JSON_KEY_CREATED_TIME, ORCA_JSON_KEY_RENAMED_FROM,
};

// A key reaches a real option if it is defined, is an alias of one, or
// handle_legacy() renames it to one. handle_legacy() clearing the key cannot be
// taken as recognition: its last act is to clear every key print_config_def does
// not have, so a typo comes back from it looking exactly like a retired option.
bool key_reaches_an_option(const std::string& key)
{
    if (print_config_def.get(key) != nullptr)
        return true;
    for (const auto& opt : print_config_def.options)
        for (const t_config_option_key& alias : opt.second.aliases)
            if (alias == key)
                return true;
    t_config_option_key renamed = key;
    std::string         value;
    PrintConfigDef::handle_legacy(renamed, value);
    return ! renamed.empty() && renamed != key && print_config_def.get(renamed) != nullptr;
}

} // namespace

// A key that no ConfigDef knows is dropped on load without a word: the preset
// silently inherits the default instead, so a typo in a shipped profile looks
// like a setting that has no effect. Catch it here, where the registry itself is
// the reference, rather than in orca_profile_tool.py, which only checks preset
// structure and ids.
//
// Vendors outside kVendorsAssertedClean carry a backlog of keys this build does
// not define (settings from other forks, options retired without a profile
// sweep). Those are reported, not failed, so the check can still hold the line
// for the vendors we do maintain instead of being disabled outright.
TEST_CASE("Every setting key in a shipped profile is a registered config option", "[Preset][Config][Profiles]")
{
    namespace fs = boost::filesystem;

    const std::set<std::string> kVendorsAssertedClean = {"FibreSeeker3"};

    std::map<std::string, bool>                       known_cache;
    std::map<std::string, std::map<std::string, int>>  unknown_by_vendor;
    size_t presets = 0;

    for (fs::recursive_directory_iterator it{fs::path(PROFILES_DIR)}, end; it != end; ++it) {
        if (! fs::is_regular_file(it->status()) || it->path().extension() != ".json")
            continue;

        nlohmann::json doc;
        {
            boost::nowide::ifstream file(it->path().string());
            try {
                file >> doc;
            } catch (const nlohmann::json::exception&) {
                FAIL(it->path().string() << " is not parseable JSON");
            }
        }
        if (! doc.is_object())
            continue;
        // Vendor indexes and machine-model stubs carry their own metadata
        // vocabulary and never go through set_deserialize. A preset declares one
        // of the three preset types and is named; the filament id/name lookup
        // tables declare a type but have no name and are not presets.
        const auto type = doc.find(BBL_JSON_KEY_TYPE);
        if (type == doc.end() || ! type->is_string())
            continue;
        if (Preset::get_type_from_string(type->get<std::string>()) == Preset::TYPE_INVALID)
            continue;
        if (doc.find(BBL_JSON_KEY_NAME) == doc.end())
            continue;
        ++presets;

        const std::string vendor = fs::relative(it->path(), fs::path(PROFILES_DIR)).begin()->stem().string();
        for (const auto& entry : doc.items()) {
            const std::string& key = entry.key();
            if (kPresetMetadataKeys.count(key))
                continue;
            auto cached = known_cache.find(key);
            if (cached == known_cache.end())
                cached = known_cache.emplace(key, key_reaches_an_option(key)).first;
            if (! cached->second)
                ++unknown_by_vendor[vendor][key];
        }
    }

    REQUIRE(presets > 0);                                  // PROFILES_DIR actually resolved

    std::ostringstream backlog;
    size_t backlog_keys = 0;
    for (const auto& [vendor, keys] : unknown_by_vendor) {
        if (kVendorsAssertedClean.count(vendor))
            continue;
        backlog << ' ' << vendor << '(' << keys.size() << ')';
        backlog_keys += keys.size();
    }
    if (backlog_keys > 0)
        WARN("profiles carry " << backlog_keys << " key(s) no ConfigDef defines, in vendors this test only"
             " reports on:" << backlog.str());

    // A SECTION per vendor would re-walk every profile for each one, so the
    // assertions share this single pass and scope their message with a block.
    for (const std::string& vendor : kVendorsAssertedClean) {
        const auto unknown = unknown_by_vendor.find(vendor);
        std::ostringstream report;
        if (unknown != unknown_by_vendor.end())
            for (const auto& [key, count] : unknown->second)
                report << "\n  " << key << " (in " << count << " preset(s))";
        INFO(vendor << " presets set keys no ConfigDef defines, so they load as defaults:" << report.str());
        CHECK(unknown == unknown_by_vendor.end());
    }
}

// Bodies are laid out like the rest of the test suite rather than collapsed
// onto the brace line.
// clang-format off
TEST_CASE("Every PrintRegionConfig field is registered in a preset key list", "[Preset][Config]")
{
    check_keys_are_in_a_preset(PrintRegionConfig::defaults().keys(), "PrintRegionConfig");
}

TEST_CASE("Every PrintObjectConfig field is registered in a preset key list", "[Preset][Config]")
{
    check_keys_are_in_a_preset(PrintObjectConfig::defaults().keys(), "PrintObjectConfig");
}

TEST_CASE("FibreSeeker printer variants label as Plastic and Plastic+CF", "[Preset]")
{
    CHECK(printer_variant_display_name("FibreSeeker3 SK3", "0.4FFF") == "Plastic");
    CHECK(printer_variant_display_name("FibreSeeker3 SK3", "0.4CF") == "Plastic+CF");
    CHECK(printer_variant_display_name("FibreSeeker3 SK3", "0.4") == "0.4");
    CHECK(printer_variant_display_name("Bambu Lab X1 Carbon", "0.4") == "0.4");
    CHECK(printer_variant_display_name("Bambu Lab X1 Carbon", "0.4HF") == "0.4HF");
}
// clang-format on
