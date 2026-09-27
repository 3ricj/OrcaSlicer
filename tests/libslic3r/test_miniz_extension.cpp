#include <catch2/catch_all.hpp>

#include "libslic3r/miniz_extension.hpp"

#include "test_utils.hpp"

#include <boost/filesystem.hpp>

#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
namespace fs = boost::filesystem;

namespace {

void write_zip(const fs::path &zip_file, const std::vector<std::pair<std::string, std::string>> &entries)
{
    mz_zip_archive zip;
    mz_zip_zero_struct(&zip);
    REQUIRE(open_zip_writer(&zip, zip_file.string()));
    for (const auto &[name, content] : entries)
        REQUIRE(mz_zip_writer_add_mem(&zip, name.c_str(), content.data(), content.size(), MZ_DEFAULT_COMPRESSION));
    REQUIRE(mz_zip_writer_finalize_archive(&zip));
    REQUIRE(close_zip_writer(&zip));
}

std::string read_file(const fs::path &file)
{
    std::ifstream in(file.string(), std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

} // namespace

TEST_CASE("Confined extraction writes a well-formed archive under the target directory", "[MinizExtension]")
{
    ScopedTemporaryDir tmp;
    const fs::path     zip_file = tmp.path() / "bundle.zip";
    const fs::path     target   = tmp.path() / "cache";
    fs::create_directories(target);
    write_zip(zip_file, {{"vendor/", ""}, {"vendor/machine/", ""}, {"vendor.json", "{\"a\":1}"}, {"vendor/machine/printer.json", "{\"b\":2}"}});

    REQUIRE(extract_archive_confined(zip_file.string(), target.string()));
    CHECK(fs::is_directory(target / "vendor"));
    CHECK(read_file(target / "vendor.json") == "{\"a\":1}");
    CHECK(read_file(target / "vendor" / "machine" / "printer.json") == "{\"b\":2}");
}

TEST_CASE("Confined extraction rejects an archive with an entry outside the target directory", "[MinizExtension]")
{
    ScopedTemporaryDir tmp;
    const fs::path     zip_file = tmp.path() / "bundle.zip";
    const fs::path     target   = tmp.path() / "cache";
    fs::create_directories(target);

    const std::string escaping_entry = GENERATE(std::string("../escape.txt"), std::string("..\\escape.txt"),
                                                std::string("sub/../../escape.txt"), std::string("C:/escape.txt"));
    // The normal entry comes first so a per-entry check would already have written it.
    write_zip(zip_file, {{"normal.json", "{}"}, {escaping_entry, "escaped"}});

    CAPTURE(escaping_entry);
    CHECK_FALSE(extract_archive_confined(zip_file.string(), target.string()));
    CHECK_FALSE(fs::exists(tmp.path() / "escape.txt"));
    CHECK(fs::is_empty(target));
}
