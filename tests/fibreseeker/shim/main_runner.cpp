// Minimal Catch2 driver for the dependency-free Fiber/ unit suites. The real
// build uses the vendored Catch2 main; this exists so the shipped code can be
// compiled and RUN in a worktree with no CMake tree and no Boost.
#include <catch2/catch_all.hpp>
#include <string>

int main(int argc, char** argv)
{
    std::string filter = argc > 1 ? argv[1] : std::string();
    return Catch::run_all(filter);
}
