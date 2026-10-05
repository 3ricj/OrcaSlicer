// Minimal Catch2 stand-in for running the fiber unit tests with no CMake tree
// and no Catch2 install. Implements only what test_fiber_tail_release.cpp uses:
// TEST_CASE registration, REQUIRE/CHECK with abort-on-REQUIRE, and Approx with
// an explicit margin. Approx's default epsilon mirrors Catch's documented
// default so verdicts agree with the real harness.
#pragma once

#include <cmath>
#include <cstddef>
#include <exception>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace Catch {

class Approx
{
public:
    explicit Approx(double value) : m_value(value), m_margin(0.0), m_epsilon(1e-12) {}

    Approx& margin(double m) { m_margin = m; return *this; }
    Approx& epsilon(double e) { m_epsilon = e; return *this; }

    double value() const { return m_value; }
    double margin_value() const { return m_margin; }
    double epsilon_value() const { return m_epsilon; }

private:
    double m_value;
    double m_margin;
    double m_epsilon;
};

inline bool operator==(double lhs, const Approx& rhs)
{
    if (rhs.margin_value() > 0.0)
        return std::fabs(lhs - rhs.value()) <= rhs.margin_value();
    // Catch's default: epsilon * (1 + max(|lhs|,|rhs|)).
    const double scale = 1.0 + std::fmax(std::fabs(lhs), std::fabs(rhs.value()));
    return std::fabs(lhs - rhs.value()) <= rhs.epsilon_value() * scale;
}

inline bool operator==(const Approx& lhs, double rhs) { return rhs == lhs; }
inline bool operator!=(double lhs, const Approx& rhs) { return !(lhs == rhs); }

// ---- counters --------------------------------------------------------------

inline int& assertion_counter() { static int n = 0; return n; }
inline int& failure_counter()   { static int n = 0; return n; }

struct RequireFailed : std::exception
{
    const char* what() const noexcept override { return "REQUIRE failed"; }
};

// ---- registration / runner -------------------------------------------------

struct TestCase
{
    std::string name;
    std::string tags;
    void (*fn)();
};

inline std::vector<TestCase>& registry()
{
    static std::vector<TestCase> r;
    return r;
}

struct Registrar
{
    Registrar(const char* name, const char* tags, void (*fn)())
    {
        registry().push_back(TestCase{name, tags, fn});
    }
};

inline int run_all(const std::string& filter = std::string())
{
    int cases_run = 0, cases_failed = 0;
    for (const TestCase& tc : registry()) {
        if (!filter.empty() && tc.name.find(filter) == std::string::npos)
            continue;
        ++cases_run;
        const int before = failure_counter();
        std::cout << "case: " << tc.name << "\n";
        try {
            tc.fn();
        } catch (const RequireFailed&) {
            // already reported
        } catch (const std::exception& e) {
            ++failure_counter();
            std::cout << "  EXCEPTION: " << e.what() << "\n";
        }
        if (failure_counter() > before)
            ++cases_failed;
    }
    std::cout << (cases_failed ? "FAILED " : "PASSED ")
              << (cases_run - cases_failed) << "/" << cases_run << " cases, "
              << assertion_counter() << " assertions\n";
    return cases_failed ? 1 : 0;
}

} // namespace Catch

#define FS_CAT2(a, b) a##b
#define FS_CAT(a, b) FS_CAT2(a, b)

// One test per source line, so __LINE__ is a sufficient unique suffix and the
// registration and the definition cannot end up naming different functions.
#define TEST_CASE(name, tags)                                                                              \
    static void FS_CAT(fs_test_fn_, __LINE__)();                                                           \
    static ::Catch::Registrar FS_CAT(fs_test_reg_, __LINE__)(name, tags, &FS_CAT(fs_test_fn_, __LINE__));  \
    static void FS_CAT(fs_test_fn_, __LINE__)()

#define CHECK(expr)                                                                                        \
    do {                                                                                                   \
        ++::Catch::assertion_counter();                                                                    \
        if (!(expr)) {                                                                                     \
            ++::Catch::failure_counter();                                                                  \
            std::cout << "  CHECK failed: " << #expr << "  (" << __FILE__ << ":" << __LINE__ << ")\n";     \
        }                                                                                                  \
    } while (false)

#define CHECK_FALSE(expr) CHECK(!(expr))

#define REQUIRE(expr)                                                                                      \
    do {                                                                                                   \
        ++::Catch::assertion_counter();                                                                    \
        if (!(expr)) {                                                                                     \
            ++::Catch::failure_counter();                                                                  \
            std::cout << "  REQUIRE failed: " << #expr << "  (" << __FILE__ << ":" << __LINE__ << ")\n";   \
            throw ::Catch::RequireFailed{};                                                                \
        }                                                                                                  \
    } while (false)

#define REQUIRE_FALSE(expr) REQUIRE(!(expr))
