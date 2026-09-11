#pragma once

#include <cmath>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace ir_sim::test {

struct TestCase {
    std::string name;
    std::function<void()> func;
};

inline std::vector<TestCase>& get_registry() {
    static std::vector<TestCase> registry;
    return registry;
}

struct AutoRegister {
    AutoRegister(std::string_view name, std::function<void()> func) {
        get_registry().push_back({std::string(name), std::move(func)});
    }
};

class TestFailure : public std::runtime_error {
public:
    explicit TestFailure(const std::string& msg) : std::runtime_error(msg) {}
};

inline void require_impl(bool condition, const char* expr, const char* file, int line) {
    if (!condition) {
        throw TestFailure(std::string("Assertion failed: (") + expr + ") at " + file + ":" + std::to_string(line));
    }
}

inline void require_near_impl(double a, double b, double tolerance, const char* expr_a, const char* expr_b, const char* file, int line) {
    const double diff = std::abs(a - b);
    if (diff > tolerance) {
        throw TestFailure(std::string("Near assertion failed: |") + expr_a + " - " + expr_b + "| = " +
                          std::to_string(diff) + " > " + std::to_string(tolerance) +
                          " (" + std::to_string(a) + " vs " + std::to_string(b) + ") at " +
                          file + ":" + std::to_string(line));
    }
}

inline int run_all_tests() {
    int passed = 0;
    int failed = 0;
    const auto& tests = get_registry();

    std::cout << "\n======================================================\n";
    std::cout << " Running " << tests.size() << " Test Suites\n";
    std::cout << "======================================================\n\n";

    for (const auto& test : tests) {
        std::cout << "[ RUN      ] " << test.name << "\n";
        try {
            test.func();
            std::cout << "[       OK ] " << test.name << "\n";
            passed++;
        } catch (const std::exception& e) {
            std::cout << "[  FAILED  ] " << test.name << "\n";
            std::cout << "             " << e.what() << "\n";
            failed++;
        }
    }

    std::cout << "\n------------------------------------------------------\n";
    std::cout << " Total: " << tests.size() << " | Passed: " << passed << " | Failed: " << failed << "\n";
    std::cout << "------------------------------------------------------\n\n";

    return (failed == 0) ? 0 : 1;
}

} // namespace ir_sim::test

#define TEST_CASE(name) \
    static void test_func_##name(); \
    static const ::ir_sim::test::AutoRegister reg_##name(#name, test_func_##name); \
    static void test_func_##name()

#define REQUIRE(expr) ::ir_sim::test::require_impl((expr), #expr, __FILE__, __LINE__)
#define REQUIRE_NEAR(a, b, tol) ::ir_sim::test::require_near_impl((a), (b), (tol), #a, #b, __FILE__, __LINE__)
