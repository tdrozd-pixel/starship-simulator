#pragma once

// Minimal test harness so the project has no test-framework dependency yet.
// Swap for Catch2 or GoogleTest whenever it starts to feel limiting.

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace test {

struct TestCase {
    std::string name;
    std::function<void()> fn;
};

std::vector<TestCase>& registry();
void reportFailure(const char* file, int line, const std::string& message);

struct Registrar {
    Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

}  // namespace test

#define TEST_CASE(name)                                          \
    static void name();                                          \
    static const ::test::Registrar name##_registrar(#name, name); \
    static void name()

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) ::test::reportFailure(__FILE__, __LINE__, #cond);   \
    } while (0)

#define CHECK_NEAR(actual, expected, tol)                                                          \
    do {                                                                                           \
        const double a_ = (actual);                                                                \
        const double e_ = (expected);                                                              \
        if (!(std::abs(a_ - e_) <= (tol))) {                                                       \
            ::test::reportFailure(__FILE__, __LINE__,                                              \
                                  std::string(#actual " = ") + std::to_string(a_) + ", expected " + \
                                      std::to_string(e_) + " +/- " + std::to_string(tol));         \
        }                                                                                          \
    } while (0)

#define CHECK_THROWS(expr)                                                                   \
    do {                                                                                     \
        bool threw_ = false;                                                                 \
        try {                                                                                \
            (void)(expr);                                                                    \
        } catch (...) {                                                                      \
            threw_ = true;                                                                   \
        }                                                                                    \
        if (!threw_) ::test::reportFailure(__FILE__, __LINE__, "expected throw: " #expr);    \
    } while (0)
