#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>

#include "TestFramework.hpp"

namespace test {

namespace {
int g_failures = 0;
}

std::vector<TestCase>& registry() {
    static std::vector<TestCase> cases;
    return cases;
}

void reportFailure(const char* file, int line, const std::string& message) {
    ++g_failures;
    std::printf("    FAIL %s:%d: %s\n", file, line, message.c_str());
}

}  // namespace test

int main(int argc, char** argv) {
    const std::string filter = argc > 1 ? argv[1] : "";
    int run = 0;
    int failedCases = 0;
    for (const auto& tc : test::registry()) {
        if (!filter.empty() && tc.name.find(filter) == std::string::npos) {
            continue;
        }
        ++run;
        const int before = test::g_failures;
        std::printf("[ RUN  ] %s\n", tc.name.c_str());
        try {
            tc.fn();
        } catch (const std::exception& e) {
            test::reportFailure(__FILE__, __LINE__, std::string("unexpected exception: ") + e.what());
        }
        const bool ok = test::g_failures == before;
        if (!ok) {
            ++failedCases;
        }
        std::printf("[ %s ] %s\n", ok ? " OK " : "FAIL", tc.name.c_str());
    }
    std::printf("\n%d test(s) run, %d failed\n", run, failedCases);
    return failedCases == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
