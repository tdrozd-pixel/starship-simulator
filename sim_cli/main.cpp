// Headless runner for the hop scenario (Starship V3 Ship, hop to Starbase Pad 2).
//
//   sim_cli                         real time
//   sim_cli --fast                  as fast as possible
//   sim_cli --time-scale 0.25       slow motion
//   sim_cli --ideal-sensors         perfect sensors ("truth mode")
//   sim_cli --seed 42 --log hop.csv

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>

#include "fsw/HopFlightComputer.hpp"
#include "sim/core/Pacer.hpp"
#include "sim/vehicle/HopScenario.hpp"

namespace {

void printUsage() {
    std::puts(
        "usage: sim_cli [--fast] [--time-scale X] [--max-time S] [--seed N]\n"
        "               [--ideal-sensors] [--log FILE.csv]");
}

}  // namespace

int main(int argc, char** argv) {
    sim::vehicle::HopScenarioOptions options;
    sim::PacerOptions pacer;
    pacer.maxSimTime = 120.0;

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            auto next = [&]() -> std::string {
                if (i + 1 >= argc) {
                    throw std::invalid_argument("missing value for " + arg);
                }
                return argv[++i];
            };
            if (arg == "--fast") {
                pacer.realtime = false;
            } else if (arg == "--time-scale") {
                pacer.timeScale = std::stod(next());
            } else if (arg == "--max-time") {
                pacer.maxSimTime = std::stod(next());
            } else if (arg == "--seed") {
                options.seed = std::stoull(next());
            } else if (arg == "--ideal-sensors") {
                options.idealSensors = true;
            } else if (arg == "--log") {
                options.logPath = next();
            } else if (arg == "--help" || arg == "-h") {
                printUsage();
                return EXIT_SUCCESS;
            } else {
                throw std::invalid_argument("unknown argument " + arg);
            }
        }

        auto scenario = sim::vehicle::buildHopScenario(options);
        std::printf("%s, target %s\n", options.vehicle.name, options.site.name);
        std::printf("Hop scenario: seed %llu, %s sensors, %s\n", static_cast<unsigned long long>(options.seed),
                    options.idealSensors ? "ideal" : "modeled",
                    pacer.realtime ? "real-time pacing" : "unpaced");

        const sim::PacerStats stats = sim::runPaced(*scenario.sim, pacer);

        const auto& vehicle = *scenario.vehicle;
        const auto& td = vehicle.touchdown();
        const double propUsed = options.vehicle.initialPropellant - vehicle.state().propellantMass;

        std::printf("\nFinal FSW mode      : %s\n", fsw::toString(scenario.fsw->computer().mode()));
        std::printf("Max altitude        : %.2f m\n", vehicle.maxAltitude());
        if (td.occurred) {
            std::printf("Touchdown           : t=%.2f s, %.3f m/s vertical, %.3f m/s horizontal\n", td.time,
                        td.verticalSpeed, td.horizontalSpeed);
            std::printf("Miss from target    : %.3f m (east %+.3f m, north %+.3f m)\n", std::hypot(td.east, td.north),
                        td.east, td.north);
        } else {
            std::printf("Touchdown           : none\n");
        }
        std::printf("Propellant used     : %.1f kg\n", propUsed);
        std::printf("Sim time / wall time: %.2f s / %.3f s (%.1fx real time)\n", stats.simTime, stats.wallTime,
                    stats.speedFactor());
        if (pacer.realtime) {
            std::printf("Max lag             : %.2f ms (%llu lag events)\n", stats.maxLag * 1e3,
                        static_cast<unsigned long long>(stats.lagEvents));
        }
        if (!options.logPath.empty()) {
            std::printf("Log written to      : %s\n", options.logPath.c_str());
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        printUsage();
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
