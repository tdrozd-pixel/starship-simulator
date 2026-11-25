// Headless runner for the hop scenario (Starship V3 Ship, hop to Starbase Pad 2).
//
//   sim_cli                         real time
//   sim_cli --fast                  as fast as possible
//   sim_cli --time-scale 0.25       slow motion
//   sim_cli --ideal-sensors         perfect sensors ("truth mode")
//   sim_cli --landing pad           land on the pad center (default: catch with the tower arms)
//   sim_cli --no-gust               remove the wind gust (a 100 m/s eastward one-cosine gust at t = 8 s, 1 s long)
//   sim_cli --wind-speed 12 --wind-from 0   steady wind, 12 m/s at the ground (1.6x at 1 km) from the north; default: no mean wind
//   sim_cli --seed 42 --log hop.csv

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <vector>
#include <numbers>

#include "fsw/HopFlightComputer.hpp"
#include "sim/core/Pacer.hpp"
#include "sim/vehicle/HopScenario.hpp"

namespace {

void printUsage() {
    std::puts(
        "usage: sim_cli [--fast] [--time-scale X] [--max-time S] [--seed N]\n"
        "               [--ideal-sensors] [--landing pad|catch] [--no-gust]\n"
        "               [--wind-speed M_PER_S] [--wind-from DEG] [--log FILE.csv]");
}


// Landing sequence timing, computed from the run (reporting only). The clocks start the first time on the descent
// (flight software mode Descent or later) that the base is at or below the threshold and stop when the flight
// software enters Caught:
//   T1 = from 200 ft (60.96 m) above the ground, T2 = from 200 ft above the catch seat.
struct TimingProbe {
    static constexpr double kHeightFeet200 = 60.96;  // [m] 200 ft
    double seatBaseAltitude = 0.0;                   // [m]
    double t1Start = -1.0;                           // [s] negative: not yet
    double t2Start = -1.0;                           // [s]
    double caughtTime = -1.0;                        // [s]

    void sample(const sim::vehicle::HopScenario& sc) {
        const int mode = static_cast<int>(sc.fsw->computer().mode());
        if (mode < static_cast<int>(fsw::HopMode::Descent)) {
            return;
        }
        const double t = sc.sim->time();
        const double altitude = sc.vehicle->basePosition().z();
        if (t1Start < 0.0 && altitude <= kHeightFeet200) {
            t1Start = t;
        }
        if (t2Start < 0.0 && altitude <= seatBaseAltitude + kHeightFeet200) {
            t2Start = t;
        }
        if (caughtTime < 0.0 && sc.fsw->computer().mode() == fsw::HopMode::Caught) {
            caughtTime = t;
        }
    }
};

}  // namespace

int main(int argc, char** argv) {
    sim::vehicle::HopScenarioOptions options;
    sim::PacerOptions pacer;
    pacer.maxSimTime = 200.0;
    double windSpeed = -1.0;  // [m/s] ground wind speed, negative: the default profile
    double windFrom = 270.0;  // [deg] meteorological direction the wind comes from (0 north, 90 east)

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
            } else if (arg == "--landing") {
                const std::string mode = next();
                if (mode == "pad") {
                    options.landing = fsw::LandingMode::Pad;
                } else if (mode == "catch") {
                    options.landing = fsw::LandingMode::Catch;
                } else {
                    throw std::invalid_argument("--landing must be pad or catch");
                }
            } else if (arg == "--no-gust") {
                options.weather.gusts.clear();
            } else if (arg == "--wind-speed") {
                windSpeed = std::stod(next());
            } else if (arg == "--wind-from") {
                windFrom = std::stod(next());
            } else if (arg == "--log") {
                options.logPath = next();
            } else if (arg == "--help" || arg == "-h") {
                printUsage();
                return EXIT_SUCCESS;
            } else {
                throw std::invalid_argument("unknown argument " + arg);
            }
        }

        if (windSpeed >= 0.0) {
            // Steady wind profile: windSpeed at the ground, 1.6x at 1 km, 3x at 10 km (the default has no mean wind).
            const double pi = std::numbers::pi;
            const double k = windSpeed / 5.0;
            const Eigen::Vector3d dir(-std::sin(windFrom * pi / 180.0), -std::cos(windFrom * pi / 180.0), 0.0);
            options.weather.profile = sim::env::WindProfile(std::vector<sim::env::WindProfilePoint>{
                {0.0, 5.0 * k * dir}, {1000.0, 8.0 * k * dir}, {10000.0, 15.0 * k * dir}});
        }
        auto scenario = sim::vehicle::buildHopScenario(options);
        std::printf("%s, target %s\n", options.vehicle.name, options.site.name);
        std::printf("Hop scenario: seed %llu, %s sensors, %s\n", static_cast<unsigned long long>(options.seed),
                    options.idealSensors ? "ideal" : "modeled",
                    pacer.realtime ? "real-time pacing" : "unpaced");
        std::printf("Landing mode: %s, wind gust %s\n", options.landing == fsw::LandingMode::Catch ? "catch" : "pad",
                    options.weather.gusts.empty() ? "off" : "on");

        TimingProbe timing;
        timing.seatBaseAltitude = scenario.tower->seat().baseAltitude;
        auto observe = [&]() {
            timing.sample(scenario);
            return false;
        };
        const sim::PacerStats stats = sim::runPaced(*scenario.sim, pacer, observe);

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
        if (options.landing == fsw::LandingMode::Catch) {
            static const char* const kTowerState[] = {"Open", "Closing", "Closed", "Holding", "Opening"};
            static const char* const kResult[] = {"None", "Caught", "ArmStrike", "TowerStrike"};
            const auto& tower = *scenario.tower;
            std::printf("Catch result        : %s (tower %s, arm angles %.1f / %.1f deg, FSW %s)\n",
                        kResult[static_cast<int>(tower.result())], kTowerState[static_cast<int>(tower.state())],
                        tower.armAngle(0) * 180.0 / 3.14159265358979323846,
                        tower.armAngle(1) * 180.0 / 3.14159265358979323846,
                        scenario.fsw->computer().catchAborted() ? "aborted to pad" : "no abort");
        }
        if (options.landing == fsw::LandingMode::Catch) {
            if (timing.caughtTime >= 0.0 && timing.t1Start >= 0.0 && timing.t2Start >= 0.0) {
                std::printf("Landing time T1     : %.2f s (200 ft above the ground to Caught)\n", timing.caughtTime - timing.t1Start);
                std::printf("Landing time T2     : %.2f s (200 ft above the catch seat to Caught)\n", timing.caughtTime - timing.t2Start);
            } else {
                std::printf("Landing time T1/T2  : not caught\n");
            }
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
