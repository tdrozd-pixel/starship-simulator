// Real-time truth server for the vertical hop scenario.
//
//   sim_server                          http://localhost:8080/ (all interfaces)
//   sim_server --host 127.0.0.1         local only
//   sim_server --port 9000 --time-scale 0.25 --seed 42 --ideal-sensors
//   sim_server --web-root path/to/web
//
// On first run Windows may ask to allow the firewall for Private networks so
// other devices on the LAN can connect.

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <thread>

#include "sim/server/JsonMessages.hpp"
#include "sim/server/NetUtil.hpp"
#include "sim/server/SimRunner.hpp"
#include "sim/server/SnapshotRing.hpp"
#include "sim/server/TruthServer.hpp"

#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>
#endif

#ifndef SIM_DEFAULT_WEB_ROOT
#define SIM_DEFAULT_WEB_ROOT "web"
#endif

namespace {

std::atomic<bool> gSignalled{false};
static_assert(std::atomic<bool>::is_always_lock_free, "signal handler needs a lock-free flag");

std::atomic<bool> gMainDone{false};

extern "C" void onSignal(int) { gSignalled.store(true); }

#ifdef _WIN32
// std::signal covers only Ctrl-C reliably; console close and Ctrl-Break need this.
BOOL WINAPI onConsoleEvent(DWORD type) {
    switch (type) {
        case CTRL_C_EVENT:
        case CTRL_BREAK_EVENT:
            gSignalled.store(true);
            return TRUE;
        case CTRL_CLOSE_EVENT:
            gSignalled.store(true);
            // Windows kills the process shortly after this returns; give main time to shut down.
            for (int i = 0; i < 200 && !gMainDone.load(); ++i) {
                Sleep(10);
            }
            return TRUE;
        default:
            return FALSE;
    }
}
#endif

void printUsage() {
    std::puts(
        "usage: sim_server [--port N] [--host ADDR] [--web-root DIR] [--seed N]\n"
        "                  [--ideal-sensors] [--time-scale X] [--autostart]");
}

}  // namespace

int main(int argc, char** argv) {
    sim::server::SimRunnerConfig runnerConfig;
    runnerConfig.autostart = false;  // wait for a viewer unless --autostart
    sim::server::TruthServerConfig serverConfig;
    serverConfig.webRoot = SIM_DEFAULT_WEB_ROOT;

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            auto next = [&]() -> std::string {
                if (i + 1 >= argc) {
                    throw std::invalid_argument("missing value for " + arg);
                }
                return argv[++i];
            };
            if (arg == "--port") {
                serverConfig.port = std::stoi(next());
            } else if (arg == "--host") {
                serverConfig.host = next();
            } else if (arg == "--web-root") {
                serverConfig.webRoot = next();
            } else if (arg == "--seed") {
                runnerConfig.hop.seed = std::stoull(next());
            } else if (arg == "--ideal-sensors") {
                runnerConfig.hop.idealSensors = true;
            } else if (arg == "--time-scale") {
                runnerConfig.timeScale = std::stod(next());
            } else if (arg == "--autostart") {
                runnerConfig.autostart = true;
            } else if (arg == "--help" || arg == "-h") {
                printUsage();
                return EXIT_SUCCESS;
            } else {
                throw std::invalid_argument("unknown argument " + arg);
            }
        }
        if (!std::isfinite(runnerConfig.timeScale) || !(runnerConfig.timeScale > 0.0) ||
            runnerConfig.timeScale > 100.0) {
            throw std::invalid_argument("--time-scale must be finite and in (0, 100]");
        }
    } catch (const std::invalid_argument& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        printUsage();
        return EXIT_FAILURE;
    } catch (const std::out_of_range& e) {
        std::fprintf(stderr, "error: value out of range (%s)\n", e.what());
        printUsage();
        return EXIT_FAILURE;
    }
    serverConfig.timeScale = runnerConfig.timeScale;

    try {
        if (!std::filesystem::exists(std::filesystem::path(serverConfig.webRoot) / "index.html")) {
            std::fprintf(stderr, "warning: %s/index.html not found; only the WebSocket will be useful\n",
                         serverConfig.webRoot.c_str());
        }

        sim::server::SnapshotRing ring;
        sim::server::TruthServer* serverPtr = nullptr;  // set before the runner starts
        runnerConfig.clientPresent = [&serverPtr] { return serverPtr != nullptr && serverPtr->clientCount() > 0; };
        runnerConfig.onWaitingForClient = [] {
            std::puts("Waiting for a client to connect...");
            std::fflush(stdout);
        };
        sim::server::SimRunner runner(runnerConfig, ring);

        sim::server::HelloInfo hello;
        const sim::vehicle::HopperParams& vehicle = runnerConfig.hop.vehicle;
        hello.vehicleLength = vehicle.length;
        hello.vehicleRadius = vehicle.radius;

        sim::server::TruthServer server(
            serverConfig, ring, [&runner] { return runner.runId(); }, [&runner] { runner.requestRestart(); }, hello);

        serverPtr = &server;
        std::signal(SIGINT, onSignal);
        std::signal(SIGTERM, onSignal);
#ifdef _WIN32
        SetConsoleCtrlHandler(onConsoleEvent, TRUE);
#endif

        runner.start();
        server.start();

        const int port = server.boundPort();
        std::printf("Truth server: http://localhost:%d/\n", port);
        if (serverConfig.host == "0.0.0.0") {
            for (const std::string& ip : sim::server::localIPv4Addresses()) {
                std::printf("              http://%s:%d/\n", ip.c_str(), port);
            }
            std::puts("(If Windows asks, allow access on Private networks so other devices can connect.)");
        } else {
            std::printf("              http://%s:%d/\n", serverConfig.host.c_str(), port);
        }
        std::printf("Time scale %.3g, seed %llu, %s sensors. Ctrl-C to stop.\n", runnerConfig.timeScale,
                    static_cast<unsigned long long>(runnerConfig.hop.seed),
                    runnerConfig.hop.idealSensors ? "ideal" : "modeled");
        std::fflush(stdout);

        while (!gSignalled.load() && !runner.failed()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::puts("Shutting down...");
        runner.stop();
        server.stop();
        std::printf("Snapshots dropped (ring full): %llu, dropped for slow clients: %llu\n",
                    static_cast<unsigned long long>(runner.droppedSnapshots()),
                    static_cast<unsigned long long>(server.droppedForClients()));
        std::fflush(stdout);
        gMainDone = true;
        return runner.failed() ? EXIT_FAILURE : EXIT_SUCCESS;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return EXIT_FAILURE;
    }
}
