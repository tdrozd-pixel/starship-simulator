#include "sim/vehicle/HopScenario.hpp"

#include "sim/core/Random.hpp"
#include "sim/vehicle/TelemetryLogger.hpp"

namespace sim::vehicle {

HopScenario buildHopScenario(const HopScenarioOptions& options) {
    HopScenario scenario;
    scenario.sim = std::make_unique<Simulation>(SimConfig{1000.0, options.seed});
    Simulation& sim = *scenario.sim;

    HopperState initial;
    initial.propellantMass = options.vehicle.initialPropellant;
    HopperDynamics& vehicle = sim.emplaceSystem<HopperDynamics>(options.vehicle, initial);
    scenario.vehicle = &vehicle;

    ImuSpec imuSpec = options.imu;
    imuSpec.ideal = imuSpec.ideal || options.idealSensors;
    AltimeterSpec altSpec = options.altimeter;
    altSpec.ideal = altSpec.ideal || options.idealSensors;

    // Order matters: sensors, then flight software, then logging.
    sim.scheduler().emplace<ImuModel>("imu", options.imuRateHz, vehicle, imuSpec,
                                      deriveSeed(options.seed, "imu"));
    sim.scheduler().emplace<RadarAltimeterModel>("radar_altimeter", options.altimeterRateHz, vehicle, altSpec,
                                                 deriveSeed(options.seed, "radar_altimeter"));

    fsw::HopConfig hop = options.hop;
    hop.dryMass = options.vehicle.dryMass;
    hop.initialPropellant = options.vehicle.initialPropellant;
    hop.maxThrust = options.vehicle.maxThrust;
    hop.ispSeaLevel = options.vehicle.ispSeaLevel;
    hop.minThrottle = options.vehicle.minThrottle;
    scenario.fsw = &sim.scheduler().emplace<FlightSoftwareModel>("flight_software", hop);

    if (!options.logPath.empty()) {
        sim.scheduler().emplace<TelemetryLogger>("telemetry_logger", options.logRateHz, vehicle, options.logPath);
    }

    sim.initialize();
    return scenario;
}

}  // namespace sim::vehicle
