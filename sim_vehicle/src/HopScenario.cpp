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
    initial.attitude = options.initialAttitude;
    initial.angularRate = options.initialAngularRate;
    HopperDynamics& vehicle = sim.emplaceSystem<HopperDynamics>(options.vehicle, initial);
    scenario.vehicle = &vehicle;

    scenario.weather = &sim.scheduler().emplace<env::WeatherModel>(
        "weather", options.seed, options.weather);

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
    hop.vehicleRadius = options.vehicle.radius;
    hop.vehicleLength = options.vehicle.length;
    hop.engineLeverArm = options.vehicle.engineLeverArm;
    hop.gimbalLimit = options.vehicle.gimbalLimit;
    scenario.fsw = &sim.scheduler().emplace<FlightSoftwareModel>("flight_software", hop);

    if (!options.logPath.empty()) {
        sim.scheduler().emplace<TelemetryLogger>("telemetry_logger", options.logRateHz, vehicle, options.logPath);
    }

    if (options.snapshotSink != nullptr) {
        scenario.snapshotPublisher = &sim.scheduler().emplace<TruthSnapshotPublisher>(
            "truth_snapshot", options.snapshotRateHz, vehicle, *options.snapshotSink);
    }

    sim.initialize();
    return scenario;
}

}  // namespace sim::vehicle
