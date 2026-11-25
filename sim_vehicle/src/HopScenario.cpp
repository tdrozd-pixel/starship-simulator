#include "sim/vehicle/HopScenario.hpp"

#include <stdexcept>

#include "sim/core/Random.hpp"
#include "sim/vehicle/TelemetryLogger.hpp"

namespace sim::vehicle {

HopScenario buildHopScenario(const HopScenarioOptions& options) {
    if (const char* problem = options.tower.validate()) {
        throw std::invalid_argument(problem);
    }
    HopScenario scenario;
    scenario.downlinkQueue = std::make_unique<DownlinkQueue>();
    scenario.sim = std::make_unique<Simulation>(SimConfig{1000.0, options.seed});
    Simulation& sim = *scenario.sim;

    scenario.site = std::make_unique<env::LocalTangentFrame>(options.site.target);

    HopperState initial = HopperDynamics::stateOnPad(
        options.vehicle, Eigen::Vector3d(options.site.startEast, options.site.startNorth, 0.0),
        options.initialAttitude);
    initial.angularRate = options.initialAngularRate;
    HopperDynamics& vehicle = sim.emplaceSystem<HopperDynamics>(options.vehicle, initial);
    scenario.vehicle = &vehicle;

    scenario.weather = &sim.scheduler().emplace<env::WeatherModel>(
        "weather", options.seed, options.weather);

    ImuSpec imuSpec = options.imu;
    imuSpec.ideal = imuSpec.ideal || options.idealSensors;
    AltimeterSpec altSpec = options.altimeter;
    altSpec.ideal = altSpec.ideal || options.idealSensors;
    GimbalFeedbackSpec gimbalSpec = options.gimbalFeedback;
    gimbalSpec.ideal = gimbalSpec.ideal || options.idealSensors;
    EngineFeedbackSpec engineSpec = options.engineFeedback;
    engineSpec.ideal = engineSpec.ideal || options.idealSensors;
    GnssSpec gnssSpec = options.gnss;
    gnssSpec.ideal = gnssSpec.ideal || options.idealSensors;

    // Order matters: sensors, then flight software, then the tower, then the downlink, then logging.
    sim.scheduler().emplace<ImuModel>("imu", options.imuRateHz, vehicle, imuSpec,
                                      deriveSeed(options.seed, "imu"));
    sim.scheduler().emplace<RadarAltimeterModel>("radar_altimeter", options.altimeterRateHz, vehicle, altSpec,
                                                 deriveSeed(options.seed, "radar_altimeter"));
    const HopperParams& v = options.vehicle;
    const Eigen::Vector3d antenna(v.gnssAntennaPosition[0], v.gnssAntennaPosition[1], v.gnssAntennaPosition[2]);
    sim.scheduler().emplace<GnssModel>("gnss", options.gnssModelRateHz, vehicle, *scenario.site, antenna, gnssSpec,
                                       deriveSeed(options.seed, "gnss"),
                                       deriveSeed(options.seed, "gnss_constellation"));
    sim.scheduler().emplace<GimbalFeedbackModel>("gimbal_feedback", options.gimbalFeedbackRateHz, vehicle, gimbalSpec,
                                                 deriveSeed(options.seed, "gimbal_feedback"));
    sim.scheduler().emplace<EngineFeedbackModel>("engine_feedback", options.engineFeedbackRateHz, vehicle, engineSpec,
                                                 deriveSeed(options.seed, "engine_feedback"));

    // What the flight software is told about the vehicle and the mission (loaded
    // before flight, like a real mass properties table and target coordinates).
    const HopperMassProperties mp = hopperMassProperties(v);
    fsw::HopConfig hop = options.hop;
    hop.target = {options.site.target.latitude, options.site.target.longitude, options.site.target.height};
    hop.dryMass = v.dryMass;
    hop.initialPropellant = v.initialPropellant;
    hop.vacuumThrust = v.vacuumThrust();
    hop.nozzleExitArea = v.nozzleExitArea();
    hop.vacuumIsp = v.vacuumIsp;
    hop.rvacVacuumThrust = v.rvacVacuumThrust;
    hop.rvacVacuumIsp = v.rvacVacuumIsp;
    hop.minThrottle = v.minThrottle;
    hop.engineLeverArm = mp.cgHeight - v.gimbalHeight;
    hop.engineClusterRadius = v.engineClusterRadius;
    hop.engineAzimuth0 = v.engineAzimuth0;
    hop.gimbalLimit = v.gimbalLimit;
    hop.transverseInertiaDry = mp.transverseInertiaDry;
    hop.transverseInertiaPropellant = mp.transverseInertiaPropellant;
    hop.axialInertiaDry = mp.axialInertiaDry;
    hop.axialInertiaPropellant = mp.axialInertiaPropellant;
    for (int i = 0; i < 3; ++i) {
        hop.gnssAntennaPosition[i] = v.gnssAntennaPosition[i];
    }
    // The IMU model sits at the CG (TODO(phase 6, item 30): its own mounting point).
    hop.imuPosition[2] = mp.cgHeight;
    // Catch mission data derived from the tower and the Ship pins (target frame = sim ENU).
    const TowerParams& tw = options.tower;
    const TowerParams::SeatPoint seat = tw.seatPointEnu(v);
    hop.landingMode = options.landing;
    fsw::CatchConfig& cc = hop.catchConfig;
    cc.seat[0] = seat.east;
    cc.seat[1] = seat.north;
    cc.seat[2] = seat.baseAltitude;
    cc.approachDir[0] = -tw.approachSide[0];
    cc.approachDir[1] = -tw.approachSide[1];
    cc.pinHeight = v.catchPinHeight;
    cc.armTriggerHeight = tw.closeTriggerHeight;
    cc.windowLateral = tw.windowLateral;
    cc.windowAlong = tw.windowAlong;
    cc.windowSink = tw.windowSink;
    cc.windowHorizontalSpeed = tw.windowHorizontalSpeed;
    cc.windowTilt = tw.windowTilt;
    cc.windowHeading = tw.windowHeading;
    cc.towerPosition[0] = tw.position[0];
    cc.towerPosition[1] = tw.position[1];
    cc.mastHalfWidth = tw.mastHalfWidth;
    cc.mastHeight = tw.mastHeight;
    cc.hingeHalfSpacing = tw.hingeHalfSpacing;
    cc.hingeSetback = tw.hingeSetback;
    cc.armLength = tw.armLength;
    cc.armOpenAngle = tw.armOpenAngle;
    cc.armClosedAngle = tw.phi(tw.psiClosed(v.catchPinOffset));
    scenario.fsw = &sim.scheduler().emplace<FlightSoftwareModel>("flight_software", hop);
    scenario.fsw->setDownlinkQueue(scenario.downlinkQueue.get());
    scenario.tower = &sim.scheduler().emplace<TowerModel>("tower", 1000.0, vehicle, options.tower);
    if (options.telemetrySink != nullptr) {
        scenario.downlink = &sim.scheduler().emplace<DownlinkModel>(
            "downlink", options.downlinkRateHz, *scenario.downlinkQueue, *options.telemetrySink, options.downlink,
            deriveSeed(options.seed, "downlink"));
    }

    if (!options.logPath.empty()) {
        sim.scheduler().emplace<TelemetryLogger>("telemetry_logger", options.logRateHz, vehicle, options.logPath);
    }

    if (options.snapshotSink != nullptr) {
        scenario.snapshotPublisher = &sim.scheduler().emplace<TruthSnapshotPublisher>(
            "truth_snapshot", options.snapshotRateHz, vehicle, *options.snapshotSink);
        scenario.snapshotPublisher->setTower(scenario.tower);
    }

    sim.initialize();
    return scenario;
}

}  // namespace sim::vehicle
