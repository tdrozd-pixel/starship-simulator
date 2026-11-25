#pragma once

#include <cmath>
#include <numbers>

#include "sim/env/Geodesy.hpp"

namespace sim::vehicle {

constexpr double kDegToRad = std::numbers::pi / 180.0;
constexpr double kTonneForce = 1000.0 * 9.80665;  // [N] per metric tonne-force
constexpr double kSeaLevelPressure = 101325.0;     // [Pa] US76, defines the rated sea-level thrust

// Engines on the Ship: index 0..2 are the sea-level Raptors (inner ring, gimbaled, the only
// ones that fly the hop), index 3..5 are the Raptor Vacuums (outer ring, fixed, no gimbal).
constexpr int kSeaLevelEngineCount = 3;
constexpr int kRvacCount = 3;
constexpr int kEngineCount = kSeaLevelEngineCount + kRvacCount;
constexpr int kFirstRvac = kSeaLevelEngineCount;

// Starship V3 Ship in a hop configuration (in the spirit of the SN5/SN6 150 m hops),
// flying on its three sea-level Raptor 3 engines with a partial propellant load.
//
// Sources (as of Nov 2026). SpaceX does not publish most of these; anything not
// from a public source is marked "estimate" and should be updated as data appears.
//  [R3]  Raptor 3 performance card, SpaceX, 3 Aug 2024, as reported by e.g.
//        https://www.teslaoracle.com/2024/08/08/raptor-3-starship-engine-is-lighter-less-complicated-but-more-powerful-and-reusable/
//        280 tf sea-level thrust, 350 s Isp, 1525 kg. SpaceX later clarified the
//        350 s is the VACUUM Isp of the sea-level engine. Some pages list 250 tf instead.
//  [WIKI] https://en.wikipedia.org/wiki/SpaceX_Starship_(spacecraft) and
//         https://en.wikipedia.org/wiki/SpaceX_Raptor (9 m diameter, 40-100% throttle).
//  [V3]  https://satnews.com/2026/05/14/spacex-debuts-starship-v3-redefining-heavy-lift-launch-capability/
//        and https://newspaceeconomy.ca/2026/04/16/detailed-review-of-starship-v3/
//        (Ship about 52 m, 1600 t propellant capacity).
//
// The three Raptor Vacuum engines (RVac) are modeled with ESTIMATED numbers (SpaceX does not
// publish them). Their mass is in dryMass (not added again). The hop never lights them: a
// vacuum nozzle at sea level suffers flow separation, modeled as a start inhibit above an
// ambient pressure threshold.
// Not modeled yet: flaps, RCS, legs.
struct HopperParams {
    const char* name = "Starship V3 Ship, hop configuration";

    // Geometry. Body frame origin for these heights is the base (aft skirt) on the centerline.
    double length = 52.0;  // [m] [V3] approximate
    double radius = 4.5;   // [m] [WIKI] 9 m diameter

    // Mass properties.
    // TODO(phase 1, item 8): live CG and inertia from tank geometry and fill levels.
    double dryMass = 150000.0;  // [kg] estimate: public figures range ~100 t (Musk, 2021)
                                // to ~166 t (independent bottom-up V3 estimate)
    double propellantCapacity = 1600000.0;  // [kg] [V3]
    double initialPropellant = 450000.0;    // [kg] hop load, chosen so all three engines at
                                            // minimum throttle weigh less than the vehicle (derived)
    double dryCgHeight = 22.0;              // [m] above base, estimate (engines low, nose and flaps high)
    double propellantCgHeight = 10.0;       // [m] above base, estimate (partial load in the aft tanks)
    double propellantColumnLength = 12.0;   // [m] estimate, for the propellant inertia

    // Raptor 3 (sea level), per engine.
    double seaLevelThrust = 280.0 * kTonneForce;  // [N] [R3] at full throttle, sea level
    double vacuumIsp = 350.0;                     // [s] [R3] (vacuum Isp of the SL engine)
    double nozzleExitDiameter = 1.3;              // [m] estimate (public photos)
    double minThrottle = 0.40;                    // fraction of rated flow [WIKI]
    double ignitionDelay = 1.0;                   // [s] estimate: command to thrust, placeholder for
                                                  // the start sequence. TODO(phase 2, item 9)
    double throttleTimeConstant = 0.15;           // [s] estimate
    double throttleRateLimit = 1.0;               // [1/s] estimate

    // Engine layout: the three sea-level engines form the inner ring.
    double engineClusterRadius = 1.55;  // [m] estimate, centerline to engine axis
    double engineAzimuth0 = 90.0 * kDegToRad;  // [rad] estimate, from body +x toward +y
    double gimbalHeight = 3.5;          // [m] above base, estimate (gimbal pivot at the engine top)

    // Raptor Vacuum (RVac), per engine, outer ring, fixed (no gimbal). All values are estimates.
    double rvacVacuumThrust = 250.0 * kTonneForce;  // [N] estimate: ~250 tf vacuum thrust at full flow
    double rvacVacuumIsp = 380.0;                   // [s] estimate (public figures ~380 s)
    double rvacNozzleExitDiameter = 2.4;            // [m] estimate (public photos)
    double rvacMinThrottle = 0.40;                  // fraction of rated flow, estimate (as the SL engine)
    double rvacIgnitionDelay = 1.0;                 // [s] estimate: same structure as the SL engine
    double rvacThrottleTimeConstant = 0.15;         // [s] estimate
    double rvacThrottleRateLimit = 1.0;             // [1/s] estimate
    // Flow separation / start inhibit: the engine refuses to run (and shuts down) while the
    // ambient pressure is above this. Estimate from the Summerfield criterion: separation when
    // nozzle exit pressure < ~0.4 * ambient; an exit pressure of ~5 kPa gives ~12 kPa (~15 km).
    double rvacMaxAmbientPressure = 12000.0;        // [Pa] estimate
    double rvacClusterRadius = 3.6;                 // [m] estimate, outer ring, centerline to engine axis
    // Offset 60 deg from the sea-level engines so the six form a hexagonal-like pattern.
    double rvacAzimuth0 = 150.0 * kDegToRad;        // [rad] estimate, from body +x toward +y

    // Thrust vector control, per sea-level engine.
    double gimbalLimit = 15.0 * kDegToRad;      // [rad] estimate (not public)
    double gimbalRateLimit = 20.0 * kDegToRad;  // [rad/s] estimate
    double gimbalTimeConstant = 0.05;           // [s] estimate

    // Aerodynamics (placeholder until the tables of build order item 17).
    double axialDragCoefficient = 0.8;      // on the base area, estimate
    double crossflowDragCoefficient = 0.7;  // on length x diameter, estimate (supercritical cylinder)
    double centerOfPressureHeight = 26.0;   // [m] above base, estimate (mid-length, flaps ignored)

    // Sensor mounting (from base, body frame).
    double gnssAntennaPosition[3] = {0.0, 4.5, 46.0};  // [m] estimate: on the skin near the nose
    // The IMU is modeled at the CG for now. TODO(phase 6, item 30): IMU mounting point.

    // Catch pins (placeholder for the tower catch hardware). The real Ship has no catch pins
    // (the Booster catches on grid fin pins); two pins on the Ship are a HYPOTHETICAL ESTIMATE
    // so the tower arms have something to engage. Pin 0 sits on body +y, pin 1 on body -y.
    double catchPinHeight = 30.0;   // [m] above the base, estimate (hypothetical, near the CG-to-nose region)
    // Stub length: the pins stand 2.5 m clear of the 4.5 m skin so the closed arms (inner edge at the arm tips
    // about 5.7 m from the axis, see TowerParams) clear the hull by more than 1 m while the Ship descends
    // between them. A smaller offset made the closed arm tips intersect the hull (about 1.5 m at 5.0 m).
    double catchPinOffset = 7.0;    // [m] lateral from the vehicle axis, estimate (hypothetical stub length)
    double catchPinRadius = 0.25;   // [m] estimate (hypothetical pin radius)

    // Derived.
    double nozzleExitArea() const { return std::numbers::pi * 0.25 * nozzleExitDiameter * nozzleExitDiameter; }
    double rvacNozzleExitArea() const {
        return std::numbers::pi * 0.25 * rvacNozzleExitDiameter * rvacNozzleExitDiameter;
    }
    // Thrust at full rated flow in vacuum: F_vac = F_sl + p_sl * A_exit.
    double vacuumThrust() const { return seaLevelThrust + kSeaLevelPressure * nozzleExitArea(); }
    double cgHeight() const {
        const double m = dryMass + initialPropellant;
        return (dryMass * dryCgHeight + initialPropellant * propellantCgHeight) / m;
    }
};

// Where the hop happens. The sim ENU origin is the landing target.
//  [LL2] Launch Library 2, pad 235, "Orbital Launch Pad 2" / Orbital Launch Mount B,
//        https://ll.thespacedevs.com/2.3.0/pads/235/ (third-party, not a SpaceX survey).
struct LaunchSite {
    const char* name = "Starbase Pad 2 (landing target)";
    // Placeholder for the tower catch point: the pad center, at ground level.
    env::Geodetic target{25.996603 * kDegToRad, -97.158251 * kDegToRad,
                         // Ellipsoidal height: about 4 m above mean sea level (estimate)
                         // plus a geoid undulation of about -26 m (EGM2008, estimate).
                         -22.0};
    // Liftoff point, east/north of the target [m]. Illustrative placeholder, not a real stand.
    double startEast = -30.0;
    double startNorth = -20.0;
};

}  // namespace sim::vehicle
