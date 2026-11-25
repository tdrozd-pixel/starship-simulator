#pragma once

#include <cmath>

#include "sim/vehicle/HopperParams.hpp"

namespace sim::vehicle {

// Catch tower with two chopstick arms (HYPOTHETICAL ESTIMATE, not a SpaceX survey). Every number
// is an estimate unless it says otherwise: the real Starbase tower geometry is not public in
// this detail, and the real Ship has no catch pins (the Booster catches on grid fin pins), so
// the Ship pins in HopperParams are a placeholder too. Public context: the Mechazilla tower is
// about 140 to 145 m tall with two arms that close around the Booster.
//
// Frames: the sim local ENU world (z up, origin at the pad center). The tower frame uses
//   a = approachSide  (unit vector from the tower toward the side the Ship comes from),
//   b = a rotated +90 deg about z,
// so the Ship flies along -a into the arms. Arm 0 sits on the +b side, arm 1 on -b. The flat
// approach axis must be axis-aligned for now (the attitude controller holds world yaw 0, so the
// pins stay on the east-west body axis); see validate(). TODO(phase 7, item 40): general heading.
//
// Arm angle psi is measured from fully open (psi = 0) toward closed. The arm direction measured
// from a, positive inward (toward the approach axis), is phi(psi) = -armOpenAngle + psi.
// Simplification: the arms do not move vertically (no catch height travel), documented.
struct TowerParams {
    // Mast: square footprint, vertical, centered on the tower position.
    double position[2] = {60.0, 0.0};         // [m] east, north, estimate (60 m east of the pad center)
    double approachSide[2] = {-1.0, 0.0};     // [] unit vector a, east, north; the Ship arrives from the west
    double mastHalfWidth = 6.0;               // [m] estimate (12 m square mast)
    double mastHeight = 145.0;                // [m] estimate (public figures: about 140 to 145 m)

    // Arm hinges: on the mast corners facing the approach side.
    double hingeSetback = 6.0;                // [m] along a from the tower center, estimate
    double hingeHalfSpacing = 8.0;            // [m] lateral (along b), either side, estimate

    // Arm body (a box with its long axis along the arm direction).
    double armLength = 16.0;                  // [m] from the hinge, estimate
    double armHalfWidth = 1.0;                // [m] estimate
    double armThickness = 1.5;                // [m] estimate
    double railHeight = 75.0;                 // [m] arm top surface above the origin plane, estimate

    // Arm drive: first order servo with a rate limit, psi_dot = clamp(gain * (cmd - psi), +/-rate).
    // Integrated with Euler at 1 kHz. Closing takes about 5.7 s (to 0.1 deg) with the 7 m pin offset.
    double armOpenAngle = 20.0 * kDegToRad;   // [rad] open arm direction, outward from a, estimate
    double servoGain = 2.0;                   // [1/s] estimate
    double servoRateLimit = 5.5 * kDegToRad;  // [rad/s] estimate

    // Cradle pad on each arm that the pin sits on.
    double cradleDistance = 12.0;             // [m] from the hinge along the arm to the pad center, estimate
    double cradleHalfLength = 1.5;            // [m] along the arm, estimate
    double cradleHalfWidth = 0.8;             // [m] across the arm, estimate

    // Tower state machine (TowerState: 0 Open, 1 Closing, 2 Closed, 3 Holding, 4 Opening).
    // Open -> Closing when the pin midpoint is over the seat point (horizontal radius), descending, and either
    // below a trigger height above the seat or predicted to reach the seat within a lead time (constant
    // deceleration to a stop at the seat: time = 2 * height / sink rate). Both are estimates; the lead time is
    // the closing time (about 5.7 s to the 0.1 deg tolerance) plus margin, so the arms are closed before
    // contact. The flight software is told the trigger height as mission data (CatchConfig::armTriggerHeight)
    // and keeps its profile slow enough to arrive after the arms are closed; it never reads the tower.
    double closeTriggerRadius = 6.0;          // [m] horizontal, estimate
    double closeTriggerHeight = 45.0;         // [m] pin midpoint above the seat, estimate
    double closeTriggerLeadTime = 8.0;        // [s] predicted time to the seat, estimate (closing time plus margin)
    double closeTriggerMinHeight = 0.0;       // [m] no trigger once the pins are at or below the seat, estimate
    double closeTriggerMaxHeight = 130.0;     // [m] no trigger above this, estimate
    double closeTriggerMinSink = 0.1;         // [m/s] must be descending at least this fast, estimate
    double closedTolerance = 0.1 * kDegToRad; // [rad] psi within this of closed counts as Closed (and open when psi < this)
    double zoneRadius = 10.0;                 // [m] horizontal zone; leaving it (or the height zone) re-opens, estimate
    double zoneHeightAbove = 135.0;           // [m] vertical zone above the seated pin height, estimate
    double zoneHeightBelow = 8.0;             // [m] vertical zone below the seated pin height, estimate
    double zoneExitTime = 1.0;                // [s] outside the zone this long -> Opening, estimate
    double closedTimeout = 45.0;              // [s] Closed without a catch -> Opening, estimate (nominal Closed to contact is about 8 s)

    // Catch window, evaluated for both pins at first contact. All estimates.
    double windowClosure = 0.98;              // [] arm closure fraction psi / psiClosed
    double windowLateral = 0.7;               // [m] pin lateral offset from the pad center
    double windowAlong = 1.5;                 // [m] pin offset along the arm from the pad center
    double windowSink = 1.5;                  // [m/s]
    double windowHorizontalSpeed = 0.5;       // [m/s]
    double windowTilt = 3.0 * kDegToRad;      // [rad]
    double windowHeading = 3.0 * kDegToRad;   // [rad]
    double windowAngularRate = 2.0 * kDegToRad;  // [rad/s]

    // Hold and halt (placeholder, no structural model).
    double haltAfterEnginesOff = 2.0;         // [s] held with no engine running -> sim halts, estimate
    double haltAfterHold = 20.0;              // [s] held this long -> sim halts regardless, estimate

    // Pure geometry helpers. All in the tower frame unless stated: along = distance along a from
    // the tower center, lateral = along b. Arm 1 mirrors arm 0 (negate lateral).
    struct TowerPoint {
        double along;    // [m]
        double lateral;  // [m]
    };

    // Arm direction from a, positive inward [rad], for arm angle psi.
    double phi(double psi) const { return -armOpenAngle + psi; }

    // Arm angle at which the cradle centers sit exactly pinOffset from the approach axis:
    // phi_c = asin((hingeHalfSpacing - pinOffset) / cradleDistance).
    double psiClosed(double pinOffset) const {
        return armOpenAngle + std::asin((hingeHalfSpacing - pinOffset) / cradleDistance);
    }

    // Cradle center of arm 0 (+b side) for arm angle psi.
    TowerPoint padCenterTower(double psi) const {
        const double p = phi(psi);
        return {hingeSetback + cradleDistance * std::cos(p), hingeHalfSpacing - cradleDistance * std::sin(p)};
    }

    // Gap between the hull cylinder (radius hullRadius about the approach axis) and the closed arms: the
    // smallest lateral distance of any corner of the arm box from the axis, minus the hull radius. Positive
    // means the Ship can descend between the closed arms; the tightest point is the inner corner of the tip.
    double closedArmClearance(double pinOffset, double hullRadius) const {
        const double p = phi(psiClosed(pinOffset));
        double nearest = 1e9;
        for (const double s : {0.0, armLength}) {
            for (const double side : {-1.0, 1.0}) {
                const double lateral = hingeHalfSpacing - s * std::sin(p) + side * armHalfWidth * std::cos(p);
                nearest = lateral < nearest ? lateral : nearest;
            }
        }
        return nearest - hullRadius;
    }

    // Seated pose: the pin midpoint on the approach axis with the arms closed, and the base
    // altitude of the Ship when the pins sit on the cradles (pin center = rail top + pin radius).
    struct SeatPoint {
        double east;          // [m]
        double north;         // [m]
        double baseAltitude;  // [m] seated base above the origin plane
        double pinHeight;     // [m] seated pin center above the origin plane
    };
    SeatPoint seatPointEnu(const HopperParams& vehicle) const {
        const TowerPoint c = padCenterTower(psiClosed(vehicle.catchPinOffset));
        const double pinZ = railHeight + vehicle.catchPinRadius;
        return {position[0] + c.along * approachSide[0], position[1] + c.along * approachSide[1],
                pinZ - vehicle.catchPinHeight, pinZ};
    }

    // Returns null if the parameters are usable, else a message. The approach axis must be
    // a unit vector along east or north (see the note above).
    const char* validate() const {
        const bool eastWest = approachSide[1] == 0.0 && std::abs(approachSide[0]) == 1.0;
        const bool northSouth = approachSide[0] == 0.0 && std::abs(approachSide[1]) == 1.0;
        if (!eastWest && !northSouth) {
            return "TowerParams: approachSide must be an axis-aligned unit vector (general heading is TODO)";
        }
        if (hingeHalfSpacing <= 0.0 || cradleDistance <= 0.0 || cradleDistance > armLength) {
            return "TowerParams: bad hinge or cradle geometry";
        }
        return nullptr;
    }
};

}  // namespace sim::vehicle
