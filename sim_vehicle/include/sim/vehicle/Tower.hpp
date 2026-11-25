#pragma once

#include <cstdint>
#include <string>

#include "sim/core/Model.hpp"
#include "sim/vehicle/HopperDynamics.hpp"
#include "sim/vehicle/TowerParams.hpp"

namespace sim::vehicle {

// Values are the wire numbers of the truth stream (towerState, catchResult).
enum class TowerState : uint32_t { Open = 0, Closing = 1, Closed = 2, Holding = 3, Opening = 4 };
enum class CatchResult : uint32_t { None = 0, Caught = 1, ArmStrike = 2, TowerStrike = 3 };

// How one pin relates to its arm. Arm-local frame: origin at the hinge, `along` toward the arm
// tip, `across` perpendicular in the horizontal plane, z above the origin plane.
enum class PinContact { None, Cradle, ArmBody };

// Inputs to the pure catch test. Everything is already in arm-local or scalar form so tests can
// call evaluateContact() without a vehicle.
struct ContactInput {
    double along[2] = {};        // [m] pin position along its arm from the hinge, pin 0 then pin 1
    double across[2] = {};       // [m] pin position across its arm from the arm centerline
    double z[2] = {};            // [m] pin center height above the origin plane
    double closure = 0.0;        // [] min over both arms of psi / psiClosed
    double sinkRate = 0.0;       // [m/s] positive down
    double horizontalSpeed = 0.0;  // [m/s]
    double tilt = 0.0;           // [rad] body axis from vertical
    double headingError = 0.0;   // [rad] absolute
    double angularRate = 0.0;    // [rad/s] magnitude
};

// Classifies one pin against its arm (pin center inside the rail slab, then cradle footprint or arm body).
PinContact classifyPin(const TowerParams& tower, double pinRadius, double along, double across, double z);

// Pure contact and catch test: None if neither pin touches its arm, ArmStrike if a pin lands on
// the arm body outside the cradle or the catch window is missed, Caught if every window item passes.
CatchResult evaluateContact(const TowerParams& tower, double pinRadius, const ContactInput& in);

// Close trigger (pure): the pin midpoint is within the radius of the seat point, descending, and either below
// the trigger height or predicted to reach the seat within the lead time (2 * height / sink).
bool shouldCloseArms(const TowerParams& tower, double horizontal, double height, double sink);

// Pure hull test: does a vertical hull disc of `hullRadius` centered at tower-frame point (along, lateral)
// overlap the arm box (hinge, direction phi(psi), length and half width from `tower`) on side +1 (arm 0) or -1?
bool hullTouchesArm(const TowerParams& tower, double hullRadius, double psi, double side, double along, double lateral);

// The catch tower: two chopstick arms driven by a rate-limited servo, a state machine that closes
// them on an approaching Ship, contact tests for the two catch pins and a mast strike test.
// Reads vehicle truth (ground infrastructure placeholder, TODO(phase 10, item 48)); no RNG, no
// bus traffic. On a catch it holds the vehicle (HopperDynamics::hold); on a strike it freezes it.
class TowerModel final : public Model {
public:
    TowerModel(std::string name, double rateHz, HopperDynamics& vehicle, const TowerParams& tower);

    void update(const StepContext& ctx) override;

    TowerState state() const { return state_; }
    CatchResult result() const { return result_; }
    double armAngle(int arm) const { return psi_[arm]; }  // ACTUAL [rad], from open
    double closedAngle() const { return psiClosed_; }     // [rad]
    const TowerParams& params() const { return tower_; }
    const TowerParams::SeatPoint& seat() const { return seat_; }

private:
    void stepServo(double dt);
    void updateStateMachine(double dt);
    void checkContacts();
    void checkMast();
    void checkHullArms();
    bool outsideZone() const;

    HopperDynamics& vehicle_;
    TowerParams tower_;
    double pinRadius_ = 0.0;     // [m]
    double pinOffset_ = 0.0;     // [m]
    double psiClosed_ = 0.0;     // [rad]
    double length_ = 0.0;        // [m] vehicle length, for the mast test
    double hullRadius_ = 0.0;    // [m]
    TowerParams::SeatPoint seat_{};
    double psi_[2] = {0.0, 0.0};  // [rad]
    TowerState state_ = TowerState::Open;
    CatchResult result_ = CatchResult::None;
    double stateTime_ = 0.0;      // [s] in the current state
    double outsideTime_ = 0.0;    // [s] continuously outside the zone
};

}  // namespace sim::vehicle
