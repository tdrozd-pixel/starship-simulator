#include "fsw/VerticalEstimator.hpp"

namespace fsw {

void VerticalEstimator::predict(double specificForceUp, double dt) {
    const double accel = specificForceUp - config_.gravity;
    altitude_ += velocity_ * dt + 0.5 * accel * dt * dt;
    velocity_ += accel * dt;
}

void VerticalEstimator::correct(double measuredAltitude, double dt) {
    const double residual = measuredAltitude - altitude_;
    altitudeInnovation_ = residual;
    altitude_ += config_.altitudeGain * residual * dt;
    velocity_ += config_.velocityGain * residual * dt;
}

void VerticalEstimator::reset(double altitude, double velocity) {
    altitude_ = altitude;
    velocity_ = velocity;
}

}  // namespace fsw
