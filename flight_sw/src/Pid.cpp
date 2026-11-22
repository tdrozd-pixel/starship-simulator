#include "fsw/Pid.hpp"

#include <algorithm>

namespace fsw {

double Pid::update(double setpoint, double measurement, double dt) {
    const double error = setpoint - measurement;

    p_ = gains_.kp * error;

    d_ = 0.0;
    if (hasLast_ && dt > 0.0) {
        d_ = -gains_.kd * (measurement - lastMeasurement_) / dt;
    }
    lastMeasurement_ = measurement;
    hasLast_ = true;

    const double candidateIntegral = integral_ + error * dt;
    double candidateI = gains_.ki * candidateIntegral;
    candidateI = std::clamp(candidateI, -gains_.integratorLimit, gains_.integratorLimit);

    const double unsaturated = p_ + candidateI + d_;
    const double output = std::clamp(unsaturated, gains_.outputMin, gains_.outputMax);

    // Only accept the new integral if it does not push further into saturation.
    const bool saturatedHigh = unsaturated > gains_.outputMax && error > 0.0;
    const bool saturatedLow = unsaturated < gains_.outputMin && error < 0.0;
    if (!saturatedHigh && !saturatedLow && gains_.ki != 0.0) {
        integral_ = candidateI / gains_.ki;
    }
    i_ = gains_.ki * integral_;

    return output;
}

void Pid::reset() {
    integral_ = 0.0;
    hasLast_ = false;
    p_ = i_ = d_ = 0.0;
}

}  // namespace fsw
