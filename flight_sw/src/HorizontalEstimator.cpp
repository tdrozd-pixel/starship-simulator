#include "fsw/HorizontalEstimator.hpp"

#include <algorithm>
#include <cmath>

namespace fsw {

void HorizontalEstimator::initialize(double east, double north, double positionSigma, double time) {
    const double measured[2] = {east, north};
    for (int i = 0; i < 2; ++i) {
        Axis& a = axis_[i];
        a = Axis{};
        positionInnovation_[i] = 0.0;
        a.x[0] = measured[i];
        a.p[0][0] = positionSigma * positionSigma;
        a.p[1][1] = config_.initialVelocitySigma * config_.initialVelocitySigma;
        a.p[2][2] = config_.initialBiasSigma * config_.initialBiasSigma;
    }
    initialized_ = true;
    time_ = time;
    lastMeasurementTime_ = time;
    newestIndex_ = 0;
    count_ = 1;
    Snapshot& first = history_[0];
    first = Snapshot{};
    first.time = time;
    first.axis[0] = axis_[0];
    first.axis[1] = axis_[1];
}

void HorizontalEstimator::predictAxes(Axis (&axes)[2], const double accel[2], double dt) const {
    const double q = config_.accelNoiseSigma * config_.accelNoiseSigma;
    const double qBias = config_.biasRandomWalk * config_.biasRandomWalk;
    predictAxis(axes[0], accel[0], dt, q, qBias);
    predictAxis(axes[1], accel[1], dt, q, qBias);
}

void HorizontalEstimator::predict(double accelEast, double accelNorth, double dt, double time) {
    if (!initialized_ || dt <= 0.0) {
        return;
    }
    const double accel[2] = {accelEast, accelNorth};
    predictAxes(axis_, accel, dt);
    time_ = time;
    newestIndex_ = (newestIndex_ + 1) % kHistoryCapacity;
    count_ = std::min(count_ + 1, kHistoryCapacity);
    Snapshot& s = history_[newestIndex_];
    s.time = time;
    s.axis[0] = axis_[0];
    s.axis[1] = axis_[1];
    s.accel[0] = accelEast;
    s.accel[1] = accelNorth;
    s.dt = dt;
    s.zeroVelocity = false;
}

void HorizontalEstimator::syncNewest() {
    if (count_ > 0) {
        newest().axis[0] = axis_[0];
        newest().axis[1] = axis_[1];
    }
}

void HorizontalEstimator::removeBias(double east, double north) {
    const double shift[2] = {east, north};
    for (int i = 0; i < 2; ++i) {
        axis_[i].x[2] -= shift[i];
    }
    // History: move the stored acceleration and the stored bias together, so every replayed step
    // still uses the same true acceleration (accel - bias) it used before.
    for (int k = 0; k < count_; ++k) {
        Snapshot& s = entry(k);
        for (int i = 0; i < 2; ++i) {
            s.axis[i].x[2] -= shift[i];
            s.accel[i] -= shift[i];
        }
    }
}

void HorizontalEstimator::zeroVelocityUpdate() {
    if (!initialized_) {
        return;
    }
    correctVelocity(0.0, 0.0, kZeroVelocitySigma);
    newest().zeroVelocity = true;
}

HorizontalEstimator::DelayedResult HorizontalEstimator::correctDelayed(
    double now, double measurementTime, double east, double north, double positionSigma, double velocityEast,
    double velocityNorth, double velocitySigma) {
    if (!initialized_) {
        initialize(east, north, positionSigma, now);
        correctVelocity(velocityEast, velocityNorth, velocitySigma);
        return DelayedResult::Initialized;
    }
    if (measurementTime <= lastMeasurementTime_ + kTimeTolerance || measurementTime > newest().time + kTimeTolerance) {
        ++outOfOrderDrops_;
        return DelayedResult::OutOfOrder;
    }
    if (measurementTime < entry(0).time - kTimeTolerance) {
        ++staleDrops_;
        return DelayedResult::Stale;
    }

    // Nearest stored state to the measurement time.
    int j = 0;
    double best = std::abs(entry(0).time - measurementTime);
    for (int k = 1; k < count_; ++k) {
        const double d = std::abs(entry(k).time - measurementTime);
        if (d < best) {
            best = d;
            j = k;
        }
    }
    Snapshot& at = entry(j);

    // The steps are 10 ms but a fix can fall between them: move the measurement the remaining
    // residual (|residual| <= half a step) to the stored time with the measured velocity and the
    // true acceleration of the step that spans it.
    const double residual = measurementTime - at.time;  // [s]
    const Snapshot* span = residual >= 0.0 ? (j + 1 < count_ ? &entry(j + 1) : nullptr) : &at;
    const double measured[2] = {east, north};
    const double measuredVelocity[2] = {velocityEast, velocityNorth};
    double position[2];
    double velocity[2];
    for (int i = 0; i < 2; ++i) {
        const double accel = span != nullptr ? span->accel[i] - at.axis[i].x[2] : 0.0;  // [m/s^2]
        position[i] = measured[i] - measuredVelocity[i] * residual;
        velocity[i] = measuredVelocity[i] - accel * residual;
    }

    positionInnovation_[0] = position[0] - at.axis[0].x[0];  // [m] measurement minus prediction at that time
    positionInnovation_[1] = position[1] - at.axis[1].x[0];
    const double positionR = positionSigma * positionSigma;
    const double velocityR = velocitySigma * velocitySigma;
    for (int i = 0; i < 2; ++i) {
        correctAxis(at.axis[i], 0, position[i], positionR);
        correctAxis(at.axis[i], 1, velocity[i], velocityR);
    }

    // Replay the inputs after that time (exact: the filter is linear in the stored inputs).
    for (int k = j + 1; k < count_; ++k) {
        Snapshot& s = entry(k);
        s.axis[0] = entry(k - 1).axis[0];
        s.axis[1] = entry(k - 1).axis[1];
        predictAxes(s.axis, s.accel, s.dt);
        if (s.zeroVelocity) {
            correctAxis(s.axis[0], 1, 0.0, kZeroVelocitySigma * kZeroVelocitySigma);
            correctAxis(s.axis[1], 1, 0.0, kZeroVelocitySigma * kZeroVelocitySigma);
        }
    }
    axis_[0] = newest().axis[0];
    axis_[1] = newest().axis[1];
    lastMeasurementTime_ = measurementTime;
    return DelayedResult::Applied;
}

void HorizontalEstimator::correctPosition(double east, double north, double sigma) {
    if (!initialized_) {
        initialize(east, north, sigma, time_);
        return;
    }
    positionInnovation_[0] = east - axis_[0].x[0];  // [m]
    positionInnovation_[1] = north - axis_[1].x[0];  // [m]
    correctAxis(axis_[0], 0, east, sigma * sigma);
    correctAxis(axis_[1], 0, north, sigma * sigma);
    syncNewest();
}

void HorizontalEstimator::correctVelocity(double velocityEast, double velocityNorth, double sigma) {
    if (!initialized_) {
        return;
    }
    correctAxis(axis_[0], 1, velocityEast, sigma * sigma);
    correctAxis(axis_[1], 1, velocityNorth, sigma * sigma);
    syncNewest();
}

double HorizontalEstimator::positionSigma(int axis) const { return std::sqrt(axis_[axis].p[0][0]); }

double HorizontalEstimator::velocitySigma(int axis) const { return std::sqrt(axis_[axis].p[1][1]); }

void HorizontalEstimator::predictAxis(Axis& a, double accel, double dt, double q, double qBias) {
    // True acceleration = measured - bias.
    // F = [1 dt -dt^2/2; 0 1 -dt; 0 0 1]
    const double dt2 = dt * dt;
    const double trueAccel = accel - a.x[2];
    a.x[0] += a.x[1] * dt + 0.5 * trueAccel * dt2;
    a.x[1] += trueAccel * dt;

    const double f[3][3] = {{1.0, dt, -0.5 * dt2}, {0.0, 1.0, -dt}, {0.0, 0.0, 1.0}};
    double fp[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            fp[i][j] = f[i][0] * a.p[0][j] + f[i][1] * a.p[1][j] + f[i][2] * a.p[2][j];
        }
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            a.p[i][j] = fp[i][0] * f[j][0] + fp[i][1] * f[j][1] + fp[i][2] * f[j][2];
        }
    }
    // White acceleration noise on position/velocity, random walk on the bias.
    a.p[0][0] += q * dt2 * dt2 / 4.0;
    a.p[0][1] += q * dt2 * dt / 2.0;
    a.p[1][0] += q * dt2 * dt / 2.0;
    a.p[1][1] += q * dt2;
    a.p[2][2] += qBias * dt;
}

void HorizontalEstimator::correctAxis(Axis& a, int state, double z, double r) {
    // Scalar measurement of one state: H = e_state.
    const double s = a.p[state][state] + r;
    double k[3];
    for (int i = 0; i < 3; ++i) {
        k[i] = a.p[i][state] / s;
    }
    const double innovation = z - a.x[state];
    for (int i = 0; i < 3; ++i) {
        a.x[i] += k[i] * innovation;
    }
    double row[3];
    for (int j = 0; j < 3; ++j) {
        row[j] = a.p[state][j];
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            a.p[i][j] -= k[i] * row[j];
        }
    }
}

}  // namespace fsw
