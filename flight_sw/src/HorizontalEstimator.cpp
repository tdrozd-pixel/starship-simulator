#include "fsw/HorizontalEstimator.hpp"

#include <cmath>

namespace fsw {

void HorizontalEstimator::initialize(double east, double north, double positionSigma) {
    const double measured[2] = {east, north};
    for (int i = 0; i < 2; ++i) {
        Axis& a = axis_[i];
        a = Axis{};
        a.x[0] = measured[i];
        a.p[0][0] = positionSigma * positionSigma;
        a.p[1][1] = config_.initialVelocitySigma * config_.initialVelocitySigma;
        a.p[2][2] = config_.initialBiasSigma * config_.initialBiasSigma;
    }
    initialized_ = true;
}

void HorizontalEstimator::predict(double accelEast, double accelNorth, double dt) {
    if (!initialized_ || dt <= 0.0) {
        return;
    }
    const double q = config_.accelNoiseSigma * config_.accelNoiseSigma;
    const double qBias = config_.biasRandomWalk * config_.biasRandomWalk;
    predictAxis(axis_[0], accelEast, dt, q, qBias);
    predictAxis(axis_[1], accelNorth, dt, q, qBias);
}

void HorizontalEstimator::correctPosition(double east, double north, double sigma) {
    if (!initialized_) {
        initialize(east, north, sigma);
        return;
    }
    correctAxis(axis_[0], 0, east, sigma * sigma);
    correctAxis(axis_[1], 0, north, sigma * sigma);
}

void HorizontalEstimator::correctVelocity(double velocityEast, double velocityNorth, double sigma) {
    if (!initialized_) {
        return;
    }
    correctAxis(axis_[0], 1, velocityEast, sigma * sigma);
    correctAxis(axis_[1], 1, velocityNorth, sigma * sigma);
}

double HorizontalEstimator::positionSigma(int axis) const { return std::sqrt(axis_[axis].p[0][0]); }

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
