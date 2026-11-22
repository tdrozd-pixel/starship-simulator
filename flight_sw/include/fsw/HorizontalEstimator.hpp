#pragma once

namespace fsw {

// Horizontal (east, north) navigation of the reference point. One decoupled
// 3-state Kalman filter per axis: position, velocity and an apparent acceleration
// bias. IMU acceleration (rotated with the estimated attitude) drives the
// prediction; GNSS position and velocity correct it.
//
// The bias state matters: a tilt error e in the gyro-integrated attitude makes the
// rotated IMU report a horizontal acceleration of about g * e that GNSS does not
// see. The filter estimates it, and the flight computer feeds it back as a tilt
// correction to the attitude (GNSS levelling), then removes it with removeBias().
// TODO(phase 9, item 43): full navigation EKF (attitude, sensor biases, lever arms,
// latency as states) with sensor voting.
class HorizontalEstimator {
public:
    struct Config {
        double accelNoiseSigma = 0.05;       // [m/s^2] white acceleration error
        double biasRandomWalk = 0.003;       // [m/s^2/sqrt(s)] drift of the apparent bias
        double initialVelocitySigma = 1.0;   // [m/s]
        double initialBiasSigma = 0.1;       // [m/s^2]
    };

    explicit HorizontalEstimator(Config config) : config_(config) {}

    // Starts the filter at a measured position with zero velocity (vehicle on the pad).
    void initialize(double east, double north, double positionSigma);
    bool initialized() const { return initialized_; }

    void predict(double accelEast, double accelNorth, double dt);
    void correctPosition(double east, double north, double sigma);
    void correctVelocity(double velocityEast, double velocityNorth, double sigma);
    // The caller corrected the error source (attitude) by this much: subtract it
    // from the bias states (closed-loop error-state reset, covariance unchanged).
    void removeBias(double east, double north) {
        axis_[0].x[2] -= east;
        axis_[1].x[2] -= north;
    }
    // Known-zero velocity (on the ground), with a tight sigma.
    void zeroVelocityUpdate() { correctVelocity(0.0, 0.0, 0.01); }

    double east() const { return axis_[0].x[0]; }
    double north() const { return axis_[1].x[0]; }
    double velocityEast() const { return axis_[0].x[1]; }
    double velocityNorth() const { return axis_[1].x[1]; }
    // Apparent acceleration bias: measured (rotated IMU) minus true acceleration [m/s^2].
    double accelBiasEast() const { return axis_[0].x[2]; }
    double accelBiasNorth() const { return axis_[1].x[2]; }
    double positionSigma(int axis) const;

private:
    struct Axis {
        double x[3] = {};      // position [m], velocity [m/s], bias [m/s^2]
        double p[3][3] = {};   // covariance
    };

    static void predictAxis(Axis& a, double accel, double dt, double q, double qBias);
    static void correctAxis(Axis& a, int state, double z, double r);

    Config config_;
    Axis axis_[2];
    bool initialized_ = false;
};

}  // namespace fsw
