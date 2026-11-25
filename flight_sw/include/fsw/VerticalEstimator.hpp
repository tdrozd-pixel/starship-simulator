#pragma once

namespace fsw {

// Second-order complementary filter for altitude and vertical velocity:
// predicts with IMU specific force, corrects with the altimeter.
//
// Assumes the vehicle stays upright (body z = world up), which holds for the
// vertical hop. TODO(phase 9, item 43): replace with a full EKF using attitude
// from integrated gyros plus GNSS.
class VerticalEstimator {
public:
    struct Config {
        double gravity = 9.80665;
        double altitudeGain = 4.0;   // [1/s], position correction bandwidth
        double velocityGain = 4.0;   // [1/s^2], velocity correction from altitude residual
    };

    explicit VerticalEstimator(Config config) : config_(config) {}

    void predict(double specificForceUp, double dt);
    void correct(double measuredAltitude, double dt);
    void reset(double altitude, double velocity);

    double altitude() const { return altitude_; }
    double velocity() const { return velocity_; }
    // Latest altimeter residual, measured minus estimated altitude before the correction [m].
    // This is a fixed-gain filter: there is no covariance, so no sigma is available.
    double altitudeInnovation() const { return altitudeInnovation_; }

private:
    Config config_;
    double altitude_ = 0.0;
    double velocity_ = 0.0;
    double altitudeInnovation_ = 0.0;  // [m]
};

}  // namespace fsw
