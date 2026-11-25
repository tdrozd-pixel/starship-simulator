#pragma once

namespace fsw {

// Horizontal (east, north) navigation of the reference point. One decoupled
// 3-state Kalman filter per axis: position, velocity and an apparent acceleration
// bias. IMU acceleration (rotated with the estimated attitude) drives the
// prediction; GNSS position and velocity correct it.
//
// Latency: a GNSS solution describes the vehicle at its time of validity, tens of
// milliseconds before the flight computer has it. The filter keeps a fixed ring of
// its own history (state, covariance and the IMU input of every prediction step).
// correctDelayed() applies the update to the state of the measurement time (so the
// innovation compares like with like), then replays the stored inputs to now. The
// filter is linear, so the replay is exact. Closed-loop corrections (removeBias,
// zero-velocity updates) are applied to the stored history too, so a replay never
// loses them. Memory is fixed at construction; no allocation in any call.
//
// The bias state matters: a tilt error e in the gyro-integrated attitude makes the
// rotated IMU report a horizontal acceleration of about g * e that GNSS does not
// see. The filter estimates it, and the flight computer feeds it back as a tilt
// correction to the attitude (GNSS levelling), then removes it with removeBias().
// TODO(phase 9, item 43): full navigation EKF (attitude, sensor biases, lever arms,
// latency as states) with sensor voting.
class HorizontalEstimator {
public:
    // History length in prediction steps: 0.64 s at the 100 Hz flight rate, many times the
    // GNSS latency (50 ms) plus jitter.
    static constexpr int kHistoryCapacity = 64;

    // Outcome of correctDelayed().
    enum class DelayedResult {
        Applied,      // update applied at the measurement time and replayed to now
        Initialized,  // filter was not running: started from this fix
        Stale,        // older than the history: dropped and counted
        OutOfOrder,   // not newer than the last applied fix, or stamped in the future: dropped and counted
    };

    struct Config {
        double accelNoiseSigma = 0.05;       // [m/s^2] white acceleration error
        double biasRandomWalk = 0.003;       // [m/s^2/sqrt(s)] drift of the apparent bias
        double initialVelocitySigma = 1.0;   // [m/s]
        double initialBiasSigma = 0.1;       // [m/s^2]
    };

    explicit HorizontalEstimator(Config config) : config_(config) {}

    // Starts the filter at a measured position with zero velocity (vehicle on the pad).
    // `time` [s] labels this first state in the history (the flight time of the fix being used).
    void initialize(double east, double north, double positionSigma, double time = 0.0);
    bool initialized() const { return initialized_; }

    // Advances the state by dt [s] with the measured (rotated, apparent) horizontal acceleration
    // [m/s^2]. The overload without a time uses the internal clock (previous time + dt); the one
    // with a time labels the new state with it (flight time of the IMU sample).
    void predict(double accelEast, double accelNorth, double dt) { predict(accelEast, accelNorth, dt, time_ + dt); }
    void predict(double accelEast, double accelNorth, double dt, double time);

    // GNSS position and velocity (reference point, already lever-arm corrected) valid at
    // `measurementTime` [s], received at `now` [s]. Applies them to the state of the measurement time
    // and replays to now. The position innovation is measurement minus the prediction at that time.
    DelayedResult correctDelayed(double now, double measurementTime, double east, double north,
                                 double positionSigma, double velocityEast, double velocityNorth,
                                 double velocitySigma);

    // Immediate (no latency) corrections: for a fix that is valid now. Not replayed.
    void correctPosition(double east, double north, double sigma);
    void correctVelocity(double velocityEast, double velocityNorth, double sigma);
    // The caller corrected the error source (attitude) by this much: subtract it
    // from the bias states (closed-loop error-state reset, covariance unchanged).
    // The correction is applied to the whole history as well (the stored accelerations and bias
    // states move together), as if the attitude had been right all along.
    void removeBias(double east, double north);
    // Known-zero velocity (on the ground), with a tight sigma. Recorded in the history so a
    // delayed update replays it.
    void zeroVelocityUpdate();

    double east() const { return axis_[0].x[0]; }
    double north() const { return axis_[1].x[0]; }
    double velocityEast() const { return axis_[0].x[1]; }
    double velocityNorth() const { return axis_[1].x[1]; }
    // Apparent acceleration bias: measured (rotated IMU) minus true acceleration [m/s^2].
    double accelBiasEast() const { return axis_[0].x[2]; }
    double accelBiasNorth() const { return axis_[1].x[2]; }
    double positionSigma(int axis) const;
    // 1-sigma from the covariance diagonal: position [m] / velocity [m/s] (axis 0 east, 1 north).
    double velocitySigma(int axis) const;
    // Latest GNSS position innovation, measurement minus prediction [m] (axis 0 east, 1 north).
    // Zero until the first correction after initialize().
    double positionInnovation(int axis) const { return positionInnovation_[axis]; }
    // Fixes dropped because they were older than the history / not newer than the last fix.
    int staleDrops() const { return staleDrops_; }
    int outOfOrderDrops() const { return outOfOrderDrops_; }
    // Flight time of the newest state [s].
    double time() const { return time_; }

private:
    struct Axis {
        double x[3] = {};      // position [m], velocity [m/s], bias [m/s^2]
        double p[3][3] = {};   // covariance
    };

    // One entry of the history: the state after the step ending at `time`, and what produced it.
    struct Snapshot {
        double time = 0.0;      // [s]
        Axis axis[2];           // state after the step (and any zero-velocity update)
        double accel[2] = {};   // [m/s^2] measured acceleration used by the step
        double dt = 0.0;        // [s] step length (0 for the initial entry)
        bool zeroVelocity = false;  // a zero-velocity update followed the step
    };

    static constexpr double kZeroVelocitySigma = 0.01;  // [m/s]
    static constexpr double kTimeTolerance = 1.0e-6;    // [s] stamp comparison slack

    static void predictAxis(Axis& a, double accel, double dt, double q, double qBias);
    static void correctAxis(Axis& a, int state, double z, double r);
    void predictAxes(Axis (&axes)[2], const double accel[2], double dt) const;
    Snapshot& newest() { return history_[newestIndex_]; }
    // k = 0 is the oldest entry, count_ - 1 the newest.
    Snapshot& entry(int k) { return history_[(newestIndex_ - (count_ - 1) + k + kHistoryCapacity) % kHistoryCapacity]; }
    void syncNewest();  // the live state changed outside predict(): store it in the newest entry

    Config config_;
    Axis axis_[2];
    double positionInnovation_[2] = {};  // [m]
    bool initialized_ = false;
    Snapshot history_[kHistoryCapacity];
    int newestIndex_ = 0;
    int count_ = 0;
    double time_ = 0.0;              // [s] time of the newest state
    double lastMeasurementTime_ = 0.0;  // [s] of the last applied delayed fix
    int staleDrops_ = 0;
    int outOfOrderDrops_ = 0;
};

}  // namespace fsw
