#pragma once

#include <cstdint>
#include <initializer_list>
#include <vector>

#include <Eigen/Core>

#include "sim/core/Model.hpp"
#include "sim/core/Random.hpp"

namespace sim::env {

struct WindProfilePoint {
    double altitude = 0.0;  // [m] geometric altitude
    Eigen::Vector3d wind = Eigen::Vector3d::Zero();  // ENU [m/s]
};

// Piecewise-linear altitude wind profile. End points are held outside the table.
class WindProfile {
public:
    WindProfile();
    WindProfile(std::initializer_list<WindProfilePoint> points);
    explicit WindProfile(std::vector<WindProfilePoint> points);

    void set(std::vector<WindProfilePoint> points);
    void addPoint(double altitude, const Eigen::Vector3d& wind);
    Eigen::Vector3d at(double altitude) const;
    Eigen::Vector3d windAt(double altitude) const { return at(altitude); }

private:
    std::vector<WindProfilePoint> points_;
};

struct GustParameters {
    Eigen::Vector3d amplitude = Eigen::Vector3d::Zero();  // ENU [m/s]
    double startTime = 0.0;  // [s]
    double duration = 0.0;   // [s]
};

// Standard one-cosine gust: zero at both ends and maximum in the middle.
class Gust {
public:
    explicit Gust(GustParameters parameters = {});
    Eigen::Vector3d at(double time) const;
    Eigen::Vector3d evaluate(double time) const { return at(time); }
    bool active(double time) const;

    const GustParameters& parameters() const { return parameters_; }

private:
    GustParameters parameters_;
};

struct TurbulenceParameters {
    Eigen::Vector3d sigma = Eigen::Vector3d::Zero();      // RMS velocity [m/s]
    Eigen::Vector3d scaleLength = Eigen::Vector3d::Constant(250.0);  // [m]
};

// Dryden-inspired exponentially correlated turbulence. The implementation is
// altitude-independent except for the supplied parameters.
class DrydenTurbulence {
public:
    explicit DrydenTurbulence(uint64_t seed = 1,
                              TurbulenceParameters parameters = {});

    void reset();
    Eigen::Vector3d step(double dt, double trueAirspeed);
    Eigen::Vector3d sample(double dt, double trueAirspeed) { return step(dt, trueAirspeed); }

    const Eigen::Vector3d& value() const { return value_; }
    const TurbulenceParameters& parameters() const { return parameters_; }

private:
    Rng rng_;
    TurbulenceParameters parameters_;
    Eigen::Vector3d value_ = Eigen::Vector3d::Zero();
};

// Von Karman is represented by two-pole low-pass shaping. This is a numerically
// stable real-time approximation suitable for a fixed-step simulation.
class VonKarmanTurbulence {
public:
    explicit VonKarmanTurbulence(uint64_t seed = 1,
                                 TurbulenceParameters parameters = {});

    void reset();
    Eigen::Vector3d step(double dt, double trueAirspeed);
    Eigen::Vector3d sample(double dt, double trueAirspeed) { return step(dt, trueAirspeed); }

    const Eigen::Vector3d& value() const { return value_; }
    const TurbulenceParameters& parameters() const { return parameters_; }

private:
    Rng rng_;
    TurbulenceParameters parameters_;
    Eigen::Vector3d value_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d derivative_ = Eigen::Vector3d::Zero();
};

struct WeatherSample {
    Eigen::Vector3d meanWind = Eigen::Vector3d::Zero();  // ENU [m/s]
    Eigen::Vector3d turbulence = Eigen::Vector3d::Zero();  // ENU [m/s]
    Eigen::Vector3d gust = Eigen::Vector3d::Zero();  // ENU [m/s]
    Eigen::Vector3d wind = Eigen::Vector3d::Zero();  // ENU [m/s], total

    Eigen::Vector3d totalWind() const { return wind; }
};

struct WeatherConditions {
    double altitude = 0.0;  // [m]
    double trueAirspeed = 0.0;  // [m/s]
};

struct WeatherParameters {
    WindProfile profile;
    TurbulenceParameters turbulence;
    std::vector<GustParameters> gusts;

    WeatherParameters();
};

using WeatherConfig = WeatherParameters;
using WindSample = WeatherSample;

// Discrete 1 kHz weather model. Publishes the WeatherSample bus channel.
class WeatherModel final : public Model {
public:
    WeatherModel(const char* name, uint64_t masterSeed,
                 WeatherParameters parameters = {});

    void initialize(Bus& bus) override;
    void update(const StepContext& ctx) override;

    const WeatherSample& current() const { return current_; }
    WeatherSample at(double altitude, double time) const;
    const WeatherParameters& parameters() const { return parameters_; }

private:
    WeatherParameters parameters_;
    DrydenTurbulence turbulence_;
    std::vector<Gust> gusts_;
    WeatherSample current_;
    const Channel<WeatherConditions>* conditionsChannel_ = nullptr;
    Channel<WeatherSample>* weatherChannel_ = nullptr;
};

}  // namespace sim::env
