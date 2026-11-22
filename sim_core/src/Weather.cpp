#include "sim/env/Weather.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

#include "sim/core/Bus.hpp"

namespace sim::env {
namespace {

constexpr double kMinimumAirspeed = 1.0;

double interpolate(double x0, double y0, double x1, double y1, double x) {
    if (x1 <= x0) {
        return y0;
    }
    const double fraction = (x - x0) / (x1 - x0);
    return y0 + fraction * (y1 - y0);
}

}  // namespace

WindProfile::WindProfile() : points_{{0.0, Eigen::Vector3d::Zero()}} {}

WindProfile::WindProfile(std::initializer_list<WindProfilePoint> points)
    : points_(points) {
    std::sort(points_.begin(), points_.end(),
              [](const WindProfilePoint& a, const WindProfilePoint& b) {
                  return a.altitude < b.altitude;
              });
    if (points_.empty()) {
        points_.push_back({0.0, Eigen::Vector3d::Zero()});
    }
}

WindProfile::WindProfile(std::vector<WindProfilePoint> points) : points_(std::move(points)) {
    std::sort(points_.begin(), points_.end(),
              [](const WindProfilePoint& a, const WindProfilePoint& b) {
                  return a.altitude < b.altitude;
              });
    if (points_.empty()) {
        points_.push_back({0.0, Eigen::Vector3d::Zero()});
    }
}

WeatherParameters::WeatherParameters()
    : profile({
          {0.0, Eigen::Vector3d(5.0, 0.0, 0.0)},
          {1000.0, Eigen::Vector3d(8.0, 0.0, 0.0)},
          {10000.0, Eigen::Vector3d(15.0, 0.0, 0.0)},
      }) {
    // Illustrative generic conditions, not Starship or Starbase flight data.
    turbulence.sigma = Eigen::Vector3d(1.0, 1.0, 0.5);
}

void WindProfile::set(std::vector<WindProfilePoint> points) {
    points = std::move(points);
    std::sort(points.begin(), points.end(),
              [](const WindProfilePoint& a, const WindProfilePoint& b) {
                  return a.altitude < b.altitude;
              });
    if (points.empty()) {
        points.push_back({0.0, Eigen::Vector3d::Zero()});
    }
    points_ = std::move(points);
}

void WindProfile::addPoint(double altitude, const Eigen::Vector3d& wind) {
    points_.push_back({altitude, wind});
    std::sort(points_.begin(), points_.end(),
              [](const WindProfilePoint& a, const WindProfilePoint& b) {
                  return a.altitude < b.altitude;
              });
}

Eigen::Vector3d WindProfile::at(double altitude) const {
    if (points_.size() == 1 || altitude <= points_.front().altitude) {
        return points_.front().wind;
    }
    if (altitude >= points_.back().altitude) {
        return points_.back().wind;
    }
    for (std::size_t i = 1; i < points_.size(); ++i) {
        if (altitude <= points_[i].altitude) {
            return Eigen::Vector3d(
                interpolate(points_[i - 1].altitude, points_[i - 1].wind.x(),
                            points_[i].altitude, points_[i].wind.x(), altitude),
                interpolate(points_[i - 1].altitude, points_[i - 1].wind.y(),
                            points_[i].altitude, points_[i].wind.y(), altitude),
                interpolate(points_[i - 1].altitude, points_[i - 1].wind.z(),
                            points_[i].altitude, points_[i].wind.z(), altitude));
        }
    }
    return points_.back().wind;
}

Gust::Gust(GustParameters parameters) : parameters_(parameters) {}

Eigen::Vector3d Gust::at(double time) const {
    if (parameters_.duration <= 0.0 ||
        time < parameters_.startTime ||
        time > parameters_.startTime + parameters_.duration) {
        return Eigen::Vector3d::Zero();
    }
    const double elapsed = time - parameters_.startTime;
    const double fraction = std::clamp(elapsed / parameters_.duration, 0.0, 1.0);
    return parameters_.amplitude * (1.0 - std::cos(2.0 * std::numbers::pi * fraction)) * 0.5;
}

bool Gust::active(double time) const {
    return parameters_.duration > 0.0 &&
           time >= parameters_.startTime &&
           time <= parameters_.startTime + parameters_.duration;
}

DrydenTurbulence::DrydenTurbulence(uint64_t seed, TurbulenceParameters parameters)
    : rng_(seed), parameters_(parameters) {}

void DrydenTurbulence::reset() {
    value_.setZero();
}

Eigen::Vector3d DrydenTurbulence::step(double dt, double trueAirspeed) {
    const double speed = std::max(kMinimumAirspeed, trueAirspeed);
    for (int i = 0; i < 3; ++i) {
        const double length = std::max(1.0, parameters_.scaleLength[i]);
        const double tau = length / speed;
        const double alpha = std::exp(-std::max(0.0, dt) / tau);
        const double sigma = std::max(0.0, parameters_.sigma[i]);
        value_[i] = alpha * value_[i] +
                    sigma * std::sqrt(std::max(0.0, 1.0 - alpha * alpha)) * rng_.normal();
    }
    return value_;
}

VonKarmanTurbulence::VonKarmanTurbulence(uint64_t seed, TurbulenceParameters parameters)
    : rng_(seed), parameters_(parameters) {}

void VonKarmanTurbulence::reset() {
    value_.setZero();
    derivative_.setZero();
}

Eigen::Vector3d VonKarmanTurbulence::step(double dt, double trueAirspeed) {
    const double h = std::max(0.0, dt);
    const double speed = std::max(kMinimumAirspeed, trueAirspeed);
    for (int i = 0; i < 3; ++i) {
        const double length = std::max(1.0, parameters_.scaleLength[i]);
        const double omega = 2.0 * speed / length;
        const double damping = std::sqrt(2.0);
        const double sigma = std::max(0.0, parameters_.sigma[i]);
        derivative_[i] += h *
            (-omega * omega * value_[i] - 2.0 * damping * omega * derivative_[i] +
             omega * omega * sigma * rng_.normal());
        value_[i] += h * derivative_[i];
    }
    return value_;
}

WeatherModel::WeatherModel(const char* name, uint64_t masterSeed,
                           WeatherParameters parameters)
    : Model(name, 1000.0),
      parameters_(std::move(parameters)),
      turbulence_(deriveSeed(masterSeed, "weather_turbulence"),
                  parameters_.turbulence) {
    for (std::size_t i = 0; i < parameters_.gusts.size(); ++i) {
        gusts_.push_back(Gust(parameters_.gusts[i]));
    }
}

void WeatherModel::initialize(Bus& bus) {
    conditionsChannel_ = &bus.channel<WeatherConditions>("weather_conditions");
    weatherChannel_ = &bus.channel<WeatherSample>("weather");
}

WeatherSample WeatherModel::at(double altitude, double time) const {
    WeatherSample sample;
    sample.meanWind = parameters_.profile.at(altitude);
    sample.turbulence = current_.turbulence;
    for (const Gust& gust : gusts_) {
        sample.gust += gust.at(time);
    }
    sample.wind = sample.meanWind + sample.turbulence + sample.gust;
    return sample;
}

void WeatherModel::update(const StepContext& ctx) {
    WeatherConditions conditions;
    if (conditionsChannel_ != nullptr && conditionsChannel_->latest().valid()) {
        conditions = conditionsChannel_->latest().data;
    }
    current_.turbulence = turbulence_.step(ctx.dt, conditions.trueAirspeed);
    current_.meanWind = parameters_.profile.at(conditions.altitude);
    current_.gust.setZero();
    for (const Gust& gust : gusts_) {
        current_.gust += gust.at(ctx.t);
    }
    current_.wind = current_.meanWind + current_.turbulence + current_.gust;
    if (weatherChannel_ != nullptr) {
        weatherChannel_->publish(current_, ctx.t);
    }
}

}  // namespace sim::env
