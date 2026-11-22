#include "sim/env/Atmosphere.hpp"

#include <array>
#include <cmath>

namespace sim::env {
namespace {

constexpr double kG0 = 9.80665;            // [m/s^2]
constexpr double kRStar = 8.31432;         // universal gas constant used by US76 [J/(mol K)]
constexpr double kMolarMass = 0.0289644;   // [kg/mol]
constexpr double kRAir = kRStar / kMolarMass;  // [J/(kg K)]
constexpr double kGamma = 1.4;
constexpr double kEarthRadius = 6356766.0;  // US76 effective radius [m]

struct Layer {
    double baseGeopotentialAltitude;  // [m]
    double baseTemperature;           // [K]
    double lapseRate;                 // [K/m]
    double basePressure;              // [Pa]
};

constexpr std::array<Layer, 7> kLayers{{
    {0.0, 288.15, -0.0065, 101325.0},
    {11000.0, 216.65, 0.0, 22632.06},
    {20000.0, 216.65, 0.0010, 5474.889},
    {32000.0, 228.65, 0.0028, 868.0187},
    {47000.0, 270.65, 0.0, 110.9063},
    {51000.0, 270.65, -0.0028, 66.93887},
    {71000.0, 214.65, -0.0020, 3.956420},
}};

constexpr double kTopGeopotential = 84852.0;  // 86 km geometric

double pressureInLayer(const Layer& layer, double h, double temperature) {
    const double dh = h - layer.baseGeopotentialAltitude;
    if (layer.lapseRate == 0.0) {
        return layer.basePressure * std::exp(-kG0 * kMolarMass * dh / (kRStar * layer.baseTemperature));
    }
    return layer.basePressure *
           std::pow(layer.baseTemperature / temperature, kG0 * kMolarMass / (kRStar * layer.lapseRate));
}

}  // namespace

AtmosphereSample StandardAtmosphere1976::at(double geometricAltitude) const {
    const double h = kEarthRadius * geometricAltitude / (kEarthRadius + geometricAltitude);

    AtmosphereSample s;
    if (h <= kTopGeopotential) {
        std::size_t i = 0;
        while (i + 1 < kLayers.size() && h >= kLayers[i + 1].baseGeopotentialAltitude) {
            ++i;
        }
        const Layer& layer = kLayers[i];
        s.temperature = layer.baseTemperature + layer.lapseRate * (h - layer.baseGeopotentialAltitude);
        s.pressure = pressureInLayer(layer, h, s.temperature);
    } else {
        // Placeholder above 86 km: isothermal extrapolation from the top of the table.
        const Layer& top = kLayers.back();
        const double topTemperature = top.baseTemperature + top.lapseRate * (kTopGeopotential - top.baseGeopotentialAltitude);
        const double topPressure = pressureInLayer(top, kTopGeopotential, topTemperature);
        s.temperature = topTemperature;
        s.pressure = topPressure * std::exp(-kG0 * kMolarMass * (h - kTopGeopotential) / (kRStar * topTemperature));
    }

    s.density = s.pressure / (kRAir * s.temperature);
    s.speedOfSound = std::sqrt(kGamma * kRAir * s.temperature);
    return s;
}

}  // namespace sim::env
