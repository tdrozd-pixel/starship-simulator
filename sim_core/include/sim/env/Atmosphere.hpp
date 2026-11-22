#pragma once

namespace sim::env {

struct AtmosphereSample {
    double temperature = 0.0;   // [K]
    double pressure = 0.0;      // [Pa]
    double density = 0.0;       // [kg/m^3]
    double speedOfSound = 0.0;  // [m/s]
};

// U.S. Standard Atmosphere 1976, 0 to 86 km geometric altitude.
// Above 86 km this extrapolates the top layer exponentially, which is only a
// placeholder. TODO(phase 0, item 4): NRLMSISE-00 table above ~80 km for entry.
class StandardAtmosphere1976 {
public:
    AtmosphereSample at(double geometricAltitude) const;
};

}  // namespace sim::env
