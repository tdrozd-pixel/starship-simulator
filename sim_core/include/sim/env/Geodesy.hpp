#pragma once

#include <Eigen/Core>

// WGS-84 geodesy for the simulator side: geodetic <-> ECEF <-> local ENU.
// The flight software has its own independent implementation (fsw/Geodesy.hpp)
// because it may not depend on sim code; tests cross-check the two.
//
// The dynamics still run in a flat local ENU frame (TODO(phase 0, item 2): ECI/ECEF
// dynamics and Earth rotation). This module ties that flat frame to the real Earth:
// sim ENU is the tangent plane at a site origin, so sensors like GNSS can report
// real geodetic coordinates.
namespace sim::env {

// WGS-84 defining constants (NGA.STND.0036_1.0.0_WGS84).
inline constexpr double kWgs84A = 6378137.0;                 // semi-major axis [m]
inline constexpr double kWgs84F = 1.0 / 298.257223563;       // flattening
inline constexpr double kWgs84B = kWgs84A * (1.0 - kWgs84F);  // semi-minor axis [m]
inline constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);  // first eccentricity squared

struct Geodetic {
    double latitude = 0.0;   // geodetic latitude [rad]
    double longitude = 0.0;  // [rad], east positive
    double height = 0.0;     // height above the WGS-84 ellipsoid [m]
};

Eigen::Vector3d geodeticToEcef(const Geodetic& g);

// Bowring's method with fixed iterations (deterministic, sub-millimeter at any
// altitude the vehicle reaches).
Geodetic ecefToGeodetic(const Eigen::Vector3d& ecef);

// Local East-North-Up tangent frame at a geodetic origin.
class LocalTangentFrame {
public:
    explicit LocalTangentFrame(const Geodetic& origin);

    Eigen::Vector3d enuToEcef(const Eigen::Vector3d& enu) const;
    Eigen::Vector3d ecefToEnu(const Eigen::Vector3d& ecef) const;
    Geodetic enuToGeodetic(const Eigen::Vector3d& enu) const { return ecefToGeodetic(enuToEcef(enu)); }
    Eigen::Vector3d geodeticToEnu(const Geodetic& g) const { return ecefToEnu(geodeticToEcef(g)); }

    const Geodetic& origin() const { return origin_; }
    const Eigen::Matrix3d& enuToEcefRotation() const { return rotation_; }

private:
    Geodetic origin_;
    Eigen::Vector3d originEcef_;
    Eigen::Matrix3d rotation_;  // columns: east, north, up expressed in ECEF
};

}  // namespace sim::env
