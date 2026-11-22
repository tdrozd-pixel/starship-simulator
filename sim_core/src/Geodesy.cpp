#include "sim/env/Geodesy.hpp"

#include <cmath>

namespace sim::env {

Eigen::Vector3d geodeticToEcef(const Geodetic& g) {
    const double sinLat = std::sin(g.latitude);
    const double cosLat = std::cos(g.latitude);
    const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sinLat * sinLat);  // prime vertical radius [m]
    return {(n + g.height) * cosLat * std::cos(g.longitude), (n + g.height) * cosLat * std::sin(g.longitude),
            (n * (1.0 - kWgs84E2) + g.height) * sinLat};
}

Geodetic ecefToGeodetic(const Eigen::Vector3d& ecef) {
    constexpr double ep2 = (kWgs84A * kWgs84A - kWgs84B * kWgs84B) / (kWgs84B * kWgs84B);  // second ecc. squared
    const double p = std::hypot(ecef.x(), ecef.y());
    Geodetic g;
    g.longitude = std::atan2(ecef.y(), ecef.x());

    // Bowring: parametric latitude start, a few fixed-point refinements.
    double beta = std::atan2(ecef.z() * kWgs84A, p * kWgs84B);
    double lat = 0.0;
    for (int i = 0; i < 4; ++i) {
        const double sb = std::sin(beta);
        const double cb = std::cos(beta);
        lat = std::atan2(ecef.z() + ep2 * kWgs84B * sb * sb * sb, p - kWgs84E2 * kWgs84A * cb * cb * cb);
        beta = std::atan2((1.0 - kWgs84F) * std::sin(lat), std::cos(lat));
    }
    g.latitude = lat;

    const double sinLat = std::sin(lat);
    const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sinLat * sinLat);
    // Height formula that stays well conditioned at all latitudes.
    g.height = p * std::cos(lat) + (ecef.z() + kWgs84E2 * n * sinLat) * sinLat - n;
    return g;
}

LocalTangentFrame::LocalTangentFrame(const Geodetic& origin)
    : origin_(origin), originEcef_(geodeticToEcef(origin)) {
    const double sLat = std::sin(origin.latitude);
    const double cLat = std::cos(origin.latitude);
    const double sLon = std::sin(origin.longitude);
    const double cLon = std::cos(origin.longitude);
    rotation_.col(0) = Eigen::Vector3d(-sLon, cLon, 0.0);                     // east
    rotation_.col(1) = Eigen::Vector3d(-sLat * cLon, -sLat * sLon, cLat);     // north
    rotation_.col(2) = Eigen::Vector3d(cLat * cLon, cLat * sLon, sLat);       // up
}

Eigen::Vector3d LocalTangentFrame::enuToEcef(const Eigen::Vector3d& enu) const {
    return originEcef_ + rotation_ * enu;
}

Eigen::Vector3d LocalTangentFrame::ecefToEnu(const Eigen::Vector3d& ecef) const {
    return rotation_.transpose() * (ecef - originEcef_);
}

}  // namespace sim::env
