#include "fsw/Geodesy.hpp"

#include <cmath>

namespace fsw {
namespace {

constexpr double kSemiMajorAxis = 6378137.0;           // WGS-84 [m]
constexpr double kFlattening = 1.0 / 298.257223563;    // WGS-84
constexpr double kEccentricitySquared = kFlattening * (2.0 - kFlattening);

}  // namespace

void geodeticToEcef(const GeodeticPosition& position, double ecef[3]) {
    const double sinLat = std::sin(position.latitude);
    const double cosLat = std::cos(position.latitude);
    const double primeVertical = kSemiMajorAxis / std::sqrt(1.0 - kEccentricitySquared * sinLat * sinLat);
    ecef[0] = (primeVertical + position.height) * cosLat * std::cos(position.longitude);
    ecef[1] = (primeVertical + position.height) * cosLat * std::sin(position.longitude);
    ecef[2] = (primeVertical * (1.0 - kEccentricitySquared) + position.height) * sinLat;
}

LocalEnuFrame::LocalEnuFrame(const GeodeticPosition& origin) {
    geodeticToEcef(origin, originEcef_);
    const double sLat = std::sin(origin.latitude);
    const double cLat = std::cos(origin.latitude);
    const double sLon = std::sin(origin.longitude);
    const double cLon = std::cos(origin.longitude);
    east_[0] = -sLon;
    east_[1] = cLon;
    east_[2] = 0.0;
    north_[0] = -sLat * cLon;
    north_[1] = -sLat * sLon;
    north_[2] = cLat;
    up_[0] = cLat * cLon;
    up_[1] = cLat * sLon;
    up_[2] = sLat;
}

void LocalEnuFrame::toEnu(const GeodeticPosition& position, double enu[3]) const {
    double ecef[3];
    geodeticToEcef(position, ecef);
    const double d[3] = {ecef[0] - originEcef_[0], ecef[1] - originEcef_[1], ecef[2] - originEcef_[2]};
    enu[0] = east_[0] * d[0] + east_[1] * d[1] + east_[2] * d[2];
    enu[1] = north_[0] * d[0] + north_[1] * d[1] + north_[2] * d[2];
    enu[2] = up_[0] * d[0] + up_[1] * d[1] + up_[2] * d[2];
}

}  // namespace fsw
