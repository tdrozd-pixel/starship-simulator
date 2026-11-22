#pragma once

// WGS-84 geodesy for the flight software. Independent of the simulator's
// implementation on purpose (flight software depends on nothing in the repo).

namespace fsw {

struct GeodeticPosition {
    double latitude = 0.0;   // WGS-84 geodetic [rad]
    double longitude = 0.0;  // [rad], east positive
    double height = 0.0;     // above the ellipsoid [m]
};

void geodeticToEcef(const GeodeticPosition& position, double ecef[3]);

// East-North-Up frame tangent to the ellipsoid at a reference point (for the hop:
// the landing target).
class LocalEnuFrame {
public:
    LocalEnuFrame() = default;
    explicit LocalEnuFrame(const GeodeticPosition& origin);

    void toEnu(const GeodeticPosition& position, double enu[3]) const;

private:
    double originEcef_[3] = {};
    double east_[3] = {};
    double north_[3] = {};
    double up_[3] = {};
};

}  // namespace fsw
