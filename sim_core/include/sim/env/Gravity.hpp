#pragma once

#include <Eigen/Core>

namespace sim::env {

constexpr double kStandardGravity = 9.80665;  // [m/s^2]

// Placeholder: uniform gravity in a flat local ENU frame (z up).
// Fine for short hops; replaced by point mass + J2 + EGM2008 in WGS-84 frames
// (phase 0, items 2 and 3) before any trajectory that goes downrange.
class UniformGravity {
public:
    Eigen::Vector3d acceleration(const Eigen::Vector3d& position) const {
        (void)position;
        return {0.0, 0.0, -kStandardGravity};
    }
};

}  // namespace sim::env
