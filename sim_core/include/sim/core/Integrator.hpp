#pragma once

namespace sim {

// Classic fixed-step 4th-order Runge-Kutta.
//
// Requirements on the types (found by ADL):
//   Deriv  f(const State&, double t)            derivative function
//   State  addScaled(const State&, const Deriv&, double h)   x + h * dx
//   Deriv  operator+(const Deriv&, const Deriv&)
//   Deriv  operator*(double, const Deriv&)
//
// State and Deriv are separate types so that, for example, a quaternion can be
// renormalized inside addScaled.
template <typename State, typename DerivFn>
State rk4Step(const State& x, double t, double h, DerivFn&& f) {
    const auto k1 = f(x, t);
    const auto k2 = f(addScaled(x, k1, 0.5 * h), t + 0.5 * h);
    const auto k3 = f(addScaled(x, k2, 0.5 * h), t + 0.5 * h);
    const auto k4 = f(addScaled(x, k3, h), t + h);
    const auto sum = k1 + 2.0 * k2 + 2.0 * k3 + k4;
    return addScaled(x, sum, h / 6.0);
}

}  // namespace sim
