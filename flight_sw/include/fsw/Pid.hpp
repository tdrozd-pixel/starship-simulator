#pragma once

namespace fsw {

struct PidGains {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double outputMin = -1e300;
    double outputMax = 1e300;
    double integratorLimit = 1e300;  // clamp on |ki * integral|
};

// PID with derivative on measurement (no kick on setpoint steps) and
// conditional-integration anti-windup (stops integrating while saturated).
class Pid {
public:
    explicit Pid(PidGains gains) : gains_(gains) {}

    double update(double setpoint, double measurement, double dt);
    void reset();

    const PidGains& gains() const { return gains_; }
    void setGains(const PidGains& gains) { gains_ = gains; }

    double lastP() const { return p_; }
    double lastI() const { return i_; }
    double lastD() const { return d_; }

private:
    PidGains gains_;
    double integral_ = 0.0;
    double lastMeasurement_ = 0.0;
    bool hasLast_ = false;
    double p_ = 0.0;
    double i_ = 0.0;
    double d_ = 0.0;
};

}  // namespace fsw
