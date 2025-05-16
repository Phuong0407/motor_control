#ifndef MOTORSTATE_HPP
#define MOTORSTATE_HPP

#include <cmath>
#include <algorithm>

static constexpr int MAX_PWM                = 255;
static constexpr double SAFTY_OFFSET        = 0.8;
static constexpr double SCALEUP_FACTOR_PWM  = 0.72549019607;
static constexpr int BACKWARD               = -1;
static constexpr int FORWARD                = +1;
static constexpr int LEFT                   = -1;
static constexpr int RIGHT                  = +1;
static constexpr int STOP                   = 0;
static constexpr double max_rps             = 0.86;
static constexpr int DEAD_PWM               = 70;

int computePWMFromRPS(double u_rps) {
    double norm_rps = std::clamp(u_rps / max_rps, 0.0, 1.0);
    int pwm = MAX_PWM * static_cast<int>(std::round(norm_rps * SAFTY_OFFSET));
    if (pwm < DEAD_PWM) pwm = 0;
    else pwm = static_cast<int>(pwm - DEAD_PWM) / SCALEUP_FACTOR_PWM;
    return pwm;
}

class MotorState{
public:
    int pwm = 0;
    int dir  = FORWARD;
};

#endif // MOTORSTATE_HPP