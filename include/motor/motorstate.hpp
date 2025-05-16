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
private:
    int pwm;
    int dir;
    double ref;
    double measure;
    double compute;

public:
    MotorState() : pwm(0), dir(0), ref(0.0), measure(0.0), compute(0.0) {}

    inline void setPWM(int pwm) { this->pwm = pwm; }
    inline void setDir(int dir) { this->dir = dir; }
    inline void setRef(double ref) { this->ref = ref; }
    inline void setMeasure(double measure) { this->measure = measure; }
    inline void setCompute(double compute) { this->compute = compute; }

    inline int getPWM() const { return pwm; }
    inline int getDir() const { return dir; }
    inline double getRef() const { return ref; }
    inline double getMeasure() const { return measure; }
    inline double getCompute() const { return compute; }
};

#endif // MOTORSTATE_HPP