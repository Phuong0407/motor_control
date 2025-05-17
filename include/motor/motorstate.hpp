#ifndef MOTORSTATE_HPP
#define MOTORSTATE_HPP

#include <cmath>
#include <algorithm>
#include <iostream>

static constexpr int MAX_PWM                = 255;
static constexpr double SAFETY_OFFSET       = 0.7;
static constexpr double SCALEUP_FACTOR_PWM  = 0.72549019607;
static constexpr int BACKWARD               = -1;
static constexpr int FORWARD                = +1;
static constexpr int LEFT                   = -1;
static constexpr int RIGHT                  = +1;
static constexpr int STOP                   = 0;
static constexpr double MAX_RPS             = 0.86;
static constexpr int DEAD_PWM               = 70;

class MotorState {
public:
    int pwm = 0;
    int dir = FORWARD;

    static int computePWMFromRPS(double u_rps) {
        double norm_rps = std::clamp(u_rps / MAX_RPS, 0.0, 1.0);
        int pwm_value = static_cast<int>(MAX_PWM * norm_rps * SAFETY_OFFSET);

        if (pwm_value < DEAD_PWM) return 0;
        return static_cast<int>((pwm_value - DEAD_PWM) / SCALEUP_FACTOR_PWM);
    }
    
public:
    MotorState() = default;
    MotorState(int pwm, int dir) { setMotorState(pwm, dir);}
    
    void setMotorState(int pwm, int dir) {
        this->pwm = std::clamp(pwm, 0, MAX_PWM);
        this->dir = dir;        
    }

    void setMotorStateRPS(double rps) {
        if(rps == 0.0) {
            setMotorState(0, STOP);
            return;
        }
        dir = (rps > 0) ? FORWARD : BACKWARD;
        pwm = computePWMFromRPS(rps);
    }

    void stop() { pwm = 0; }
    void reverse() { dir = (dir == FORWARD) ? BACKWARD : FORWARD; }

    inline int getPWM() const { return pwm; }
    inline int getDirection() const { return dir; }
};

#endif // MOTORSTATE_HPP