#ifndef PID_HPP
#define PID_HPP

#include "config.h"
#include <algorithm>

constexpr double max_out = 1.0;
constexpr double max_intgr = 80.0;

class PID {
private:
    double kp, ki, kd;
    double intgr, prev_err;

public:
    PID() = default;

    PID(
        double kp,
        double ki,
        double kd
    ) :
    kp(kp), ki(ki), kd(kd),
    intgr(0.0), prev_err(0.0)
    {}

    void setUpPIDParameters(
        double kp,
        double ki,
        double kd
    ) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->intgr = 0.0;
        this->prev_err = 0.0;
    }

    double compute(double err) {
        intgr += err * smpl_itv;
        intgr = std::clamp(intgr, -max_intgr, max_intgr);
        double dev = (err - prev_err) / smpl_itv;
        double ctrl_sgnl = kp * err + ki * intgr + kd * dev;
        ctrl_sgnl = std::clamp(ctrl_sgnl, -max_out, max_out);
        prev_err = err;
        return ctrl_sgnl;
    }

    void reset() {
        intgr = 0.0;
        prev_err = 0.0;
    }
};

#endif // PID_HPP