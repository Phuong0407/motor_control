#ifndef PID_HPP
#define PID_HPP

#include "config.h"
#include <algorithm>

class PID {
private:
    double max_out;
    double kp, ki, kd;
    double intgr, prev_err;
    double max_intgr;

public:
    PID() = default;

    PID(
        double kp,
        double ki,
        double kd,
        double max_out
    ) :
    kp(kp), ki(ki), kd(kd),
    intgr(0.0), prev_err(0.0),
    max_out(max_out), max_intgr(max_out / smpl_itv)
    {}

    void setUpPIDParameters(
        double kp,
        double ki,
        double kd,
        double max_out
    ) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->intgr = 0.0;
        this->prev_err = 0.0;
        this->max_out = max_out;
        this->max_intgr = max_out / smpl_itv;
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