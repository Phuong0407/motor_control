#ifndef PID_HPP
#define PID_HPP

#include <algorithm>

class PID {
private:
    double kp, ki, kd;
    double smpl_intv;
    double intgr, prev_err;
    double max_out, max_intgr;

public:
    PID() = default;

    PID(
        double kp,
        double ki,
        double kd,
        double max_out,
        double max_intgr,
        double smpl_intv
    ) :
        kp(kp), ki(ki), kd(kd),
        smpl_intv(smpl_intv),
        max_out(max_out),
        max_intgr(max_intgr),
        intgr(0.0), prev_err(0.0)
    {}

    double compute(double ref, double measured) {
        double err = ref - measured;
        intgr += err * smpl_intv;
        intgr = std::clamp(intgr, -max_intgr, max_intgr);
        double dev = (err - prev_err) / smpl_intv;
        double ctrl_sgnl = kp * err + ki * intgr + kd * dev;
        ctrl_sgnl = std::clamp(ctrl_sgnl, -max_out, max_out);
        prev_err = err;
        return ctrl_sgnl;
    }
};

#endif // PID_HPP