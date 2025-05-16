#ifndef PID_HPP
#define PID_HPP

#include <algorithm>

constexpr double max_out = 1.0;
constexpr double max_intgr = 80.0;

class PID {
private:
    double kp, ki, kd;
    double smpl_intv;
    double intgr, prev_err;

public:
    PID() = default;

    PID(
        double kp,
        double ki,
        double kd,
        double smpl_intv
    ) :
    kp(kp), ki(ki), kd(kd),
    smpl_intv(smpl_intv),
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