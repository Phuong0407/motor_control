#ifndef PID_HPP
#define PID_HPP

#include <cmath>
#include <algorithm>

class PID {
private:
    double kp, ki, kd, Alpha, smpl_intv;
    double intgr, prev_fltrerr;
    double max_out, max_intgr = 80.0;

    static double calcAlphaEMA(double cutoff_freq) {
        if (cutoff_freq <= 0.0)
            return 1.0;
        const double Alpha_cutoff = std::cos(2 * M_PI * cutoff_freq);
        return Alpha_cutoff - 1 + std::sqrt(Alpha_cutoff * Alpha_cutoff - 4 * Alpha_cutoff + 3);
    }

public:
PID(
        double kp,
        double ki,
        double kd,
        double max_out,
        double cutoff_freq,
        double smpl_intv
    ) :
    kp(kp), ki(ki), kd(kd),
    Alpha(calcAlphaEMA(cutoff_freq * smpl_intv)),
    smpl_intv(smpl_intv),
    intgr(0.0), prev_fltrerr(0.0),
    max_out(max_out)
    {}

    double compute(double ref, double measured) {
        double err = ref - measured;
        double fltrerr = Alpha * err + (1.0 - Alpha) * prev_fltrerr;
        double dev = (fltrerr - prev_fltrerr) / smpl_intv;
        double new_intgr = intgr + err * smpl_intv;
        intgr = std::clamp(new_intgr, -max_intgr, max_intgr);
        double ctrl_sgnl = kp * err + ki * intgr + kd * dev;
        ctrl_sgnl = std::clamp(ctrl_sgnl, -max_out, max_out);
        prev_fltrerr = fltrerr;
        return ctrl_sgnl;
    }
};

#endif // PID_HPP
