#ifndef PID_HPP
#define PID_HPP

#include "config.h"

#include <cmath>
#include <algorithm>

class PID {
private:
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double intgr = 0.0;
    double prev_err = 0.0;
    double ema_coef = 0.0;
    double max_out = 0.0;

public:
    PID() = default;

    PID(double kp, double ki, double kd, double cutoff_freq, double max_out) 
        : kp(kp), ki(ki), kd(kd), max_out(max_out)
    {
        this->ki = ki;
        this->kd = kd;
        setEMACutoff(cutoff_freq);
    }


    void setPIDParameters(double kp, double ki, double kd, double cutoff_freq, double max_out) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->max_out = max_out;
        setEMACutoff(cutoff_freq);
    }

    double compute(double input) {
        double err = ref - input;
        double new_intgr = intgr + err * smpl_itv;
        double dev = EMAFltCoeff * (prev_input - input);
        prev_input = input;

        double output = kp * err + ki * new_intgr + kd * dev;

        output = std::clamp(output, -max_out, max_out);
        if (std::abs(output) != max_out) intgr = new_intgr;
        return output;
    }

    inline void setEMACutoff(double cutoff_freq) {
        if (cutoff_freq <= 0.0) {
            EMAFltCoeff = 1.0;
        } else {
            double normalizedFreq = cutoff_freq * smpl_itv;
            EMAFltCoeff = 1.0 / (1.0 + normalizedFreq);
        }
    }

    inline void setRef(double ref) { this->ref = ref; }
    inline void resetIntegral() { intgr = 0.0; }
};

#endif // PID_HPP