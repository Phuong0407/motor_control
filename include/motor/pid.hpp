#ifndef PID_HPP
#define PID_HPP

#include <cmath>
#include <algorithm>

class PID {
private:
    double kp               = 0.0;
    double ki               = 0.0;
    double kd               = 0.0;
    double alpha            = 0.0;
    double max_out          = 0.0;
    double intgr            = 0.0;
    double old_filter_err   = 0.0;

    inline double computeAlphaEMA(double cutoff_freq) {
        if (cutoff_freq <= 0)
            return 1.0;
        double alpha = std::cos(2.0 * float(M_PI) * cutoff_freq);
        return alpha - 1.0 + std::sqrt(alpha * alpha - 4.0 * alpha + 3.0);
    }

    inline void setEMACutoff(double cutoff_freq) {
        double normalizedFreq = cutoff_freq * 0.10;
        alpha = cutoff_freq == 0.0 ? 1.0 : computeAlphaEMA(cutoff_freq);
    }

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

    inline void resetPID() { intgr = 0.0; old_filter_err = 0.0; }

    double compute(double ref, double measured) {
        double err = ref - measured;
        double filter_err = alpha * err + (1 - alpha) * old_filter_err;
        double filter_dev = (filter_err - old_filter_err) / 0.10;
        double new_intgr = intgr + err * 0.10;
        double ctrl_sgnl = kp * err + ki * new_intgr + kd * filter_dev;

        if (ctrl_sgnl > max_out)        ctrl_sgnl = max_out;
        else if (ctrl_sgnl < -max_out)  ctrl_sgnl = -max_out;
        else                            intgr = new_intgr;
        
        old_filter_err = filter_err;
        return ctrl_sgnl;
    }
};

#endif // PID_HPP