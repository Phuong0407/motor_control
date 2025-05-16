#ifndef PID_HPP
#define PID_HPP

#include <cmath>
#include <algorithm>

class PID {
public:
    PID() = default;

    PID(
        double kp,
        double ki,
        double kd,
        double cutoff_freq,
        double max_out
    ) :
    kp(kp),
    ki(ki),
    kd(kd),
    max_out(max_out)
    { 
        setEMACutoff(cutoff_freq);
    }

    double compute(double input) {
        double error = ref - input;
        intgr += error * smpl_itv;
        intgr = std::clamp(intgr, -max_out, max_out);
        double dev = EMAFltCoeff * (prev_input - input);
        prev_input -= dev;
        double output = kp * error + ki * intgr + kd * dev;
        output = std::clamp(output, -max_out, max_out);
        prev_input = input;
        return output;
    }

    void setKp(double kp) { this->kp = kp; }
    void setKi(double ki) { this->ki = ki * smpl_itv; }
    void setKd(double kd) { this->kd = kd / smpl_itv; }

    double getKp() const { return kp; }
    double getKi() const { return ki / smpl_itv; }
    double getKd() const { return kd * smpl_itv; }

    void setEMACutoff(double cutoff_freq) {
        if (cutoff_freq <= 0.0) {
            EMAFltCoeff = 1.0;
        } else {
            double normalizedFreq = cutoff_freq * smpl_itv;
            EMAFltCoeff = 1.0 / (1.0 + normalizedFreq);
        }
    }

    void setSetpoint(double ref) { this->ref = ref;
    }

    double getSetpoint() const { return ref; }

    void resetIntegral() { intgr = 0.0; }

    void setMaxOutput(double max_out) {
        this->max_out = max_out;
    }

    double getMaxOutput() const { return max_out; }

private:
    double kp = 1.0;
    double ki = 0.0;
    double kd = 0.0;
    double max_out = 0.0;
    double EMAFltCoeff = 1.0;
    double prev_input = 0.0;
    double intgr = 0.0;
    double ref = 0.0;
};

#endif // PID_HPP