#ifndef ORIENTATION_PID_HPP
#define ORIENTATION_PID_HPP

#include <chrono>
#include <algorithm>

class OrientationPID {
private:
    double kp;
    double ki;
    double kd;
    double max_output;
    double smpl_intv;

    double prev_err;
    double intgr;

public:
    OrientationPID(
        double kp,
        double ki,
        double kd,
        double max_output,
        double smpl_intv
    ) :
    kp(kp),
    ki(ki),
    kd(kd),
    max_output(max_output),
    smpl_intv(smpl_intv),
    prev_err(0.0),
    intgr(0.0)
    {}

    double compute(double ref, double curr) {
        double error = ref - curr;
        intgr += error * smpl_intv;
        double derivative = (error - prev_err) / smpl_intv;
        double output = (kp * error) + (ki * intgr) + (kd * derivative);
        output = std::clamp(output, -max_output, max_output);
        prev_err = error;
        return output;
    }
};

#endif // ORIENTATION_PID_HPP