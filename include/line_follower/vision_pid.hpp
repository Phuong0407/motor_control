#ifndef VISION_PID_HPP
#define VISION_PID_HPP

#include <cmath>
#include <algorithm>

class VisionPID {
private:
    double kp, ki, kd;

public:
    VisionPID(
        double kp,
        double ki,
        double kd,
    ) {}

    double compute(double ref, double measured) {
        
    }
};

#endif // VISION_PID_HPP