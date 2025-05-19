#ifndef PD_H
#define PD_H

#include <cmath>

class PDController {
public:
    PDController(float kp, float kd);
    float calculate(float error);

private:
    float Kp;
    float Kd;
    float previous_error;
};

PDController::PDController(float kp, float kd) : Kp(kp), Kd(kd), previous_error(0.0f) {}

float PDController::calculate(float error) {
    float derivative = error - previous_error;
    previous_error = error;

    float control_signal = (Kp * error) + (Kd * derivative);

    return control_signal;
}

#endif // PD_H