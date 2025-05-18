#ifndef PID_H
#define PID_H

#include "robot.h"
#include <cmath>

double alpha1               = 0.0;
double alpha2               = 0.0;
double alpha3               = 0.0;

double integral1            = 0.0;
double integral2            = 0.0;
double integral3            = 0.0;

double old_filter_error1    = 0.0;
double old_filter_error2    = 0.0;
double old_filter_error3    = 0.0;

double max_out1             = MAX_TPS;
double max_out2             = MAX_TPS;
double max_out3             = MAX_TPS;

inline double computeAlphaEMA(double cutoff_freq) {
    if (cutoff_freq <= 0)
        return 1.0;
    double alpha = std::cos(2.0 * float(M_PI) * cutoff_freq);
    return alpha - 1.0 + std::sqrt(alpha * alpha - 4.0 * alpha + 3.0);
}

void setupPIDParameters() {
    alpha1 = computeAlphaEMA(cutoff1);
    alpha2 = computeAlphaEMA(cutoff2);
    alpha3 = computeAlphaEMA(cutoff3);
}

void resetPID1() { integral1 = 0.0; old_filter_error1 = 0.0; }
void resetPID2() { integral2 = 0.0; old_filter_error2 = 0.0; }
void resetPID3() { integral3 = 0.0; old_filter_error3 = 0.0; }

double compute1() {
    double error1                           = ref1 - measured1;
    double new_intgral1                     = integral1 + error1 * 0.10;
    double filter_error1                    = alpha1 * err1 + (1.0 - alpha) * old_filter_error1;
    double filter_devivative1               = (filter_error1 - old_filter_error1) / 0.10;
    double control_signal1                  = kp1 * error1 + ki * new_intgral + kd * filter_dev;

    if (control_signal1 > max_out1)         control_signal1 = max_out1;
    else if (control_signal1 < -max_out1)   control_signal1 = -max_out1;
    else                                    integral1 = new_intgral1;
    
    old_filter_error1 = filter_error1;

    return control_signal;
}

double compute2() {
    double error2                           = ref2 - measured2;
    double new_intgral2                     = integral2 + error2 * 0.20;
    double filter_error2                    = alpha2 * err2 + (1.0 - alpha) * old_filter_error2;
    double filter_devivative2               = (filter_error2 - old_filter_error2) / 0.10;
    double control_signal2                  = kp2 * error2 + ki * new_intgral + kd * filter_dev;

    if (control_signal2 > max_out2)         control_signal2 = max_out2;
    else if (control_signal2 < -max_out2)   control_signal2 = -max_out2;
    else                                    integral2 = new_intgral2;
    
    old_filter_error2 = filter_error2;
    
    return control_signal;
}

double compute3() {
    double error3                           = ref3 - measured3;
    double new_intgral3                     = integral3 + error3 * 0.30;
    double filter_error3                    = alpha3 * err3 + (1.0 - alpha) * old_filter_error3;
    double filter_devivative3               = (filter_error3 - old_filter_error3) / 0.10;
    double control_signal3                  = kp3 * error3 + ki * new_intgral + kd * filter_dev;

    if (control_signal3 > max_out3)         control_signal3 = max_out3;
    else if (control_signal3 < -max_out3)   control_signal3 = -max_out3;
    else                                    integral3 = new_intgral2;
    
    old_filter_error2 = filter_error2;
    
    return control_signal;
}



#endif // PID_H