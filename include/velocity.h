#ifndef VELOCITY_H
#define VELOCITY_H

#include "robot.h"
#include <cmath>

double old_error = 0.0;

inline void calculateOffsetDifference() {
    for (size_t i = 1; i < N_SLICES; i++) {
        dir_offset_diffs[i] = dir_offsets[i] - dir_offsets[i - 1];
    }
}

inline bool detectPerpendicularTurn() {
    for (size_t i = 1; i < N_SLICES; i++) {
        if (std::abs(dir_offset_diffs[i]) >= TURN_THRESHOLD) {
            return true;
        }
    }
    return false;
}

inline double computeDirectionControlSignal(double error) {
    double derivative = error - old_error;
    old_error = error;
    return kp_dir * error + kd_dir * derivative;
}

void computeRobotVelocity() {
    int     num_valid_slice     = 0;
    double  dir_offset_tot      = 0.0;

    for (size_t i = 0; i < N_SLICES; i++) {
        if (contain_lines[i]) {
            dir_offset_tot += static_cast<double>(dir_offsets[i]);
            num_valid_slice++;
        }
    }

    double avg_dir_offset = (num_valid_slice > 0) ? dir_offset_tot / static_cast<double>(num_valid_slice) : 0.0;
    omega = computeDirectionControlSignal(avg_dir_offset);
    printf("omega = %.3f", omega);
    if (detectPerpendicularTurn()) {
        base_speed  *= TURN_SPEED_DECREASE;
        omega       *= TURN_SPEED_INCREASE;
    }
}

void computeRefTPSFromVelocity() {
    ref1 = (1.0 / r) * (base_speed - L1 * omega);
    ref2 = (1.0 / r) * (base_speed + L1 * omega);
    ref3 = (1.0 / r) * omega * L2;
}

void * computeRefTPSFromVision(void * arg) {
    computeRobotVelocity();
    computeRefTPSFromVelocity();
    return nullptr;
}

#endif // VELOCITY_H