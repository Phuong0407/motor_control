#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include "vision.hpp"
#include "orientation_pid.hpp"
#include "kinematic_model.hpp"
#include "../motor/motor.hpp"

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>

class Navigation{
private:
    double vx;
    double cam_offset;
    double kp_vy, kp_omega;
    
    Vision vision;
    OrientationPID orientation_y;
    OrientationPID orientation_omega;
    KinematicModel kinemator;
    MotorDriver motor;

public:
    Navigation(
        double vx,
        double kp_vy,
        double kp_omega,
        double L1,
        double L2,
        double r,
        double cam_offset
    ) :
    vx(vx),
    cam_offset(cam_offset),
    orientation_y(kp_vy, 0.0, 0.0, 10.0, 2.0),
    orientation_omega(kp_omega, 0.0, 0.0, 10.0, 2.0),
    kinemator(L1, L2, r),
    motor(2.0, 0.01, 0.5, 0.1, 0.7 * 0.5 / 0.1, 0x0f, 0x0d)
    { }

    void Navigation::updateNavigation(const cv::Mat& frame) {
        double contourX = vision.
        std::vector<cv::Point2f> deviationPoints = vision.getDeviationPoints(frame);

        cv::Point2f avgDeviation(0.0f, 0.0f);
        for (const auto& point : deviationPoints) {
            avgDeviation += point;
        }
        avgDeviation.x /= deviationPoints.size();
        avgDeviation.y /= deviationPoints.size();
    
        float vy = -kp_vy * avgDeviation.x;
        float vx = base_speed;

        float omega = -kp_omega * avgDeviation.x;
    
        double omega1, omega2, omega3;
        kinematic.computeWheelVelocityFromRobotVelocity(vx, vy, omega, omega1, omega2, omega3);
    
        motor.setTargetSpeed(0, omega1);
        motor.setTargetSpeed(1, omega2);
        motor.setTargetSpeed(2, omega3);
    }
};


#endif // NAVIGATION_HPP 