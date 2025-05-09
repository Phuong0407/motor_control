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
    // OrientationPID orientation_y;
    // OrientationPID orientation_omega;
    KinematicModel kinemator;
    MotorDriver motor;

public:
    Navigation(
        double vx,
        double kp_vy,
        // double kp_omega,
        double L1,
        double L2,
        double r,
        double cam_offset
    ) :
    vx(vx),
    cam_offset(cam_offset),
    kp_vy(kp_vy),
    kp_omega(0.0),
    // orientation_y(kp_vy, 0.0, 0.0, 10.0, 2.0),
    // orientation_omega(kp_omega, 0.0, 0.0, 10.0, 2.0),
    kinemator(L1, L2, r),
    motor(2.0, 0.01, 0.5, 0.1, 0.7 * 0.5 / 0.1, 0x0f, 0x0d)
    {}

    void navigate(const cv::Mat& frame) {
        cv::Mat image2(480, 640, CV_8UC3);
        vision.getOutputVision(image1, image2);
        double contourX = vision.getCentroidXFirstSlices();

        double vy = kp_vy * (contourX - cam_offset);
        double vx = base_speed;

        double omega1, omega2, omega3;
        kinematic.computeWheelVelocityFromRobotVelocity(vx, vy, 0.0, omega1, omega2, omega3);
        motor.controlAngularVelocity(omega1, omega2, omega3);
    }
};


#endif // NAVIGATION_HPP 