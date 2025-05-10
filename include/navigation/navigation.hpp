#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include "vision.hpp"
#include "kinematic_model.hpp"
#include "../motor/motor.hpp"

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>
#include <numeric>

class Navigation {
private:
    double vx;
    double kp_vy;
    double kp_omega;
    double cam_offset;

    Vision vision;
    KinematicModel kinemator;
    MotorDriver motor;

public:
    Navigation(
        double L1,
        double L2,
        double whl_r,
        double cam_offset,
        
        double vx,
        double kp_vy,
        double kp_omega
    ) :
    vx(vx),
    kp_vy(kp_vy),
    kp_omega(kp_omega),
    cam_offset(cam_offset),
    kinemator(L1, L2, whl_r),
    motor(2.0, 0.01, 0.5, 0.1, 0.7 * 0.5 / 0.1, 0x0f, 0x0d)
    {}

    void followLine() {
        lccv::PiCamera cam;
        cam.options->video_width = 640;
        cam.options->video_height = 480;
        cam.options->framerate = 30;
        cam.startVideo();

        cv::Mat frame(480, 640, CV_8UC3);
        int ch = 0;
        while (ch != 27) {
            cam.getVideoFrame(frame, 1000);
            double avg_centroids = vision.getCentroids();
            cv::imshow("Processed Frame", frame);
            ch = cv::waitKey(5);
            
            double y_error = avg_centroid_x - cam_offset;
            double vy = kp_vy * y_error;
            double omega = kp_omega * y_error;

            vy = std::clamp(vy, -1.0, 1.0);
            omega = std::clamp(omega, -1.0, 1.0);

            double omega1, omega2, omega3;
            kinemator.computeWheelVelocityFromRobotVelocity(vx, vy, omega, omega1, omega2, omega3);
            motor.controlAngularVelocity(omega1, omega2, omega3, 20.0);
        }
        motor.stop_motor();
        cleanupEncoders();
    }
};

#endif // NAVIGATION_HPP