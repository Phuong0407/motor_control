#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include "camera.hpp"
#include "image_processor.hpp"
#include "kinematic_model.hpp"
#include "../motor/motor.hpp"

#include <numeric>
#include <vector>
#include <stdio.h>

/**
 * @class Navigation
 * @brief Implements navigation control logic using camera input, image processing, and motor control.
 */
class Navigation {
private:
    double v;                      ///< Linear velocity in the X direction.
    double kp_omega;                ///< Proportional gain for angular velocity correction.

    Camera camera;                  ///< Camera interface object.
    ImageProcessor<4> imager;       ///< Image processing object with 4 slices.
    KinematicModel kinemator;       ///< Kinematic model for robot control.

public:
    /**
     * @brief Constructor for the Navigation class.
     * @brief Initializes the kinematic model, motor driver, and other parameters.
     * @param L1 Distance from the center of the robot to the left wheel.
     * @param L2 Distance from the center of the robot to the right wheel.
     * @param wheel_radius Radius of the wheels.
     * @param pixel_offset Offset of the camera from the robot's center axis.
     *        pixel_offset = (cam_offset_cm*focal_length_px​)/distance_to_ground
     * @param v Desired linear velocity.
     * @param kp_omega Proportional gain for angular velocity control.
     */
    Navigation(
        double L1,
        double L2,
        double wheel_radius,
        int frame_width,
        int frame_height,
        int frame_rate,
        bool verbose,
        double v,
        double kp_omega
    ) :
    v(v),
    kp_omega(kp_omega),
    camera(frame_width, frame_height, frame_rate, verbose),
    kinemator(L1, L2, wheel_radius) {
        startEncoders();
    }

    /**
     * @brief Executes the line-following behavior using camera input and motor control.
     * 
     * @brief Captures video frames from the camera.
     * @brief Extracts binary mask using image processing.
     * @brief Identifies contours and computes centroids.
     * @brief Applies motor control using kinematic model calculations.
     * @brief Executes the line-following behavior using camera input and motor control.
     * @brief Captures video frames from the camera.
     * @brief Converts the frame to a binary mask.
     * @brief Processes the binary mask to calculate the direction offset.
     * @brief Adjusts `omega` based on the calculated direction.
     * @brief Computes motor velocities using kinematic model and applies control signals.
     * @brief Stops the motor and releases resources after the loop.
     * 
     * @param timeout Time in milliseconds to wait for a frame capture.
     */
    void followLine() {
        const int width = camera.getVideoWidth();
        const int height = camera.getVideoHeight();
        cv::Mat frame(height, width, CV_8UC3);
        cv::Mat image(height, width, CV_8UC3);
        cv::Mat binaryMask(height, width, CV_8UC1);

        int direction = 0;
        int ch = 0;
        while (ch != 27) {
            if (!camera.captureFrame(frame, 1000)) {
                printf("[ERROR] Failed to capture frame.\n");
                continue;
            } if (frame.empty()) {
                printf("[ERROR] Empty frame captured.\n");
                continue;
            }

            printf("\nFUCK YOU\n");

            direction = imager.postProcessImage(binaryMask, frame);
            double omega = kp_omega * direction;
            omega = std::clamp(omega, -1.0, 1.0);

            double omega1, omega2, omega3;
            kinemator.computeWheelVelocityFromRobotVelocity(v, omega, omega1, omega2, omega3);
            controlAngularVelocity(omega1, omega2, omega3);
            cv::imshow("PROCESS IMAGE", image);
            ch = cv::waitKey(1);
        }
        cv::destroyAllWindows();
        stop_motor();
    }

};

#endif // NAVIGATION_HPP