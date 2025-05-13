#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include "../include/vision/camera.hpp"
#include "../include/vision/image_postprocessor.hpp"
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
    double kp_vy;                   ///< Proportional gain for lateral velocity correction.
    double kp_omega;                ///< Proportional gain for angular velocity correction.
    double pixel_offset;            ///< Camera offset for calibration.

    Camera camera;                  ///< Camera interface object.
    ImagePostprocessor<4> imager;   ///< Image processing object with 4 slices.
    KinematicModel kinemator;       ///< Kinematic model for robot control.
    MotorDriver motor;              ///< Motor control interface.

    /// Threshold values for line detection and curvature analysis.
    static const double RESIDUAL_THRESHOLD = 3.0;
    static const double CURVATURE_THRESHOLD = 0.05;
    static const double ANGLE_THRESHOLD = 0.2;

    /**
     * @brief Computes the line residual for a given contour and fitted line.
     * @param contour The contour points to evaluate.
     * @param line Fitted line parameters (v, vy, x0, y0).
     * @return Average residual distance of contour points from the fitted line.
     */
    double computeLineResidual(const std::vector<cv::Point>& contour, const cv::Vec4f& line) {
        double v = line[0];
        double vy = line[1];
        double x0 = line[2];
        double y0 = line[3];

        double residual_sum = 0.0;
        for (const auto& pt : contour) {
            double dx = pt.x - x0;
            double dy = pt.y - y0;
            double distance = std::abs(vy * dx - v * dy) / std::sqrt(v * v + vy * vy);
            residual_sum += distance;
        }
        return residual_sum / contour.size();
    }

    /**
     * @brief Computes the curvature of a contour using cubic polynomial fitting.
     * @param contour The contour points to fit and evaluate.
     * @return Curvature value at the midpoint of the contour.
     */
    double computeCurvature(const std::vector<cv::Point>& contour) {
        std::vector<double> X, Y;
        for (const auto& pt : contour) {
            X.push_back(pt.x);
            Y.push_back(pt.y);
        }

        cv::Mat A(X.size(), 4, CV_64F);
        cv::Mat B(Y.size(), 1, CV_64F);

        for (size_t i = 0; i < X.size(); i++) {
            A.at<double>(i, 0) = X[i] * X[i] * X[i];
            A.at<double>(i, 1) = X[i] * X[i];
            A.at<double>(i, 2) = X[i];
            A.at<double>(i, 3) = 1.0;
            B.at<double>(i, 0) = Y[i];
        }

        cv::Mat coeffs;
        cv::solve(A, B, coeffs, cv::DECOMP_SVD);

        double a = coeffs.at<double>(0, 0);
        double b = coeffs.at<double>(1, 0);
        double c = coeffs.at<double>(2, 0);

        double x_mid = X[X.size() / 2];
        double dy = 3 * a * x_mid * x_mid + 2 * b * x_mid + c;
        double ddy = 6 * a * x_mid + 2 * b;

        return std::abs(ddy) / std::pow(1 + dy * dy, 1.5);
    }

    /**
     * @brief Determines if the given contour is approximately a straight line.
     * @param contour The contour to evaluate.
     * @return True if the contour is a straight line, false otherwise.
     */
    bool isStraightLine(const std::vector<cv::Point>& contour) {
        cv::Vec4f line;
        cv::fitLine(contour, line, cv::DIST_L2, 0, 0.01, 0.01);
        double residual = computeLineResidual(contour, line);
        double curvature = computeCurvature(contour);
        return (residual < RESIDUAL_THRESHOLD && curvature < CURVATURE_THRESHOLD);
    }

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
     * @param kp_vy Proportional gain for lateral velocity control.
     * @param kp_omega Proportional gain for angular velocity control.
     */
    Navigation(
        double L1,
        double L2,
        double wheel_radius,
        int pixel_offset,
        int frame_width,
        int frame_height,
        int frame_rate,
        bool verbos,
        double v,
        double kp_vy,
        double kp_omega
    ) :
    v(v),
    kp_vy(kp_vy),
    kp_omega(kp_omega),
    pixel_offset(pixel_offset),
    camera(frame_width, frame_height, frame_rate, verbose),
    kinemator(L1, L2, wheel_radius),
    motor(2.0, 0.01, 0.5, 0.1, 0.7 * 0.5 / 0.1, 0x0f, 0x0d) {}

    /**
     * @brief Executes the line-following behavior using camera input and motor control.
     * 
     * @brief Captures video frames from the camera.
     * @brief Extracts binary mask using image processing.
     * @brief Identifies contours and computes centroids.
     * @brief Computes velocity adjustments based on contour alignment.
     * @brief Applies motor control using kinematic model calculations.
     * @brief Executes the line-following behavior using camera input and motor control.
     * @brief Captures video frames from the camera.
     * @brief Converts the frame to a binary mask.
     * @brief Processes the binary mask to calculate the direction offset.
     * @brief Adjusts `vy` and `omega` based on the calculated direction.
     * @brief Computes motor velocities using kinematic model and applies control signals.
     * @brief Stops the motor and releases resources after the loop.
     * 
     * @param timeout Time in milliseconds to wait for a frame capture.
     */
    void followLine(int timeout) {
        const int width = camera.getVideoWidth();
        const int height = camera.getVideoHeight();
        cv::Mat frame(height, width, CV_8UC3);
        cv::Mat image(height, width, CV_8UC3);
        cv::Mat binaryMask(height, width, CV_8UC1);

        double adjusted_center_x = (width / 2.0) + cam_offset;
        int direction = 0;

        int ch = 0;
        while (ch != 27) {
            if (!camera.captureFrame(image, timeout)) {
                printf("[ERROR] Failed to capture frame.\n");
                continue;
            } if (image.empty()) {
                printf("[ERROR] Empty frame captured.\n");
                continue;
            }

            direction = imager.postProcessImage(binaryMask, image);
            double error = direction - adjusted_center_x;
            double omega = kp_omega * error;
            vy = std::clamp(vy, -1.0, 1.0);
            omega = std::clamp(omega, -1.0, 1.0);

            double omega1, omega2, omega3;
            kinemator.computeWheelVelocityFromRobotVelocity(v, omega, omega1, omega2, omega3);
            motor.controlAngularVelocity(omega1, omega2, omega3, 20.0);
            cv::imshow("PROCESS IMAGE", image);
            ch = cv::waitKey(1);
        }
        cv::destroyAllWindows();
        motor.stop_motor();
        cleanupEncoders();
    }

};

#endif // NAVIGATION_HPP