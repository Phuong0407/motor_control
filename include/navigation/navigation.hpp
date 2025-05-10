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
    const double RESIDUAL_THRESHOLD = 3.0;
    const double CURVATURE_THRESHOLD = 0.05;
    const double ANGLE_THRESHOLD = 0.2;

    double calculateLineResidual(const std::vector<cv::Point>& contour, const cv::Vec4f& line) {
        double vx = line[0];
        double vy = line[1];
        double x0 = line[2];
        double y0 = line[3];

        double residual_sum = 0.0;

        for (const auto& pt : contour) {
            double dx = pt.x - x0;
            double dy = pt.y - y0;
            double distance = std::abs(vy * dx - vx * dy) / std::sqrt(vx * vx + vy * vy);
            residual_sum += distance;
        }
        return residual_sum / contour.size();
    }

    /**
     * @brief CUBIC Polynomial Fitting Algorithm
     */
    double calculateCurvature(const std::vector<cv::Point>& contour) {
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

    bool isStraightLine(const std::vector<cv::Point>& contour) {
        cv::Vec4f line;
        cv::fitLine(contour, line, cv::DIST_L2, 0, 0.01, 0.01);

        double residual = calculateLineResidual(contour, line);
        double curvature = calculateCurvature(contour);

        return (residual < RESIDUAL_THRESHOLD && curvature < CURVATURE_THRESHOLD);
    }

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

        cv::Mat frame, binary;
        int ch = 0;

        while (ch != 27) {
            cam.getVideoFrame(frame, 1000);
            vision.getBinaryMask(frame, binary);

            std::vector<std::vector<cv::Point>> contours = vision.getCentroidPoints();
            if (centroids.empty()) continue;

            bool is_straight = isStraightLine(main_contour);
            double avg_centroid_x = 0.0;
            for (const auto& pt : centroids)
                avg_centroid_x += pt.x;
            avg_centroid_x /= centroids.size();
            
            double vy = kp_vy * y_error;
            double omega = kp_omega * y_error;

            if (!is_straight) {
                double curvature = calculateCurvature(main_contour);
                omega += 0.5 * curvature;
            }

            vy = std::clamp(vy, -1.0, 1.0);
            omega = std::clamp(omega, -1.0, 1.0);

            double omega1, omega2, omega3;
            kinematics.computeWheelVelocityFromRobotVelocity(vx, vy, omega, omega1, omega2, omega3);
            motor.controlAngularVelocity(omega1, omega2, omega3, 20.0);

            cv::imshow("Processed Frame", binary);
            ch = cv::waitKey(5);
        }
        motor.stop_motor();
        cleanupEncoders();
    }
};

#endif // NAVIGATION_HPP
