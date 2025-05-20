#ifndef BALL_DETECTION_H
#define BALL_DETECTION_H

#include <opencv2/opencv.hpp>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <vector>
#include <algorithm>

const cv::Scalar    LOWER_RED_1(0,     120, 70);
const cv::Scalar    UPPER_RED_1(10,    255, 255);
const cv::Scalar    LOWER_RED_2(170,   120, 70);
const cv::Scalar    UPPER_RED_2(180,   255, 255);

static constexpr int    frameheight     = 480;
static constexpr int    framewidth      = 640;

cv::Mat                 bin_mask;

bool                    found_sphere    = false;
double                  center_x        = 0.0;
double                  center_y        = 0.0;

static constexpr double img_center_x    = static_cast<double>(framewidth) / 2.0;
static constexpr double img_center_y    = static_cast<double>(frameheight) - 1.0;

using Contour_t                         = std::vector<cv::Point>;
using Contours_t                        = std::vector<Contour_t>;

Contours_t              contours;

double rho                              = 0.0;
double alpha                            = 0.0;
double base_speed                       = 0.0;
double omega                            = 0.0;

constexpr double k_rho                  = 10.0;
constexpr double k_omega                = 10.0;
constexpr double MAX_SPEED              = 5.0;
constexpr double MAX_OMEGA              = 1.0;

void extractRedMask() {
    cv::Mat hsvImage, mask1, mask2;
    cv::cvtColor(img,       hsvImage,       cv::COLOR_BGR2HSV);
    cv::inRange(hsvImage,   LOWER_RED_1,    UPPER_RED_1, mask1);
    cv::inRange(hsvImage,   LOWER_RED_2,    UPPER_RED_2, mask2);
    bin_mask = mask1 | mask2;
}

void findLargestContourCenter() {
    contours.clear();
    cv::findContours(bin_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) { 
        found_sphere = false;
        return;
    }

    auto max_contour = *std::max_element(contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
            return cv::contourArea(c1) < cv::contourArea(c2);
        });

    cv::Moments M = cv::moments(max_contour);
    if (M.m00 == 0.0) {
        found_sphere = false;
        return;
    } else {
        center_x = M.m10 / M.m00;
        center_y = M.m01 / M.m00;
    }
}

inline void computeRhoAndAlpha() {
    double dx   = static_cast<double>(center_x - img_center_x);
    double dy   = static_cast<double>(center_y - img_center_y);
    rho         = std::sqrt(dx * dx + dy * dy);
    alpha       = std::atan2(dy, dx);
}

inline void computeRobotVelocity() {
    base_speed  = k_rho * rho;
    omega       = k_omega * alpha;
}

void processFrame() {
    extractRedMask();
    findLargestContourCenter();

    if (found_sphere) {
        computeRhoAndAlpha();
        computeRobotVelocity();
    } else {
        rho         = 0.0;
        alpha       = 0.0;
        base_speed  = 0.0;
        omega       = 0.0;
    }
}

#endif // BALL_DETECTION_H