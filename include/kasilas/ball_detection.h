#ifndef BALL_DETECTION_H
#define BALL_DETECTION_H

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <chrono>
#include <vector>
#include <algorithm>

using Contour_t     = std::vector<cv::Point>;
using Contours_t    = std::vector<Contour_t>;

const cv::Scalar LOWER_RED_1(0,     120, 70);
const cv::Scalar UPPER_RED_1(10,    255, 255);
const cv::Scalar LOWER_RED_2(170,   120, 70);
const cv::Scalar UPPER_RED_2(180,   255, 255);

constexpr double    PIXELS_PER_CM_X     = 10.0;
constexpr double    PIXELS_PER_CM_Y     = 10.0;
constexpr double    INTERVAL            = 0.1;



cv::Mat             img;
cv::Mat             bin_mask;

bool                found_sphere        = false;
int                 center_x            = 0;
int                 center_y            = 0;
int                 img_center_x        = 0;
int                 img_center_y        = 0;

Contours_t          contours;



cv::Mat extractRedMask(const cv::Mat& image) {
    cv::Mat hsvImage, mask1, mask2, bin_mask;
    cv::cvtColor(image,     hsvImage,       cv::COLOR_BGR2HSV);
    cv::inRange(hsvImage,   LOWER_RED_1,    UPPER_RED_1, mask1);
    cv::inRange(hsvImage,   LOWER_RED_2,    UPPER_RED_2, mask2);
    bin_mask = mask1 | mask2;
    return bin_mask;
}


void findLargestContourCenter(const cv::Mat& mask) {
    contours.clear();
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) { 
        found_sphere = false;
        return;
    }

    auto max_contour = *std::max_element(contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
            return cv::contourArea(c1) < cv::contourArea(c2);
        });

    cv::Moments M = cv::moments(max_contour);
    if (M.m00 == 0) {
        found_sphere = false;
        return;
    } else {
        center = cv::Point2f(static_cast<float>(M.m10 / M.m00), static_cast<float>(M.m01 / M.m00));
    }
}








#endif // BALL_DETECTION_H