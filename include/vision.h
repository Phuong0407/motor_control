#ifndef VISION_H
#define VISION_H

#include "robot.h"
#include <vector>
#include <stdio.h>
#include <limits>
#include <cmath>

constexpr double        MIN_CONTOUR_AREA            = 100.0;
constexpr double        MAX_EXTENT_RATIO            = 0.7;
constexpr int           CONTOUR_OFFSET_THRESHOLD    = 5;
const     cv::Scalar    CONTOUR_COLOR               = cv::Scalar(0, 255, 0);
const     cv::Scalar    IMAGE_CENTER_COLOR          = cv::Scalar(0, 0, 255);
const     cv::Scalar    TEXT_COLOR                  = cv::Scalar(200, 0, 200);
constexpr int           MARKER_RADIUS               = 5;
constexpr int           TEXT_OFFSET_Y               = 30;
constexpr int           FOUND_LINE                  = 1;
constexpr int           NO_LINE_FOUND               = std::numeric_limits<int>::max();

constexpr int           N_SLICES                    = 5;
int                     contour_center_x1           = 0;
int                     contour_center_x2           = 0;
int                     contour_center_x3           = 0;
int                     contour_center_x4           = 0;
int                     contour_center_x5           = 0;

// inline cv::Mat extrackBinMask(const cv::Mat& img) {
//     cv::Mat img_hsv, red1, red2, blue, bin_mask;
//     cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

//     cv::inRange(img_hsv, cv::Scalar(0, 120, 70),    cv::Scalar(10, 255, 255),   red1);
//     cv::inRange(img_hsv, cv::Scalar(170, 120, 70),  cv::Scalar(180, 255, 255),  red2);
//     cv::inRange(img_hsv, cv::Scalar(100, 150, 50),  cv::Scalar(140, 255, 255),  blue);

//     return red1 | red2 | blue;
// }

// inline cv::Point computeCenter(const std::vector<cv::Point>& contour) const {
//     if (contour.empty()) return cv::Point(0, 0);
//     cv::Moments moments = cv::moments(contour);
//     if (moments.m00 == 0) return cv::Point(0, 0);
//     return cv::Point(
//         static_cast<int>(moments.m10 / moments.m00),
//         static_cast<int>(moments.m01 / moments.m00)
//     );
// }

// double computeExtent(const std::vector<cv::Point>& contour) const {
//     if (contour.empty()) return 0.0;
//     double area = cv::contourArea(contour);
//     cv::Rect boundingRect = cv::boundingRect(contour);
//     double rect_area = static_cast<double>(boundingRect.width * boundingRect.height);
//     return (rect_area > 0.0) ? (area / rect_area) : 0.0;
// }

// void correctMainContour(int prevCenterX) {
//     for (const auto& contour : contours) {
//         cv::Point center = computeCenter(contour);
//         if (center.x != 0 && std::abs(center.x - prevCenterX) < CONTOUR_OFFSET_THRESHOLD) {
//             mainContour = contour;
//             contourCenterX = center.x;
//             break;
//         }
//     }
// }

// void drawMarkers(const cv::Point& center, double extent) {
//     cv::drawContours(image, std::vector<std::vector<cv::Point>>{mainContour}, -1, CONTOUR_COLOR, 2);
//     cv::circle(image, center, MARKER_RADIUS, cv::Scalar(255, 255, 255), -1);
//     cv::circle(image, cv::Point(imageCenterX, imageCenterY), 3, IMAGE_CENTER_COLOR, -1);
//     cv::putText(image, "Offset: " + std::to_string(imageCenterX - contourCenterX),
//                 cv::Point(contourCenterX + 20, imageCenterY),
//                 cv::FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1);
//     cv::putText(image, "Extent: " + std::to_string(extent),
//                 cv::Point(contourCenterX + 20, imageCenterY + TEXT_OFFSET_Y),
//                 cv::FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 1);
// }

void * computeBarycenter(void *arg) {
    cv::Mat img, img_hsv;
    cv::Mat red1, red2, blue;
    cv::Mat bin_mask;

    while (true) {
        if (!cam.getVideoFrame(img, 1000)) {
            printf("Timeout error while grabbing frame.\n");
            continue;
        }

        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
        cv::inRange(img_hsv, cv::Scalar(0, 120, 70),    cv::Scalar(10, 255, 255),   red1);
        cv::inRange(img_hsv, cv::Scalar(170, 120, 70),  cv::Scalar(180, 255, 255),  red2);
        cv::inRange(img_hsv, cv::Scalar(100,150,50),    cv::Scalar(140, 255, 255),  blue);
        bin_mask = red1 | red2 | blue;
        cv::imshow("BINARY MASK", bin_mask);

        std::vector<cv::Vec4i>              hierarchy;
        std::vector<std::vector<cv::Point>> contours;
        findContours(bin_mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

        cv::Mat img_cpy1 = img.clone();
        cv::drawContours(img_cpy1, contours, -1, cv::Scalar(0, 255, 0), 2);
        
        cv::Mat img_cpy2    = img.clone();
        cv::Moments moment  = cv::moments(bin_mask, true);
        x                   = moment.m10 / moment.m00;

        cv::Point p(x, moment.m01 / moment.m00);
        cv::circle(img_cpy2, p, 5, cv::Scalar(128,0,0), -1);
        cv::imshow("CAMERA", img_cpy2);

        printf("x = %d\n", x);

        char key = static_cast<char>(cv::waitKey(5));
        if (key == 27) break;
    }
    return NULL;
}

#endif // VISION_H