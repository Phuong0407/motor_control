#ifndef VISION_H
#define VISION_H

#include "robot.h"
// #include "imageprocessor.h"
#include <vector>
#include <stdio.h>
#include <limits>
#include <cmath>

// extern pthread_mutex_t VISION_MUTEX;

inline bool detectLineFromContours(const Contours_t& contours) {
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < MIN_CONTOUR_AREA)
            continue;

        cv::RotatedRect rect = cv::minAreaRect(contour);
        double width = rect.size.width;
        double height = rect.size.height;
        if (w == 0 || h == 0) continue;

        double aspect_ration = std::max(width, height) / std::min(width, height);
        if (aspect_ration > MIN_ASPECT_RATIO) {
            return true;
        }
    }
    return false;
}

void * computeBarycenter(void *arg) {
    cv::Mat img_hsv, red1, red2, blue;

    while (true) {
        if (!cam.getVideoFrame(img, 1000)) {
            printf("[ERROR] Timeout error while grabbing frame.\n");
            continue;
        }

        cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
        cv::inRange(img_hsv, cv::Scalar(0, 120, 70),    cv::Scalar(10, 255, 255),   red1);
        cv::inRange(img_hsv, cv::Scalar(170, 120, 70),  cv::Scalar(180, 255, 255),  red2);
        cv::inRange(img_hsv, cv::Scalar(100,150,50),    cv::Scalar(140, 255, 255),  blue);
        bin_mask = red1 | red2 | blue;

        std::vector<cv::Vec4i>              hierarchy;
        Contours_t                          contours;
        findContours(bin_mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        
        CONTAIN_LINE = detectLineFromContours(contours);
        if (!CONTAIN_LINE)
            continue;

        cv::Moments moment  = cv::moments(bin_mask, true);
        if (moment.m00 != 0) {
            x = moment.m10 / moment.m00;
            y = moment.m01 / moment.m00;
            cv::Point barycent(static_cast<int>(x), static_cast<int>(y));

            cv::drawContours(img, contours, -1, CONTOUR_COLOR, 2);
            cv::circle(img, barycent, 5, CONTOUR_CENTER_COLOR, -1);
            cv::imshow("IMAGE", img);
        }
        char key = static_cast<char>(cv::waitKey(5));
        if (key == 27) break;
    }
    return NULL;
}

#endif // VISION_H