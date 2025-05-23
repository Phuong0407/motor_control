#ifndef VISION_H
#define VISION_H

#include "robot.h"
#include <vector>
#include <stdio.h>
#include <limits>
#include <cmath>

inline bool detectLineFromContours(const Contours_t& contours) {
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < MIN_CONTOUR_AREA)
            continue;

        cv::RotatedRect rect = cv::minAreaRect(contour);
        double width = rect.size.width;
        double height = rect.size.height;
        if (width == 0 || height == 0) continue;

        double aspect_ration = std::max(width, height) / std::min(width, height);
        if (aspect_ration > MIN_ASPECT_RATIO) {
            return true;
        }
    }
    return false;
}

void * computeBarycenter(void *arg) {
    cv::Mat img_hsv, red1, red2, blue;

    while (!TERMINATE_PROGRAM) {
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
        
        pthread_mutex_lock(&VISION_MUTEX);
        CONTAIN_LINE = detectLineFromContours(contours);
        pthread_mutex_unlock(&VISION_MUTEX);
        
        if (!CONTAIN_LINE) continue;

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
    return nullptr;
}

void * extractBallCenter(void * arg) {
    double radius;
    cv::Point2d center;
    while (!TERMINATE_PROGRAM) {
        if (!cam.getVideoFrame(img, 1000)) {
            printf("[ERROR] Timeout error while grabbing frame.\n");
            continue;
        }

        cv::Mat hsv, mask1, mask2, bin_mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 120, 70),    cv::Scalar(10, 255, 255),   mask1);
        cv::inRange(hsv, cv::Scalar(170, 120, 70),  cv::Scalar(180, 255, 255),  mask2);
        bin_mask = mask1 | mask2;

        cv::erode(bin_mask, bin_mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(bin_mask, bin_mask, cv::Mat(), cv::Point(-1, -1), 2);

        contours.clear();
        cv::findContours(bin_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < MIN_CONTOUR_AREA)
                continue;
            else {
                cv::minEnclosingCircle(contour, center, radius);
                break;
            }
        }
  
        double diameter = static_cast<double>(radius) * 2.0;
        y = DEPTH_MULTIPLIER / ball_diam;
        x = static_cast<double>(center.x - FRAME_WIDTH / 2) / ball_diam * BALL_DIAMETER_CM;

        cv::circle(frame, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
        cv::putText(frame,
                    "Z = " + std::to_string(depth).substr(0, 5) + " cm "
                    + "X =" + std::to_string(deviation).substr(0, 5) + " cm",
                    center + cv::Point2d(10, -20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 255), 1);
        cv::imshow("IMAGE", frame);
    }
    return nullptr;
}

#endif // VISION_H