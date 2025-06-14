#ifndef VISION_H
#define VISION_H

#include "robot.h"
#include "motor.h"

#include <vector>
#include <stdio.h>
#include <limits>
#include <cmath>

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
        if (key == 27) {
            TERMINATE_PROGRAM = true;
        }
    }
    return NULL;
}


void * extractBallCenter(void * arg) {
    float radius;
    cv::Point2f center;
    while (!TERMINATE_PROGRAM) {
        if (!cam.getVideoFrame(img, 1000)) {
            printf("[ERROR] Timeout error while grabbing frame.\n");
            continue;
        }

        cv::Mat hsv, mask1, mask2, bin_mask;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 120, 70),    cv::Scalar(10, 255, 255),   mask1);
        cv::inRange(hsv, cv::Scalar(170, 120, 70),  cv::Scalar(180, 255, 255),  mask2);
        bin_mask = mask1 | mask2;

        cv::erode(bin_mask, bin_mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(bin_mask, bin_mask, cv::Mat(), cv::Point(-1, -1), 2);

        std::vector<cv::Vec4i>              hierarchy;
        Contours_t                          contours;
        findContours(bin_mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < MIN_CONTOUR_AREA) {
                pthread_mutex_lock(&VISION_MUTEX);
                CONTAIN_BALL = false;
                pthread_mutex_unlock(&VISION_MUTEX);
                continue;
            }
            else {
                cv::minEnclosingCircle(contour, center, radius);
                pthread_mutex_lock(&VISION_MUTEX);
                CONTAIN_BALL = true;
                pthread_mutex_unlock(&VISION_MUTEX);
                break;
            }
        }

        if (CONTAIN_BALL) {
            double ball_diam = static_cast<double>(radius) * 2.0;
            y = DEPTH_MULTIPLIER / ball_diam;
            x = static_cast<double>(center.x - FRAMEWIDTH / 2) / ball_diam * BALL_DIAMETER_CM;

            cv::circle(img, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
            cv::putText(img,
                        "Z = " + std::to_string(y).substr(0, 5) + " cm "
                        + "X =" + std::to_string(x).substr(0, 5) + " cm",
                        center + cv::Point2f(10, -20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 255), 1);
        }
        cv::imshow("IMAGE", img);
    }
    return nullptr;
}

#endif // VISION_H