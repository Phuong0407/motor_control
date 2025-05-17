#ifndef VISION_HPP
#define VISION_HPP

#include <lccv.hpp>
#include <opencv2/opencv.hpp>


#include <vector>
#include <stdio.h>





static constexpr int    framewidth      = 640;
static constexpr int    frameheight     = 480;
static constexpr int    framerate       = 30;
static constexpr bool   verbose         = false;



lccv::PiCamera cam;



void startCamera() {
    cam.options->video_width    = framewidth;
    cam.options->video_height   = frameheight;
    cam.options->framerate      = framerate;
    cam.options->verbose        = verbose;
    cam.startVideo();
}





void * computeBaryCenter(void *arg) {
    cv::Mat img, img_hsv;
    cv::Mat red1, red2, blue;
    cv::Mat bin_mask;

    while (true) {
        if (!cam.getVideoFrame(img, 1000)) {
            printf("Timeout error while grabbing frame.\n");
            continue;
        }

        cv::cvtColor(img, img_hsv, COLOR_BGR2HSV);
        cv::inRange(img_hsv, cv::Scalar(0, 120, 70),    cv::Scalar(10, 255, 255),   red1);
        cv::inRange(img_hsv, cv::Scalar(170, 120, 70),  cv::Scalar(180, 255, 255),  red2);
        cv::inRange(img_hsv, cv::Scalar(100,150,50),    cv::Scalar(140, 255, 255),  blue);
        bin_mask = red1 | red2 | blue;

        std::vector<Vec4i>              hierarchy;
        std::vector<std::vector<Point>> contours;
        findContours(bin_mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

        cv::Mat img_cpy1 = img.clone();
        cv::drawContours(img_cpy1, contours, -1, Scalar(0, 255, 0), 2);
        
        cv::Mat img_cpy2 = img.clone();
        cv::Moments moment = cv::moments(bin_mask, true);
        barycenter  = moment.m10 / moment.m00;
        cv::Point p(barycenter, moment.m01 / moment.m00);

        cv::circle(image_cpy2, p, 5, Scalar(128,0,0), -1);
        cv::imshow("CAMERA", img_cpy2);

        char key = static_cast<char>(cv::waitKey(5));
        if (key == 27) break;
    }
}

#endif // VISION_HPP