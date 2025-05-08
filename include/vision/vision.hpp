#ifndef VISION_HPP
#define VISION_HPP

#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <vector>
#include <stdio.h>


struct RedHSV{
    static inline const cv::Scalar lower = cv::Scalar(0, 100, 100);
    static inline const cv::Scalar upper = cv::Scalar(10, 255, 255);
};

struct RedHSV{
    static inline const cv::Scalar lower = cv::Scalar(100, 150, 0);
    static inline const cv::Scalar upper = cv::Scalar(140, 255, 255);
};



class Vision {
private:
    cv::Mat image;
    lccv::PiCamera cam;
public:
    Vision(
        int frame_width = 640,
        int frame_height = 480,
        int framerate = 30,
        bool verbose = false
    ) {
        cam.options->video_width = frame_width;
        cam.options->video_height = frame_height;
        cam.options->framerate = framerate;
        cam.options->verbose = verbose;
        cv::Mat image(frame_height, frame_width, CV_8UC3);
        camera.startVideo();
        cv::namedWindow("Video", cv::WINDOW_NORMAL);
    }
    ~Vision() {
        camera.stopVideo();
    }
    void captureFrame(int timeout = 1000) {
        if (!camera.captureFrame(image, timeout))
            printf("[ERROR] The program stops now!\n");
    }
    void extrackRouteBinaryMap(cv::Mat& mask, int timeout = 1000) {
        if (!camera.captureFrame(image, timeout))
            printf("[ERROR] The program stops now!\n");
        
        cv::Mat red_mask, blue_mask;
        red_extractor.extractColoredMask(image, red_mask);
        blue_extractor.extractColoredMask(image, blue_mask);
        mask = red_mask | blue_mask;
    }
    
    void extractPath(const cv::Mat &mask, cv::Mat &path) {
        CV_Assert(mask.type() == CV_8UC1);
    
        cv::Mat processed = mask.clone();
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

        cv::morphologyEx(processed, processed, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(processed, processed, cv::MORPH_OPEN, kernel);
    
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(processed, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    
        path = cv::Mat::zeros(mask.size(), CV_8UC3);
    
        if (!contours.empty()) {
            std::vector<cv::Point> c = contours[0];
            double maxArea = contourArea(c);

            for (auto &contour : contours) {
                double area = contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    c = contour;
                }
            }

            cv::Moments M = moments(c);
            if (M.m00 != 0) {
                int cx = int(M.m10 / M.m00);
                int cy = int(M.m01 / M.m00);

                printf("CX: %d \t CY: %d \n", cx, cy);

                if (cx >= 120) {
                    // mov6eLeft();
                } else if (cx < 120 && cx > 40) {
                    // moveForward();
                } else {
                    // moveRight();
                }

                cv::circle(mask, cv::Point(cx, cy), 5, cv::Scalar(255, 255, 255), -1);
            }
        }
    }                       

};

#endif // VISION_HPP
