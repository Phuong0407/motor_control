#ifndef VISION_HPP
#define VISION_HPP

#include "camera.hpp"
#include "color_extractor.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <vector>


class Vision {
private:
    cv::Mat image;
    Camera camera;
    ColorExtractor<RedHSV>  red_extractor;
    ColorExtractor<BlueHSV> blue_extractor;
public:
    Vision(
        int frame_width = 800,
        int frame_height = 640,
        int framerate = 30,
        bool verbose = false
    ) :
    camera(frame_width, frame_height, framerate, verbose),
    red_extractor(),
    blue_extractor()
    {
        cv::Mat image(frame_width, frame_height, CV_8UC3);
        camera.startVideo();
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

        cv::imshow("FRAME", image);
        
        cv::Mat red_mask, blue_mask;
        red_extractor.extractColoredMask(image, red_mask);
        blue_extractor.extractColoredMask(image, blue_mask);
        mask = red_mask | blue_mask;
    }
    
    void extractPath(const cv::Mat &mask, cv::Mat &path, std::vector<cv::Point> &waypoints) {
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

            Moments M = moments(c);
            if (M.m00 != 0) {
                int cx = int(M.m10 / M.m00);
                int cy = int(M.m01 / M.m00);

                cout << "CX: " << cx << " CY: " << cy << endl;

                if (cx >= 120) {
                    // mov6eLeft();
                } else if (cx < 120 && cx > 40) {
                    // moveForward();
                } else {
                    // moveRight();
                }

                circle(frame, Point(cx, cy), 5, Scalar(255, 255, 255), -1);
            }
        }
    }

};

#endif // VISION_HPP
