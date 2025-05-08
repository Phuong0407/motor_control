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
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(processed, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
        path = cv::Mat::zeros(mask.size(), CV_8UC3);
    
        for (const auto &contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 100) {
                cv::drawContours(path, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);
            }
        }
    
        cv::Mat binaryPath;
        cv::cvtColor(path, binaryPath, cv::COLOR_BGR2GRAY);
        cv::threshold(binaryPath, binaryPath, 1, 255, cv::THRESH_BINARY);
    
        cv::Mat skeleton;
        cv::ximgproc::thinning(binaryPath, skeleton, cv::ximgproc::THINNING_ZHANGSUEN);
    
        waypoints.clear();
        for (int y = 0; y < skeleton.rows; y++) {
            for (int x = 0; x < skeleton.cols; x++) {
                if (skeleton.at<uchar>(y, x) == 255) {
                    waypoints.push_back(cv::Point(x, y));
                    cv::circle(path, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), -1);  // Draw waypoint
                }
            }
        }
    
        cv::imshow("Path with Centerline", path);
        cv::waitKey(0);
    }

};

#endif // VISION_HPP
