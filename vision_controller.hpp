#ifndef VISION_CONTROLLER_HPP
#define VISION_CONTROLLER_HPP

#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>

struct RedHSV {
    static inline const cv::Scalar lower = cv::Scalar(0, 100, 100);
    static inline const cv::Scalar upper = cv::Scalar(10, 255, 255);
};

struct BlueHSV {
    static inline const cv::Scalar lower = cv::Scalar(100, 150, 0);
    static inline const cv::Scalar upper = cv::Scalar(140, 255, 255);
};

template<typename RGBColor>
class VisionController {
private:
    cv::Mat currframe;

    void updateFrame(const cv::Mat& frame) {
        frame.copyTo(currframe);
    }

    void extractColoredMask(const cv::Mat& image, cv::Mat& red_mask) {
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, RGBColor::lower, RGBColor::upper, red_mask);
    }

    bool detectLine(const cv::Mat& frame, cv::Point2f& targetCenter) {
        cv::Mat mask;
        extractColoredMask(frame, mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty()) {
            return false;
        }

        size_t largestContourIdx = 0;
        double maxArea = 0;
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area > maxArea) {
                maxArea = area;
                largestContourIdx = i;
            }
        }

        cv::Moments M = cv::moments(contours[largestContourIdx]);
        if (M.m00 != 0) {
            targetCenter = cv::Point2f(static_cast<float>(M.m10 / M.m00),
                                       static_cast<float>(M.m01 / M.m00));
            return true;
        }
        return false;
    }

    void extractTractableLine() {
        // Stub for future implementation
    }

    void isStraightLine() {
        // Stub for future implementation
    }

    void computeCurvatureRadius() {
        // Stub for future implementation
    }

    void computeKimenatic() {
        // Stub for future implementation
    }

public:
    VisionController() = default;
    void extractRedMask(cv::Mat &red_mask, const cv::Mat& image) {
        cv::Mat hsvImage, mask1, mask2, binaryMask;
        cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);
        cv::inRange(hsvImage, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
        binaryMask = mask1 | mask2;
        return binaryMask;
    }

    void processFrame(const cv::Mat& frame) {
        updateFrame(frame);

        cv::Point2f targetCenter;
        if (detectLine(currframe, targetCenter)) {
            std::cout << "Line detected at: " << targetCenter << std::endl;
        } else {
            std::cout << "No line detected." << std::endl;
        }
    }

    bool detectTarget(const cv::Mat& frame, cv::Point2f& targetCenter) {
        return detectLine(frame, targetCenter);
    }
};

#endif // VISION_CONTROLLER_HPP
