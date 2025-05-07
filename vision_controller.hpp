#ifndef VISION_CONTROLLER_HPP
#define VISION_CONTROLLER_HPP

#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>

class VisionController {
private:
    cv::Mat currframe;

    void updateNewFrame(cosnt cv::Mat& frame);
    bool detectTarget(const cv::Mat& frame, cv::Point2f& targetCenter);

public:
    VisionController() = default;
    void processFrame(const cv::Mat& frame);
    void extractRedMask(cv::Mat &red_mask, const cv::Mat& image) {
        cv::Mat hsvImage, mask1, mask2, binaryMask;
        cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);
        cv::inRange(hsvImage, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
        binaryMask = mask1 | mask2;
        return binaryMask;
    }
};

#endif // VISION_CONTROLLER_HPP
