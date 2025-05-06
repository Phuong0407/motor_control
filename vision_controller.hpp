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

};


#endif // VISION_CONTROLLER_HPP