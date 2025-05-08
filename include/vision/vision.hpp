#ifndef VISION_HPP
#define VISION_HPP

#include "camera.hpp"
#include "color_extractor.hpp"

#include <opencv2/opencv.hpp>

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
        cv::Mat red_mask, blue_mask;
        red_extractor.extractColoredMask(image, red_mask);
        blue_extractor.extractColoredMask(image, blue_mask);
        mask = red_mask | blue_mask;
    }
}

#endif // VISION_HPP