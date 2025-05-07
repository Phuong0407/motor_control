#ifndef CAMERA_CONTROLLER_HPP
#define CAMERA_CONTROLLER_HPP

#include <opencv2/opencv.hpp>
#include "lccv.hpp"
#include <iostream>

class CameraController{
private:
    lccv::PiCamera cam;
    int frame_width;
    int frame_height;
    int frame_rate;
    bool verbose_mode;

public:
    CameraController(
        int frame_width = 640,
        int frame_height = 480,
        int frame_rate = 30,
        bool verbose_mode = false
    ) :
    frame_width(frame_width), 
    frame_height(frame_height),
    frame_rate(frame_rate),
    verbose_mode(verbose_mode)
    {
        cam.options->width = frame_width;
        cam.options->height = frame_height;
        cam.options->framerate = frame_rate;
        if (verbose_mode) {
            std::cout << "[INFO] Camera initialized successfully: "
                      << frame_width << "x" << frame_height << "@" << frame_rate << "fps.\n";
        }
    }

    ~CameraController() {
        cam.stopVideo();
        if (verbose_mode) {
            std::cout << "[CameraController] Camera stopped.\n";
        }
    }

    bool getFrame(cv::Mat& frame) {
        if (!cam.getVideoFrame(frame, 1000)) {
            if (verbose_mode) {
                std::cerr << "[ERROR] Failed to capture frame (timeout).\n";
            }
            return false;
        }
        return true;
    }
};



#endif // CAMERA_CONTROLLER_HPP
