#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

class CameraController{
private:
    cv::Mat image;
    lccv::PiCamera cam;

public:
    CameraController(
        int photo_width = 640,
        int photo_height = 480,
        bool verbose = false
    ) {
        cam.options->photo_width = photo_width;
        cam.options->photo_height = photo_height;
        cam.options->verbose = verbose;
        std::cout << "[INFO] Camera initialized successfully with resolution "
                  << photo_width << "x" << photo_height << ".\n";
    }

    ~CameraController() {
        cam.stopVideo();
        std::cout << "[INFO] Camera stopped.\n";
    }

    bool getFrame(cv::Mat& frame) {
        if(!cam.capturePhoto(image)) {
            std::cerr<<"[ERROR] Camera error.\n";
            return false;
        }
        return true;
    }
};



#endif // CAMERA_HPP