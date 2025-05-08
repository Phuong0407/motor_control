#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <opencv2/opencv.hpp>
#include <lccv.hpp>
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

    bool getFrame(/*cv::Mat& frame*/) {
        cv::namedWindow("Image",cv::WINDOW_NORMAL);
        if(!cam.capturePhoto(image)) {
            std::cout<<"[ERROR] Camera error.\n";
            return false;
        }
        cv::imshow("Image",image);
        cv::waitKey(30);
        cv::waitKey();
        cv::destroyWindow("Image");
        return true;
    }
};



#endif // CAMERA_HPP