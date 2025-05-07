#ifndef CAMERA_CONTROLLER_HPP
#define CAMERA_CONTROLLER_HPP

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
    ) :
    {
        cam.options->photo_width = photo_width;
        cam.options->photo_height = photo_height;
        cam.options->verbose = verbose;
        std::cout << "[INFO] Camera initialized successfully: "
                  << frame_width << "x" << frame_height << "@" << frame_rate << "fps.\n";

        cv::namedWindow("Image",cv::WINDOW_NORMAL);
        if(!cam.capturePhoto(image)) {
            std::cout<<"Camera error"<<std::endl;
        }
        cv::imshow("Image",image);
        cv::waitKey(30);
        cv::waitKey();
        cv::destroyWindow("Image");
    }

    ~CameraController() {
        cam.stopVideo();
        if (verbose_mode) {
            std::cout << "[CameraController] Camera stopped.\n";
        }
    }

    bool getFrame(cv::Mat& frame) {
        cv::namedWindow("Image",cv::WINDOW_NORMAL);                                                            27         if(!cam.capturePhoto(image)) {                                                                         28             std::cout<<"Camera error"<<std::endl;                                                              29         }                                                                                                      30         cv::imshow("Image",image);                                                                             31         cv::waitKey(30);                                             
        cv::waitKey();
        cv::destroyWindow("Image");
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
