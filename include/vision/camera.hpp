#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "vision.hpp"

#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <thread>

class Camera{
private:
    lccv::PiCamera cam;
    Vision<RedHSV> v;

public:
    Camera(
        int frame_width = 800,
        int frame_height = 640,
        int framerate = 30,
        bool verbose = false
    ) {
        cam.options->video_width = frame_width;
        cam.options->video_height = frame_height;
        cam.options->framerate;
        cam.options->verbose = verbose;
        printf("[INFO] Camera initialized successfully with resolution %dx%d.\n", frame_width, frame_height);
    }

    ~Camera() {
        cam.stopVideo();
        printf("[INFO] Camera stopped.\n");
    }

    bool getFrame() {
        cam.startVideo();
        printf("Camera starts recording video.\n");
//      cv::namedWindow("Video", cv::WINDOW_NORMAL);
        cv::Mat image(cam.options->video_width, cam.options->video_height, CV_8UC3);

        int ch = 0;
        // PRESS ESC KEY TO STOP
        while (ch != 27) {
            if (!cam.getVideoFrame(image, 1000)) {
                printf("[ERROR] Timeout while grabbing frame.\n");
                return false;
            }
            // TODO 
            cv::Mat mask;
            v.extractColoredMask(image, mask);
            v.displayMaskAsASCII(mask);
            cv::imshow("Binary Mask", mask);
            cv::waitKey(100);
//            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        }
        cv::destroyWindow("Binary Mask");
        return true;
    }
};



#endif // CAMERA_HPP
