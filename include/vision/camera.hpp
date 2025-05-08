#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "vision.hpp"

#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

class CameraController{
private:
    lccv::PiCamera cam;
    Vision v;

public:
    CameraController(
        int frame_width = 640,
        int frame_height = 480,
        int framerate = 30,
        bool verbose = false
    ) {
        cam.options->photo_width = photo_width;
        cam.options->photo_height = photo_height;
        cam.options->framefrate;
        cam.options->verbose = verbose;
        printf("[INFO] Camera initialized successfully with resolution %dx%d.\n", photo_width, photo_height);
    }

    ~CameraController() {
        cam.stopVideo();
        printf("[INFO] Camera stopped.\n");
    }

    bool getFrame(cv::Mat& frame) {
        cam.startVideo();
        printf("Camera starts recording video.\n");
        cv::namedWindown("Video", cv::WINDOW_NORMAL);
        cv::Mat image(cam.options->video_width, cam.options->video_height, CV_8UC3);

        int char = 0;
        // PRESS ESC KEY TO STOP
        while (ch != 27) {
            if (!cam.getVideoFrame(image, 1000)) {
                printf("[ERROR] Timeout while grabbing frame.\n");
                return false;
            }
            // TODO 
            cv::Mat<RedHSV> mask;
            v.extractColoredMask(image, mask);
            v.displayMaskAsASCII(mask);
        }
        return true;
    }
};



#endif // CAMERA_HPP
