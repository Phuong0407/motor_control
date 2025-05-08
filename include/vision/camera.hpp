#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

class Camera{
private:
    lccv::PiCamera cam;
public:
    Camera(
        int frame_width = 800,
        int frame_height = 640,
        int framerate = 30,
        bool verbose = false
    ) {
        cam.options->video_width = frame_width;
        cam.options->video_height = frame_height;
        cam.options->framerate = framerate;
        cam.options->verbose = verbose;
        printf("[INFO] Camera initialized successfully with resolution %dx%d.\n", frame_width, frame_height);
    }
    void startVideo() {
        cam.startVideo();
        printf("[INFO] Camera starts recording video.\n");
    }
    void stopVideo() {
        cam.stopVideo();
        printf("[INFO] Camera stops recording video.\n");
    }
    bool captureFrame(cv::Mat &image, int timeout = 1000) {
        if (!cam.getVideoFrame(image, timeout)) {
            printf("[ERROR] Timeout while grabbing frame.\n");
            return false;
        }
        return true;
    }
    
};



#endif // CAMERA_HPP
