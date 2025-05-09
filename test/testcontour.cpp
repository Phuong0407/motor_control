#include "../include/vision/image_processor_helper.hpp"
#include "../include/vision/image_processor.hpp"

#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

int main() {
    lccv::PiCamera cam;
    cam.options->video_width = 480;
    cam.options->video_height = 640;
    cam.options->framerate = 30;
    cam.options->verbose = true;

    cv::Mat image(640, 480, CV_8UC3);

    ImageProcessorHelper helper;

    int ret_code = cam.startVideo();
    if (ret_code)
        printf("[ERROR] Failed to initialize camera with error code: %d\n", ret_code);
    else
        printf("[INFO] Camera initialized successfully with resolution %dx%d.\n", 640, 480);

    while (true) {
        if (!cam.getVideoFrame(image, 1000)) {
            printf("[ERROR] Timeout while grabbing frame.\n");
            break;
        }
        image = helper.removeBackground(image, true);
    }
}