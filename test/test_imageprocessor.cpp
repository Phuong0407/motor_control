#include "../include/vision/image_processor.hpp"
#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#define N_SLICES 4

int main() {
    lccv::PiCamera cam;
    cam.options->video_width = 640;
    cam.options->video_height = 480;
    cam.options->framerate = 30;
    cam.options->verbose = true;
    cam.startVideo();

    cv::Mat image(480, 640, CV_8UC3);
    cv::Mat processedImage;
    std::vector<Image> slices(N_SLICES);

    int ch = 0;

    while (ch != 27) {
        if (!cam.getVideoFrame(image, 1000)) {
            printf("[Error] Timeout error while grabbing frame.");
            continue;
        }

        SlicePart(image, slices, N_SLICES);
        RepackImages(slices, processedImage);
        cv::imshow("Processed", processedImage);
        ch = cv::waitKey(5);
    }

    cam.stopVideo();
    return 0;
}
