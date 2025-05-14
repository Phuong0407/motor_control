#include "image_processor.hpp"
#include <opencv2/opencv.hpp>

#include <iostream>

void displayImage(const cv::Mat& image, const std::string& windowName) {
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 640, 480);
    cv::imshow(windowName, image);
    cv::waitKey(0);
    cv::destroyWindow(windowName);
}

int main() {
    lccv::PiCamera cam;
    cam.options->video_width = 640;
    cam.options->video_height = 480;
    cam.options->framerate = 30;
    cam.options->verbose = true;
    cam.startVideo();
    
    cv::Mat image1(480, 640, CV_8UC3);
    cv::Mat image2(480, 640, CV_8UC3);

    ImageProcessor<4> vision;

    int ch = 0;
    while (ch != 27) {
        if (!cam.getVideoFrame(image1, 1000)) {
            std::cout << "Timeout error while grabbing frame." << std::endl;
            continue;
        }
        vision.postProcessImage(image1, image2);
        displayImage(image2, "Processed Image");
        ch = cv::waitKey(5);
    }
}
