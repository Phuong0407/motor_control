#include "./include/vision/camera.hpp"
#include "./include/vision/vision.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    Camera camera(640, 480, 30, false);
    Vision<RedHSV> red_visualizer;
    cv::Mat image(480, 640, CV_8UC3);
    int timeout = 1000;
    int ch = 0;
    while(ch != 27) {
        if (!camera.captureFrame(image, timeout)) {
            printf("[ERROR] The program stops now!\n");
        }
        cv::Mat mask;
        red_visualizer.extractColoredMask(image, mask);
        cv::imshow("Binary Mask", mask);
        cv::waitKey(100);
    }
    cv::destroyWindow("Binary Mask");
    return 0;
}
