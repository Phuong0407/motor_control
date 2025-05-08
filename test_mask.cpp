#include "./include/vision/camera.hpp"
#include "./include/vision/vision.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Initialize the Camera
    CameraController camera(640, 480, true);

    // Create a VisionController for Red Color
    VisionController<RedHSV> redVision;

    // Capture a frame
    cv::Mat frame;
    if (!camera.getFrame(frame)) {
        std::cerr << "[ERROR] Unable to capture frame.\n";
        return -1;
    }

    // Extract the red mask
    redVision.updateFrame(frame);
    cv::Mat redMask;
    redVision.extractColoredMask(frame);

    // Display the mask as ASCII output
    redVision.displayMaskAsASCII(frame);

    return 0;
}
