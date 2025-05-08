#include "./include/vision/camera.hpp"
#include "./include/vision/vision.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Initialize the Camera
    Camera camera(640, 480, true);
    camera.getFrame();
    return 0;
}
