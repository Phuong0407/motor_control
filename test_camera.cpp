#include "camera_controller.hpp"

int main() {
    CameraController cam(640, 480, 30, true);
    cv::Mat frame;

    while (true) {
        if (!cam.getFrame(frame)) {
            std::cout << "Frame grab failed. Skipping..." << std::endl;
            continue;
        }
    }

    return 0;
}

