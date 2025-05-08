#include "./include/vision/vision.hpp"

int main() {
    Vision vision(800, 640, 30, false);
    cv::Mat binary_mask;
    int timeout = 1000;
    int ch = 0;
    while (ch != 27) {
        vision.extrackRouteBinaryMap(binary_mask, timeout);
        cv::imshow("Binary Mask", binary_mask);
        cv::waitKey(500);
    }

}