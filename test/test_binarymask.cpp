#include "../include/vision/vision.hpp"

#include <stdio.h>

int main() {
    Vision vision(640, 480, 30, false);

    cv::Mat binary_mask;
    cv::Mat pathImage;

    int timeout = 500;
    int ch = 0;
    while (ch != 27) {
        vision.extrackRouteBinaryMap(binary_mask, timeout);
        vision.extractPath(binary_mask, pathImage);
        cv::imshow("Path with Centerline", pathImage);
        printf("end one waypoints.\n");
        ch = cv::waitKey(0);
    }
}
