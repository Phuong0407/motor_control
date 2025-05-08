#include "../include/vision/vision.hpp"

#include <stdio.h>

int main() {
    Vision vision(800, 640, 30, false);

    cv::Mat binary_mask;
    cv::Mat pathImage;
    std::vector<cv::Point> waypoints;

    int timeout = 5000;
    int counter = 0;
    while (counter <= 1000) {
        vision.extrackRouteBinaryMap(binary_mask, timeout);
        vision.extractPath(binary_mask, pathImage, waypoints);
        cv::imshow("Binary Mask", binary_mask);
        cv::imshow("Path with Centerline", pathImage);
        std::cout << "Waypoints:" << std::endl;
        for (const auto &wp : waypoints) {
            std::cout << "(" << wp.x << ", " << wp.y << ")" << std::endl;
        }
        printf("end one waypoints.\n");
        cv::waitKey(1);
        counter++;
    }
}
