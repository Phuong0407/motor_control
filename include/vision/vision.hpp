#ifndef VISION_HPP
#define VISION_HPP

#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>

struct RedHSV {
    static inline const cv::Scalar lower = cv::Scalar(0, 120, 50);
    static inline const cv::Scalar upper = cv::Scalar(10, 255, 255);
};

struct BlueHSV {
    static inline const cv::Scalar lower = cv::Scalar(100, 150, 0);
    static inline const cv::Scalar upper = cv::Scalar(140, 255, 255);
};

const std::string asciiChars = " .:-=+*#%@";

template<typename RGBColor>
class Vision {
private:
    cv::Mat currentFrame;

public:
    Vision() = default;

    void updateFrame(const cv::Mat& frame) {
        frame.copyTo(currentFrame);
    }

    void extractColoredMask(const cv::Mat& image, cv::Mat &mask) {
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_image, RGBColor::lower, RGBColor::upper, mask);
    }

    void displayMaskAsASCII(cv::Mat mask) {
        int charLevels = asciiChars.length() - 1;
        printf("\nASCII Mask Display:\n");
        for (int y = 0; y < mask.rows; ++y) {
            for (int x = 0; x < mask.cols; ++x) {
                uchar pixel = mask.at<uchar>(y, x);
                int index = static_cast<int>((pixel / 255.0) * charLevels);
                printf("%c", asciiChars[index]);
            }
            printf("\n");
        }
    }
};

#endif // VISION_HPP
