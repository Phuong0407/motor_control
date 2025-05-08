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

template<typename RGBColor>
class Vision {
private:
    cv::Mat currentFrame;
    /**
     * Character set for ASCII representation
     * Adjust this string to change the output characters
     */
    const std::string asciiChars = " .:-=+*#%@";

public:
    Vision() = default;

    /**
     * Updates the current frame.
     */
    void updateFrame(const cv::Mat& frame) {
        frame.copyTo(currentFrame);
    }

    /**
     * Extracts the target color mask based on the HSV range.
     */
    void extractColoredMask(const cv::Mat& image, cv::Mat &mask) {
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_image, RGBColor::lower, RGBColor::upper, mask);
    }

    /**
     * Displays the mask in ASCII format in the terminal.
     * @param width  Desired width of the ASCII output
     * @param height Desired height of the ASCII output
     */
    void displayMaskAsASCII(cv::Mat mask) {
        int charLevels = asciiChars.length() - 1;
        std::cout << "\nASCII Mask Display:\n";
        for (int y = 0; y < mask.rows; ++y) {
            for (int x = 0; x < mask.cols; ++x) {
                uchar pixel = mask.at<uchar>(y, x);
                int index = static_cast<int>((pixel / 255.0) * charLevels);
                std::cout << asciiChars[index];
            }
            std::cout << std::endl;
        }
    }
};

#endif // VISION_HPP
