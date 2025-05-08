#ifndef VISION_HPP
#define VISION_HPP

#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

struct RedHSV {
    static inline const cv::Scalar lower = cv::Scalar(0, 100, 100);
    static inline const cv::Scalar upper = cv::Scalar(10, 255, 255);
};

struct BlueHSV {
    static inline const cv::Scalar lower = cv::Scalar(100, 150, 0);
    static inline const cv::Scalar upper = cv::Scalar(140, 255, 255);
};

template<typename RGBColor>
class VisionController {
private:
    /**
     * Character set for ASCII representation
     * Adjust this string to change the output characters
     */
    const std::string asciiChars = " .:-=+*#%@";

public:
    VisionController() = default;

    /**
     * Updates the current frame.
     */
    void updateFrame(const cv::Mat& frame) {
        frame.copyTo(currentFrame);
    }

    /**
     * Extracts the target color mask based on the HSV range.
     */
    void extractColoredMask(cv::Mat& mask) {
        cv::Mat hsv;
        cv::cvtColor(currentFrame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, RGBColor::lower, RGBColor::upper, mask);
    }

    /**
     * Displays the mask in ASCII format in the terminal.
     * @param width  Desired width of the ASCII output
     * @param height Desired height of the ASCII output
     */
    void displayMaskAsASCII(int width = 60, int height = 30) {
        cv::Mat mask;
        extractColoredMask(mask);

        // Resize the mask to the specified width and height
        cv::Mat resizedMask;
        cv::resize(mask, resizedMask, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);

        // Calculate the scaling factor for ASCII intensity
        int charLevels = asciiChars.length() - 1;

        std::cout << "\nASCII Mask Display:\n";
        for (int y = 0; y < resizedMask.rows; ++y) {
            for (int x = 0; x < resizedMask.cols; ++x) {
                uchar pixel = resizedMask.at<uchar>(y, x);

                // Map pixel value to ASCII characters
                int index = static_cast<int>((pixel / 255.0) * charLevels);
                std::cout << asciiChars[index];
            }
            std::cout << std::endl;
        }
    }
};

#endif // VISION_HPP