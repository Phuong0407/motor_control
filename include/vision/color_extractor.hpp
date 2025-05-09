#ifndef COLOR_EXTRACTOR_HPP
#define COLOR_EXTRACTOR_HPP

#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>

struct RedHSV {
    static inline const cv::Scalar lower = cv::Scalar(0, 100, 100);
    static inline const cv::Scalar upper = cv::Scalar(10, 255, 255);
};

struct BlueHSV {
    static inline const cv::Scalar lower = cv::Scalar(100, 150, 0);
    static inline const cv::Scalar upper = cv::Scalar(140, 255, 255);
};

template<typename HSVColor>
class ColorExtractor {
public:
    ColorExtractor() = default;
    void extractColoredMask(const cv::Mat& image, cv::Mat &mask) {
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
        cv::inRange(hsv_image, HSVColor::lower, HSVColor::upper, mask);
    }
};

#endif // COLOR_EXTRACTOR_HPP
