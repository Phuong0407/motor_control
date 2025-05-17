#ifndef IMAGE_PREPROCESSOR_HPP
#define IMAGE_PREPROCESSOR_HPP

#include <lccv.hpp>
#include <opencv2/opencv.hpp>

/**
 * @struct RedHSV
 * @brief Defines the HSV color range for detecting red color.
 */
struct RedHSV {
    static inline const cv::Scalar lower = cv::Scalar(0, 100, 100);
    static inline const cv::Scalar upper = cv::Scalar(10, 255, 255);
};

/**
 * @struct BlueHSV
 * @brief Defines the HSV color range for detecting blue color.
 */
struct BlueHSV {
    static inline const cv::Scalar lower = cv::Scalar(170, 100, 100);
    static inline const cv::Scalar upper = cv::Scalar(180, 255, 255);
};

/**
 * @class ColorExtractor
 * @brief Template class for extracting a specific color mask from a precomputed HSV image.
 * @tparam HSVColor The HSV color struct defining the lower and upper HSV bounds for color detection.
 */
template<typename HSVColor>
class ColorExtractor {
public:
    /**
     * @brief Extracts a binary mask from the given HSV image based on the defined HSV range.
     * @param hsv_image Precomputed HSV image.
     * @param mask Output binary mask for the specified color range.
     */
    void extractMask(const cv::Mat& hsv_image, cv::Mat &mask) const {
        cv::inRange(hsv_image, HSVColor::lower, HSVColor::upper, mask);
    }
};

/**
 * @class BinaryMaskExtractor
 * @brief Extracts binary masks for multiple colors and combines them into a single mask.
 */
class BinaryMaskExtractor {
private:
    ColorExtractor<RedHSV>    red_extractor;
    ColorExtractor<BlueHSV>   blue_extractor;

public:
    /**
     * @brief Extracts a combined mask for red, blue, and yellow colors from the input image.
     * Converts the input BGR image to HSV once and extracts color masks sequentially, minimizing memory usage.
     * @param image Input image in BGR format.
     * @param mask Output binary mask combining red, blue, and yellow masks.
     */
    void extractColoredMask(const cv::Mat& image, cv::Mat &mask) {
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
        cv::GaussianBlur(hsv_image, hsv_image, cv::Size(5, 5), 0);
    
        cv::Mat red_mask;
        cv::Mat blue_mask;
        red_extractor.extractMask(hsv_image, red_mask);
        blue_extractor.extractMask(hsv_image, blue_mask);
        cv::bitwise_or(red_mask, blue_mask, mask);
    }


    BinaryMaskExtractor() = default;
};

#endif // IMAGE_PREPROCESSOR_HPP
