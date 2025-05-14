#ifndef CONTOUR_ANALYZER_HPP
#define CONTOUR_ANALYZER_HPP

#include "image_preprocessor.hpp"

#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>
#include <stdio.h>

/**
 * @class ContourAnalyzer
 * @brief It extracts contours from a binary mask.
 * @brief It computes contour centers and its extent.
 * @brief It draws red circle to represent the center of the contour.
 * @brief It draws white circle to represent the center of the image.
 * @note center and extent will be used later for direction computation.
 */
class ContourAnalyzer {
private:
    /**
     * @note In the query of contour coordinate and image center coordiante,
     * @note their Y-coordinate is the same.
     */
    cv::Mat image;                                 ///< Input image.
    int contourCenterX = 0;                        ///< X-coordinate of the main contour center.
    int imageCenterX = 0;                          ///< X-coordinate of the image center.
    int imageCenterY = 0;                          ///< Y-coordinate of the image center.
    int directionOffset = 0;                       ///< Horizontal offset from image center to contour center.
    std::vector<std::vector<cv::Point>> contours;  ///< Detected contours.
    std::vector<cv::Point> mainContour;            ///< Main contour identified.
    std::vector<cv::Point> previousContour;        ///< Previous main contour for tracking.
    BinaryMaskExtractor binaryExtractor;
    
    /**
     * @brief Computes the centroid of a given contour using image moments.
     * @param contour The contour for which the center point is computed.
     * @return Center point as a cv::Point object. Returns (0, 0) if the contour area is zero.
     */
    cv::Point computeContourCenter(const std::vector<cv::Point>& contour) const {
        cv::Moments moments = cv::moments(contour);
        if (moments.m00 == 0) return cv::Point(0, 0);
        
        return cv::Point(
            static_cast<int>(moments.m10 / moments.m00), 
            static_cast<int>(moments.m01 / moments.m00)
        );
    }

    /**
     * @brief Computes the extent of a contour (area-to-bounding-rectangle ratio).
     * @param contour The contour for which the extent is computed.
     * @return The extent ratio as a double. Returns 0.0 if the bounding rectangle area is zero.
     */
    double computeContourExtent(const std::vector<cv::Point>& contour) const {
        double area = cv::contourArea(contour);
        cv::Rect boundingRect = cv::boundingRect(contour);
        double rectArea = static_cast<double>(boundingRect.width * boundingRect.height);
        return (rectArea > 0.0) ? (area / rectArea) : 0.0;
    }

    /**
     * @brief Corrects the main contour selection based on proximity to the previous center.
     * @param prevCenterX The X-coordinate of the previous main contour center.
     */
    void correctMainContour(int prevCenterX) {
        for (const auto& contour : contours) {
            cv::Point center = computeContourCenter(contour);
            if (center.x != 0 && std::abs(center.x - prevCenterX) < 5) {
                mainContour = contour;
                contourCenterX = center.x;
                break;
            }
        }
    }

    /**
     * @brief Draws visual markers on the image, including the main contour, center, and extent.
     * @param center Center point of the main contour.
     * @param extent Extent ratio of the main contour.
     */
    void drawMarkers(const cv::Point& center, double extent) {
        cv::drawContours(image, std::vector<std::vector<cv::Point>>{mainContour}, -1, cv::Scalar(0, 255, 0), 2);
        cv::circle(image, center, 5, cv::Scalar(255, 255, 255), -1);
        cv::circle(image, cv::Point(imageCenterX, imageCenterY), 3, cv::Scalar(0, 0, 255), -1);
        cv::putText(image, "Offset: " + std::to_string(imageCenterX - contourCenterX), 
                    cv::Point(contourCenterX + 20, imageCenterY), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 0, 200), 1);
        cv::putText(image, "Extent: " + std::to_string(extent), 
                    cv::Point(contourCenterX + 20, imageCenterY + 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 200), 1);
    }

public:
    ContourAnalyzer() = default;

    /**
     * @brief Detects contours, identifies the main contour, and draws visual markers.
     * @brief Extracts binary mask using `extractBinaryMask`.
     * @brief Detects contours using OpenCV's `findContours`.
     * @brief Identifies the largest contour by area.
     * @brief Computes center and extent of the main contour.
     * @brief Draws visual markers (contour outline, center point, extent info).
     */
    void analyzeContours() {
        if (image.empty()) {
            printf("[ERROR] Image not loaded!\n");
            return;
        }

        cv::Mat binaryMask;
        binaryExtractor.extractColoredMask(image, binaryMask);

        cv::findContours(binaryMask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        
        if (contours.empty()) {
            printf("[INFO] No line detected. Stopping motors.\n");
            directionOffset = 0;  // Reset direction offset
            return;
        }

        // Parameters for contour validation
        constexpr double MIN_CONTOUR_AREA = 100.0;   // Adjust as necessary
        constexpr double MAX_EXTENT_RATIO = 0.7;     // Adjust as necessary

        mainContour.clear();  // Reset main contour

        // Iterate through contours to find a valid line contour
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            double extent = computeContourExtent(contour);

            if (area >= MIN_CONTOUR_AREA && extent <= MAX_EXTENT_RATIO) {
                mainContour = contour;
                break;
            }
        }

        if (mainContour.empty()) {
            printf("[INFO] No valid line contour detected. Stopping motors.\n");
            directionOffset = 0;  // Reset direction offset
            return;
        }
                
        previousContour = mainContour;

        if (!contours.empty()) {
            mainContour = *std::max_element(contours.begin(), contours.end(), 
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            int prevCenterX = contourCenterX;
            cv::Point center = computeContourCenter(mainContour);
            if (center.x != 0) {
                contourCenterX = center.x;
                if (std::abs(prevCenterX - contourCenterX) > 5.0)
                    correctMainContour(prevCenterX);
            } else contourCenterX = 0;

            double extent = computeContourExtent(mainContour);
            directionOffset = static_cast<int>((imageCenterX - contourCenterX) * extent);
            drawMarkers(center, extent);
        }
    }

    void setImage(const cv::Mat& inputImage) {
        image = inputImage.clone();
        imageCenterX = image.cols / 2;
        imageCenterY = image.rows / 2;
    }
    const cv::Mat& getImage() const { return image; }
    int getContourCenterX() const { return contourCenterX; }
    int getImageCenterX() const { return imageCenterX; }
    int getImageCenterY() const { return imageCenterY; }
    int getDirectionOffset() const { return directionOffset; }
    const std::vector<std::vector<cv::Point>>& getContours() const { return contours; }
    const std::vector<cv::Point>& getMainContour() const { return mainContour; }
    const std::vector<cv::Point>& getPreviousContour() const { return previousContour; }
};

#endif // CONTOUR_ANALYZER_HPP