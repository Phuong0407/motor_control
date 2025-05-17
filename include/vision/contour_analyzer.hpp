#ifndef CONTOUR_ANALYZER_HPP
#define CONTOUR_ANALYZER_HPP

#include "image_preprocessor.hpp"

#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>
#include <stdio.h>
#include <limits>

constexpr double MIN_CONTOUR_AREA       = 100.0;
constexpr double MAX_EXTENT_RATIO       = 0.7;
constexpr int CONTOUR_OFFSET_THRESHOLD  = 5;
const     cv::Scalar CONTOUR_COLOR      = cv::Scalar(0, 255, 0);
const     cv::Scalar IMAGE_CENTER_COLOR = cv::Scalar(0, 0, 255);
const     cv::Scalar TEXT_COLOR         = cv::Scalar(200, 0, 200);
constexpr int MARKER_RADIUS             = 5;
constexpr int TEXT_OFFSET_Y             = 30;
constexpr int FOUND_LINE                = 1;
constexpr int NO_LINE_FOUND             = std::numeric_limits<int>::max();

/**
 * @class ContourAnalyzer
 * @brief Extracts contours, computes centers and extent, and visualizes contour data.
 */
class ContourAnalyzer {
private:
    cv::Mat image;
    int contourCenterX = 0;
    int imageCenterX = 0;
    int imageCenterY = 0;
    int directionOffset = 0;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> mainContour;
    std::vector<cv::Point> previousContour;
    BinaryMaskExtractor binaryExtractor;

    cv::Point computeContourCenter(const std::vector<cv::Point>& contour) const {
        if (contour.empty()) return cv::Point(0, 0);
        cv::Moments moments = cv::moments(contour);
        if (moments.m00 == 0) return cv::Point(0, 0);
        return cv::Point(
            static_cast<int>(moments.m10 / moments.m00),
            static_cast<int>(moments.m01 / moments.m00)
        );
    }

    double computeContourExtent(const std::vector<cv::Point>& contour) const {
        if (contour.empty()) return 0.0;
        double area = cv::contourArea(contour);
        cv::Rect boundingRect = cv::boundingRect(contour);
        double rectArea = static_cast<double>(boundingRect.width * boundingRect.height);
        return (rectArea > 0.0) ? (area / rectArea) : 0.0;
    }

    void correctMainContour(int prevCenterX) {
        for (const auto& contour : contours) {
            cv::Point center = computeContourCenter(contour);
            if (center.x != 0 && std::abs(center.x - prevCenterX) < CONTOUR_OFFSET_THRESHOLD) {
                mainContour = contour;
                contourCenterX = center.x;
                break;
            }
        }
    }

    void drawMarkers(const cv::Point& center, double extent) {
        cv::drawContours(image, std::vector<std::vector<cv::Point>>{mainContour}, -1, CONTOUR_COLOR, 2);
        cv::circle(image, center, MARKER_RADIUS, cv::Scalar(255, 255, 255), -1);
        cv::circle(image, cv::Point(imageCenterX, imageCenterY), 3, IMAGE_CENTER_COLOR, -1);
        cv::putText(image, "Offset: " + std::to_string(imageCenterX - contourCenterX),
                    cv::Point(contourCenterX + 20, imageCenterY),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1);
        cv::putText(image, "Extent: " + std::to_string(extent),
                    cv::Point(contourCenterX + 20, imageCenterY + TEXT_OFFSET_Y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 1);
    }

    void extractContours(const cv::Mat& binaryMask) {
        contours.clear();
        cv::findContours(binaryMask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    }

    void identifyMainContour() {
        mainContour.clear();
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            double extent = computeContourExtent(contour);
            if (area >= MIN_CONTOUR_AREA && extent <= MAX_EXTENT_RATIO) {
                mainContour = contour;
                break;
            }
        }
    }

public:
    ContourAnalyzer() = default;

    void setImage(const cv::Mat& inputImage) {
        image = inputImage.clone();
        imageCenterX = image.cols / 2;
        imageCenterY = image.rows / 2;
    }

    int analyzeContours() {
        cv::Mat binaryMask;
        binaryExtractor.extractColoredMask(image, binaryMask);
        extractContours(binaryMask);
        cv::imshow("binary", binaryMask);

        if (contours.empty()) return NO_LINE_FOUND;
        identifyMainContour();
        if (mainContour.empty()) return NO_LINE_FOUND;

        previousContour = mainContour;
        cv::Point center = computeContourCenter(mainContour);
        contourCenterX = center.x;
        double extent = computeContourExtent(mainContour);
        directionOffset = static_cast<int>((imageCenterX - contourCenterX) * extent);
        drawMarkers(center, extent);

        return FOUND_LINE;
    }

    const cv::Mat& getImage() const { return image; }
    inline int getContourCenterX() const { return contourCenterX; }
    inline int getImageCenterX() const { return imageCenterX; }
    inline int getImageCenterY() const { return imageCenterY; }
    inline int getDirectionOffset() const { return directionOffset; }
    const std::vector<std::vector<cv::Point>>& getContours() const { return contours; }
    const std::vector<cv::Point>& getMainContour() const { return mainContour; }
    const std::vector<cv::Point>& getPreviousContour() const { return previousContour; }
};

#endif // CONTOUR_ANALYZER_HPP