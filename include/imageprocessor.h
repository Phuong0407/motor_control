#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include "robot.h"
#include <opencv2/opencv.hpp>
#include <array>

constexpr double        MIN_CONTOUR_AREA            = 100.0;
constexpr double        MAX_EXTENT_RATIO            = 0.7;
constexpr int           CONTOUR_OFFSET_THRESHOLD    = 5;
const     cv::Scalar    CONTOUR_COLOR               = cv::Scalar(0, 255, 0);
const     cv::Scalar    IMAGE_CENTER_COLOR          = cv::Scalar(0, 0, 255);
const     cv::Scalar    TEXT_COLOR                  = cv::Scalar(200, 0, 200);
constexpr int           MARKER_RADIUS               = 5;
constexpr int           TEXT_OFFSET_Y               = 30;
constexpr int           FOUND_LINE                  = 1;
constexpr int           NO_LINE_FOUND               = 0;
constexpr int           N_SLICES                    = 5;
using                   Contour_t                   = std::vector<cv::Point>;
using                   Contours_t                  = std::vector<Contour_t>;

inline void computeContourCenter(const Contour_t &contour, int &center_x, int &center_y) {
    cv::Moments moments = cv::moments(contour);
    center_x = static_cast<int>(moments.m10 / moments.m00);
    center_y = static_cast<int>(moments.m01 / moments.m00);
}

inline double computeContourExtent(const Contour_t &contour, double area) {
    cv::Rect boundRect = cv::boundingRect(contour);
    double rect_area = static_cast<double>(boundRect.width * boundRect.height);
    return (rect_area > 0.0) ? (area / rect_area) : 0.0;
}



struct SliceData {
    cv::Mat             img;
    cv::Mat             bin_mask;
    static Contours_t   contours;
    Contour_t           contour;
    int                 center_x        = 0;
    int                 center_y        = 0;
    int                 img_center_x    = 0;
    int                 img_center_y    = 0;
    int                 dir_offset      = 0;
    double              extent          = 0.0;
    bool                has_line        = true;

    inline void         computeSliceCenter();
    inline void         computeSliceExtent(double area);
    double                identifyMainContour();
    inline void         computeDirectionOffset();
    inline void         processSliceImage();
    inline void         extractContour();
    void                drawMarker();
};
Contours_t SliceData::contours;

inline void SliceData::computeSliceCenter() {
    if (contour.empty()) {
        has_line = false;
        return;
    }
    computeContourCenter(contour, center_x, center_y);
}

inline void SliceData::computeSliceExtent(double area) {
    if (contour.empty()) {
        has_line = false;
        return;
    }
    extent = computeContourExtent(contour, area);
}

inline void SliceData::computeDirectionOffset() {
    dir_offset = static_cast<int>((img_center_x - center_x) * extent);
}

double SliceData::identifyMainContour() {
    double area = 0.0;
    contour.clear();
    for (const auto& contour_iter : contours) {
        area = cv::contourArea(contour_iter);
        if (area < MIN_CONTOUR_AREA)
            continue;
        
        extent = computeContourExtent(contour_iter, area);
        if (extent <= MAX_EXTENT_RATIO) {
            contour = contour_iter;
            break;
        }
    }
    return area;
}

inline void SliceData::extractContour() {
    SliceData::contours.clear();
    cv::findContours(bin_mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
}

void SliceData::processSliceImage() {
    this->img_center_x  = this->bin_mask.cols / 2;
    this->img_center_y  = this->bin_mask.rows / 2;

    extractContour();
    if (contours.empty()) {
        has_line = false;
        return;
    }
    
    double area = identifyMainContour();
    if (contour.empty()) {
        has_line = false;
        return;
    }

    computeSliceCenter();
    computeSliceExtent(area);
    computeDirectionOffset();
}

void SliceData::drawMarker() {
    for (int i = 0; i < N_SLICES; ++i) {
        cv::Point contour_center = cv::Point(center_x, center_y);
        cv::Point slice_center   = cv::Point(img_center_x, img_center_y);
        
        cv::drawContours(img, std::vector<std::vector<cv::Point>>{contour}, -1, CONTOUR_COLOR, 2);
        cv::circle(img, contour_center, MARKER_RADIUS, cv::Scalar(255, 255, 255), -1);
        cv::circle(img, slice_center, MARKER_RADIUS, IMAGE_CENTER_COLOR, -1);
        
        cv::putText(
                img, "Offset: " + std::to_string(img_center_x - center_x),
                contour_center, cv::FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1
                );
                cv::putText(
                    img, "Extent: " + std::to_string(extent),
                    cv::Point(center_x + 20, center_y + TEXT_OFFSET_Y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 1
                );
    }
}


class ImageProcessor {
public:
    ImageProcessor() = default;
    void        processImage(cv::Mat& img);
    cv::Mat     getOutputImage() const;

private:
    cv::Mat     img;
    cv::Mat     output;
    cv::Mat     bin_mask;
    SliceData   slices[N_SLICES];

    void        extractBinMask();
    void        sliceBinMask();
    void        repackSlice();
};

cv::Mat ImageProcessor::getOutputImage() const {
    return output;
}

void ImageProcessor::extractBinMask() {
    cv::Mat img_hsv, red1, red2, blue;
    
    cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(img_hsv, cv::Scalar(0, 120, 70),    cv::Scalar(10, 255, 255),   red1);
    cv::inRange(img_hsv, cv::Scalar(170, 120, 70),  cv::Scalar(180, 255, 255),  red2);
    cv::inRange(img_hsv, cv::Scalar(100, 150, 50),  cv::Scalar(140, 255, 255),  blue);

    bin_mask = red1 | red2 | blue;
}

void ImageProcessor::sliceBinMask() {
    int width           = bin_mask.cols;
    int height          = bin_mask.rows;
    int slice_height    = height / N_SLICES;

    for (int i = 0; i < N_SLICES; i++) {
        int start_y = slice_height * i;
        cv::Rect slice_rect(0, start_y, width, slice_height);
        slices[i].bin_mask  = bin_mask(slice_rect).clone();
        slices[i].img       = img(slice_rect).clone();
    }
}

void ImageProcessor::processImage(cv::Mat& img) {
    this->img = img;
    extractBinMask();
    sliceBinMask();
    for (int i = 0; i < N_SLICES; ++i) {
        slices[i].processSliceImage();
        slices[i].drawMarker();
    }
    repackSlice();
}

void ImageProcessor::repackSlice() {
    output = slices[0].img.clone();
    for (size_t i = 1; i < N_SLICES; ++i) {
        cv::vconcat(output, slices[i].img, output);
    }
}

#endif // IMAGEPROCESSOR_H