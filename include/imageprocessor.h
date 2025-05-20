#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include "robot.h"
#include <opencv2/opencv.hpp>

cv::Mat     slice_img[N_SLICES];
cv::Mat     slice_bin_mask[N_SLICES];
Contour_t   slice_contour[N_SLICES];
Contours_t  slice_contours[N_SLICES];
double      slice_area[N_SLICES];



void identifyMainContour(
    const Contours_t & contours, Contour_t &contour,
    double & area, double & extent)
{
    contour.clear();
    for (const auto& contour_it : contours) {
        double larea = cv::contourArea(contour_it);
        if (larea < MIN_CONTOUR_AREA)
            continue;
        else
            area = larea;

        cv::Rect bound_rect = cv::boundingRect(contour_it);
        double rect_area = static_cast<double>(bound_rect.width * bound_rect.height);

        double lextent = (rect_area > 0.0) ? (larea / rect_area) : 0.0;
        if (lextent <= MAX_EXTENT_RATIO) {
            contour = contour_it;
            extent = lextent;
        }
    }
}

void processSliceImage() {
    for (unsigned int i = 0; i < N_SLICES; ++i) {
        img_center_xs[i] = slice_bin_mask[i].cols / 2;
        img_center_ys[i] = slice_bin_mask[i].rows / 2;

        slice_contours[i].clear();
        cv::findContours(bin_mask, slice_contours[i], cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        
        if (slice_contours[i].empty())
            contain_lines[i] = false;
        
        identifyMainContour(slice_contours[i], slice_contour[i], slice_area[i], extents[i]);
        if (slice_contour[i].empty()) {
            contain_lines[i] = false;
            continue;
        }

        cv::Moments moments = cv::moments(slice_contour[i]);
        center_xs[i] = static_cast<int>(moments.m10 / moments.m00);
        center_ys[i] = static_cast<int>(moments.m01 / moments.m00);

        dir_offsets[i] = static_cast<int>(static_cast<double>(img_center_xs[i] - center_xs[i]) * extents[i]);
        printf("dir offset %d,\t%d\n", i, dir_offsets[i]);
    }
}

void drawMarker() {
    for (unsigned int i = 0; i < N_SLICES; ++i) {
        cv::Point contour_center    = cv::Point(center_xs[i], center_ys[i]);
        cv::Point slice_center      = cv::Point(img_center_xs[i], img_center_ys[i]);
        cv::Point extent_center     = cv::Point(center_xs[i] + 20, center_ys[i] + TEXT_OFFSET_Y);

        cv::drawContours(slice_img[i], Contours_t{slice_contour[i]}, -1, CONTOUR_COLOR, 2);
        cv::circle(slice_img[i], contour_center, MARKER_RADIUS, CONTOUR_CENTER_COLOR, -1);
        cv::circle(slice_img[i], slice_center,   MARKER_RADIUS, IMAGE_CENTER_COLOR,  -1);

        cv::putText(slice_img[i], "Offset: " + std::to_string(img_center_xs[i] - center_xs[i]),
                    contour_center, cv::FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 1);

        cv::putText(slice_img[i], "Extent: " + std::to_string(extents[i]), extent_center,
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 1);
    }
}

void extractBinMask() {
    cv::Mat img_hsv, red1, red2, blue;
    
    cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(img_hsv, cv::Scalar(0, 120, 70),    cv::Scalar(10, 255, 255),   red1);
    cv::inRange(img_hsv, cv::Scalar(170, 120, 70),  cv::Scalar(180, 255, 255),  red2);
    cv::inRange(img_hsv, cv::Scalar(100, 150, 50),  cv::Scalar(140, 255, 255),  blue);

    bin_mask = red1 | red2 | blue;
}

void sliceImageToMask() {
    int width           = bin_mask.cols;
    int height          = bin_mask.rows;
    int slice_height    = height / N_SLICES;

    for (unsigned int i = 0; i < N_SLICES; i++) {
        int start_y = slice_height * i;
        cv::Rect slice_rect(0, start_y, width, slice_height);
        slice_img[i]        = img(slice_rect).clone();
        slice_bin_mask[i]   = bin_mask(slice_rect).clone();
        cv::imshow("bin_mask, ", slice_bin_mask[i]);
    }
}

void repackSlice() {
    output = slice_img[0].clone();
    for (size_t i = 1; i < N_SLICES; ++i) {
        cv::vconcat(output, slice_img[i], output);
    }
}

void processImage() {
    extractBinMask();
    sliceImageToMask();
    processSliceImage();
    drawMarker();
    repackSlice();
}

#endif // IMAGEPROCESSOR_H