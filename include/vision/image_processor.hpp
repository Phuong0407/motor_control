#ifndef IMAGE_HPP
#define IMAGE_HPP

#include "color_extractor.hpp"

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>
#include <cmath>

class Image {
private:
    ColorExtractor<RedHSV>  red_extractor;
    ColorExtractor<BlueHSV> blue_extractor;

    cv::Point getContourCenter(const std::vector<cv::Point>& contour) {
        cv::Moments M = cv::moments(contour);
        if (M.m00 == 0)
            return cv::Point(0, 0);

        int x = static_cast<int>(M.m10 / M.m00);
        int y = static_cast<int>(M.m01 / M.m00);
        return cv::Point(x, y);
    }

    double getContourExtent(const std::vector<cv::Point>& contour) {
        double area = cv::contourArea(contour);
        cv::Rect boundingRect = cv::boundingRect(contour);
        double rectArea = static_cast<double>(boundingRect.width) * boundingRect.height;

        return (rectArea > 0.0) ? static_cast<double>(area / rectArea) : 0.0;
    }

    void correctMainContour(int prev_cx) {
        for (const auto& contour : contours) {
            cv::Point center = getContourCenter(contour);
            if (center.x != 0) {
                int tmp_cx = center.x;

                if (std::abs(tmp_cx - prev_cx) < 5.0) {
                    main_contour = contour;
                    contour_centerX = tmp_cx;
                    break;
                }
            }
        }
    }

    void extractBinaryMask(const cv::Mat &image, cv::Mat &bin_mask) {
        cv::Mat red_mask, blue_mask;
        red_extractor.extractColoredMask(image, red_mask);
        blue_extractor.extractColoredMask(image, blue_mask);
        bin_mask = red_mask | blue_mask;
    }

public:

    cv::Mat image;
    int contour_centerX = 0;
    int middleX = 0;
    int middleY = 0;
    int dir = 0;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> main_contour;
    std::vector<cv::Point> prev_MC;

    void detectAndDrawContour() {
        if (image.empty()) {
            printf("[ERROR] Image not loaded!");
            return;
        }

        cv::Mat bin_mask;
        extractBinaryMask(image, bin_mask);
        cv::findContours(bin_mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        prev_MC = main_contour;
        if (!contours.empty()) {
            main_contour = *std::max_element(
                                contours.begin(), contours.end(), 
                                [](
                                    const std::vector<cv::Point>& a,
                                    const std::vector<cv::Point>& b)
                                    {
                                        return cv::contourArea(a) < cv::contourArea(b);
                                    }
                            );

            int height = image.rows;
            int width = image.cols;

            middleX = width / 2.0;
            middleY = height / 2.0;

            int prev_cX = contour_centerX;
            cv::Point center = getContourCenter(main_contour);

            if (center.x != 0.0) {
                contour_centerX = center.x;

                if (std::abs(prev_cX - contour_centerX) > 5.0)
                    correctMainContour(prev_cX);
            } else {
                contour_centerX = 0;
            }

            double extent = getContourExtent(main_contour);
            dir = static_cast<int>((middleX - contour_centerX) * extent);

            cv::drawContours(image, std::vector<std::vector<cv::Point>>{main_contour}, -1, cv::Scalar(0, 255, 0), 3);
            cv::circle(image, cv::Point(contour_centerX, middleY), 7, cv::Scalar(255, 255, 255), -1);
            cv::circle(image, cv::Point(middleX, middleY), 3, cv::Scalar(0, 0, 255), -1);

            cv::putText(image, std::to_string(middleX - contour_centerX), 
                        cv::Point(contour_centerX + 20, middleY), 
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 0, 200), 2);
            
            cv::putText(image, "Weight:" + std::to_string(extent), 
                        cv::Point(contour_centerX + 20, middleY + 35), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 200), 1);
        }
    }

    void slicePart(const cv::Mat& im, std::vector<Image>& images, int slices) {
        int height = im.rows;
        int width = im.cols;
        int sliceHeight = height / slices;
    
        for (int i = 0; i < slices; i++) {
            int startY = sliceHeight * i;
            cv::Rect sliceRect(0, startY, width, sliceHeight);
            images[i].image = im(sliceRect).clone();
            images[i].Process();
        }
    }
    
    void repackImages(const std::vector<Image>& images, cv::Mat& output) {
        if (images.empty()) {
            output.release();
            return;
        }
    
        output = images[0].image.clone();
        for (size_t i = 1; i < images.size(); i++) {
            cv::vconcat(output, images[i].image, output);
        }
    }

    Image() = default;
};

#endif // IMAGE_PROCESSOR_HPP