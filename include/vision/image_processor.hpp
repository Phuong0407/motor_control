#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

class ImageProcessor {
public:
    cv::Mat image;
    int contour_centerX;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> main_contour;

    ImageProcessor() {
        contour_centerX = 0;
    }

    void analyzeContours() {
        if (image.empty()) {
            printf("[ERROR] Empty binary mask!\n");
            return;
        }

        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        cv::Mat thresh;
        cv::threshold(gray, thresh, 100, 255, cv::THRESH_BINARY_INV);

        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            mainContour = *std::max_element(
                                contours.begin(),
                                contours.end(),
                                []( const std::vector<cv::Point>& a,
                                    const std::vector<cv::Point>& b ) {
                                        return cv::contourArea(a) < cv::contourArea(b);
                                    }
                                );

            int height = image.rows;
            int width = image.cols;

            int middleX = width / 2;
            int middleY = height / 2;

            int prevCenterX = contourCenterX;
            cv::Point center = getContourCenter(mainContour);

            if (center.x != -1) {
                contourCenterX = center.x;

                if (std::abs(prevCenterX - contourCenterX) > 5) {
                    correctMainContour(prevCenterX);
                }

                double extent = getContourExtent(mainContour);
                int direction = static_cast<int>((middleX - contourCenterX) * extent);

                cv::drawContours(image, std::vector<std::vector<cv::Point>>{mainContour}, -1, cv::Scalar(0, 255, 0), 2);
                cv::circle(image, cv::Point(contourCenterX, middleY), 7, cv::Scalar(0, 255, 255), -1);
                cv::circle(image, cv::Point(middleX, middleY), 3, cv::Scalar(0, 0, 255), -1);

                cv::putText(image, "Offset: " + std::to_string(middleX - contourCenterX), 
                            cv::Point(contourCenterX + 20, middleY), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(200, 0, 200), 2);

                cv::putText(image, "Extent: " + std::to_string(extent), 
                            cv::Point(contourCenterX + 20, middleY + 30), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 0, 200), 1);
            }
        }
    }

private:
    cv::Point getContourCenter(const std::vector<cv::Point>& contour) {
        cv::Moments M = cv::moments(contour);
        if (M.m00 == 0) {
            return cv::Point(-1, -1);
        }
        int x = static_cast<int>(M.m10 / M.m00);
        int y = static_cast<int>(M.m01 / M.m00);
        return cv::Point(x, y);
    }

    double getContourExtent(const std::vector<cv::Point>& contour) {
        double area = cv::contourArea(contour);
        cv::Rect boundingRect = cv::boundingRect(contour);
        double rectArea = boundingRect.width * boundingRect.height;
        if (rectArea > 0) {
            return area / rectArea;
        }
        return 0.0;
    }

    void correctMainContour(int prevCenterX) {
        for (const auto& contour : contours) {
            cv::Point center = getContourCenter(contour);
            if (center.x != -1) {
                if (std::abs(a - b) < 5.0) {
                    main_contour = contour;
                    contour_centerX = center.x;
                    break;
                }
            }
        }
    }
};

#endif // IMAGE_PROCESSOR_HPP