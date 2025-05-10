#ifndef VISION_HPP
#define VISION_HPP

#include "../image/image.hpp"

#include <vector>
#include <stdio.h>

#define N_SLICES 4

class Vision{
private:
    std::vector<Image> slices;
    Image image_processor;

    cv::Point Center(cv::Moments moments) {
        if (moments.m00 == 0) {
            return cv::Point(0, 0);
        }
        int x = static_cast<int>(moments.m10 / moments.m00);
        int y = static_cast<int>(moments.m01 / moments.m00);
        return cv::Point(x, y);
    }

    void slicePart(const cv::Mat& im, std::vector<Image>& images, int n_slices) {
        int height = im.rows;
        int width = im.cols;
        int sliceHeight = height / n_slices;
        for (int i = 0; i < n_slices; i++) {
            int startY = sliceHeight * i;
            cv::Rect sliceRect(0, startY, width, sliceHeight);
            images[i].image = im(sliceRect).clone();
            images[i].detectAndDrawContour();
        }
    }
    
    void repackImages(const std::vector<Image>& images, cv::Mat& output) {
        output = images[0].image.clone();
        for (size_t i = 1; i < images.size(); i++)
            cv::vconcat(output, images[i].image, output);
    }

public:
    Vision() : slices(N_SLICES) {}

    void getOutputVision(const cv::Mat& input, cv::Mat& output) {
        image_processor.image = input;
        image_processor.detectAndDrawContour();
        slicePart(input, slices, N_SLICES);
        repackImages(slices, output);
    }

    std::vector<cv::Point> getCentroidPoints() {
        std::vector<cv::Point> centroid_points;
        centroid_points.reserve(N_SLICES);
        printf("\nPRINT CENTROIDS\n");
        for (const auto& slice : slices) {
            int x_centroid = slice.contour_centerX;
            int y_centroid = slice.middleY;
            centroid_points.emplace_back(x_centroid, y_centroid);
            print("(%d,\t%d)\t", x_centroid, y_centroid);
        }
        printf("\n");
        return centroid_points;
    }

    double getAvgCentroids() {
        double centroids = 0.0;
        for (const auto& slice : slices) {
            centroids += slice.contour_centerX;
            printf("contour_centerX %.3f\t", slice.contour_centerX);
        }
        printf("%d");
        return centroids;
    }
};


#endif // VISION_HPP