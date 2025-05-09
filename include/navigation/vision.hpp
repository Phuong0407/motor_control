#ifndef VISION_HPP
#define VISION_HPP

#include "../include/image/image.hpp"

#include <vector>
#include <stdio.h>

#define N_SLICES 4

class Vision{
private:
    std::vector<Image> slices;

    cv::Point Center(cv::Moments moments) {
        if (moments.m00 == 0) {
            return cv::Point(0, 0);
        }   
        int x = static_cast<int>(moments.m10 / moments.m00);
        int y = static_cast<int>(moments.m01 / moments.m00);
        return cv::Point(x, y);
    }

    void slicePart(const cv::Mat& im, std::vector<Image>& images, int slices) {
        int height = im.rows;
        int width = im.cols;
        int sliceHeight = height / slices;
    
        for (int i = 0; i < slices; i++) {
            int startY = sliceHeight * i;
            cv::Rect sliceRect(0, startY, width, sliceHeight);
            images[i].image = im(sliceRect).clone();
            images[i].detectAndDrawContour();
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

public:
    Vision() : slices(N_SLICES)
    {}

    void getOutputVision()
};


#endif // VISION_HPP