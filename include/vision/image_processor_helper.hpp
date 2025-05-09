#ifndef IMAGE_PROCESSOR_HELPER_HPP
#define IMAGE_PROCESSOR_HELPER_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

class ImageProcessorHelper {
private:
    cv::Mat image;
    
public:
    ImageProcessorHelper() = default;
    // void slicePart(const cv::Mat& im, std::vector<ImageProcessorHelper>& images, int slices) {
    //     int height = im.rows;
    //     int width = im.cols;
    //     int sl = height / slices;

    //     for (int i = 0; i < slices; i++) {
    //         int part = sl * i;

    //         int sliceHeight = (i == slices - 1) ? (height - part) : sl;
    //         cv::Mat crop_img = im(cv::Rect(0, part, width, sliceHeight)).clone();

    //         images[i].image = crop_img;
    //         images[i].Process();
    //     }
    // }

    // cv::Mat repackImages(const std::vector<ImageProcessorHelper>& images) {
    //     cv::Mat img = images[0].image.clone();

    //     for (size_t i = 1; i < images.size(); i++) {
    //         cv::vconcat(img, images[i].image, img);
    //     }

    //     return img;
    // }

    cv::Point computeCenter(const cv::Moments& moments) {
        if (moments.m00 == 0)
            return cv::Point(0, 0);

        int x = static_cast<int>(moments.m10 / moments.m00);
        int y = static_cast<int>(moments.m01 / moments.m00);

        return cv::Point(x, y);
    }

    cv::Mat removeBackground(const cv::Mat& image, bool removeBackground) {
        int up = 100;

        if (removeBackground) {
            cv::Mat mask, result;
            cv::inRange(image, cv::Scalar(0, 0, 0), cv::Scalar(up, up, up), mask);
            cv::bitwise_not(mask, mask);
            cv::bitwise_and(image, image, result, mask);
            return result;
        } else {
            return image.clone();
        }
    }
};

#endif // IMAGE_PROCESSOR_HELPER_HPP