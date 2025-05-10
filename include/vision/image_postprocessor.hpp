#ifndef IMAGE_POSTPROCESSOR_HPP
#define IMAGE_POSTPROCESSOR_HPP

#include "contour_analyzer.hpp"

#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <vector>

/**
 * @class ImagePostprocessor
 * @brief Processes an image to produce multiple vertical slices.
 * @tparam n_slices Number of vertical slices to divide the image into. Must be greater than 0.
 */
template<unsigned int n_slices>
class ImagePostprocessor {
    static_assert(n_slices > 0, "Number of slices must be greater than 0.");

private:
    ContourAnalyzer image_processor;          ///< Full image processor.
    std::vector<ContourAnalyzer> slices;      ///< Sliced image processors.

    /**
     * @brief Calculates the centroid point from image moments.
     * @param moments Image moments.
     * @return Centroid as a cv::Point object.
     */
    cv::Point computeCenters(const cv::Moments& moments) const {
        if (moments.m00 == 0) return cv::Point(0, 0);
        int x = static_cast<int>(moments.m10 / moments.m00);
        int y = static_cast<int>(moments.m01 / moments.m00);
        return cv::Point(x, y);
    }

    /**
     * @brief Divides the input image into vertical slices and processes each slice.
     * @param im Input image.
     * @param slices Vector to store the processed slices.
     */
    void sliceImageToPart(const cv::Mat& im, std::vector<ContourAnalyzer>& slices) {
        int width = im.cols;
        int height = im.rows;
        int sliceHeight = height / n_slices;
        slices.resize(n_slices);
        
        for (unsigned int i = 0; i < n_slices; i++) {
            int startY = sliceHeight * i;
            cv::Rect sliceRect(0, startY, width, sliceHeight);
            cv::Mat sliceImage = im(sliceRect).clone();
            slices[i].setImage(sliceImage);
            slices[i].analyzeContours();
        }
    }

    /**
     * @brief Recombines processed slices into a single output image.
     * @param slices Vector of processed slices.
     * @param output Output concatenated image.
     */
    void repackSlicesToImage(const std::vector<ContourAnalyzer>& slices, cv::Mat& output) {
        if (slices.empty()) return;
        output = slices[0].getImage().clone();
        for (size_t i = 1; i < slices.size(); i++)
            cv::vconcat(output, slices[i].getImage(), output);
    }

public:
    ImagePostprocessor() = default;

    /**
     * @brief Slices the input image, calculates the accumulated direction offset, and repacks the slices.
     * @brief This method performs the following steps:
     * @brief Divides the input image into `n_slices` vertical slices using `slicePart()`.
     * @brief Processes each slice to calculate its direction offset and accumulates these offsets.
     * @brief Recombines the processed slices into a single output image using `repackImages()`.
     * @param input Input image in BGR format.
     * @param output Output image that contains all processed slices concatenated vertically.
     * @return The accumulated direction offset calculated by summing the direction offsets of each slice.
     */
    int postProcessImage(const cv::Mat& input, cv::Mat& output) {
        int accumulatedDirection = 0;
        image_processor.setImage(input);
        image_processor.analyzeContours();
        slicePart(input, slices);
        for (const auto& slice : slices)
            accumulatedDirection += slice.getDirectionOffset();
        repackImages(slices, output);
        return accumulatedDirection;
    }

    /**
     * @brief Gets the centroid points for each slice.
     * @return Vector of centroid points for each slice.
     */
    std::vector<cv::Point> getCentroidPoints() const {
        std::vector<cv::Point> centroid_points;
        centroid_points.reserve(n_slices);

        for (const auto& slice : slices) {
            int x_centroid = slice.getContourCenterX();
            int y_centroid = slice.getImageCenterY();
            centroid_points.emplace_back(x_centroid, y_centroid);
        }
        return centroid_points;
    }

};

#endif // IMAGE_POSTPROCESSOR_HPP