#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include "contour_analyzer.hpp"

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @class ImageProcessor
 * @brief Processes an image to produce multiple vertical slices.
 * @tparam n_slices Number of vertical slices to divide the image into. Must be greater than 0.
 */
template<unsigned int n_slices>
class ImageProcessor {
    static_assert(n_slices > 0, "Number of slices must be greater than 0.");

private:
    std::vector<ContourAnalyzer> slices;
    /**
     * @brief Calculates the centroid point from image moments.
     * @brief This method is static as it only performs a computation based on input moments.
     * @param moments Image moments.
     * @return Centroid as a cv::Point object.
     */
    static cv::Point computeCenters(const cv::Moments& moments) {
        if (moments.m00 == 0) return cv::Point(0, 0);
        int x = static_cast<int>(moments.m10 / moments.m00);
        int y = static_cast<int>(moments.m01 / moments.m00);
        return cv::Point(x, y);
    }

    /**
     * @brief Divides the input image into vertical slices and processes each slice.
     * @brief This method is static as it processes slices without modifying class members.
     * @param im Input image.
     * @param slices Vector to store the processed slices.
     */
    static void sliceImageToPart(const cv::Mat& im, std::vector<ContourAnalyzer>& slices) {
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
     * @brief This method is static as it only operates on the provided slices.
     * @param slices Vector of processed slices.
     * @param output Output concatenated image.
     */
    static void repackSlicesToImage(const std::vector<ContourAnalyzer>& slices, cv::Mat& output) {
        if (slices.empty()) {
            output = cv::Mat();
            return;
        }
        output = slices[0].getImage().clone();
        for (size_t i = 1; i < slices.size(); i++)
            cv::vconcat(output, slices[i].getImage(), output);
    }

public:

    ImageProcessor() : slices(n_slices) {} 

    /**
     * @brief Slices the input image, calculates the accumulated direction offset, and repacks the slices.
     * @brief This method performs the following steps:
     * @brief Divides the input image into `n_slices` vertical slices using `sliceImageToPart()`.
     * @brief Processes each slice to calculate its direction offset and accumulates these offsets.
     * @brief Recombines the processed slices into a single output image using `repackSlicesToImage()`.
     * @param input Input image in BGR format.
     * @param output Output image with concatenated slices.
     * @return The accumulated direction offset calculated by summing the direction offsets of each slice.
     */
    int postProcessImage(const cv::Mat& input, cv::Mat& output) {
        int accumulatedDirection = 0;
        sliceImageToPart(input, slices);
        for (std::size_t i = 1; i < slices.size() - 1; ++i) {
            accumulatedDirection += slices[i].getDirectionOffset();
            printf("Accumulated direction offset: %d\n", accumulatedDirection);
        }
        printf("\n");
        repackSlicesToImage(slices, output);
        return accumulatedDirection;
    }

    /**
     * @brief Gets the centroid points for each slice.
     * @return Vector of centroid points for each slice.
     */
    std::vector<cv::Point> getCentroidPoints() {
        std::vector<cv::Point> centroidPoints;
        centroidPoints.reserve(n_slices);
        for (const auto& slice : slices) {
            int x_centroid = slice.getContourCenterX();
            int y_centroid = slice.getImageCenterY();
            centroidPoints.emplace_back(x_centroid, y_centroid);
        }
        return centroidPoints;
    }
};

#endif // IMAGE_PROCESSOR_HPP