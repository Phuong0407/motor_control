#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include "contour_analyzer.hpp"

#include <opencv2/opencv.hpp>

#include <vector>

/**
 * @class ImageProcessor
 * @brief Processes an image to produce multiple vertical slices and compute direction offsets.
 * @tparam n_slices Number of vertical slices to divide the image into. Must be greater than 0.
 */
template<unsigned int n_slices>
class ImageProcessor {
    static_assert(n_slices > 0, "Number of slices must be greater than 0.");

private:
    std::vector<ContourAnalyzer> slices = std::vector<ContourAnalyzer>(n_slices);

    /**
     * @brief Slices the input image into vertical sections.
     * @param input The input image.
     */
    void sliceImage(const cv::Mat& input) {
        int width = input.cols;
        int height = input.rows;
        int sliceHeight = height / n_slices;

        for (unsigned int i = 0; i < n_slices; i++) {
            int startY = sliceHeight * i;
            cv::Rect sliceRect(0, startY, width, sliceHeight);
            cv::Mat sliceImage = input(sliceRect).clone();

            ContourAnalyzer analyzer;
            analyzer.setImage(sliceImage);
            slices[i] = analyzer;
        }
    }

    bool analyzeSliceContour() {
        bool LineDetectionStatus = false;
        for (auto& slice : slices) {
            int DetectionStatus = slice.analyzeContours();
            if (DetectionStatus == NO_LINE_FOUND) {
                printf("No line found.\n");
            } else {
                printf("Line found in slice.\n");
                LineDetectionStatus = true;
            }
        }
        return LineDetectionStatus;
    }

    /**
     * @brief Calculates the accumulated direction offset across all slices.
     * @return The accumulated direction offset.
     */
    int calculateDirectionOffset() const {
        int accumulatedDirection = 0;
        for (size_t i = 1; i < slices.size() - 1; ++i) {
            accumulatedDirection += slices[i].getDirectionOffset();
        }
        return accumulatedDirection;
    }

    /**
     * @brief Recombines the processed slices into a single output image.
     * @param output Output concatenated image.
     */
    void repackSlices(cv::Mat& output) const {
        if (slices.empty()) {
            output = cv::Mat();
            return;
        }
        output = slices[0].getImage().clone();
        for (size_t i = 1; i < slices.size(); ++i) {
            cv::vconcat(output, slices[i].getImage(), output);
        }
    }

public:

    ImageProcessor() = default;

    /**
     * @brief Processes the input image and calculates the direction offset.
     * @param input Input image in BGR format.
     * @param output Output image with concatenated slices.
     * @return The accumulated direction offset.
     */
    int processImage(const cv::Mat& input, cv::Mat& output) {
        sliceImage(input);
        bool LineDetectionStatus = analyzeSliceContour();
        repackSlices(output);
        if (!LineDetectionStatus)
            return NO_LINE_FOUND;
        else
            return calculateDirectionOffset();
    }

    /**
     * @brief Gets the centroid points for each slice.
     * @return Vector of centroid points for each slice.
     */
    std::vector<cv::Point> getCentroidPoints() const {
        std::vector<cv::Point> centroidPoints;
        for (const auto& slice : slices) {
            int x_centroid = slice.getContourCenterX();
            int y_centroid = slice.getImageCenterY();
            centroidPoints.emplace_back(x_centroid, y_centroid);
        }
        return centroidPoints;
    }
};

#endif // IMAGE_PROCESSOR_HPP