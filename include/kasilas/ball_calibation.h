#ifndef BALL_CALIBRATION_H
#define BALL_CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <cmath>

class BallCalibration {
public:
    BallCalibration();
    void setupCamera();
    void displayInstructions() const;
    void collectPoints();
    void calculateCalibration();
    
private:
    cv::VideoCapture cap;
    std::vector<cv::Point2f> clickedPoints;

    static void onMouse(int event, int x, int y, int, void* userdata);
    std::pair<float, float> getRealWorldLengths() const;
};

BallCalibration::BallCalibration() : cap(0) {}

void Calibration::setupCamera() {
    cap.open(0);
    if (!cap.isOpened()) {
        fprintf(stderr, "Error: Camera not detected!\n");
        exit(EXIT_FAILURE);
    }
}

void BallCalibration::displayInstructions() const {
    printf("== Dual-Axis Calibration ==\n");
    printf("1. Click 2 points horizontally (e.g., a ruler along the ground).\n");
    printf("2. Click 2 points vertically (e.g., a vertical stick or edge).\n");
}

void BallCalibration::onMouse(int event, int x, int y, int, void* userdata) {
    auto* self = reinterpret_cast<Calibration*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN && self->clickedPoints.size() < 4) {
        self->clickedPoints.emplace_back(x, y);
        printf("Clicked Point %lu: (%d, %d)\n", self->clickedPoints.size(), x, y);
    }
}

void BallCalibration::collectPoints() {
    cv::namedWindow("CALIBRATION", cv::WINDOW_NORMAL);
    cv::setMouseCallback("CALIBRATION", onMouse, this);

    cv::Mat frame;
    while (clickedPoints.size() < 4) {
        cap >> frame;
        if (frame.empty()) continue;

        for (const auto& pt : clickedPoints) {
            cv::circle(frame, pt, 5, cv::Scalar(0, 255, 0), -1);
        }

        cv::imshow("Calibration", frame);
        if (cv::waitKey(10) == 27) break;
    }
    cv::destroyWindow("CALIBRATION");
    cap.release();
}

std::pair<double, double> BallCalibration::getRealWorldLengths() const {
    double knownLengthX, knownLengthY;
    printf("\nEnter the real-world horizontal length in cm (between points 1 and 2): ");
    scanf("%f", &knownLengthX);
    printf("Enter the real-world vertical length in cm (between points 3 and 4): ");
    scanf("%f", &knownLengthY);
    return {knownLengthX, knownLengthY};
}

void BallCalibration::calculateCalibration() {
    if (clickedPoints.size() != 4) {
        fprintf(stderr, "Error: Not enough points selected for calibration.\n");
        return;
    }

    auto realWorldLengths = getRealWorldLengths();

    float pixelDistanceX = cv::norm(clickedPoints[0] - clickedPoints[1]);
    float pixelDistanceY = cv::norm(clickedPoints[2] - clickedPoints[3]);

    float pixelsPerCmX = pixelDistanceX / realWorldLengths.first;
    float pixelsPerCmY = pixelDistanceY / realWorldLengths.second;

    printf("\n--- Calibration Results ---\n");
    printf("Horizontal pixel distance: %.2f px\n", pixelDistanceX);
    printf("Vertical pixel distance: %.2f px\n", pixelDistanceY);
    printf("Pixels per cm (X): %.4f\n", pixelsPerCmX);
    printf("Pixels per cm (Y): %.4f\n", pixelsPerCmY);
    printf("CM per pixel (X): %.4f\n", 1.0f / pixelsPerCmX);
    printf("CM per pixel (Y): %.4f\n", 1.0f / pixelsPerCmY);
}


#endif // BALL_CALIBRATION_H