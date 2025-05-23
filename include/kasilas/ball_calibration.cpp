#include <opencv2/opencv.hpp>
#include <iostream>
#include <lccv.hpp>  // Make sure lccv.hpp is available

// Constants
const float BALL_DIAMETER_CM = 4.6f;            // Real-world ball diameter
const float CAMERA_CONSTANT = 346.1f;       // Based on your calibration

// Depth estimation using known diameter and pixel size
float estimateBallDepth(float pixelDiameter, float realDiameterCm = BALL_DIAMETER_CM) {
    return (CAMERA_CONSTANT * realDiameterCm) / pixelDiameter;
}

int main() {
    // Initialize PiCamera
    lccv::PiCamera cam;
    cam.options->video_width = 640;
    cam.options->video_height = 480;
    cam.options->framerate = 30;
    cam.options->verbose = true;
    cam.startVideo();

    cv::Mat frame, hsv, mask1, mask2, mask;

    while (true) {
        // Capture frame
        if (!cam.getVideoFrame(frame, 1000)) {
            std::cerr << "[ERROR] Timeout error while grabbing frame." << std::endl;
            continue;
        }
        if (frame.empty()) continue;

        // Convert to HSV color space
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Detect red using two HSV ranges (red wraps around HSV hue)
        cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);     // Lower red
        cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);   // Upper red
        cv::bitwise_or(mask1, mask2, mask);

        // Optional: smooth and clean the mask
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            float area = cv::contourArea(contour);
            if (area < 200) continue;  // Skip small blobs

            // Fit circle to contour
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);

            float diameter_px = radius * 2.0f;
            float depth_cm = estimateBallDepth(diameter_px);

            float pixelsPerCm = diameter_px / BALL_DIAMETER_CM;
            float x_cm = center.x / pixelsPerCm;
            float y_cm = center.y / pixelsPerCm;

            // Draw circle and depth text
            cv::circle(frame, center, (int)radius, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, "Z = " + std::to_string(depth_cm) + " cm",
                        center + cv::Point2f(10, -20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 255), 1);

            // Print 3D position
            std::cout << "Ball 3D Position: (X = " << x_cm
                      << " cm, Y = " << y_cm
                      << " cm, Z = " << depth_cm << " cm)" << std::endl;
            std::cout << "pixel_dia" << diameter_px << std::endl;
        }

        // Show result
        cv::imshow("Red Ball Tracking", frame);
        if (cv::waitKey(10) == 27) break;  // Exit on ESC
    }

    // Cleanup
    cam.stopVideo();
    cv::destroyAllWindows();
    return 0;
}