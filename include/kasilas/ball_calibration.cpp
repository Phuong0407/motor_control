#include <opencv2/opencv.hpp>
#include <iostream>
#include <lccv.hpp>  // Include the lccv PiCamera wrapper

const float BALL_DIAMETER_CM = 4.6f;
const float CAMERA_CONSTANT = 311.3292f;  // Based on your calibration

float estimateBallDepth(float pixelDiameter, float realDiameterCm = BALL_DIAMETER_CM) {
    return (CAMERA_CONSTANT * realDiameterCm) / pixelDiameter;
}

int main() {
    // Setup PiCamera
    lccv::PiCamera cam;
    cam.options->video_width = 640;
    cam.options->video_height = 480;
    cam.options->framerate = 30;
    cam.options->verbose = true;
    cam.startVideo();

    cv::Mat frame, hsv, mask;

    while (true) {
        if (!cam.read(frame)) continue; // Grab frame
        if (frame.empty()) continue;

        // Convert to HSV
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Threshold for orange ball (adjust HSV if needed)
        cv::Scalar lower_orange(5, 100, 100);
        cv::Scalar upper_orange(25, 255, 255);
        cv::inRange(hsv, lower_orange, upper_orange, mask);

        // Morphological filtering
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            float area = cv::contourArea(contour);
            if (area < 200) continue; // Filter small areas

            // Fit enclosing circle
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);

            float diameter_px = radius * 2.0f;
            float depth_cm = estimateBallDepth(diameter_px);

            float pixelsPerCm = diameter_px / BALL_DIAMETER_CM;
            float x_cm = center.x / pixelsPerCm;
            float y_cm = center.y / pixelsPerCm;

            // Draw ball + depth info
            cv::circle(frame, center, (int)radius, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, "Z = " + std::to_string(depth_cm) + " cm",
                        center + cv::Point2f(10, -20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 255), 1);

            std::cout << "Ball 3D Position: (X = " << x_cm
                      << " cm, Y = " << y_cm
                      << " cm, Z = " << depth_cm << " cm)" << std::endl;
        }

        // Show video
        cv::imshow("Ball Tracking", frame);
        if (cv::waitKey(10) == 27) break;  // ESC to exit
    }

    cam.stopVideo();  // Cleanly stop the camera
    cv::destroyAllWindows();
    return 0;
}