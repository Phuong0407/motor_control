#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <lccv.hpp>

cv::Mat extractRedMask(const cv::Mat& image) {
    cv::Mat hsvImage, mask1, mask2, binaryMask;
    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);
    cv::inRange(hsvImage, cv::Scalar(0, 120, 70),  cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsvImage, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
    binaryMask = mask1 | mask2;
    return binaryMask;
}

int main() {
    lccv::PiCamera cam;
    cam.options->video_width = 640;
    cam.options->video_height = 480;
    cam.options->framerate = 30;
    cam.options->verbose = true;
    cam.startVideo();

    std::vector<std::pair<double, cv::Point2f>> ballTrajectory;
    const double interval = 0.1; // seconds
    auto lastTime = std::chrono::steady_clock::now();

    // === Calibration: Pixels per centimeter (adjust this based on your setup) ===
    const double pixels_per_cm_x = 10.0f;  // 10 pixels = 1 cm in X
    const double pixels_per_cm_y = 10.0f;  // 10 pixels = 1 cm in Y

    std::cout << "Tracking red ball. Press ESC to exit." << std::endl;

    while (true) {
        cv::Mat frame;
        if (!cam.getVideoFrame(frame, 1000)) {
            std::cerr << "[ERROR] Timeout error while grabbing frame." << std::endl;
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - lastTime;

        if (elapsed.count() >= interval) {
            lastTime = now;

            cv::Mat redMask = extractRedMask(frame);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(redMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            if (!contours.empty()) {
                auto maxContour = *std::max_element(contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
                        return cv::contourArea(c1) < cv::contourArea(c2);
                    });

                cv::Moments M = cv::moments(maxContour);
                if (M.m00 > 0) {
                    cv::Point2f pixelCenter(M.m10 / M.m00, M.m01 / M.m00);
                    
                    // Convert to real-world coordinates
                    cv::Point2f realCenter;
                    realCenter.x = pixelCenter.x / pixels_per_cm_x;
                    realCenter.y = pixelCenter.y / pixels_per_cm_y;

                    double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
                    ballTrajectory.emplace_back(timestamp, realCenter);

                    std::cout << "Time: " << timestamp
                              << " | Pixel Position: (" << pixelCenter.x << ", " << pixelCenter.y << ")"
                              << " | Real Position: (" << realCenter.x << "cm, " << realCenter.y << "cm)" << std::endl;

                    // Draw markers
                    cv::circle(frame, pixelCenter, 8, cv::Scalar(0, 255, 0), -1);
                }
            }

            cv::imshow("Ball Detection", frame);
            if (cv::waitKey(1) == 27) break; // ESC to quit
        }
    }

    cam.stopVideo();
    cv::destroyAllWindows();
    return 0;
}
