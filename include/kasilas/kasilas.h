#ifndef KASILAS_H
#define KASILAS_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>

constexpr double BALL_DIAMETER_CM   = 4.6;
constexpr double DEPTH_MULTIPLIER   = 1592.06;

constexpr int    FRAME_WIDTH        = 640;
constexpr int    FRAME_HEIGHT       = 480;
constexpr int    FRAME_RATE         = 30;
constexpr bool   VERBOSE            = false;

using Contour_t  = std::vector<cv::Point>;
using Contours_t = std::vector<Contour_t>;

cv::Mat     frame;
Contours_t  contours;

double x = 0.0;
double z = 0.0;

void extractBallCenter() {
    cv::Mat hsv, mask1, mask2, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv, cv::Scalar(0, 120, 70),  cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
    cv::bitwise_or(mask1, mask2, mask);

    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    contours.clear();
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 200.0) continue;

        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);

        double diameter_px = static_cast<double>(radius) * 2.0;
        z = DEPTH_MULTIPLIER / diameter_px;
        x = ((static_cast<double>(center.x) - (FRAME_WIDTH / 2.0)) / diameter_px) * BALL_DIAMETER_CM;

        cv::circle(frame, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, "Z = " + std::to_string(z).substr(0, 5) + " cm",
                    center + cv::Point2f(10, -20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 255), 1);
    }
}

void initMotorDriver(int &fd1, int &fd2) {
    wiringPiSetup();
    fd1 = wiringPiI2CSetup(0x0f);
    fd2 = wiringPiI2CSetup(0x0d);
}

void setMotors(int fd1, int fd2, int leftSpeed, int rightSpeed, int frontSpeed) {
    std::cout << "[Motor] L: " << leftSpeed << " R: " << rightSpeed << " F: " << frontSpeed << std::endl;
    
    leftSpeed   = std::max(-250, std::min(250, leftSpeed));
    rightSpeed  = std::max(-250, std::min(250, rightSpeed));
    frontSpeed  = std::max(-250, std::min(250, frontSpeed));

    uint8_t l = std::abs(leftSpeed);
    uint8_t r = std::abs(rightSpeed);
    uint8_t f = std::abs(frontSpeed);

    uint8_t dir = 0x00;

    if (leftSpeed < 0 && rightSpeed < 0) {
        dir = 0x06;  // both forward
    } else if (leftSpeed > 0 && rightSpeed > 0) {
        dir = 0x09;  // both reverse
    }

    wiringPiI2CWriteReg16(fd1, 0xAA, dir);

    uint16_t speedVal = (l << 8) | r;

    wiringPiI2CWriteReg16(fd1, 0x82, speedVal);

    wiringPiI2CWriteReg16(fd2, 0x82, 0x0000);

}

cv::Mat extractRedMask(const cv::Mat& image) {
    cv::Mat hsvImage, mask1, mask2, binaryMask;
    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);
    cv::inRange(hsvImage, cv::Scalar(0, 120, 70),  cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsvImage, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
    binaryMask = mask1 | mask2;
    return binaryMask;
}

double predictGoalLineX(const std::vector<std::pair<double, cv::Point2f>>& trajectory) {
    if (trajectory.size() < 3) {
        std::cerr << "[predictGoalLineX] Not enough points for prediction.\n";
        return -1;
    }

    double sumX = 0, sumY = 0, sumY2 = 0, sumXY = 0;
    int n = trajectory.size();

    for (const auto& [t, pt] : trajectory) {
        double x = pt.x; // in cm
        double y = pt.y; // in cm

        sumX  += x;
        sumY  += y;
        sumY2 += y * y;
        sumXY += x * y;
    }

    double denominator = (n * sumY2 - sumY * sumY);
    if (denominator == 0) {
        std::cerr << "[predictGoalLineX] Linear regression failed: divide by zero.\n";
        return -1;
    }

    double a = (n * sumXY - sumX * sumY) / denominator;
    double b = (sumX * sumY2 - sumY * sumXY) / denominator;

    double predictedX = a * 0 + b; // because we want x when y = 0
    std::cout << "[Trajectory] Predicted impact at X = " << predictedX << " cm (when Y = 0)\n";

    return predictedX;
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

    // Simulated robot starting at center of goal (X = 0 cm)
    double botX = 0.0f;

    // Movement parameters
    const double Kp = 10.0f;
    const int maxSpeed = 250;
    const double movementThreshold = 2.0f; // cm

    int fd1;
    int fd2;
    initMotorDriver(fd1, fd2);

    std::cout << "Goalkeeper running. Press ESC to quit." << std::endl;

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

                    int frameWidth = frame.cols;
                    int frameHeight = frame.rows;

                    double xCenterPx = frameWidth / 2.0f;
                    double yBottomPx = frameHeight;

                    cv::Point2f realCenter;
                    realCenter.x = (pixelCenter.x - xCenterPx) / pixels_per_cm_x;
                    realCenter.y = (yBottomPx - pixelCenter.y) / pixels_per_cm_y;

                    double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
                    ballTrajectory.emplace_back(timestamp, realCenter);

                    // Clear trajectory if ball reaches the goal line
                    if (realCenter.y <= 0) {
                        ballTrajectory.clear();
                        std::cout << "[Info] Ball reached goal line, trajectory reset.\n";
                    }

                    std::cout << "Time: " << timestamp
                              << " | Pixel Position: (" << pixelCenter.x << ", " << pixelCenter.y << ")"
                              << " | Real Position: (" << realCenter.x << "cm, " << realCenter.y << "cm)" << std::endl;

                    // Draw markers
                    cv::circle(frame, pixelCenter, 8, cv::Scalar(0, 255, 0), -1);

                    if (ballTrajectory.size() >= 5) {
                        double predictedX = predictGoalLineX(ballTrajectory);

                        double error = predictedX - botX;
                        
                        if (std::abs(error) > movementThreshold) {
                            int direction = (error > 0) ? 1 : -1;
                            int speed = std::min(maxSpeed, static_cast<int>(std::abs(error) * Kp));
                            speed *= direction;

                            setMotors(fd1, fd2, speed, speed, 0);
                            botX = predictedX;
                            std::cout << "[Move] Bot moves " << (direction > 0 ? "RIGHT" : "LEFT")
                                      << " with speed " << speed << std::endl;
                        } else {
                            setMotors(fd1, fd2, 0, 0, 0); // Stop
                            std::cout << "[Hold] Bot in position.\n";
                        }
                    }

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

#endif // KASILAS_H