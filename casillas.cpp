#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>
#include <lccv.hpp>
#include <wiringPiI2C.h>
#include <wiringPi.h>

// === Constants ===
const float BALL_DIAMETER_CM = 4.6f;        // Real ball diameter
const float CAMERA_CONSTANT = 346.1f;       // Calibration constant

// === Depth Estimation ===
float estimateBallDepth(float pixelDiameter) {
    return (CAMERA_CONSTANT * BALL_DIAMETER_CM) / pixelDiameter;
}

// === Motor Initialization ===
void initMotorDriver(int &fd1, int &fd2) {
    wiringPiSetup();
    fd1 = wiringPiI2CSetup(0x0f);
    fd2 = wiringPiI2CSetup(0x0d);
}

// === Motor Control ===
void setMotors(int fd1, int fd2, int leftSpeed, int rightSpeed, int frontSpeed) {
    std::cout << "[Motor] L: " << leftSpeed << " R: " << rightSpeed << " F: " << frontSpeed << std::endl;

    leftSpeed = std::max(-250, std::min(250, leftSpeed));
    rightSpeed = std::max(-250, std::min(250, rightSpeed));
    frontSpeed = std::max(-250, std::min(250, frontSpeed));

    uint8_t l = std::abs(leftSpeed);
    uint8_t r = std::abs(rightSpeed);
    uint8_t f = std::abs(frontSpeed);

    uint8_t dir = 0x00;
    if (leftSpeed < 0 && rightSpeed < 0) {
        dir = 0x06;
    } else if (leftSpeed > 0 && rightSpeed > 0) {
        dir = 0x09;
    }

    wiringPiI2CWriteReg16(fd1, 0xAA, dir);
    uint16_t speedVal = (l << 8) | r;
    wiringPiI2CWriteReg16(fd1, 0x82, speedVal);
    wiringPiI2CWriteReg16(fd2, 0x82, 0x0000);
}

// === Predict where ball crosses goal line ===
float predictGoalLineX(const std::vector<std::pair<double, cv::Point2f>>& trajectory) {
    if (trajectory.size() < 3) {
        std::cerr << "[predictGoalLineX] Not enough points for prediction.\n";
        return -1;
    }

    float sumX = 0, sumY = 0, sumY2 = 0, sumXY = 0;
    int n = trajectory.size();

    for (const auto& [t, pt] : trajectory) {
        float x = pt.x;
        float y = pt.y;
        sumX  += x;
        sumY  += y;
        sumY2 += y * y;
        sumXY += x * y;
    }

    float denominator = (n * sumY2 - sumY * sumY);
    if (denominator == 0) {
        std::cerr << "[predictGoalLineX] Linear regression failed.\n";
        return -1;
    }

    float a = (n * sumXY - sumX * sumY) / denominator;
    float b = (sumX - a * sumY) / n;
    float predictedX = b;

    std::cout << "[Trajectory] Predicted impact at X = " << predictedX << " cm (Y = 0)\n";
    return predictedX;
}

// === Main Program ===
int main() {
    lccv::PiCamera cam;
    cam.options->video_width = 640;
    cam.options->video_height = 480;
    cam.options->framerate = 30;
    cam.options->verbose = true;
    cam.startVideo();

    std::vector<std::pair<double, cv::Point2f>> ballTrajectory;
    const double interval = 0.1;
    
    float botX = 0.0f;
    const float Kp = 10.0f;
    const int maxSpeed = 250;
    const float movementThreshold = 2.0f;

    int fd1, fd2;
    initMotorDriver(fd1, fd2);
    
    auto lastTime = std::chrono::steady_clock::now();
    
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

            cv::Mat hsv, mask1, mask2, redMask;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
            cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
            redMask = mask1 | mask2;

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(redMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            if (!contours.empty()) {
                auto maxContour = *std::max_element(contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
                        return cv::contourArea(c1) < cv::contourArea(c2);
                    });

                // Fit a circle to get the pixel diameter
                cv::Point2f circleCenter;
                float radius;
                cv::minEnclosingCircle(maxContour, circleCenter, radius);
                float diameter_px = radius * 2.0f;
                float depth_cm = estimateBallDepth(diameter_px);

                float pixelsPerCm = diameter_px / BALL_DIAMETER_CM;
                float x_cm = (circleCenter.x - (frame.cols / 2.0f)) / pixelsPerCm;

                cv::Point2f realCenter(x_cm, depth_cm);
                double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
                ballTrajectory.emplace_back(timestamp, realCenter);

                if (realCenter.y <= 0.0f) {
                    ballTrajectory.clear();
                    std::cout << "[Info] Ball reached goal line, trajectory reset.\n";
                }

                std::cout << "Time: " << timestamp
                          << " | Ball 3D Position: (X = " << realCenter.x
                          << " cm, Y = " << realCenter.y << " cm, diameter = "
                          << diameter_px << " px)\n";

                cv::circle(frame, circleCenter, (int)radius, cv::Scalar(0, 255, 0), 2);
                cv::putText(frame, "Z = " + std::to_string(depth_cm) + " cm",
                            circleCenter + cv::Point2f(10, -20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(0, 255, 255), 1);

                if (ballTrajectory.size() >= 5) {
                    float predictedX = predictGoalLineX(ballTrajectory);
                    float error = predictedX - botX;

                    if (std::abs(error) > movementThreshold) {
                        int direction = (error > 0) ? 1 : -1;
                        int speed = 250;
			// std::min(maxSpeed, static_cast<int>(std::abs(error) * Kp));
                        speed *= direction;

                        setMotors(fd1, fd2, speed, speed, 0);
                        botX = predictedX;

                        std::cout << "[Move] Bot moves " << (direction > 0 ? "RIGHT" : "LEFT")
                                  << " with speed " << speed << std::endl;
                    } else {
                        setMotors(fd1, fd2, 0, 0, 0);
                        std::cout << "[Hold] Bot in position.\n";
                    }
                }
            }

            cv::imshow("Ball Detection", frame);
            if (cv::waitKey(1) == 27) break;
        }
    }

    cam.stopVideo();
    cv::destroyAllWindows();
    return 0;
}
