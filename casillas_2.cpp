#include <opencv2/opencv.hpp>
#include <lccv.hpp>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>
#include <stdio.h>

constexpr double    BALL_DIAMETER_CM    = 4.6;
constexpr double    DEPTH_MULTIPLIER    = 1592.06;
constexpr int       MAX_PWM             = 255;
constexpr int       DEAD_PWM            = 40;
constexpr int       FRAME_WIDTH         = 640;
constexpr int       FRAME_HEIGHT        = 480;
constexpr int       FRAME_RATE          = 30;
constexpr bool      VERBOSE             = false;

using Contour_t  = std::vector<cv::Point>;
using Contours_t = std::vector<Contour_t>;

cv::Mat     frame;
cv::Mat     bin_mask;
Contours_t  contours;

int i2c_fd = -1;

int pwm = 0;
double x = 0.0;
double z = 0.0;
double speed = 0.0;

void extractBallCenter() {
    cv::Mat hsv, mask1, mask2, bin_mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 120, 70),    cv::Scalar(10, 255, 255),   mask1);
    cv::inRange(hsv, cv::Scalar(170, 120, 70),  cv::Scalar(180, 255, 255),  mask2);
    bin_mask = mask1 | mask2;
    cv::erode(bin_mask, bin_mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(bin_mask, bin_mask, cv::Mat(), cv::Point(-1, -1), 2);

    contours.clear();

    cv::findContours(bin_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 200.0) continue;

        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);

        z = DEPTH_MULTIPLIER / static_cast<double>(radius) * 0.5;
        x = ((static_cast<double>(center.x - FRAME_WIDTH / 2)) / diameter_px) * BALL_DIAMETER_CM;

        cv::circle(frame, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, "Z = " + std::to_string(z).substr(0, 5) + " cm",
                    center + cv::Point2f(10, -20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 255), 1);
    }
}

inline int computePWMFromUnsignedRPS(double utps) {
    double norm_rps = std::clamp(utps / MAX_TPS, 0.0, 1.0);
    int pwm_value = static_cast<int>(MAX_PWM * norm_rps * SAFETY_OFFSET);
    if (pwm_value < DEAD_PWM) return 0;
    return static_cast<int>((pwm_value - DEAD_PWM) / DEADZONE_SCALEUP);
}

inline int computeDirection(int dir) {
    return (dir == +1) ? 0x06 : 0x0a;
}

void setMotors() {
    printf("[Motor] L\t=\t%.3f\tR\t=\t%.3f", leftSpeed, rightSpeed);
    int dir = (speed > 0) ? +1 : -1;
    dir = computeDirection(dir);
    pwm = computePWMFromUnsignedRPS(std::abs(speed));

    wiringPiI2CWriteReg16(i2c_fd, 0xaa, dir);
    wiringPiI2CWriteReg16(i2c_fd, 0x82, (pwm << 8) | pwm);
}

int main() {

    lccv::PiCamera cam;
    cam.options->video_width    = FRAME_WIDTH;
    cam.options->video_height   = FRAME_HEIGHT;
    cam.options->framerate      = FRAME_RATE;
    cam.options->verbose        = VERBOSE
    cam.startVideo();

    double kp_x = 10.0, kp_z = 10.0;

    while (true) {
        cv::Mat frame;
        if (!cam.getVideoFrame(frame, 1000)) {
            printf("[ERROR] Timeout error while grabbing frame.\n");
            continue;
        }
        extractBallCenter();
        speed = kp_x * x + kd_z * z;     
        setMotors();   
    }
    cam.stopVideo();
    cv::destroyAllWindows();
    return 0;
}