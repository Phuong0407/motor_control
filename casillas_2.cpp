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
constexpr double    DEADZONE_SCALEUP    = 0.843137254901961;
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
double prev_x = 0.0;
double prev_z = 0.0;
double curr_x = 0.0;
double curr_z = 0.0;
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

        double ball_diam = static_cast<double>(radius) * 2.0;
        curr_z = DEPTH_MULTIPLIER / ball_diam;
        curr_x = ((static_cast<double>(center.x - FRAME_WIDTH / 2)) / ball_diam) * BALL_DIAMETER_CM;

        cv::circle(frame, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, "Z = " + std::to_string(z).substr(0, 5) + " cm " + "X =" + std::to_string(z).substr(0, 5) + " cm",
                    center + cv::Point2f(10, -20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 255), 1);
    }
    cv::imshow("IMAGE", frame);
}

inline int computePWMFromUnsignedRPS(double uspeed) {
    int pwm_value = std::clamp(static_cast<int>(uspeed), 0, MAX_PWM);
    if (pwm_value < DEAD_PWM) return 0;
    return static_cast<int>((pwm_value - DEAD_PWM) / DEADZONE_SCALEUP);
}

inline int computeDirection(int dir) {
    return (dir == +1) ? 0x06 : 0x0a;
}

void setMotors() {
    printf("[Motor] SPEED\t=\t%.3f\n", speed);
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
    cam.options->verbose        = VERBOSE;
    cam.startVideo();

    wiringPiSetup();
    i2c_fd = wiringPiI2CSetup(0x0f);

    double kp_x = 2.0, kp_z = 2.0;

    while (true) {
        if (!cam.getVideoFrame(frame, 1000)) {
            printf("[ERROR] Timeout error while grabbing frame.\n");
            continue;
        }
        extractBallCenter();

        printf("x = %.3f\tz = %.3f\t\n", curr_x, curr_z);

        speed = kp_x * (curr_x - prev_x) + kp_z * (curr_z - prev_z);
        prev_x = curr_x;
        prev_z = curr_z;
        setMotors();
        char key = static_cast<char>(cv::waitKey(5));
        if (key == 27) break;
    }
    cam.stopVideo();
    cv::destroyAllWindows();
    wiringPiI2CWriteReg16(i2c_fd, 0x82, 0x0000);
    return 0;
}