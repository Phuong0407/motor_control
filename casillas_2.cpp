#include <opencv2/opencv.hpp>
#include <lccv.hpp>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>
#include <stdio.h>
#include <algorithm>

constexpr double    BALL_DIAMETER_CM    = 4.6;
constexpr double    DEPTH_MULTIPLIER    = 1592.06;
constexpr int       MAX_PWM             = 255;
constexpr int       DEAD_PWM            = 40;
constexpr double    DEADZONE_SCALEUP    = 0.843137254901961;
constexpr int       FRAME_WIDTH         = 640;
constexpr int       FRAME_HEIGHT        = 480;
constexpr int       FRAME_RATE          = 90;
constexpr bool      VERBOSE             = false;

using Contour_t  = std::vector<cv::Point>;
using Contours_t = std::vector<Contour_t>;

cv::Mat     frame;
cv::Mat     bin_mask;
Contours_t  contours;

int i2c_fd = -1;

bool CONTAINS_BALL = false;

int pwm = 0;
double prev_x = 0.0;
double prev_z = 0.0;
double curr_x = 0.0;
double curr_z = 0.0;
double speed = 0.0;

void extractBallCenter(double &deviation, double &depth) {
    cv::Mat hsv, mask1, mask2, mask3, bin_mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 120, 70),    cv::Scalar(10, 255, 255),   mask1);
    cv::inRange(hsv, cv::Scalar(170, 120, 70),  cv::Scalar(180, 255, 255),  mask2);
    cv::inRange(hsv, cv::Scalar(100,150,50),    cv::Scalar(140, 255, 255),  mask3);
    bin_mask = mask1 | mask2 | mask3;
    cv::erode(bin_mask, bin_mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(bin_mask, bin_mask, cv::Mat(), cv::Point(-1, -1), 2);

    contours.clear();
    cv::findContours(bin_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    CONTAINS_BALL = false;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 200.0) {
            CONTAINS_BALL |= false;
            continue;
        } else {
            CONTAINS_BALL |= true;
        }

        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);

        double ball_diam = static_cast<double>(radius) * 2.0;
        depth = DEPTH_MULTIPLIER / ball_diam;
        deviation = ((static_cast<double>(center.x - FRAME_WIDTH / 2)) / ball_diam) * BALL_DIAMETER_CM;

        cv::circle(frame, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, "Z = " + std::to_string(depth).substr(0, 5) + " cm " + "X =" + std::to_string(deviation).substr(0, 5) + " cm",
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
    int dir = (speed > 0) ? 0x06 : 0x09;
    pwm = computePWMFromUnsignedRPS(std::abs(speed));

    wiringPiI2CWriteReg16(i2c_fd, 0x82, (pwm << 8) | pwm);
    delay(1);
    wiringPiI2CWriteReg16(i2c_fd, 0xaa, dir);
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

    double kp_x = 100.0, kp_z = 100.0;

    while (true) {
        auto t0 = std::chrono::high_resolution_clock::now();
        if (!cam.getVideoFrame(frame, 1000)) {
            printf("[ERROR] Timeout error while grabbing frame.\n");
            continue;
        }
        extractBallCenter(curr_x, curr_z);
        auto t1 = std::chrono::high_resolution_clock::now();
        if (!CONTAINS_BALL)
            break;
        double urgency = std::clamp((70.0- curr_z) / 70.0, 0.3, 1.0);
        if (std::abs(curr_x) <= 5.0)
            speed = 0.0;
        else
            speed = - kp_x * curr_x * urgency / 0.028;

        setMotors();
        auto t2 = std::chrono::high_resolution_clock::now();
        double detect_time = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double act_time    = std::chrono::duration<double, std::milli>(t2 - t0).count();
        printf("[TIMING] Detection: %.3f ms | Full response: %32f ms\n", detect_time, act_time);
        char key = static_cast<char>(cv::waitKey(5));
        if (key == 27) break;
    }
    printf("[INFO] NO BALL FOUND. STOP.\n");
    cam.stopVideo();
    cv::destroyAllWindows();
    wiringPiI2CWriteReg16(i2c_fd, 0x82, 0x0000);
    return 0;
}
