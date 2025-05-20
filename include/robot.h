#ifndef ROBOT_H
#define ROBOT_H

#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>

#include <inttypes.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <stdio.h>

double  L1              = 0.115;
double  L2              = 0.230;
double  r               = 0.028;

int x                   = 0;
int pwm1                = 0;
int pwm2                = 0;
int pwm3                = 0;
int dir1                = 1;
int dir2                = 1;
int dir3                = 1;
double ref1             = 0.0;
double ref2             = 0.0;
double ref3             = 0.0;
double measured1        = 0.0;
double measured2        = 0.0;
double measured3        = 0.0;
double computed1        = 0.0;
double computed2        = 0.0;
double computed3        = 0.0;

static constexpr int    ADDRESS1                    = 0x0f;
static constexpr int    ADDRESS2                    = 0x0d;
static           int    i2c_fd1                     = -1;
static           int    i2c_fd2                     = -1;

static constexpr int    MOTOR1_H1                   = 21;
static constexpr int    MOTOR1_H2                   = 22;
static constexpr int    MOTOR2_H1                   = 3;
static constexpr int    MOTOR2_H2                   = 4;
static constexpr int    MOTOR3_H1                   = 27;
static constexpr int    MOTOR3_H2                   = 0;
static constexpr int    MAX_PWM                     = 255;
static constexpr int    DEAD_PWM                    = 40;
static constexpr double SAFETY_OFFSET               = 0.8;
static constexpr double DEADZONE_SCALEUP            = 0.843137254901961;
static constexpr double ERROR_THRESHOLD_PERCENT     = 0.10;
static constexpr double MIN_ERROR_TPS               = 11.0;


#ifdef NOLOAD_RUN
static constexpr double MAX_TPS                     = 12.0;
#else // LOAD_RUN
static constexpr double MAX_TPS                     = 9.0;
#endif


constexpr double        kp1                         = 0.3;
constexpr double        ki1                         = 0.005;
constexpr double        kd1                         = 0.0050;
constexpr double        cutoff1                     = 4.0;

constexpr double        kp2                         = 0.3;
constexpr double        ki2                         = 0.005;
constexpr double        kd2                         = 0.0050;
constexpr double        cutoff2                     = 4.0;

constexpr double        kp3                         = 0.3;
constexpr double        ki3                         = 0.005;
constexpr double        kd3                         = 0.0050;
constexpr double        cutoff3                     = 4.0;

volatile  int64_t       counter1                    = 0;
volatile  int64_t       counter2                    = 0;
volatile  int64_t       counter3                    = 0;


lccv::PiCamera   cam;
static constexpr int    framewidth                  = 640;
static constexpr int    frameheight                 = 480;
static constexpr int    framerate                   = 30;
static constexpr bool   verbose                     = false;



constexpr unsigned int  N_SLICES                    = 5;
constexpr double        MIN_CONTOUR_AREA            = 100.0;
constexpr double        MAX_EXTENT_RATIO            = 0.7;
constexpr int           CONTOUR_OFFSET_THRESHOLD    = 5;
const     cv::Scalar    CONTOUR_COLOR               = cv::Scalar(0, 255, 0);
const     cv::Scalar    IMAGE_CENTER_COLOR          = cv::Scalar(0, 0, 255);
const     cv::Scalar    CONTOUR_CENTER_COLOR        = cv::Scalar(0, 0, 0);
const     cv::Scalar    TEXT_COLOR                  = cv::Scalar(200, 0, 200);
constexpr int           MARKER_RADIUS               = 5;
constexpr int           TEXT_OFFSET_Y               = 30;
constexpr int           FOUND_LINE                  = 1;
constexpr int           NO_LINE_FOUND               = 0;
using                   Contour_t                   = std::vector<cv::Point>;
using                   Contours_t                  = std::vector<Contour_t>;



cv::Mat                 img;
cv::Mat                 output;
cv::Mat                 bin_mask;

bool                    contain_lines[N_SLICES]     = {true};
int                     img_center_xs[N_SLICES]     = {0};
int                     img_center_ys[N_SLICES]     = {0};
int                     center_xs[N_SLICES]         = {0};
int                     center_ys[N_SLICES]         = {0};
int                     dir_offsets[N_SLICES]       = {0};
double                  extents[N_SLICES]           = {0.0};


constexpr int           TURN_THRESHOLD              = 10;
constexpr double        TURN_SPEED_DECREASE         = 0.5;
constexpr double        TURN_SPEED_INCREASE         = 0.5;
constexpr double        kp_dir                      = 0.005;
constexpr double        kd_dir                      = 0.01;

int                     dir_offset_diffs[N_SLICES]  = {0};
double                  base_speed                  = 0.1;
double                  omega                       = 0.0;

#endif // ROBOT_H