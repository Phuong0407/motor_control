#ifndef ROBOT_H
#define ROBOT_H

#include "pid.h"

#include <lccv.hpp>

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
double computed3        
PID                     pid1;
PID                     pid2;
PID                     
lccv::PiCamera          cam;

static constexpr int    ADDRESS1                = 0x0f;
static constexpr int    ADDRESS2                = 0x0d;
static constexpr int    MOTOR1_H1               = 21;
static constexpr int    MOTOR1_H2               = 22;
static constexpr int    MOTOR2_H1               = 3;
static constexpr int    MOTOR2_H2               = 4;
static constexpr int    MOTOR3_H1               = 27;
static constexpr int    MOTOR3_H2               = 0;
static constexpr int    MAX_PWM                 = 255;
static constexpr int    DEAD_PWM                = 40;

static constexpr double SAFETY_OFFSET           = 0.8;
static constexpr double DEADZONE_SCALEUP        = 0.843137254901961;
static constexpr double ERROR_THRESHOLD_PERCENT = 0.10;
static constexpr double MIN_ERROR_TPS           = 11.0;

#ifdef NOLOAD_RUN
static constexpr double MAX_TPS                 = 12.0;
#else // LOAD_RUN
static constexpr double MAX_TPS                 = 9.0;
#endif

static           int    i2c_fd1                 = -1;
static           int    i2c_fd2                 = -1;

#endif // ROBOT_H