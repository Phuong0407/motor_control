#ifndef ROBOT_HPP
#define ROBOT_HPP

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
double computed3        = 0.0;
PID                     pid1;
PID                     pid2;
PID                     pid3;
lccv::PiCamera          cam;

#endif // ROBOT_HPP