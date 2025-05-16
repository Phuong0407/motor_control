#ifndef CONFIG_H
#define CONFIG_H

#include <wiringPi.h>
#include <wiringPiI2C.h>

constexpr int encoder_addr1 = 0x0f;
constexpr int encoder_addr2 = 0x0d;

int i2c_fd1 = -1;
int i2c_fd2 = -1;

static constexpr int MOTOR1_H1 = 21;
static constexpr int MOTOR1_H2 = 22;
static constexpr int MOTOR2_H1 = 3;
static constexpr int MOTOR2_H2 = 4;
static constexpr int MOTOR3_H1 = 27;
static constexpr int MOTOR3_H2 = 0;

static constexpr double MIN_ERROR_RPS = 0.08;
static constexpr double COUNTER_PER_REV = 144.0;

static constexpr double smpl_itv = 0.1;

#endif // CONFIG_H