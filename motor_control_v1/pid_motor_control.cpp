// This is the file to test the speed control of motor using PID

#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <chrono>

constexpr int PPR_EDGE_RISING = 3;
constexpr int PPR_EDGE_BOTH = 6;

volatile int motor_pulse_counter = 0;

void pulse_count(void) {
    motor_pulse_counter++;
}

int pulse_to_rpm(void) {
    static auto last_time = std::chrono::steady_clock::now();
    static int last_pulse_count = 0; 

    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>()
}

int rpm_to_pulse(int rpm) {
   // TO DO
}

int maint() {
    wiringPiSetup();
	int fd = wiringPiI2CSetup(0x0f);

	pinMode(3, INPUT);
	pullUpDnControl(3, PUD_UP);
	wiringPiISR(3, INT_EDGE_RISING, &pulse_ISR_count);

	while(true) {
            // TO DO
	}

//  wiringPiI2CWriteReg16(fd, 0x82, 0x0f0f);
//  wiringPiI2CWriteReg16(fd, 0xaa, 0x05);
//  wiringPiISR(3, INT_EDGE_RISING, &pulse_ISR_count);


	return 0;
}
