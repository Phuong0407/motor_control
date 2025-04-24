all: motor_calibration

motor_calibration: motor_calibration.cpp
	g++ -o $@ $< -lwiringPi