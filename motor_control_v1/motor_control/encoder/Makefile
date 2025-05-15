all: test_encoder motor_calibration

test_encoder: test_encoder.cpp
	g++ -o $@ $< -lwiringPi

motor_calibration: motor_calibration.cpp
	g++ -o $@ $< -lwiringPi
