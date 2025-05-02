all: test_encoder_v3 test_3_motor #motor_calibration

#motor_calibration: motor_calibration.cpp
#		g++ -o $@ $< -lwiringPi -latomic

test_encoder_v3: test_encoder_v3.cpp
		g++ -o $@ $< -lwiringPi

test_3_motor: test_3_motor.cpp
		g++ -o $@ $< -lwiringPi
