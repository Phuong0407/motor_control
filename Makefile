all: test_encoder_v3 #motor_calibration

#motor_calibration: motor_calibration.cpp
#		g++ -o $@ $< -lwiringPi -latomic

test_encoder_v3: test_encoder_v3.cpp
		g++ -o $@ $< -lwiringPi -latomic