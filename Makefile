all: test_motor_driver test_encoder pwm

test_motor_driver: test_motor_driver.cpp motor_driver.hpp pid_controller.hpp encoder.hpp
		g++ -o $@ $< -lwiringPi

test_encoder: test_encoder.cpp encoder.hpp
		g++ -o $@ $< -lwiringPi

pwm: pwm.cpp encoder.hpp
		g++ -o $@ $< -lwiringPi

