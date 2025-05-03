all: test_motor_driver test_encoder


test_motor_driver: test_motor_driver.cpp motor_driver.hpp pid_control.hpp encoder.hpp
	g++ -o moteur moteur.cpp -lwiringPi

test_encoder: test_encoder.cpp encoder.hpp
	g++ -o moteur moteur.cpp -lwiringPi