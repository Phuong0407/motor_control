all: moteur motor_pulse_per_revolution test_motor


moteur: moteur.cpp
	g++ -o moteur moteur.cpp -lwiringPi

motor_pulse_per_revolution: motor_pulse_per_revolution.cpp
	g++ -o $@ $< -lwiringPi

test_motor: test_motor.cpp
	g++ -o $@ $< -lwiringPi
