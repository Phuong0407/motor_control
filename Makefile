XX = g++
CXXFLAGS = -Wall -lwiringPi

SRC = test_motor_driver.cpp motor_driver.hpp pid_controler.hpp encoder.hpp
TARGET = test_motor_driver

all: $(TARGET)

$(TARGET): $(SRC)
		$(CXX) -o $@ $(SRC) $(CXXFLAGS)

clean:
		rm -f $(TARGET)
