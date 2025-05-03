XX = g++
CXXFLAGS = -Wall -lwiringPi

SRC = pwm.cpp encoder.hpp
TARGET = pwm

all: $(TARGET)

$(TARGET): $(SRC)
		$(CXX) -o $@ $(SRC) $(CXXFLAGS)

clean:
		rm -f $(TARGET)
