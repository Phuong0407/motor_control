CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17 -lwiringPi
INCLUDES = -I..

TARGETS = calibration

all: $(TARGETS)

calibration: calibration.cpp ../encoder.hpp
		$(CXX) calibration.cpp -o calibration $(CXXFLAGS) $(INCLUDES)

clean:
		rm -f $(TARGETS)

