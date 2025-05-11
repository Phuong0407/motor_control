#define NOLOADED_RUN
#include "../include/motor/motor.hpp"
#include <iostream>

int main(int argc, char* argv[]) {
    if (argc < 3)
        return 0;
    
    double omega1 = std::stod(argv[1]);
    double omega2 = std::stod(argv[2]);
    double omega3 = std::stod(argv[3]);
    controlAngularVelocity(0.625, 0.625, 0.625);
    stop_motor();
    return 0;
}
