#define NOLOADED_RUN
#include "../include/motor/motor.hpp"
#include <iostream>

int main() {
    
    startEncoders();
    controlAngularVelocity(0.625, 0.625, 0.625);
    stop_motor();
    return 0;
}
