#include <wiringPi.h>
#include <iostream>

volatile int pulseCount = 0;  // Global variable to store pulse count

// Interrupt service routine
void pulseISR() {
    pulseCount++;
}

int main() {
    int encoderPin = 0;  // WiringPi pin number (GPIO 17 is WiringPi 0)

    // Initialize WiringPi
    if (wiringPiSetup() == -1) {
        std::cerr << "WiringPi setup failed!" << std::endl;
        return 1;
    }

    // Setup GPIO pin as input with pull-up
    pinMode(encoderPin, INPUT);
    pullUpDnControl(encoderPin, PUD_UP);

    // Attach interrupt: both rising and falling edges
    if (wiringPiISR(encoderPin, INT_EDGE_RISING, &pulseISR) < 0) {
        std::cerr << "Interrupt setup failed!" << std::endl;
        return 1;
    }

    std::cout << "Rotate shaft ONE full revolution, then press Enter..." << std::endl;

    // Wait for user to rotate shaft
    getchar();

    // Output pulse count
    std::cout << "Total pulses counted: " << pulseCount << std::endl;
    std::cout << "PPR = " << pulseCount << " (if counting both edges)" << std::endl;

    return 0;
}
