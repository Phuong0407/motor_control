#include "encoder.h"

int main() {
    counter = 0;
    init_encoder();
    wiringPiISR(H2_PIN, INT_EDGE_RISING, update_motor_position);

    delay(5000);
    printf("%d\n", counter);

    return 0;
}
