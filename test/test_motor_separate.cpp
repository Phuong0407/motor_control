#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// Pin Definitions
#define IN1_PIN  5   // Motor A Direction 1
#define IN2_PIN  6   // Motor A Direction 2
#define ENA_PIN  12  // Motor A PWM

#define IN3_PIN  13  // Motor B Direction 1
#define IN4_PIN  19  // Motor B Direction 2
#define ENB_PIN  26  // Motor B PWM

void setupPins() {
    wiringPiSetupGpio();  // Use BCM GPIO numbering

    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(ENA_PIN, PWM_OUTPUT);

    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);
    pinMode(ENB_PIN, PWM_OUTPUT);

    // Set PWM range to 1024 (default is 1024)
    pwmSetRange(1024);
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(384);  // Adjust to change PWM frequency (1.2 kHz approx)
}

void controlMotorA(int speed, int direction) {
    // Clamp speed to 0 - 1024
    speed = (speed > 1024) ? 1024 : (speed < 0) ? 0 : speed;

    // Set Direction
    if (direction == 1) {
        digitalWrite(IN1_PIN, HIGH);
        digitalWrite(IN2_PIN, LOW);
    } else if (direction == -1) {
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, HIGH);
    } else {
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, LOW);
    }

    // Set PWM speed
    pwmWrite(ENA_PIN, speed);
}

void controlMotorB(int speed, int direction) {
    // Clamp speed to 0 - 1024
    speed = (speed > 1024) ? 1024 : (speed < 0) ? 0 : speed;

    // Set Direction
    if (direction == 1) {
        digitalWrite(IN3_PIN, HIGH);
        digitalWrite(IN4_PIN, LOW);
    } else if (direction == -1) {
        digitalWrite(IN3_PIN, LOW);
        digitalWrite(IN4_PIN, HIGH);
    } else {
        digitalWrite(IN3_PIN, LOW);
        digitalWrite(IN4_PIN, LOW);
    }

    // Set PWM speed
    pwmWrite(ENB_PIN, speed);
}

int main(int argc, char *argv[]) {
    if (argc != 5) {
        printf("Usage: %s <motorA_speed> <motorA_dir> <motorB_speed> <motorB_dir>\n", argv[0]);
        printf("Direction: 1 = Forward, -1 = Backward, 0 = Stop\n");
        return 1;
    }

    // Parse command line arguments
    int motorA_speed = atoi(argv[1]);
    int motorA_dir = atoi(argv[2]);
    int motorB_speed = atoi(argv[3]);
    int motorB_dir = atoi(argv[4]);

    setupPins();

    // Control Motors
    controlMotorA(motorA_speed, motorA_dir);
    controlMotorB(motorB_speed, motorB_dir);

    printf("Motor A: Speed %d, Direction %d\n", motorA_speed, motorA_dir);
    printf("Motor B: Speed %d, Direction %d\n", motorB_speed, motorB_dir);

    printf("Press CTRL+C to exit...\n");
    while (1) {
        sleep(1);
    }

    return 0;
}