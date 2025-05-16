#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// Motor A (Driver 1)
#define IN1_PIN  5
#define IN2_PIN  6
#define ENA_PIN  12  // PWM0

// Motor B (Driver 1)
#define IN3_PIN  13
#define IN4_PIN  19
#define ENB_PIN  26  // PWM1

// Motor C (Driver 2)
#define IN5_PIN  16
#define IN6_PIN  20
#define ENC_PIN  21  // PWM1

void setupPins() {
    wiringPiSetupGpio();  // Use BCM GPIO numbering

    // Motor A
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(ENA_PIN, PWM_OUTPUT);

    // Motor B
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);
    pinMode(ENB_PIN, PWM_OUTPUT);

    // Motor C
    pinMode(IN5_PIN, OUTPUT);
    pinMode(IN6_PIN, OUTPUT);
    pinMode(ENC_PIN, PWM_OUTPUT);

    // Set PWM frequency and range
    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(1024);
    pwmSetClock(384);  // ~1.2 kHz PWM frequency
}

void controlMotor(int in1, int in2, int pwmPin, int speed, int direction) {
    // Clamp speed to 0 - 1024
    speed = (speed > 1024) ? 1024 : (speed < 0) ? 0 : speed;

    // Set direction
    if (direction == 1) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (direction == -1) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }

    // Set PWM speed
    pwmWrite(pwmPin, speed);
}

int main(int argc, char *argv[]) {
    if (argc != 7) {
        printf("Usage: %s <MotorA_speed> <MotorA_dir> <MotorB_speed> <MotorB_dir> <MotorC_speed> <MotorC_dir>\n", argv[0]);
        printf("Direction: 1 = Forward, -1 = Backward, 0 = Stop\n");
        return 1;
    }

    int motorA_speed = atoi(argv[1]);
    int motorA_dir = atoi(argv[2]);
    int motorB_speed = atoi(argv[3]);
    int motorB_dir = atoi(argv[4]);
    int motorC_speed = atoi(argv[5]);
    int motorC_dir = atoi(argv[6]);

    setupPins();

    // Control Motors
    controlMotor(IN1_PIN, IN2_PIN, ENA_PIN, motorA_speed, motorA_dir);
    controlMotor(IN3_PIN, IN4_PIN, ENB_PIN, motorB_speed, motorB_dir);
    controlMotor(IN5_PIN, IN6_PIN, ENC_PIN, motorC_speed, motorC_dir);

    printf("Motor A: Speed %d, Direction %d\n", motorA_speed, motorA_dir);
    printf("Motor B: Speed %d, Direction %d\n", motorB_speed, motorB_dir);
    printf("Motor C: Speed %d, Direction %d\n", motorC_speed, motorC_dir);

    printf("Press CTRL+C to exit...\n");
    while (1) {
        sleep(1);
    }

    return 0;
}