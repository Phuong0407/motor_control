#!/bin/bash

OUTPUT_FILE="motor_calibration_result.txt"
echo "Motor PWM TIME(ms) ENCODER1 ENCODER2 ENCODER3" > "$OUTPUT_FILE"

PWM_STEP=8
TIME_MIN=50
TIME_MAX=1000
TIME_STEP=50

# Function to format hex
to_hex() {
    printf "0x%02X" "$1"
}

for time in $(seq $TIME_MIN $TIME_STEP $TIME_MAX); do
    for pwm in $(seq 0 $((PWM_STEP)) 255); do
        pwm_hex=$(to_hex $pwm)

        # Test motor 1
        echo "Testing Motor 1: PWM=$pwm_hex Time=${time}ms"
        out=$(./calibration $pwm_hex 0x00 0x00 $time)
        e1=$(echo "$out" | grep -oP 'ENCODER COUNTER 1 = \K[0-9-]+')
        echo "1 $pwm_hex $time $e1 0 0" >> "$OUTPUT_FILE"

    done
done

for time in $(seq $TIME_MIN $TIME_STEP $TIME_MAX); do
    for pwm in $(seq 0 $((PWM_STEP)) 255); do
        pwm_hex=$(to_hex $pwm)

        # Test motor 2
        echo "Testing Motor 2: PWM=$pwm_hex Time=${time}ms"
        out=$(./calibration 0x00 $pwm_hex 0x00 $time)
        e2=$(echo "$out" | grep -oP 'ENCODER COUNTER 2 = \K[0-9-]+')
        echo "2 $pwm_hex $time 0 $e2 0" >> "$OUTPUT_FILE"

    done
done

for time in $(seq $TIME_MIN $TIME_STEP $TIME_MAX); do
    for pwm in $(seq 0 $((PWM_STEP)) 255); do
        pwm_hex=$(to_hex $pwm)

        # Test motor 3
        echo "Testing Motor 3: PWM=$pwm_hex Time=${time}ms"
        out=$(./calibration 0x00 0x00 $pwm_hex $time)
        e3=$(echo "$out" | grep -oP 'ENCODER COUNTER 3 = \K[0-9-]+')
        echo "3 $pwm_hex $time 0 0 $e3" >> "$OUTPUT_FILE"

    done
done
echo "Individual motor tests complete. Results saved in $OUTPUT_FILE"

