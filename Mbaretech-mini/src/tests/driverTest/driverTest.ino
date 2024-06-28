#include <Arduino.h>
#include "driver/ledc.h"
#include <math.h>

// Define pulse width values in microseconds
#define MIN_PULSE_WIDTH 1080
#define MID_PULSE_WIDTH 1500
#define MAX_PULSE_WIDTH 1920

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 37

#define RESOLUTION 14

// LEDC (PWM) settings
const int freq = 50;  // 50Hz frequency for RC control
const int period = 20000; // 20 milliseconds (20000 microseconds).

#define leftMotorChannel LEDC_CHANNEL_0
#define rightMotorChannel LEDC_CHANNEL_1

uint32_t usToDutyCycle(int pulseWidth) {
    uint32_t maxDutyCycle = pow(2, RESOLUTION) - 1;
    uint32_t dutyCycle = (pulseWidth * maxDutyCycle) / period;
    return dutyCycle;
}

void setup() {
    Serial.begin(9600);
    // Motors
    pinMode(MOTOR_LEFT, OUTPUT);
    pinMode(MOTOR_RIGHT, OUTPUT);

    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = freq,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configure LEDC channels
ledc_channel_config_t ledc_channel_left = {
    .gpio_num = MOTOR_LEFT,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = leftMotorChannel,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0
};

ledc_channel_config(&ledc_channel_left);

ledc_channel_config_t ledc_channel_right = {
    .gpio_num = MOTOR_RIGHT,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = rightMotorChannel,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0
    };

    ledc_channel_config(&ledc_channel_right);

    // Start motors at 0
    ledc_set_duty(LEDC_LOW_SPEED_MODE, leftMotorChannel, usToDutyCycle(MID_PULSE_WIDTH));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, leftMotorChannel);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, rightMotorChannel, usToDutyCycle(MID_PULSE_WIDTH));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, rightMotorChannel);
}

void loop() {
    // Test the motors with minimum pulse width
    Serial.println("Setting motors to minimum pulse width");
    ledc_set_duty(LEDC_LOW_SPEED_MODE, leftMotorChannel, usToDutyCycle(MIN_PULSE_WIDTH));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, leftMotorChannel);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, rightMotorChannel, usToDutyCycle(MIN_PULSE_WIDTH));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, rightMotorChannel);

    delay(2000); // Wait for 2 seconds

    // Test the motors with middle pulse width
    Serial.println("Setting motors to middle pulse width");
    ledc_set_duty(LEDC_LOW_SPEED_MODE, leftMotorChannel, usToDutyCycle(MID_PULSE_WIDTH));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, leftMotorChannel);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, rightMotorChannel, usToDutyCycle(MID_PULSE_WIDTH));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, rightMotorChannel);

    delay(2000); // Wait for 2 seconds

    // Test the motors with maximum pulse width
    Serial.println("Setting motors to maximum pulse width");
    ledc_set_duty(LEDC_LOW_SPEED_MODE, leftMotorChannel, usToDutyCycle(MAX_PULSE_WIDTH));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, leftMotorChannel);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, rightMotorChannel, usToDutyCycle(MAX_PULSE_WIDTH));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, rightMotorChannel);

    delay(2000); // Wait for 2 seconds
}

