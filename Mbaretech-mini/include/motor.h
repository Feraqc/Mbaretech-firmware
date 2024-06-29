#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include "driver/ledc.h"

#define RESOLUTION 10
#define PERIOD 20000

// Define pulse width values in microseconds
#define MIN_PULSE_WIDTH 1080
#define MID_PULSE_WIDTH 1500
#define MAX_PULSE_WIDTH 1920

// Motor specs  //TODO cambiar en MEGA tambien
#define FREQUENCY 50


class Motor{
    public:
        uint32_t currentSpeed;
        uint8_t pwmPin;
        ledc_channel_t pwmChannel;
        ledc_channel_config_t ledc_channel;

        Motor(uint8_t pwmPin_, ledc_channel_t pwmChannel_){
            pwmPin = pwmPin_;
            pwmChannel = pwmChannel_;
        }

        void begin(){
            pinMode(pwmPin, OUTPUT);
            
            ledc_timer_config_t ledc_timer = {
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .duty_resolution = LEDC_TIMER_10_BIT,
                .timer_num = LEDC_TIMER_0,
                .freq_hz = FREQUENCY,
                .clk_cfg = LEDC_AUTO_CLK
            };
            ledc_timer_config(&ledc_timer);

            ledc_channel_config_t ledc_channel = {
                .gpio_num = pwmPin,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel = pwmChannel,
                .intr_type = LEDC_INTR_DISABLE,
                .timer_sel = LEDC_TIMER_0,
                .duty = 0
            };
            ledc_channel_config(&ledc_channel);
        }

        void setSpeed(uint32_t percentage) {
            uint32_t pulseWidth = percentageToPulseWidth(percentage);
            uint32_t dutyCycle = usToDutyCycle(pulseWidth);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel, dutyCycle);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel);
            currentSpeed = percentage;
        }

        void forward(uint32_t percentage) {
            setSpeed(percentage);
        }

        void reverse(uint32_t percentage) {
            setSpeed(percentage);
        }

        void brake() {
            setSpeed(0);
        }

        uint32_t usToDutyCycle(int pulseWidth) {
            uint32_t maxDutyCycle = pow(2, RESOLUTION) - 1;
            uint32_t dutyCycle = (pulseWidth * maxDutyCycle) / PERIOD;
            return dutyCycle;
        }

        uint32_t percentageToPulseWidth(uint32_t percentage) {
            uint32_t pulseWidth = MIN_PULSE_WIDTH + ((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) * percentage / 100);
            return pulseWidth;
        }
};

#endif 