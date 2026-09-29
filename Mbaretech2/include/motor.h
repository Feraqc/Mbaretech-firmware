
#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include "firmwareConfig.h"
#include "driver/ledc.h"



#define FREQUENCY 20000 //40000

//MOTOR A
#define PWM_B 48
#define PIN_B0 47
#define PIN_B1 20 //45
#define CHANNEL_LEFT LEDC_CHANNEL_1

//MOTORB
#define PWM_A 35 // En el diagrama esta al reves, ignorar
#define PIN_A0 36
#define PIN_A1 37
#define CHANNEL_RIGHT LEDC_CHANNEL_0

#define MAX_DUTY_VALUE 990

class Motor{
    public:
        uint32_t currentSpeed = 0; // Applied PWM duty (0..990).
        uint8_t pwmPin;
        uint8_t A0pin;
        uint8_t A1pin;
        ledc_channel_t pwmChannel;
        ledc_channel_config_t ledc_channel{};

        Motor(uint8_t pwmPin_,uint8_t A0pin_,uint8_t A1pin_, ledc_channel_t pwmChannel_){
            pwmPin = pwmPin_;
            A0pin = A0pin_;
            A1pin = A1pin_;
            pwmChannel = pwmChannel_;
        }

        void begin(){
#if ENABLE_MOTORS
            pinMode(pwmPin, OUTPUT);
            pinMode(A0pin, OUTPUT);
            pinMode(A1pin, OUTPUT);

            ledc_timer_config_t ledc_timer = {
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .duty_resolution = LEDC_TIMER_10_BIT,
                .timer_num = LEDC_TIMER_0,
                .freq_hz = FREQUENCY,
                .clk_cfg = LEDC_AUTO_CLK
            };
            ledc_timer_config(&ledc_timer);

            ledc_channel = {
                .gpio_num = pwmPin,
                .speed_mode = LEDC_LOW_SPEED_MODE,
                .channel = pwmChannel,
                .intr_type = LEDC_INTR_DISABLE,
                .timer_sel = LEDC_TIMER_0,
                .duty = 0
            };
            ledc_channel_config(&ledc_channel);
            brake();

#else
            currentSpeed = 0;
#endif
        }

        void setSpeed(uint32_t percentage){
#if ENABLE_MOTORS
            if (percentage > 100) percentage = 100;
            int speed = map(percentage,0,100,0,1023);
            speed = constrain(speed,0,MAX_DUTY_VALUE); // Datasheet dice 98%, le capeo a casi 97% ~ 990
            ledc_set_duty(LEDC_LOW_SPEED_MODE,pwmChannel,speed);
            ledc_update_duty(LEDC_LOW_SPEED_MODE,pwmChannel);
            currentSpeed = speed;
            ledc_channel.duty = speed;

#else
            currentSpeed = 0;
#endif
        }

        void forward(uint32_t speed){
#if ENABLE_MOTORS
            digitalWrite(A0pin,0);
            digitalWrite(A1pin,1);
            setSpeed(speed);

#else
            currentSpeed = 0;
#endif
        }
        void backward(uint32_t speed){
#if ENABLE_MOTORS
            digitalWrite(A0pin,1);
            digitalWrite(A1pin,0);
            setSpeed(speed);

#else
            currentSpeed = 0;
#endif
        }
        void brake(){
#if ENABLE_MOTORS
            setSpeed(0);
            digitalWrite(A0pin,0);
            digitalWrite(A1pin,0);

#else
            currentSpeed = 0;
#endif
        }
};

#endif
