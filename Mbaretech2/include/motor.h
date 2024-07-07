
#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include "driver/ledc.h"
#include "globals.h"


#define FREQUENCY 40000

//MOTOR A
#define PWM_B 48
#define PIN_B0 47
#define PIN_B1 45
#define CHANNEL_LEFT LEDC_CHANNEL_1

//MOTORB
#define PWM_A 35 // En el diagrama esta al reves, ignorar
#define PIN_A0 36
#define PIN_A1 37
#define CHANNEL_RIGHT LEDC_CHANNEL_0

#define MAX_DUTY_VALUE 990

class Motor{
    public:
        uint32_t currentSpeed;
        uint8_t pwmPin;
        uint8_t A0pin;
        uint8_t A1pin;
        ledc_channel_t pwmChannel;
        ledc_channel_config_t ledc_channel;

        Motor(uint8_t pwmPin_,uint8_t A0pin_,uint8_t A1pin_, ledc_channel_t pwmChannel_){
            pwmPin = pwmPin_;
            A0pin = A0pin_;
            A1pin = A1pin_;
            pwmChannel = pwmChannel_;
        }

        void begin(){
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

        void setSpeed(uint32_t percentage){
            int speed = map(percentage,0,100,0,1023);
            speed = constrain(speed,1,MAX_DUTY_VALUE); // Datasheet dice 98%, le capeo a casi 97% ~ 990
            ledc_set_duty(LEDC_LOW_SPEED_MODE,pwmChannel,speed);
            ledc_update_duty(LEDC_LOW_SPEED_MODE,pwmChannel);
            //currentSpeed = speed;
        }

        void forward(uint32_t speed){
            digitalWrite(A0pin,0);
            digitalWrite(A1pin,1);
            setSpeed(speed);
        }
        void backward(uint32_t speed){
            digitalWrite(A0pin,1);
            digitalWrite(A1pin,0);
            setSpeed(speed);
        }
        void brake(){
            digitalWrite(A0pin,0);
            digitalWrite(A1pin,0);
        }
};

#endif 