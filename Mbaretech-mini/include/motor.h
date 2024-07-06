#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include <ESP32Servo.h>

#define RESOLUTION 10
#define PERIOD 20000

// Define pulse width values in microseconds
#define MIN_WIDTH 1080
#define MID_WIDTH 1500
#define MAX_WIDTH 1920

#define FORWARD_MIN_RIGHT  1425
#define FORWARD_MIN_LEFT  1406
#define FORWARD_MAX_RIGHT  1080
#define FORWARD_MAX_LEFT  1080

#define BACKWARD_MIN_RIGHT 1565
#define BACKWARD_MIN_LEFT 1580
#define BACKWARD_MAX_RIGHT 1965
#define BACKWARD_MAX_LEFT 1965

#define MAX_DUTY_CYCLE 1023

// Motor specs  //TODO cambiar en MEGA tambien
#define FREQUENCY 50
class Motor{
    public:
        uint32_t currentSpeed;
        uint8_t pwmPin;
        Servo motor;
        int maxForwardPulse;
        int minForwardPulse;
        int maxBackwardPulse;
        int minBackwarPulse;

        Motor(uint8_t pwmPin_,int maxForwardPulse_,int minForwardPulse_,int maxBackwardPulse_,int minBackwarPulse_){
            pwmPin = pwmPin_;
            maxForwardPulse = maxForwardPulse_;
            minForwardPulse = minForwardPulse_;
            maxBackwardPulse = maxBackwardPulse_;
            minBackwarPulse = minBackwarPulse_;
        }

        void begin(){
            pinMode(pwmPin, OUTPUT);
            ESP32PWM::allocateTimer(0);
            ESP32PWM::allocateTimer(1);
            ESP32PWM::allocateTimer(2);
            ESP32PWM::allocateTimer(3);
            motor.setPeriodHertz(FREQUENCY);
            motor.attach(pwmPin,1080,1920);
            motor.writeMicroseconds(1500);
        }

        void setSpeed(int speed) {
            motor.writeMicroseconds(speed);
        }

        void forward(int percentage) {
            int speed = map(percentage,0,100,minForwardPulse,maxForwardPulse);
            motor.writeMicroseconds(speed);
        }

        void backward(int percentage) {
            int speed = map(percentage,0,100,minBackwarPulse,maxBackwardPulse);
            motor.writeMicroseconds(speed);
        }

        void brake() {
            motor.writeMicroseconds(1500);
            currentSpeed = 1500;
        }



};
#endif 
