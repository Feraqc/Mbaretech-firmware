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

#define MAX_DUTY_CYCLE 1023

// Motor specs  //TODO cambiar en MEGA tambien
#define FREQUENCY 50
class Motor{
    public:
        uint32_t currentSpeed;
        uint8_t pwmPin;
        Servo motor;
        

        Motor(uint8_t pwmPin_){
            pwmPin = pwmPin_;
        }

        void begin(){
            pinMode(pwmPin, OUTPUT);
            ESP32PWM::allocateTimer(0);
            ESP32PWM::allocateTimer(1);
            ESP32PWM::allocateTimer(2);
            ESP32PWM::allocateTimer(3);
            motor.setPeriodHertz(FREQUENCY);
            motor.attach(pwmPin,MIN_WIDTH,MAX_WIDTH);
            motor.writeMicroseconds(MID_WIDTH); 
        }

        void setSpeed(int speed) {
            motor.writeMicroseconds(speed);
        }

        void forward(int percentage) {
            int speed = map(percentage,0,100,MID_WIDTH,MIN_WIDTH);
                for(int i=currentSpeed;i>speed;--i){
                    motor.writeMicroseconds(i);
                    currentSpeed = i;
                }
        }

        void backward(int percentage) {
            int speed = map(percentage,0,100,MID_WIDTH,MAX_WIDTH);
                for(int i=currentSpeed;i<speed;++i){
                    motor.writeMicroseconds(i);
                    currentSpeed = i;
                }
        }

        void brake() {
            motor.writeMicroseconds(MID_WIDTH);
            currentSpeed = MID_WIDTH;
        }
};

#endif 
