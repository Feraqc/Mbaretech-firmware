#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "freertos/semphr.h"
#include "lineSensor.h"
// #include "IMU.h"

// #include <WiFi.h>
// #include <ESPAsyncWebServer.h>
// #include <AsyncTCP.h>
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "motor.h"

#define IR1 19
#define IR2 21
#define IR3 45

#define DIPA 4
#define DIPB 5
#define DIPC 6
#define DIPD 7

#define START_PIN 8

#define SCL_PIN 9
#define SDA_PIN 10
#define INT_PIN 11

#define NUM_SAMPLES 10

// MOTOR A
#define PWM_A 12
// MOTORB
#define PWM_B 13

#define THRESHOLD 200  // veloz tiene que ser mas de 400
// 250 del lento

// SPEED AND TIMERS

#define FORWARD_SPEED 15  // percent
#define MAX_SPEED 100
#define ALMOST_MAX_SPEED 98
#define MIN_SPEED 0
#define TURN_LEFT_SPEED 10
#define TURN_RIGHT_SPEED 10

#define TURN_LEFT_DELAY 105
#define LAST_LEFT_TIMER 140

#define TURN_RIGHT_DELAY 105
#define LAST_RIGHT_TIMER 140

#define TASK_TICKS 10

#define TURKISH_TIME 2000

#define LINE_FRONT_LEFT ADC2_CHANNEL_4
#define LINE_FRONT_RIGHT ADC2_CHANNEL_6
#define ADC_WIDTH ADC_WIDTH_BIT_12

enum Sensor { LEFT, MID, RIGHT };

enum State {
    IDLE,
    FORWARD,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    BRAKE,
    MOVEMENT_U,
    MOVEMENT_45,
    SLOW_FORWARD,
    TURN_180,
    TURN_90_LEFT,
    AVOID_LINE

};

extern volatile bool irSensor[3];
extern volatile bool
    startSignal;  // Creo que debe ser volatile si le trato con interrupt
extern bool dipSwitch[4];  // de A a D
extern bool lineSensor[2];

extern Motor leftMotor;
extern Motor rightMotor;

extern int currentAngle;

bool elapsedTime(TickType_t duration);
int readLineSensor(adc2_channel_t channel);
bool checkLineSensor(bool lineSensor,int measurement);
bool readIrSensor(int irSensor);

void imuTask(void *param);
void mainTask(void *param);
// void handleState();
// void changeState(State newState);

#endif  // GLOBALS_H
