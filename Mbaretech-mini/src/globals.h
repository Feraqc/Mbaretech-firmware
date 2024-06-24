#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "IMU.h"

// CAMBIAR A LOS VERDADEROS PINES Y SENSORES !!!!!!

#define IR1 40
#define IR2 39
#define IR3 38
#define IR4 18
#define IR5 17
#define IR6 4
#define IR7 5

#define DIPA 37
#define DIPB 36
#define DIPC 35
#define DIPD 34

#define START_PIN 33

#define MOTOR_LEFT 32
#define MOTOR_RIGHT 31



// Define pulse width values in microseconds
#define MIN_PULSE_WIDTH 1080
#define MID_PULSE_WIDTH 1500
#define MAX_PULSE_WIDTH 1920

#define RESOLUTION 16

// LEDC (PWM) settings
const int freq = 50;  // 50Hz frequency for RC control
const int period = 20000; // 20 milliseconds (20000 microseconds).
const int leftMotorChannel = 0;
const int rightMotorChannel = 1;
const int resolution = 16;  // 16-bit resolution


enum Sensor { SIDE_LEFT, SHORT_LEFT, TOP_LEFT, TOP_MID, TOP_RIGHT, SIDE_RIGHT };

extern volatile bool sensorReadings[7];
extern volatile bool startSignal;  // Creo que debe ser volatile si le trato con interrupt
extern bool dipSwitchPin[4];  // de A a D

// usar IRAM_ATTR para usar la ram interna
void IR1_ISR();
void IR2_ISR();
void IR3_ISR();
void IR4_ISR();
void IR5_ISR();
void IR6_ISR();
void IR7_ISR();

enum State {
    WAIT_ON_START,
    INITIAL_MOVEMENT,
    MID_SENSOR_CHECK,
    MID_MOVE,
    TOP_SENSORS_CHECK,
    TOP_LEFT_MOVE,
    TOP_RIGHT_MOVE,
    SIDE_SENSORS_CHECK,
    SIDE_LEFT_MOVE,
    SIDE_RIGHT_MOVE,
    BOUND_MOVE,
    DEFAULT_ACTION_STATE
};

extern volatile State currentState;

extern QueueHandle_t imuDataQueue;
extern QueueHandle_t cmdQueue;

extern TaskHandle_t imuTaskHandle;

extern IMU imu;

extern int desiredAngle;

void imuTask(void *param);
void mainTask(void *param);
void handleState();
void changeState(State newState);


uint32_t usToDutyCycle(int pulseWidth);

#endif // GLOBALS_H
