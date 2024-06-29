#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "IMU.h"

#define IR1 40
#define IR2 39
#define IR3 38
#define IR4 18
#define IR5 17
#define IR6 4
#define IR7 5


// CAMBIAR A LOS VERDADEROS PINES !!!!!!
#define DIPA 42
#define DIPB 2
#define DIPC 1
#define DIPD 44

#define START_PIN 41

#define SDA_PIN 15
#define SCL_PIN 16

#define HALL_PIN 43

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 37

#define PIN_A0 45
#define PIN_A1 48

#define PIN_B0 36 
#define PIN_B1 35


#define ENCODER_LEFT 14
#define ENCODER_RIGHT 12

#define LINE_FRONT_LEFT 3
#define LINE_FRONT_RIGHT 8
#define LINE_BACK_LEFT 19
#define LINE_BACK_RIGHT 20



// Hasta aca lo que hay que cambiar

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

#endif // GLOBALS_H
