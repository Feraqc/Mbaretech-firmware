#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "IMU.h"
#include "motor.h"	
#include "lineSensor.h"

#define IR1 40
#define IR2 39
#define IR3 38
#define IR4 18
#define IR5 17
#define IR6 4
#define IR7 5

#define DIPA 42
#define DIPB 2
#define DIPC 1
#define DIPD 44

#define START_PIN 41

#define SDA_PIN 15
#define SCL_PIN 16

#define HALL_PIN 43

#define PWM_A 48   
#define PWM_B 35

#define PIN_A0 0
#define PIN_A1 45

#define PIN_B0 36 
#define PIN_B1 37

#define ENCODER_LEFT 14
#define ENCODER_RIGHT 12

#define LINE_FRONT_LEFT ADC1_CHANNEL_2
#define LINE_FRONT_RIGHT ADC1_CHANNEL_7
#define LINE_BACK_LEFT ADC2_CHANNEL_8
#define LINE_BACK_RIGHT ADC2_CHANNEL_9



// Hasta aca lo que hay que cambiar

enum Sensor { SIDE_LEFT, SHORT_LEFT, TOP_LEFT, TOP_MID, TOP_RIGHT, SIDE_RIGHT };

extern volatile bool sensorReadings[7];
extern volatile bool startSignal;  // Creo que debe ser volatile si le trato con interrupt
extern bool dipSwitchPin[4];  // de A a D

// usar IRAM_ATTR para usar la ram interna
void IRAM_ATTR IR1_ISR();
void IRAM_ATTR IR2_ISR();
void IRAM_ATTR IR3_ISR();
void IRAM_ATTR IR4_ISR();
void IRAM_ATTR IR5_ISR();
void IRAM_ATTR IR6_ISR();
void IRAM_ATTR IR7_ISR();

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
