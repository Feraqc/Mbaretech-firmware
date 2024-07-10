#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "freertos/semphr.h"

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "esp_efuse.h"
#include "esp_efuse_table.h"

#include "IMU.h"
#include "motor.h"	

// Defines
#define ADC_WIDTH ADC_WIDTH_BIT_12

#define IR1 39
#define IR2 40
#define IR3 38
#define IR4 4
#define IR5 5
#define IR6 18
#define IR7 17

#define DIPA 42
#define DIPB 2
#define DIPC 1
#define DIPD 44

#define START_PIN 41

#define SDA_PIN 15
#define SCL_PIN 16

#define HALL_PIN 43

#define ENCODER_LEFT 14
#define ENCODER_RIGHT 12

#define LINE_FRONT_LEFT ADC1_CHANNEL_2
#define LINE_FRONT_RIGHT ADC1_CHANNEL_7
#define LINE_BACK_LEFT ADC2_CHANNEL_8
#define LINE_BACK_RIGHT ADC2_CHANNEL_9
#define ADC_WIDTH ADC_WIDTH_BIT_12

// SPEED AND TIMERS
// Los comentados son los delays (y timers) de Asuncion
// Los valores no comentados son sugerencias para brasil
// Probar y ajustar para ambos bots
#ifdef MBARETECH_2
#define TURN_LEFT_SPEED 60 //percertage
#define LAST_LEFT_45_TIMER 50 //100
#define TURN_LEFT_45_DELAY 25 //40

#define LAST_LEFT_90_TIMER 140 //230
#define TURN_LEFT_90_DELAY 65 //105

#define TURN_LEFT_180_DELAY 150 // ajustar, simplemente demostrativo

#define TURN_RIGHT_SPEED 60
#define LAST_RIGHT_45_TIMER 50 //110
#define TURN_RIGHT_45_DELAY 25 //50

#define LAST_RIGHT_90_TIMER 160 //250
#define TURN_RIGHT_90_DELAY 75 //125

#define SHORT_RIGHT_DELAY  100 //140
#define SHORT_LEFT_DELAY 45 //70
#endif

#ifdef MBARETECH_1
#define TURN_LEFT_SPEED 60 //percertage
#define LAST_LEFT_45_TIMER 50 //100
#define TURN_LEFT_45_DELAY 25 //40

#define LAST_LEFT_90_TIMER 140 //230
#define TURN_LEFT_90_DELAY 65 //105

#define TURN_LEFT_180_DELAY 150 // ajustar, simplemente demostrativo

#define TURN_RIGHT_SPEED 60
#define LAST_RIGHT_45_TIMER 50 //110
#define TURN_RIGHT_45_DELAY 25 //50

#define LAST_RIGHT_90_TIMER 160 //250
#define TURN_RIGHT_90_DELAY 75 //125

#define SHORT_RIGHT_DELAY  100 //140
#define SHORT_LEFT_DELAY 45 //70
#endif

#define MAX_SPEED 85 
// En el codigo viejo con 100% patinaba, usabamos 90% en asuncion
// En brasil debe ser menos, le pongo 85 por el momento
// Probar si se puede ser 100% o mas de 85

#define FORWARD_90 90
#define FORWARD_70 70
#define FORWARD_60 60
#define FORWARD_49 49
#define FORWARD_42 42
#define FORWARD_40 40

#define THRESHOLD 150

extern TickType_t lastLeft45;
extern TickType_t lastRight45;
extern TickType_t lastLeft90;
extern TickType_t lastRight;
extern TickType_t currMove;

void lineSensorsInit();
int readLineSensorFront(adc1_channel_t channel);
bool checkLineSensor(int measurement);
extern bool lineSensor[4];

bool elapsedTime(TickType_t duration);

enum Sensor { SIDE_LEFT, SHORT_LEFT, TOP_LEFT, TOP_MID, TOP_RIGHT, SHORT_RIGHT, SIDE_RIGHT };

extern volatile bool irSensor[7];
extern volatile bool startSignal;  // Creo que debe ser volatile si le trato con interrupt
extern bool dipSwitch[4];  // de A a D


extern Motor leftMotor;
extern Motor rightMotor;

enum State {
    IDLE,
    FORWARD,
    BACKWARD,
    TURN_RIGHT,
    TURN_LEFT_45,
    TURN_RIGHT_45,
    TURN_RIGHT_90,
    TURN_LEFT_90,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    MOVEMENT_45,
    TURN_180,
    BRAKE,
    INITIAL_MOVEMENT
};

extern volatile State currentState;

//TASK HANDLERS
extern TaskHandle_t stateMachineTaskHandle;
extern TaskHandle_t lineSensorTaskHandle;

// TASKS
void stateMachineTask(void *param);
void lineSensorTask(void *param);

void changeState(State newState);

void lineSensorsInit();
int readLineSensorFront(adc1_channel_t channel);

#endif // GLOBALS_H
