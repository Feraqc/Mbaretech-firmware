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
#define TURN_LEFT_SPEED 60 //percertage
#define LAST_LEFT_45_TIMER 100
#define TURN_LEFT_45_DELAY 40

#define LAST_LEFT_90_TIMER 230
#define TURN_LEFT_90_DELAY 105

#define TURN_RIGHT_SPEED 60
#define LAST_RIGHT_45_TIMER 110
#define TURN_RIGHT_45_DELAY 50

#define LAST_RIGHT_90_TIMER 250
#define TURN_RIGHT_90_DELAY 125

#define SHORT_TURN_DELAY 70

#define FORWARD_70 70
#define FORWARD_49 49
#define FORWARD_42 42

#define MAX_SPEED 100



extern TickType_t lastLeft45;
extern TickType_t lastRight45;
extern TickType_t lastLeft90;
extern TickType_t lastRight;
extern TickType_t currMove;


void lineSensorsInit();
int readLineSensorFront(adc1_channel_t channel);
int readLineSensorBack(adc2_channel_t channel);
bool checkLineSensor(int measurement,int threshold);
extern bool lineSensor[4];


enum Sensor { SIDE_LEFT, SHORT_LEFT, TOP_LEFT, TOP_MID, TOP_RIGHT, SHORT_RIGHT, SIDE_RIGHT };

extern volatile bool irSensor[7];
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
    SNAKE,
    ATTACK,
    BRAKE,
    INITIAL_MOVEMENT
};

extern volatile State currentState;

//TASK HANDLERS
extern TaskHandle_t imuTaskHandle;
extern TaskHandle_t stateMachineTaskHandle;
extern TaskHandle_t lineSensorTaskHandle;

// TASKS
void stateMachineTask(void *param);
void imuTask(void *param);
void lineSensorTask(void *param);

//MUTEXS
extern SemaphoreHandle_t gyroDataMutex;

extern IMU imu;

extern int currentAngle;
extern int desiredAngle;

void handleState();
void changeState(State newState);

#define ADC_WIDTH ADC_WIDTH_BIT_12

void lineSensorsInit();
int readLineSensorFront(adc1_channel_t channel);
int readLineSensorBack(adc2_channel_t channel);


// Web server related functions
extern AsyncWebServer server;
extern AsyncWebSocket ws;
extern void handleRoot();
extern void sendSensorData();
extern void webSocketTask(void *pvParameters);
extern void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

#endif // GLOBALS_H
