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

#define ENCODER_LEFT 14
#define ENCODER_RIGHT 12

#define LINE_FRONT_LEFT ADC1_CHANNEL_2
#define LINE_FRONT_RIGHT ADC1_CHANNEL_7
#define LINE_BACK_LEFT ADC2_CHANNEL_8
#define LINE_BACK_RIGHT ADC2_CHANNEL_9
#define ADC_WIDTH ADC_WIDTH_BIT_12

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
    TURN_LEFT,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    ATTACK,
    INITIAL_MOVEMENT
};

extern volatile State currentState;

//TASK HANDLERS
TaskHandle_t imuTaskHandle;
TaskHandle_t stateMachineTaskHandle;
TaskHandle_t lineSensorTaskHandle;

//MUTEXS
SemaphoreHandle_t gyroDataMutex;

// QUEUESS
QueueHandle_t imuDataQueue;
QueueHandle_t cmdQueue;

extern IMU imu;

extern int currentAngle;
extern int desiredAngle;

void stateMachineTask(void *param);
void imuTask(void *param);
void lineSensorTask(void *param);

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
