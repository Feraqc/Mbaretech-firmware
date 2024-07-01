#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "IMU.h"

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

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

#define LINE_LEFT 15
#define LINE_RIGHT 17

//MOTOR A
#define PWM_A 13
#define CHANNEL_LEFT LEDC_CHANNEL_1

//MOTORB
#define PWM_B 12
#define CHANNEL_RIGHT LEDC_CHANNEL_0

// Define the tick interval in milliseconds
// 10 ms es el estandard para freertos
#define TICK_INTERVAL_MS 5

// Define ticks to turn specific angles
#define TICKS_FOR_30_DEGREES 20  // Example values
#define TICKS_FOR_45_DEGREES 30  // Example values
#define TICKS_FOR_90_DEGREES 60  // Example values
#define TICKS_FOR_180_DEGREES 120 // Example values


enum Sensor { SHORT_RIGHT, TOP_MID, SHORT_LEFT};

extern volatile bool sensorReadings[7];
extern volatile bool startSignal;  // Creo que debe ser volatile si le trato con interrupt
extern bool dipSwitchPin[4];  // de A a D

// usar IRAM_ATTR para usar la ram interna
void IR1_ISR();
void IR2_ISR();
void IR3_ISR();

extern Motor leftMotor;
extern Motor rightMotor;

extern TaskHandle_t imuTaskHandle;
extern TaskHandle_t leftTurnTaskHandle;
extern TaskHandle_t rightTurnTaskHandle;
extern TaskHandle_t forwardTaskHandle;
extern SemaphoreHandle_t motorControlMutex;
extern volatile bool stopTask;

enum State {
    WAIT_ON_START,
    INITIAL_MOVEMENT,
    MID_SENSOR_CHECK,
    MID_MOVE,
    TOP_SENSORS_CHECK,
    TOP_LEFT_MOVE,
    TOP_RIGHT_MOVE,
    SEARCH,
    BOUND_MOVE
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

// Movements tasks
void leftTurnTask(void *pvParameters);
void rightTurnTask(void *pvParameters);
void forwardMovement(uint32_t speed);

// Movement starts
void startLeftTurn(uint32_t ticks);
void startRightTurn(uint32_t ticks);
void startForward(uint32_t speed);

// Web server related functions
extern AsyncWebServer server;
extern AsyncWebSocket ws;
extern void handleRoot();
extern void sendSensorData();
extern void webSocketTask(void *pvParameters);
extern void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

#endif // GLOBALS_H
