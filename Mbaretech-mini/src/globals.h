#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "IMU.h"

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

#include "motor.h"

// CAMBIAR A LOS VERDADEROS PINES Y SENSORES !!!!!!

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

extern Motor leftMotor;
extern Motor rightMotor;

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

// Web server related functions
extern AsyncWebServer server;
extern AsyncWebSocket ws;
extern void handleRoot();
extern void sendSensorData();
extern void webSocketTask(void *pvParameters);
extern void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

#endif // GLOBALS_H
