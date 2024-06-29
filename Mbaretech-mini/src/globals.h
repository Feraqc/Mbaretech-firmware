#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "IMU.h"

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// CAMBIAR A LOS VERDADEROS PINES Y SENSORES !!!!!!

#define IR1 19
#define IR2 18
#define IR3 45


#define DIPA 4
#define DIPB 5
#define DIPC 6
#define DIPD 7

#define START_PIN 12

#define SCL_PIN 9
#define SDA_PIN 16
#define INT_PIN 17

#define MOTOR_LEFT 19
#define MOTOR_RIGHT 18

#define LINE_LEFT 8
#define LINE_RIGHT 10

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

// Web server related functions
extern WebServer server;
extern WebSocketsServer webSocket;
extern void handleRoot();
extern void sendSensorData();
extern void webSocketTask(void *pvParameters);
extern void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);

#endif // GLOBALS_H
