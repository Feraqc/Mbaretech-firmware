#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "freertos/semphr.h"
#include "lineSensor.h"
#include "IMU.h"

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
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

#define LINE_LEFT 15
#define LINE_RIGHT 17

//MOTOR A
#define PWM_A 12
//MOTORB
#define PWM_B 13

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

#define LINE_FRONT_LEFT ADC1_CHANNEL_2
#define LINE_FRONT_RIGHT ADC1_CHANNEL_7
#define LINE_BACK_LEFT ADC2_CHANNEL_8
#define LINE_BACK_RIGHT ADC2_CHANNEL_9
#define ADC_WIDTH ADC_WIDTH_BIT_12

enum Sensor { LEFT, MID, RIGHT};

extern volatile bool irSensor[3];
extern volatile bool startSignal;  // Creo que debe ser volatile si le trato con interrupt
extern bool dipSwitchPin[4];  // de A a D

// usar IRAM_ATTR para usar la ram interna
void IR1_ISR();
void IR2_ISR();
void IR3_ISR();

extern Motor leftMotor;
extern Motor rightMotor;

extern QueueHandle_t imuDataQueue;
extern QueueHandle_t cmdQueue;

extern TaskHandle_t imuTaskHandle;
extern SemaphoreHandle_t gyroDataMutex;

extern IMU imu;

extern int currentAngle;

void imuTask(void *param);
void mainTask(void *param);
//void handleState();
//void changeState(State newState);

// Web server related functions
extern AsyncWebServer server;
extern AsyncWebSocket ws;
extern void handleRoot();
extern void sendSensorData();
extern void webSocketTask(void *pvParameters);
extern void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);




#endif // GLOBALS_H
