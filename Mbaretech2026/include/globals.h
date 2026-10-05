#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "freertos/semphr.h"

//#include <WiFi.h>
//#include <ESPAsyncWebServer.h>
//#include <AsyncTCP.h>
#include "esp_efuse.h"
#include "esp_efuse_table.h"

//#include "IMU.h"
#include "motor.h"	

// Defines
#define ADC_WIDTH ADC_WIDTH_BIT_12


// array comunicacion bluetooth
const int ARRAY_PARAMETROS_SIZE = 20; //traje esto desde el globals
extern int parametros[ARRAY_PARAMETROS_SIZE];


#ifdef MBARETECH_2
#define IR1 39
#define IR2 40
#define IR3 38
#define IR4 4
#define IR5 5
#define IR6 18
#define IR7 17
#endif

#ifdef MBARETECH_1
//#define IR1 39
#define IR2 7 // short left
#define IR3 4 //top left
#define IR4 17 //top md
#define IR5 40 //top right
#define IR6 6 // short right
#endif

#define DIPA 42
#define DIPB 2
#define DIPC 1
#define DIPD 44 

#define START_PIN 41

#define SDA_PIN 15
#define SCL_PIN 16

//#define HALL_PIN 43 //ahora DIPD


// ENCODER_LEFT/RIGHT nunca se leen en ningun lado de src/ (dead code) --
// estos dos pines fueron repurpuestos 2026-09-29 para los sensores de linea
// traseros (LS3/LS4, ver LINE_BACK_LEFT/RIGHT abajo). No usar para encoders.
#define ENCODER_LEFT 14
#define ENCODER_RIGHT 12

#define LINE_FRONT_LEFT ADC1_CHANNEL_2
#define LINE_FRONT_RIGHT ADC1_CHANNEL_7

// LS3 (trasero izquierdo) puenteado a GPIO12, LS4 (trasero derecho) a
// GPIO14 -- pines libres (antes ENCODER_RIGHT/LEFT, nunca leidos), sin
// conflicto con DIPE/PIN_B1 como tenian los pines originales (ADC2_CH8/9 =
// GPIO19/20). Confirmado y activado 2026-09-29.
#define LINE_BACK_LEFT ADC2_CHANNEL_1   // GPIO12, LS3
#define LINE_BACK_RIGHT ADC2_CHANNEL_3  // GPIO14, LS4
#define LINE_BACK_INSTALLED 1

#define ADC_WIDTH ADC_WIDTH_BIT_12
#define DIPE 19

// SPEED AND TIMERS
// Los comentados son los delays (y timers) de Asuncion
// Los valores no comentados son sugerencias para brasil
// Probar y ajustar para ambos bots



#ifdef MBARETECH_2
#define TURN_LEFT_SPEED 94 //percertage parametros[5]
#define LAST_LEFT_45_TIMER 150 //100
#define TURN_LEFT_45_DELAY 55 // antes 60 -- calibrado en calibracion.cpp (0010) con bateria llena, 2026-09-25 //70 //40

#define LAST_LEFT_90_TIMER 200 //230
#define TURN_LEFT_90_DELAY 80 // antes 85 (recalibrado 2026-09-25, el 85 del 2026-09-23 queda superado) -- calibrado en calibracion.cpp (0011) con bateria llena //contra charizard tenia 95 y se pasaba //105

#define TURN_LEFT_180_DELAY 150 // ajustar, simplemente demostrativo
#define TURN_RIGHT_180_DELAY 150 // parametros[19], 2026-10-04: arranca igual que el IZQ, sin calibrar (solo lo usa calibracion.cpp 0110)

#define TURN_RIGHT_SPEED 94
#define LAST_RIGHT_45_TIMER 150 //110
#define TURN_RIGHT_45_DELAY 45 // antes 60 -- calibrado en calibracion.cpp (0010) con bateria llena, 2026-09-25 //estaba 70 //50

#define LAST_RIGHT_90_TIMER 200 //250
#define TURN_RIGHT_90_DELAY 70 // antes 100 (recalibrado 2026-09-25, el 100 del 2026-09-23 queda superado) -- calibrado en calibracion.cpp (0011) con bateria llena //125 //Contra charizar tenia 95 pero no vimos el giro

#define SHORT_RIGHT_DELAY  15 //140
#define SHORT_LEFT_DELAY 15 //70
#define THRESHOLD 250 // 2026-10-01: probado 800 y revertido (el robot iba a reversa apenas
// se activaba). Vuelto de la prueba de 500 (2026-09-29) a su valor calibrado --
// ver checkLineSensora/b/c/d en lineSensor.cpp: se saco el filtro de 7 lecturas, ahora
// blanco/negro se confirman con una sola lectura simetrica de este umbral. //145 //169

#define TURKISH_TIME 2000
#define TURKISH_DELAY 100
#endif

#define GIRO_U_DELAY 500
#define GIRO_U_L_DELAY 1000

#ifdef MBARETECH_1
#define TURN_LEFT_SPEED 80 //percertage
#define LAST_LEFT_45_TIMER 80 //100
#define TURN_LEFT_45_DELAY 40 //40

#define LAST_LEFT_90_TIMER 190 //230
#define TURN_LEFT_90_DELAY 70 //105

#define TURN_LEFT_180_DELAY 150 // ajustar, simplemente demostrativo
#define TURN_RIGHT_180_DELAY 150 // parametros[19], 2026-10-04: arranca igual que el IZQ, sin calibrar (solo lo usa calibracion.cpp 0110)

#define TURN_RIGHT_SPEED 80
#define LAST_RIGHT_45_TIMER 100 //110
#define TURN_RIGHT_45_DELAY 50 //50

#define LAST_RIGHT_90_TIMER 190 //250
#define TURN_RIGHT_90_DELAY 90 //125

#define SHORT_RIGHT_DELAY  10 //140
#define SHORT_LEFT_DELAY 10 //70
#define THRESHOLD 169

#define TURKISH_TIME 2000
#define TURKISH_DELAY 30
#endif

#define MAX_SPEED 100 
// En el codigo viejo con 100% patinaba, usabamos 90% en asuncion
// En brasil debe ser menos, le pongo 85 por el momento
// Probar si se puede ser 100% o mas de 85


#define CORRECT_SPEED 4
#define TURKISH_SPEED 70
#define FORWARD_X 94
#define FORWARD_90 90
#define FORWARD_80 80
#define FORWARD_70 70
#define FORWARD_60 60
#define FORWARD_49 49
#define FORWARD_42 42
#define FORWARD_40 40


extern TickType_t lastLeft45;
extern TickType_t lastRight45;
extern TickType_t lastLeft90;
extern TickType_t lastRight;
extern TickType_t currMove;

void lineSensorsInit();
int readLineSensorFront(adc1_channel_t channel);
bool checkLineSensora(int measurement);
bool checkLineSensorb(int measurement);
int readLineSensorBack(adc2_channel_t channel);
bool checkLineSensorc(int measurement);
bool checkLineSensord(int measurement);
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
    TURN_LEFT_45_IF,
    TURN_RIGHT_45_IF,
    TURN_RIGHT_90_IF,
    TURN_LEFT_90_IF,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    MOVEMENT_45,
    L_MOVEMENT_45,
    R_MOVEMENT_45,
    TURN_180,
    BRAKE,
    SHORT_LEFT_MOVE,
    SHORT_RIGHT_MOVE,
    LINE_RETREAT,
    INITIAL_MOVEMENT,
    SNAKE,
    TURKISH,
    GIRO_U_L,
    GIRO_U_R,
    GIRO_U_L_LONG,
    GIRO_U_R_LONG,
    TEST_FORWARD
};

extern volatile State currentState;

//TASK HANDLERS
extern TaskHandle_t stateMachineTaskHandle;
extern TaskHandle_t lineSensorTaskHandle;

extern bool fast_enemy;

// TASKS
void stateMachineTask(void *param);
void lineSensorTask(void *param);

void changeState(State newState);

void lineSensorsInit();
int readLineSensorFront(adc1_channel_t channel);


#endif // GLOBALS_H
