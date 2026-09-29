#include "globals.h"
#include "bluetoothComm.h"
#include "sensorTasks.h"
#if ENABLE_RECIPE_FSM
#include "fsm/FSMDefinitions.h"
#endif

bool dipSwitch[4];
#ifdef MBARETECH_2
Motor leftMotor(PWM_A,PIN_A0,PIN_A1,CHANNEL_LEFT);
Motor rightMotor(PWM_B,PIN_B0,PIN_B1,CHANNEL_RIGHT);
#endif
#ifdef MBARETECH_1
Motor leftMotor(PWM_A,PIN_A1,PIN_A0,CHANNEL_LEFT);
Motor rightMotor(PWM_B,PIN_B1,PIN_B0,CHANNEL_RIGHT);
#endif

bool lineSensor[4];

// TASK HANDLE
TaskHandle_t stateMachineTaskHandle;

//VOLATILE VARIABLES
volatile bool irSensor[7];
volatile bool startSignal = false;

void IRAM_ATTR KS_ISR() {
    startSignal = digitalRead(START_PIN);
}

void setup() {
    esp_efuse_write_field_cnt(ESP_EFUSE_VDD_SPI_FORCE, 1);

#if ENABLE_SERIAL
    Serial.begin(115200);
#endif
#if ENABLE_LOGGING
    communicationsInit("MBARETECH");
#endif
#if ENABLE_GYRO_TEST
    imu.begin();
#endif
#if ENABLE_LINE_SENSORS
    // Line sensors
    lineSensorsInit();
#endif
#if ENABLE_IR_SENSORS
    // IR Sensors
#ifdef MBARETECH_2
    pinMode(IR1, INPUT);
    pinMode(IR7, INPUT);
#endif
    pinMode(IR2, INPUT);
    pinMode(IR3, INPUT);
    pinMode(IR4, INPUT);
    pinMode(IR5, INPUT);
    pinMode(IR6, INPUT);
#endif
#if ENABLE_DIP_SWITCHES
    // DIPS
    pinMode(DIPA, INPUT);
    pinMode(DIPB, INPUT);
    pinMode(DIPC, INPUT);
    pinMode(DIPD, INPUT);
    pinMode(DIPE, INPUT);
#endif
    // Start pin
    pinMode(START_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(START_PIN), KS_ISR, CHANGE);
    startSignal = digitalRead(START_PIN);
#if ENABLE_FSM || ENABLE_RECIPE_FSM || ENABLE_MOTOR_TEST
    leftMotor.begin();
    rightMotor.begin();
#endif
#if ENABLE_SENSOR_TASK
    if (!startSensorTask()) {
#if ENABLE_SERIAL
        Serial.println("ERROR: sensor task creation failed; control disabled");
#endif
        return;
    }
#endif
#if ENABLE_FSM || ENABLE_RECIPE_FSM
#if ENABLE_RECIPE_FSM
    const unsigned controlStack = fsm_defs::runtime::TASK_STACK_BYTES;
    const unsigned controlPriority = fsm_defs::runtime::TASK_PRIORITY;
#else
    const unsigned controlStack = 4096;
    const unsigned controlPriority = FSM_TASK_PRIORITY;
#endif
    if (xTaskCreate(stateMachineTask, "stateMachine", controlStack, nullptr, controlPriority,
                    &stateMachineTaskHandle) != pdPASS) {
        leftMotor.brake();
        rightMotor.brake();
#if ENABLE_SERIAL
        Serial.println("ERROR: state machine task creation failed");
#endif
    }
#elif ENABLE_MOVEMENT_TEST
    xTaskCreate(stateMachineTask, "stateMachineTask", 4096, NULL, 1, &stateMachineTaskHandle);
#endif

}

static TickType_t startTime = 0;
static bool firstCall = true;
void resetElapsedTime() { firstCall = true; }

bool elapsedTime(TickType_t duration) {
    TickType_t currentTime = xTaskGetTickCount();

    if (firstCall) {
        startTime = currentTime;
        firstCall = false;
    }

    if ((currentTime - startTime) >= duration) {
        startTime = currentTime;
        firstCall = true;
        return true;
    }
    else {
        return false;
    }
}

#if !(ENABLE_FSM || ENABLE_RECIPE_FSM || ENABLE_MOVEMENT_TEST || ENABLE_GYRO_TEST || ENABLE_MOTOR_TEST || ENABLE_LINE_TEST)
void loop() { delay(10); }
#endif
