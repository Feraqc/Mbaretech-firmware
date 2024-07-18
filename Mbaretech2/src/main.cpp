#include "globals.h" 

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
TaskHandle_t lineSensorTaskHandle;

//VOLATILE VARIABLES
volatile bool irSensor[7];
volatile bool startSignal = false;

void IRAM_ATTR KS_ISR(){startSignal = digitalRead(START_PIN);};

volatile State currentState;

void setup() {
    //esp_efuse_write_field_cnt(ESP_EFUSE_VDD_SPI_FORCE, 1); 
    // Initialize Serial communication
    #ifdef DEBUG
        Serial.begin(115200);
    #endif

    // Motors
    rightMotor.begin();
    leftMotor.begin();

    #ifdef RUN_LINE_SENSOR
    // Line sensors 
    lineSensorsInit();
    #endif

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
    

    attachInterrupt(digitalPinToInterrupt(START_PIN), KS_ISR, CHANGE);

    // DIPS
    pinMode(DIPA, INPUT);
    pinMode(DIPB, INPUT);
    pinMode(DIPC, INPUT);
    pinMode(DIPD, INPUT);

    // Start pin
    pinMode(START_PIN, INPUT);

    #if defined(RUN_MOVEMENTS_TEST) || defined(RUN_TASK_TEST) || defined(RUN_MOVEMENT_SENSOR_CALIBRATION)
        xTaskCreate(stateMachineTask, "stateMachineTask", 4096, NULL, 1, &stateMachineTaskHandle);
    #endif

    startSignal = false; //Esto agregue despues de la compe, verificar
}

bool elapsedTime(TickType_t duration) {
    static TickType_t startTime = 0;
    static bool firstCall = true;
    TickType_t currentTime = xTaskGetTickCount();

    if (firstCall) {
        startTime = currentTime;
        firstCall = false;
    }

    // Serial.println(currentTime - startTime);

    if ((currentTime - startTime) >= duration) {
        startTime = currentTime;
        firstCall = true;
        return true;
    }
    else {
        return false;
    }
}

void loop() {};