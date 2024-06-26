#include <Arduino.h>
#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "IMU.h"

#define IR_TEST
//#define IMU_TEST 
#define ENCODER_TEST

#define IR1 40
#define IR2 39
#define IR3 38
#define IR4 18
#define IR5 17
#define IR6 4
#define IR7 5

#define ENCODER_RIGHT 12
#define ENCODER_LEFT 14

#define KS 41

volatile int sensorReadings[7];
volatile unsigned int encoderRightCounter = 0;
volatile unsigned int encoderLeftCounter = 0;
volatile bool killSwitchCmd = false;

QueueHandle_t sensorDataQueue;
QueueHandle_t cmdQueue;
TaskHandle_t sensorTaskHandle;

IMU imu;
void sensorTask(void *param);

void IR1_ISR() { sensorReadings[0] = digitalRead(IR1); }
void IR2_ISR() { sensorReadings[1] = digitalRead(IR2); }
void IR3_ISR() { sensorReadings[2] = digitalRead(IR3); }
void IR4_ISR() { sensorReadings[3] = digitalRead(IR4); }
void IR5_ISR() { sensorReadings[4] = digitalRead(IR5); }
void IR6_ISR() { sensorReadings[5] = digitalRead(IR6); }
void IR7_ISR() { sensorReadings[6] = digitalRead(IR7); }

void encoderRightISR(){encoderRightCounter++;}
void encoderLeftISR(){encoderLeftCounter++;}

void killSwithcISR(){killSwitchCmd = digitalRead(KS);}

void setup() {
    // Initialize Serial communication
    Serial.begin(115200);

    imu.begin();


    #ifdef IR_TEST
    pinMode(IR1, INPUT);
    pinMode(IR2, INPUT);
    pinMode(IR3, INPUT);
    pinMode(IR4, INPUT);
    pinMode(IR5, INPUT);
    pinMode(IR6, INPUT);
    pinMode(IR7, INPUT);

    attachInterrupt(digitalPinToInterrupt(IR1), IR1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR2), IR2_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR3), IR3_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR4), IR4_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR5), IR5_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR6), IR6_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR7), IR7_ISR, CHANGE);
    #endif
    
    pinMode(KS,INPUT);
    attachInterrupt(digitalPinToInterrupt(KS), killSwithcISR, CHANGE);


    pinMode(ENCODER_RIGHT,INPUT);
    pinMode(ENCODER_LEFT,INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), encoderRightISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), encoderLeftISR, RISING);

    // Create the queues
    sensorDataQueue = xQueueCreate(10, sizeof(float));
    cmdQueue = xQueueCreate(10, sizeof(float));

    if (sensorDataQueue == NULL) {
        Serial.println("Failed to create sensor data queue");
        while (true); // Halt the program
    }
    if (cmdQueue == NULL) {
        Serial.println("Failed to create command queue");
        while (true); // Halt the program
    }

    // Create the sensor task
    xTaskCreate(sensorTask, "SensorTask", 4096, NULL, 1, &sensorTaskHandle);
}

void loop() {
    //static int sensorReadings[7];
    static float currentAngle;
    #ifdef IMU_TEST
    xQueueSend(cmdQueue, &desiredAngle, portMAX_DELAY);
    // Wait for and receive the gyroFlag from the sensorDataQueue
    bool gyroFlag;
    if (xQueueReceive(sensorDataQueue, &gyroFlag, portMAX_DELAY) == pdPASS) {
    }
    if(gyroFlag){
        Serial.println("Llego papu");
    }
    #endif

    xQueueReceive(sensorDataQueue, &currentAngle, portMAX_DELAY);

    //Serial.println(killSwitchCmd);

    if(killSwitchCmd){
        for(int i=0;i<6;i++){
            Serial.print(sensorReadings[i]);
            Serial.print("\t");
        }
        Serial.print(currentAngle);
        Serial.print("\t");
        Serial.print(encoderRightCounter);
        Serial.print("\t");
        Serial.print(encoderLeftCounter);
        Serial.print("\n");
    }

    // #ifdef IR_TEST
    // if (xQueueReceive(sensorDataQueue, &sensorReadings, portMAX_DELAY) == pdPASS) {
    //     for(int i=0;i<6;i++){
    //         Serial.print(sensorReadings[i]);
    //         Serial.print("\t");
    //     }

    //     Serial.print("\t");
    //     Serial.print("\n");
    // } 
    // #endif


    

    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void sensorTask(void *param){

    int sensorReadings[7];
    float currentAngle;
    
    while (true) {
        //#ifdef IMU_TEST
        imu.getData();
        currentAngle = imu.currentAngle;

            //imu.transmitData();
            // Receive the desired angle from the cmdQueue
            // if (xQueueReceive(cmdQueue, &desiredAngle, 0) == pdPASS) {
            //     // Check rotation with the received desired angle
            //     gyroFlag = imu.checkRotation(desiredAngle);

            //     // Send the gyroFlag to the sensorDataQueue
            //     xQueueSend(sensorDataQueue, &gyroFlag, portMAX_DELAY);
            // }

        xQueueSend(sensorDataQueue, &currentAngle, portMAX_DELAY);
       // #endif

        // #ifdef IR_TEST
        // // Send sensor values to queue
        // xQueueSend(sensorDataQueue, &sensorReadings, portMAX_DELAY);
        // #endif


        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
