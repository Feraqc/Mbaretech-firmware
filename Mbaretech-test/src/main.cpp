#include <Arduino.h>
#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "IMU.h"

#define ADC_CHANNEL ADC1_CHANNEL_0 

QueueHandle_t sensorDataQueue;
QueueHandle_t cmdQueue;
TaskHandle_t sensorTaskHandle;

IMU imu;
void sensorTask(void *param);

void setup() {
    // Initialize Serial communication
    Serial.begin(115200);
    imu.begin();

    // Create the queues
    sensorDataQueue = xQueueCreate(10, sizeof(bool));
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
    float desiredAngle = 90.0;

    // Send the desired angle to the cmdQueue
    // xQueueSend(cmdQueue, &desiredAngle, portMAX_DELAY);

    // // Wait for and receive the gyroFlag from the sensorDataQueue
    // bool gyroFlag;
    // if (xQueueReceive(sensorDataQueue, &gyroFlag, portMAX_DELAY) == pdPASS) {
    // }
    // if(gyroFlag){
    //     Serial.println("Llego papu");
    // }

    // Delay to avoid flooding the queue
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void sensorTask(void *param) {
    bool gyroFlag = false;
    float desiredAngle;

    while (true) {
        imu.getData();
        imu.transmitData();

        // Receive the desired angle from the cmdQueue
        // if (xQueueReceive(cmdQueue, &desiredAngle, 0) == pdPASS) {
        //     // Check rotation with the received desired angle
        //     gyroFlag = imu.checkRotation(desiredAngle);

        //     // Send the gyroFlag to the sensorDataQueue
        //     xQueueSend(sensorDataQueue, &gyroFlag, portMAX_DELAY);
        // }

        // Delay to control the task frequency
        vTaskDelay(10 / portTICK_PERIOD_MS); // Adjust delay as needed
    }
}
