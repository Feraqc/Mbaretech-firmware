#include <Arduino.h>
#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "IMU.h"

#define IR1 40
#define IR2 39
#define IR3 38
#define IR4 18
#define IR5 17
#define IR6 4
#define IR7 5

QueueHandle_t imuDataQueue;
TaskHandle_t imuTaskHandle;

IMU imu;

void imuTask(void *param);


void setup(){
    Serial.begin(115200);

    imu.begin();

    // Create the queues
    imuDataQueue = xQueueCreate(10, sizeof(float));  // para gyro el tamanho de la cola, arbitrario
    cmdQueue = xQueueCreate(10, sizeof(float));

    if (imuDataQueue == NULL) {
        Serial.println("Failed to create sensor data queue");
        while (true);  // Halt the program
    }
    if (cmdQueue == NULL) {
        Serial.println("Failed to create command queue");
        while (true);  // Halt the program
    }

    xTaskCreate(imuTask, "imuTask", 4096, NULL, 1, &imuTaskHandle);

}

void loop() {
    static float currentAngle;
    xQueueSend(cmdQueue, &desiredAngle, portMAX_DELAY);
    // Wait for and receive the gyroFlag from the imuDataQueue
    bool gyroFlag;
    if (xQueueReceive(imuDataQueue, &gyroFlag, portMAX_DELAY) == pdPASS) {
    } //esta vacio, es correcto?
    if (gyroFlag) {
        Serial.println("Llego papu");
    }
    
    xQueueReceive(imuDataQueue, &currentAngle, portMAX_DELAY);

    Serial.print(currentAngle);
    Serial.print("\t");
    Serial.print("\n");

    vTaskDelay(10 / portTICK_PERIOD_MS);

}

void imuTask(void *param) {
    float currentAngle;

    while (true) {
        // #ifdef IMU_TEST
        imu.getData();
        currentAngle = imu.currentAngle;

        imu.transmitData();
        // Receive the desired angle from the cmdQueue
         if (xQueueReceive(cmdQueue, &desiredAngle, 0) == pdPASS) {
             // Check rotation with the received desired angle
             gyroFlag = imu.checkRotation(desiredAngle);

            // Send the gyroFlag to the imuDataQueue
            xQueueSend(imuDataQueue, &gyroFlag, portMAX_DELAY);
        }

        xQueueSend(imuDataQueue, &currentAngle, portMAX_DELAY);
        

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}