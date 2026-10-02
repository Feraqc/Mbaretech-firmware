#ifdef RUN_GYRO_TEST
#include "globals.h"  // Include the header file


void loop() {
    static float currentAngle;
    xQueueSend(cmdQueue, &desiredAngle, portMAX_DELAY);
    // Wait for and receive the gyroFlag from the imuDataQueue
    bool gyroFlag;
    if (xQueueReceive(imuDataQueue, &gyroFlag, portMAX_DELAY) == pdPASS) {
        if (gyroFlag) {
            Serial.println("Llego papu");
        }
    }
    
    if (xQueueReceive(imuDataQueue, &currentAngle, portMAX_DELAY) == pdPASS) {
        Serial.print(currentAngle);
        Serial.print("\t");
        Serial.print("\n");
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void imuTask(void *param) {
    float currentAngle;
    bool gyroFlag;

    while (true) {
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

        // Send the current angle to the imuDataQueue
        xQueueSend(imuDataQueue, &currentAngle, portMAX_DELAY);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

#endif  // RUN_GYRO_TEST
