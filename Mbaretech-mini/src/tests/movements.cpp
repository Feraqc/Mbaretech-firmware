#ifdef RUN_MOVEMENTS_TEST
#include "globals.h"

void imuTask(void *param) {
    while (true) {
        imu.getData();
        if (xSemaphoreTake(gyroDataMutex, portMAX_DELAY) == pdTRUE) {
            currentAngle = imu.currentAngle;
            xSemaphoreGive(gyroDataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

bool elapsedTime(TickType_t duration) {
    static TickType_t startTime = 0;
    static bool firstCall = true;
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

bool checkRotation(int rotationAngle) {
    static int initialAngle = 0;
    int currentAngleRotation = 0;
    static bool firstCall = true;

    if (xSemaphoreTake(gyroDataMutex, portMAX_DELAY) == pdTRUE) {
        if (firstCall) {
            firstCall = false;
            initialAngle = currentAngle;  // Initialize initialAngle only once
        }
        currentAngleRotation = currentAngle;
        xSemaphoreGive(gyroDataMutex);
    }

    if (abs(currentAngleRotation - initialAngle) >= rotationAngle) {
        initialAngle = currentAngleRotation;
        firstCall = true;
        return true;
    }
    else {
        return false;
    }
}

void motorTask(void *param) {
    typedef enum {
        IDLE,
        FORWARD,
        BACKWARD,
        TURN_LEFT,
        TURN_RIGHT,
        BRAKE
    } State;
    State currentState;
    currentState = IDLE;
    int desiredAngle = 0;

    TickType_t lastLeft = 0;
    TickType_t lastRight = 0;
    TickType_t currMove;

    currentState = TURN_LEFT;

    while (true) {
        Serial.print(currentState);
        Serial.print("\t");
        Serial.print(irSensor[LEFT]);
        Serial.print("\t");
        Serial.print(irSensor[MID]);
        Serial.print("\t");
        Serial.print(irSensor[RIGHT]);
        Serial.print("\n");

        if (startSignal) {
            switch (currentState) {
                case TURN_RIGHT:
                    Serial.println("Entering turn right");
                    
                    rightMotor.brake();
                    leftMotor.forward(10);
                    while(!elapsedTime(105)){};
                    rightMotor.brake();
                    leftMotor.brake();
                    while (!elapsedTime(1000)) {};
                    Serial.println("Getting out of turn right");
                    break;
                case TURN_LEFT:
                    rightMotor.forward(10);
                    leftMotor.brake();
                    while(!elapsedTime(105)){};
                    Serial.println("Stopping for a sec");
                    rightMotor.brake();
                    leftMotor.brake();
                    while (!elapsedTime(1000)) {};
                    Serial.println("Getting out of turn right");

                    break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        else{
            leftMotor.brake();
            rightMotor.brake();
        }
    }
}

#endif