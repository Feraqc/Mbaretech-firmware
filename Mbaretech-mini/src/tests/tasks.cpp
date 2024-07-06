#ifdef RUN_TASK_TEST
#include "globals.h"

void imuTask(void *param) {
    while (true) {
        imu.getData();
        if (xSemaphoreTake(gyroDataMutex, portMAX_DELAY) == pdTRUE) {
            currentAngle = imu.currentAngle;
            xSemaphoreGive(gyroDataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
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
    } else {
        return false;
    }
}

bool checkRotation(int rotationAngle){
    static int initialAngle = 0;
    int currentAngleRotation = 0;
    if (xSemaphoreTake(gyroDataMutex, portMAX_DELAY) == pdTRUE) {
        if (initialAngle == 0) {
            initialAngle = currentAngle;  // Initialize initialAngle only once
        }
        currentAngleRotation = currentAngle;
        xSemaphoreGive(gyroDataMutex);
    }

    if (abs(currentAngleRotation - initialAngle) >= rotationAngle) {
        initialAngle = currentAngleRotation;
        return true;
    }
    else{return false;}
}


void motorTask(void *param) {
    typedef enum {
        IDLE,
        FORWARD,
        BACKWARD,
        TURN_LEFT,
        TURN_RIGHT
    } State;
    State currentState;
    currentState = IDLE;

    int speed = 10;
    while (true) {
        // Serial.print(sensorReadings[0]);
        // Serial.print('\t');
        // Serial.print(sensorReadings[1]);
        // Serial.print('\t');
        // Serial.print(sensorReadings[2]);
        // Serial.print('\n');

        switch (currentState){
            case FORWARD:
                rightMotor.forward(0);
                leftMotor.forward(0);

                // rightMotor.writePulse(rightMotor.minForwardPulse);
                // leftMotor.writePulse(leftMotor.minForwardPulse);

                if(!startSignal){
                    currentState = IDLE;
                }
                else if(elapsedTime(1000)){
                    currentState = BACKWARD;
                }
                break;
            
            case BACKWARD:
                rightMotor.backward(0);
                leftMotor.backward(0);
                // rightMotor.writePulse(rightMotor.minBackwarPulse);
                // leftMotor.writePulse(leftMotor.minBackwarPulse);

                if(!startSignal){
                    currentState = IDLE;
                }
                else if(elapsedTime(1000)){
                    currentState = FORWARD;
                }
                break;
            
            case IDLE:
                leftMotor.brake();
                rightMotor.brake();
                if(startSignal){
                    currentState = FORWARD;
                }
                break;

            case TURN_LEFT:
                rightMotor.forward(0);
                leftMotor.backward(0);

                if(!startSignal){
                    currentState = IDLE;
                }
                else if(checkRotation(85)){
                    leftMotor.brake();
                    rightMotor.brake();
                    currentState = FORWARD;
                }
                break;

                case TURN_RIGHT:
                rightMotor.backward(0);
                leftMotor.forward(0);

                if(!startSignal){
                    currentState = IDLE;
                }
                else if(checkRotation(85)){
                    leftMotor.brake();
                    rightMotor.brake();
                    currentState = FORWARD;
                }
                break;

        }
    vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#endif