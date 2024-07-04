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

    int forwardSpeed = 50;

    while (true) {
        switch (currentState){
            case FORWARD:
                    leftMotor.forward(10);
                    rightMotor.forward(10);

                if(startSignal){
                    currentState = IDLE;
                }
                else if(elapsedTime(1500)){
                    currentState = BACKWARD;
                    leftMotor.brake();
                    rightMotor.brake();
                }
                break;
            
            case BACKWARD:

                leftMotor.backward(10);
                rightMotor.backward(10);


                if(startSignal){
                    currentState = IDLE;
                }
                else if(elapsedTime(1500)){
                    currentState = FORWARD;
                    leftMotor.brake();
                    rightMotor.brake();
                }
                break;
            
            case IDLE:
                leftMotor.brake();
                rightMotor.brake();
                if(startSignal){
                    currentState = FORWARD;
                }
                
        }
    vTaskDelay(pdMS_TO_TICKS(10));
    }
}