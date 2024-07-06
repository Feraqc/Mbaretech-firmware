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
            initialAngle = currentAngle;  
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

void stateMachineTask(void *param) {
    while (true) {

        switch (currentState){
            case FORWARD:
                rightMotor.forward(0);
                leftMotor.forward(0);

                if(!startSignal){ // KILLSWITCH
                    currentState = IDLE;
                }
                else if(elapsedTime(1000)){ //MAXIMO TIEMPO DE AVANCE
                    currentState = BACKWARD;
                }
                else if(irSensor[0]){ // 
                    currentState = TURN_LEFT;
                    desiredAngle = 90;
                }
                else if(irSensor[1]){ //
                    currentState = TURN_LEFT;
                    desiredAngle = 90;
                }
                else if(irSensor[2]){
                    currentState = TURN_LEFT;
                    desiredAngle = 90;
                }
                else if(irSensor[3]){
                    currentState = TURN_RIGHT;
                    desiredAngle = 90;
                }                
                else if(irSensor[4]){
                    currentState = TURN_RIGHT;
                    desiredAngle = 90;
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
                    currentState = TURN_RIGHT;
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

void lineSensoTask(void *param) { 
    while(true){

        if(currentState == FORWARD){
            lineSensor[0] = checkLineSensor(LINE_FRONT_LEFT,3000);
            lineSensor[1] = checkLineSensor(LINE_FRONT_RIGHT,3000);
        }
        else if(currentState == FORWARD){
            lineSensor[2] = checkLineSensor(LINE_BACK_LEFT,3000);
            lineSensor[3] = checkLineSensor(LINE_BACK_RIGHT,3000);
        }
    }
}


#endif