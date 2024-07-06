#ifdef RUN_TASK_TEST
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
    } else {
        return false;
    }
}

bool checkRotation(int rotationAngle){
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
    else{return false;}
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

    while (true) {
        Serial.print(currentState);
        Serial.print("\t");
        Serial.print(irSensor[RIGHT]);
        Serial.print("\t");
        Serial.print(irSensor[MID]);
        Serial.print("\t");
        Serial.print(irSensor[LEFT]);
        Serial.print("\n");        

        switch (currentState){
            case FORWARD:
                rightMotor.forward(0);
                leftMotor.forward(0);

                if(!startSignal){
                    currentState = IDLE;
                }
                else if(elapsedTime(1000)){
                    currentState = TURN_RIGHT;
                }
                break;
            
            case BACKWARD:
                rightMotor.backward(0);
                leftMotor.backward(0);

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
                    currentState = BRAKE;
                    irSensor[RIGHT] = digitalRead(IR1);
                    irSensor[MID] = digitalRead(IR2);
                    irSensor[LEFT] = digitalRead(IR3);
                }
                break;

            case TURN_LEFT:
               // Serial.println("TURN LEFT");
                rightMotor.forward(0);
                leftMotor.backward(0);
                if(!startSignal){
                    currentState = IDLE;
                }
                else if(checkRotation(desiredAngle)){
                    currentState = BRAKE;
                }
                break;

                case TURN_RIGHT:
               // Serial.println("TURN RIGHT");
                rightMotor.backward(0);
                leftMotor.forward(0);

                if(!startSignal){
                    currentState = IDLE;
                }
                else if(checkRotation(desiredAngle)){
                    currentState = BRAKE;
                }
                break;

                case BRAKE:
                    leftMotor.brake();
                    rightMotor.brake();

                if(!startSignal){
                    currentState = IDLE;
                }
                // else if(irSensor[0]){
                //     currentState = TURN_RIGHT;
                //     desiredAngle = 30;
                // }
                else if(irSensor[RIGHT]){
                    currentState = TURN_RIGHT;
                    desiredAngle = 5;
                    irSensor[RIGHT] = 0;
                    //irSensor[RIGHT] = 0;
                }
                else if(irSensor[LEFT]){
                    currentState = TURN_LEFT;
                    desiredAngle = 5;
                    irSensor[LEFT] = 0;
                    //irSensor[LEFT]= 0;
                }
            

        }
    vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#endif