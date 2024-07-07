#ifdef RUN_MOVEMENTS_TEST
#include "globals.h"

void imuTask(void *param)
{
    while (true)
    {
        imu.getData();
        if (xSemaphoreTake(gyroDataMutex, portMAX_DELAY) == pdTRUE)
        {
            currentAngle = imu.currentAngle;
            xSemaphoreGive(gyroDataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool elapsedTime(TickType_t duration)
{
    static TickType_t startTime = 0;
    static bool firstCall = true;
    TickType_t currentTime = xTaskGetTickCount();

    if (firstCall)
    {
        startTime = currentTime;
        firstCall = false;
    }
    // Serial.println(currentTime - startTime);
    if ((currentTime - startTime) >= duration)
    {
        startTime = currentTime;
        firstCall = true;
        return true;
    }
    else
    {
        return false;
    }
}

// bool checkRotation(int rotationAngle){
//     static int initialAngle = 0;
//     int currentAngleRotation = 0;
//     if (xSemaphoreTake(gyroDataMutex, portMAX_DELAY) == pdTRUE) {
//         if (initialAngle == 0) {
//             initialAngle = currentAngle;
//         }
//         currentAngleRotation = currentAngle;
//         xSemaphoreGive(gyroDataMutex);
//     }
//     if (abs(currentAngleRotation - initialAngle) >= rotationAngle) {
//         initialAngle = currentAngleRotation;
//         return true;
//     }
//     else{return false;}
// }

void stateMachineTask(void *param)
{
    enum State
    {
        IDLE,
        FORWARD,
        BACKWARD,
        TURN_RIGHT,
        TURN_LEFT,
        FORWARD_LEFT,
        FORWARD_RIGHT,
        ATTACK,
        BRAKE,
        INITIAL_MOVEMENT,
        TURN_90_LEFT,
        TURN_90_RIGHT,
        TURN_45_LEFT,
        TURN_45_RIGHT,
        TURN_22_LEFT,
        TURN_22_RIGHT
    };

    State currentState = TURN_90_RIGHT;
    bool counter = 0;
    TickType_t lastLeft = 0;
    TickType_t lastRight = 0;
    TickType_t currMove;

    while (true)
    {
        if (startSignal)
        {
            switch (currentState)
            {
            case FORWARD:
                Serial.println("State forward");
                rightMotor.forward(20);
                leftMotor.forward(20);

                if (!startSignal)
                { // KILLSWITCH
                    currentState = IDLE;
                }
                else if (elapsedTime(1000))
                { // MAXIMO TIEMPO DE AVANCE
                    currentState = BRAKE;
                }
                /* controlar con macro
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
                */
                break;

            case BRAKE:
                Serial.println("State brake");
                leftMotor.brake();
                rightMotor.brake();
                if (!startSignal)
                {
                    currentState = IDLE;
                }
                else if (elapsedTime(1000))
                {
                    counter = !counter;
                    if (counter)
                    {
                        Serial.print("Counter is");
                        Serial.println(counter);
                        currentState = BACKWARD;
                    }
                    else
                    {
                        Serial.print("Counter is");
                        Serial.println(counter);
                        currentState = FORWARD;
                    }
                }
                break;

            case BACKWARD:
                Serial.println("State backward");
                rightMotor.backward(20);
                leftMotor.backward(20);
                // rightMotor.writePulse(rightMotor.minBackwarPulse);
                // leftMotor.writePulse(leftMotor.minBackwarPulse);

                if (!startSignal)
                {
                    currentState = IDLE;
                }
                else if (elapsedTime(1000))
                {
                    // currentState = TURN_RIGHT;
                    currentState = BRAKE;
                }
                break;

            case IDLE:
                leftMotor.brake();
                rightMotor.brake();
                Serial.println(startSignal);
                if (startSignal)
                {
                    Serial.println("Changing state");
                    currentState = TURN_LEFT;
                }
                break;
                /*
                case TURN_LEFT:
                    currMove = xTaskGetTickCount();
                    if (currMove - lastLeft >= 2000)
                    { // 2 * delay
                        Serial.println("Entered turn left");
                        rightMotor.forward(50);
                        leftMotor.backward(50);
                        lastLeft = xTaskGetTickCount();
                    }

                    if(!startSignal){
                        currentState = IDLE;
                    }
                    else if(elapsedTime(1000)){
                        currentState = TURN_RIGHT;
                    }
                    break;

                case TURN_RIGHT:
                    currMove = xTaskGetTickCount();
                    if (currMove - lastRight >= 2000)
                    { // 2 * delay
                        Serial.println("Entered turn right");
                        rightMotor.backward(50);
                        leftMotor.forward(50);
                        lastRight = xTaskGetTickCount();
                    }

                    if(!startSignal){
                        currentState = IDLE;
                    }
                    else if(elapsedTime(1000)){
                        currentState = TURN_LEFT;
                    }
                    break;
                */
            case TURN_90_RIGHT:
                Serial.println("TURN");
                rightMotor.backward(60);
                leftMotor.forward(60);
                while (!elapsedTime(70))
                {
                }
                Serial.println("BRAKE");
                rightMotor.brake();
                leftMotor.brake();
                while (!elapsedTime(1000))
                {
                }

                if (!startSignal)
                {
                    currentState = IDLE;
                }
                break;

            case TURN_90_LEFT:
                Serial.println("TURN");
                rightMotor.forward(20);
                leftMotor.backward(20);
                while (!elapsedTime(1000))
                {
                }
                Serial.println("BRAKE");
                rightMotor.brake();
                leftMotor.brake();
                while (!elapsedTime(1000))
                {
                }
                break;

                if (!startSignal)
                {
                    currentState = IDLE;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// void lineSensoTask(void *param) {
//     while(true){

//     //     if(currentState == FORWARD){
//     //         lineSensor[0] = checkLineSensor(LINE_FRONT_LEFT,3000);
//     //         lineSensor[1] = checkLineSensor(LINE_FRONT_RIGHT,3000);
//     //     }
//     //     else if(currentState == FORWARD){
//     //         lineSensor[2] = checkLineSensor(LINE_BACK_LEFT,3000);
//     //         lineSensor[3] = checkLineSensor(LINE_BACK_RIGHT,3000);
//     //     }
//     // }
// }}

#endif