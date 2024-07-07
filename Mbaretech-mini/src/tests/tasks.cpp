#ifdef RUN_TASK_TEST
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
        vTaskDelay(pdMS_TO_TICKS(1));
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

bool checkRotation(int rotationAngle)
{
    static int initialAngle = 0;
    int currentAngleRotation = 0;
    static bool firstCall = true;

    if (xSemaphoreTake(gyroDataMutex, portMAX_DELAY) == pdTRUE)
    {
        if (firstCall)
        {
            firstCall = false;
            initialAngle = currentAngle; // Initialize initialAngle only once
        }
        currentAngleRotation = currentAngle;
        xSemaphoreGive(gyroDataMutex);
    }

    if (abs(currentAngleRotation - initialAngle) >= rotationAngle)
    {
        initialAngle = currentAngleRotation;
        firstCall = true;
        return true;
    }
    else
    {
        return false;
    }
}

void motorTask(void *param)
{
    typedef enum
    {
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

    while (true)
    {
        Serial.print(currentState);
        Serial.print("\t");
        Serial.print(irSensor[LEFT]);
        Serial.print("\t");
        Serial.print(irSensor[MID]);
        Serial.print("\t");
        Serial.print(irSensor[RIGHT]);
        Serial.print("\n");

        switch (currentState)
        {
        case FORWARD:
            rightMotor.forward(15);
            leftMotor.forward(15);

            irSensor[MID] = !digitalRead(IR2);
            irSensor[LEFT] = !digitalRead(IR1);
            irSensor[RIGHT] = !digitalRead(IR3);

            if (!startSignal)
            {
                currentState = IDLE;
            }/*
            else if (elapsedTime(1000))
            {
                currentState = TURN_RIGHT;
            }
            */
            else if (irSensor[LEFT] & !irSensor[RIGHT]){
                leftMotor.forward(98);
                rightMotor.forward(100);
            }
            else if (!irSensor[LEFT] & irSensor[RIGHT]){
                leftMotor.forward(100);
                rightMotor.forward(98);
            }
           
            else if (irSensor[LEFT] & irSensor[RIGHT]){
                leftMotor.forward(100);
                rightMotor.forward(100);
            }
            else if (!irSensor[MID]){
                currentState = BRAKE;
            }
            break;

        case BACKWARD:
            rightMotor.backward(0);
            leftMotor.backward(0);

            if (!startSignal)
            {
                currentState = IDLE;
            }
            else if (elapsedTime(1000))
            {
                currentState = FORWARD;
            }
            break;

        case IDLE:
            leftMotor.brake();
            rightMotor.brake();
            if (startSignal)
            {
                currentState = BRAKE;
            }
            break;

        case TURN_LEFT:
            currMove = xTaskGetTickCount();
            if (currMove - lastLeft >= 140)
            { // 2 * delay
                rightMotor.brake();
                leftMotor.forward(10);
                while(!elapsedTime(105)){
                    Serial.println("Turning left");
                };
                lastLeft = xTaskGetTickCount();
            }

            if (!startSignal)
            {
                currentState = IDLE;
            }
            else {
                currentState = BRAKE;
            }
            break;

        case TURN_RIGHT:
            // Serial.println("TURN RIGHT");
            currMove = xTaskGetTickCount();
            if (currMove - lastRight >= 140)
            { // 2 * delay
                rightMotor.forward(10);
                leftMotor.brake();
                while(!elapsedTime(105)){
                    Serial.println("Turning left");
                };
                lastRight = xTaskGetTickCount();
            }

            if (!startSignal)
            {
                currentState = IDLE;
            }
            else if (elapsedTime(3))
            {
                currentState = BRAKE;
            }
            // else if(checkRotation(desiredAngle)){
            //     currentState = BRAKE;
            // }
            break;

        case BRAKE:
            leftMotor.brake();
            rightMotor.brake();

            if (!startSignal)
            {
                currentState = IDLE;
            }
            // else if(irSensor[0]){
            //     currentState = TURN_RIGHT;
            //     desiredAngle = 30;
            // }
            /*
            else if (irSensor[RIGHT] && !irSensor[MID] && !irSensor[LEFT])
            {
                currentState = TURN_RIGHT;
                // desiredAngle = 15;
            }
            else if (irSensor[LEFT] && !irSensor[MID] && !irSensor[RIGHT])
            {
                currentState = TURN_LEFT;
                // desiredAngle = 15;
            }
            break;
            */
           irSensor[MID] = !digitalRead(IR2);
           irSensor[LEFT] = !digitalRead(IR1);
           irSensor[RIGHT] = !digitalRead(IR3);
           if (irSensor[MID]){
                currentState = FORWARD;
           }
           else if (irSensor[LEFT]){
                currentState = TURN_LEFT;
                //Serial.println("IR LEFT ON");
           }

           else if (irSensor[RIGHT]){
                currentState = TURN_RIGHT;
           }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#endif