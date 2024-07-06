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
        // Serial.print(currentState);
        // Serial.print("\t");
        // Serial.print(irSensor[RIGHT]);
        // Serial.print("\t");
        // Serial.print(irSensor[MID]);
        // Serial.print("\t");
        // Serial.print(irSensor[LEFT]);
        // Serial.print("\n");

        switch (currentState)
        {
        case FORWARD:
            rightMotor.forward(0);
            leftMotor.forward(0);

            if (!startSignal)
            {
                currentState = IDLE;
            }
            else if (elapsedTime(1000))
            {
                currentState = TURN_RIGHT;
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
                irSensor[0] = digitalRead(IR1);
                irSensor[1] = digitalRead(IR2);
                irSensor[2] = digitalRead(IR3);
            }
            break;

        case TURN_LEFT:
            currMove = xTaskGetTickCount();
            if (currMove - lastLeft >= 10)
            { // 2 * delay
                rightMotor.forward(50);
                leftMotor.backward(50);
                lastLeft = xTaskGetTickCount();
            }
            else
            {
                currentState = BRAKE;
            }

            if (!startSignal)
            {
                currentState = IDLE;
            }
            else if (elapsedTime(2))
            {
                currentState = BRAKE;
            }
            break;

        case TURN_RIGHT:
            // Serial.println("TURN RIGHT");
            currMove = xTaskGetTickCount();
            if (currMove - lastRight >= 10)
            { // 2 * delay
                rightMotor.backward(20);
                leftMotor.forward(20);
                lastRight = xTaskGetTickCount();
            }
            else
            {
                currentState = BRAKE;
            }

            if (!startSignal)
            {
                currentState = IDLE;
            }
            else if (elapsedTime(2))
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
            else if (irSensor[RIGHT])
            {
                currentState = TURN_RIGHT;
                // desiredAngle = 15;
            }
            else if (irSensor[LEFT])
            {
                currentState = TURN_LEFT;
                // desiredAngle = 15;
            }
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

#endif