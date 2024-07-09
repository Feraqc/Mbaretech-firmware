#ifdef RUN_LINE_SENSOR
#include "globals.h"
#include "lineSensor.h"

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

void lineSensorTask(void *param) { 
        typedef enum {
        IDLE,
        FORWARD,
        BACKWARD,
        BRAKE,
    } State;
    State currentState;
    currentState = IDLE;
    bool lineSensor[2];
    while(true){
        lineSensor[0] = checkLineSensor(0,readLineSensor(LINE_FRONT_LEFT));
        lineSensor[1] = checkLineSensor(1,readLineSensor(LINE_FRONT_RIGHT));
        Serial.print(readLineSensor(LINE_FRONT_LEFT));
        Serial.print("\t");
        Serial.print(readLineSensor(LINE_FRONT_RIGHT));
        Serial.print("\t");
        Serial.print(lineSensor[0]);
        Serial.print("\t");
        Serial.print(lineSensor[1]);
        Serial.print("\n");
        
        if(startSignal){
            switch (currentState){
                case FORWARD:
                    rightMotor.forward(20);
                    leftMotor.forward(20);
                    if(lineSensor[0]||lineSensor[1]){
                        currentState = BACKWARD;
                    }
                    break;

                case BRAKE:
                    rightMotor.brake();
                    leftMotor.brake();
                    break;

                case BACKWARD:
                    rightMotor.backward(60);
                    leftMotor.backward(60);
                    if(elapsedTime(80)){
                        currentState = BRAKE;
                    }
                    break;
            }

        
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#endif