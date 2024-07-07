#ifdef RUN_TASK_TEST
#include "globals.h"

#define LAST_LEFT_45_TIMER 100
#define TURN_LEFT_SPEED
#define TURN_LEFT_DELAY

#define LAST_RIGHT_45_TIMER
#define TURN_RIGHT_SPEED
#define TURN_RIGHT_DELAY

bool elapsedTime(TickType_t duration) {
    static TickType_t startTime = 0;
    static bool firstCall = true;
    TickType_t currentTime = xTaskGetTickCount();

    if (firstCall) {
        startTime = currentTime;
        firstCall = false;
    }

    // Serial.println(currentTime - startTime);

    if ((currentTime - startTime) >= duration) {
        startTime = currentTime;
        firstCall = true;
        return true;
    }
    else {
        return false;
    }
}

void stateMachineTask(void *param) {
    bool counter = 0;
    while (true) {
        switch (currentState) {
            case FORWARD:
                Serial.println("State forward");
                rightMotor.forward(40);  // 40% maso
                leftMotor.forward(40);

                if (!startSignal) {  // KILLSWITCH
                    currentState = IDLE;
                }
                /*
                else if (irSensor[SHORT_LEFT] & !irSensor[SHORT_RIGHT]) {
                    leftMotor.forward(ALMOST_MAX_SPEED);
                    rightMotor.forward(MAX_SPEED);
                }
                else if (!irSensor[SHORT_LEFT] & irSensor[SHORT_RIGHT]) {
                    leftMotor.forward(MAX_SPEED);
                    rightMotor.forward(ALMOST_MAX_SPEED);
                }

                else if (irSensor[SHORT_LEFT] & irSensor[SHORT_RIGHT]) {
                    leftMotor.forward(MAX_SPEED);
                    rightMotor.forward(MAX_SPEED);
                }
                else if (!irSensor[FRONT_MID]) {
                    currentState = BRAKE;
                }
                */
                break;

            case BRAKE:
                Serial.println("State brake");
                leftMotor.brake();
                rightMotor.brake();
                if (!startSignal) {
                    currentState = IDLE;
                }

                irSensor[MID] = !digitalRead(IR4);
                irSensor[LEFT] = !digitalRead(IR3);
                irSensor[RIGHT] = !digitalRead(IR5);

                if (false) {  // leer sesnor medio
                    changeState(FORWARD);
                }
                else if (irSensor[TOP_LEFT]) {
                    changeState(TURN_LEFT_45);
                }
                /*
                else if (irSensor[TOP_RIGHT]){
                    changeState(TURN_RIGHT_45);
                }


                else if (irSensor[SIDE_LEFT]){
                    changeState(TURN_LEFT_90);
                }

                else if (irSensor[SIDE_RIGHT]){
                    changeState(TURN_RIGHT_90);
                }
                */
                break;
                /*
                else if(elapsedTime(1000)){
                    counter = !counter;
                    if (counter) {
                        Serial.print("Counter is");
                        Serial.println(counter);
                        currentState = BACKWARD;
                    }
                    else {
                        Serial.print("Counter is");
                        Serial.println(counter);
                        currentState = FORWARD;
                    }
                }
                break;
                */

            case TURN_LEFT_45:
                currMove = xTaskGetTickCount();
                if (currMove - lastLeft >= LAST_LEFT_45_TIMER) {  // 2 * delay
                    rightMotor.backward(TURN_LEFT_SPEED);
                    leftMotor.forward(TURN_LEFT_SPEED);
                    while (!elapsedTime(TURN_LEFT_DELAY)) {
                        Serial.println("Turning left");
                    };
                    lastLeft = xTaskGetTickCount();
                }

                if (!startSignal) {
                    currentState = IDLE;
                }
                else {
                    currentState = BRAKE;
                }
                break;

            case BACKWARD:
                Serial.println("State backward");
                rightMotor.backward(410);
                leftMotor.backward(410);
                // rightMotor.writePulse(rightMotor.minBackwarPulse);
                // leftMotor.writePulse(leftMotor.minBackwarPulse);

                if (!startSignal) {
                    currentState = IDLE;
                }
                else if (elapsedTime(1000)) {
                    // currentState = TURN_RIGHT;
                    currentState = BRAKE;
                }
                break;

            case IDLE:
                leftMotor.brake();
                rightMotor.brake();
                Serial.println(startSignal);
                if (startSignal) {
                    Serial.println("Changing state");
                    currentState = BRAKE;
                }
                break;

            case TURN_LEFT:
                rightMotor.forward(0);
                leftMotor.backward(0);

                if (!startSignal) {
                    currentState = IDLE;
                }
                else if (checkRotation(85)) {
                    leftMotor.brake();
                    rightMotor.brake();
                    currentState = FORWARD;
                }
                break;

            case TURN_RIGHT:
                rightMotor.backward(0);
                leftMotor.forward(0);

                if (!startSignal) {
                    currentState = IDLE;
                }
                else if (checkRotation(85)) {
                    leftMotor.brake();
                    rightMotor.brake();
                    currentState = FORWARD;
                }
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void lineSensorTask(void *param) {
    while (true) {
        if (currentState == FORWARD) {
            lineSensor[0] = checkLineSensor(LINE_FRONT_LEFT, 3000);
            lineSensor[1] = checkLineSensor(LINE_FRONT_RIGHT, 3000);
        }
        else if (currentState == FORWARD) {
            lineSensor[2] = checkLineSensor(LINE_BACK_LEFT, 3000);
            lineSensor[3] = checkLineSensor(LINE_BACK_RIGHT, 3000);
        }
    }
}

#endif