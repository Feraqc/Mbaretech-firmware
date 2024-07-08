#ifdef RUN_TASK_TEST
#include "globals.h"

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

TickType_t currMove;
TickType_t lastLeft45 = 0;
TickType_t lastRight45 = 0;
TickType_t lastLeft90 = 0;
TickType_t lastRight90 = 0;

void stateMachineTask(void *param) {
    bool counter = 0;
    while (true) {
        switch (currentState) {
            case FORWARD:
                Serial.println("State forward");
                rightMotor.forward(40);  // 40% maso
                leftMotor.forward(40);

                irSensor[SHORT_LEFT] = !digitalRead(IR2);
                irSensor[TOP_MID] = !digitalRead(IR4);
                irSensor[SHORT_RIGHT] = !digitalRead(IR6);

                if (!startSignal) {  // KILLSWITCH
                    currentState = IDLE;
                }
                else if (irSensor[SHORT_LEFT] & irSensor[SHORT_RIGHT]) {
                    leftMotor.forward(MAX_SPEED);
                    rightMotor.forward(MAX_SPEED);
                }
                
                else if (irSensor[SHORT_LEFT] & !irSensor[SHORT_RIGHT]) {
                    leftMotor.forward(FORWARD_60);
                    rightMotor.forward(FORWARD_49);
                    while (!elapsedTime(SHORT_LEFT_DELAY)) { //80 ms en mb2
                    }
                }
                else if (!irSensor[SHORT_LEFT] & irSensor[SHORT_RIGHT]) {
                    leftMotor.forward(FORWARD_90);
                    rightMotor.forward(FORWARD_42);
                    while (!elapsedTime(SHORT_RIGHT_DELAY)) { //80 ms en mb2
                    }
                }

                else if (!irSensor[TOP_MID]) {
                    currentState = BRAKE;
                }
                
                break;

            case BRAKE:
                Serial.println("State brake");
                leftMotor.brake();
                rightMotor.brake();
                if (!startSignal) {
                    currentState = IDLE;
                }

                irSensor[TOP_MID] = !digitalRead(IR4);
                irSensor[TOP_LEFT] = !digitalRead(IR3);
                irSensor[TOP_RIGHT] = !digitalRead(IR5);
                

                if (irSensor[TOP_MID]) {  // irSensor[TOP_MID] leer sesnor medio
                    changeState(FORWARD);
                }
                else if (irSensor[TOP_LEFT]) {  // irSensor[TOP_LEFT]
                    changeState(TURN_LEFT_45);
                }
                else if (irSensor[TOP_RIGHT]) {  // irSensor[TOP_RIGHT]
                    changeState(TURN_RIGHT_45);
                }
                
                irSensor[SIDE_LEFT] = !digitalRead(IR1);
                irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                if (irSensor[SIDE_LEFT]) { // irSensor[SIDE_LEFT]
                    changeState(TURN_LEFT_90);
                }

                else if (irSensor[SIDE_RIGHT]) {// irSensor[SIDE_RIGHT]
                    changeState(TURN_RIGHT_90);
                } 
                break;
                

            case TURN_LEFT_45:
                currMove = xTaskGetTickCount();
                if (currMove - lastLeft45 >= LAST_LEFT_45_TIMER) {  // 2 * delay
                    rightMotor.forward(TURN_LEFT_SPEED);
                    leftMotor.backward(TURN_LEFT_SPEED);
                    while (!elapsedTime(TURN_LEFT_45_DELAY)) {
                        Serial.println("Turning left");
                    };
                    lastLeft45 = xTaskGetTickCount();
                }

                if (!startSignal) {
                    currentState = IDLE;
                }
                else {
                    currentState = BRAKE;
                }
                break;

            case TURN_RIGHT_45:
                currMove = xTaskGetTickCount();
                if (currMove - lastRight45 >=
                    LAST_RIGHT_45_TIMER) {  // 2 * delay
                    rightMotor.backward(TURN_RIGHT_SPEED);
                    leftMotor.forward(TURN_RIGHT_SPEED);
                    while (!elapsedTime(TURN_RIGHT_45_DELAY)) {
                        Serial.println("Turning right");
                    };
                    lastRight45 = xTaskGetTickCount();
                }

                if (!startSignal) {
                    currentState = IDLE;
                }
                else {
                    currentState = BRAKE;
                }
                break;

            case TURN_LEFT_90:
                currMove = xTaskGetTickCount();
                if (currMove - lastLeft90 >= LAST_LEFT_90_TIMER) {  // 2 * delay
                    rightMotor.forward(TURN_LEFT_SPEED);
                    leftMotor.backward(TURN_LEFT_SPEED);
                    while (!elapsedTime(TURN_LEFT_90_DELAY)) {
                        Serial.println("Turning left");
                    };
                    lastLeft90 = xTaskGetTickCount();
                }

                if (!startSignal) {
                    currentState = IDLE;
                }
                else {
                    currentState = BRAKE;
                }
                break;

            case TURN_RIGHT_90:
                currMove = xTaskGetTickCount();
                if (currMove - lastRight90 >=
                    LAST_RIGHT_90_TIMER) {  // 2 * delay
                    rightMotor.backward(TURN_RIGHT_SPEED);
                    leftMotor.forward(TURN_RIGHT_SPEED);
                    while (!elapsedTime(TURN_RIGHT_90_DELAY)) {
                        Serial.println("Turning right");
                    };
                    lastRight90 = xTaskGetTickCount();
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
                rightMotor.backward(40);
                leftMotor.backward(40);
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
                    if (digitalRead(DIPA)){

                    }
                    else if (digitalRead(DIPB)){

                    }
                    else if (digitalRead(DIPC)){

                    }
                    else if (digitalRead(DIPD)){

                    }
                    else{
                        currentState = BRAKE;
                    }
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