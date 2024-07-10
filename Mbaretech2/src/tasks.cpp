#ifdef RUN_TASK_TEST
#include "globals.h"

bool snake = false;
bool turkish = false;

TickType_t currTurkish = 0;
TickType_t lastTurkish = 0;

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
#ifdef DEBUG
                Serial.println("State forward");
#endif 

                if (!snake) {
                    rightMotor.forward(FORWARD_40);  // 40% maso
                    leftMotor.forward(FORWARD_40);
                }
                else {
                    leftMotor.forward(FORWARD_90);
                    rightMotor.forward(FORWARD_42);
                    while (!elapsedTime(SHORT_RIGHT_DELAY)) {  // 80 ms en mb2
                    }
                    rightMotor.forward(FORWARD_60);
                    leftMotor.forward(FORWARD_49);            // probar mas
                    while (!elapsedTime(SHORT_LEFT_DELAY)) {  // 80 ms en mb2
                    }
                }

                irSensor[SHORT_LEFT] = !digitalRead(IR2);
                irSensor[TOP_MID] = !digitalRead(IR4);
                irSensor[SHORT_RIGHT] = !digitalRead(IR6);

                if (!startSignal) {  // KILLSWITCH
                    currentState = IDLE;
                }
                else if (irSensor[SHORT_LEFT] & irSensor[SHORT_RIGHT]) {
                    if (!snake) {
                        leftMotor.forward(MAX_SPEED);
                        rightMotor.forward(MAX_SPEED);
                    }
                    else {
                        leftMotor.forward(FORWARD_90);
                        rightMotor.forward(FORWARD_70);
                        while (
                            !elapsedTime(SHORT_RIGHT_DELAY)) {  // 80 ms en mb2
                        }
                        rightMotor.forward(FORWARD_90);
                        leftMotor.forward(FORWARD_70);            // probar mas
                        while (!elapsedTime(SHORT_LEFT_DELAY)) {  // 80 ms en
                                                                  // mb2
                        }
                    }
                }

                else if (irSensor[SHORT_LEFT] & !irSensor[SHORT_RIGHT]) {
                    leftMotor.forward(FORWARD_60);
                    rightMotor.forward(FORWARD_49);
                    while (!elapsedTime(SHORT_LEFT_DELAY)) {  // 80 ms en mb2
                    }
                }
                else if (!irSensor[SHORT_LEFT] & irSensor[SHORT_RIGHT]) {
                    leftMotor.forward(FORWARD_90);
                    rightMotor.forward(FORWARD_42);
                    while (!elapsedTime(SHORT_RIGHT_DELAY)) {  // 80 ms en mb2
                    }
                }

                else if (!irSensor[TOP_MID]) {
                    leftMotor.brake();
                    rightMotor.brake();
                    currentState = BRAKE;
                }

                break;

            case BRAKE:
#ifdef DEBUG
                Serial.println("State brake");
#endif
                //leftMotor.brake();
                //rightMotor.brake();
                if (!startSignal) {
                    currentState = IDLE;
                }

                irSensor[TOP_MID] = !digitalRead(IR4);
                irSensor[TOP_LEFT] = !digitalRead(IR3);
                irSensor[TOP_RIGHT] = !digitalRead(IR5);

                if (irSensor[TOP_MID]) {  // irSensor[TOP_MID] leer sesnor medio
                    currentState = FORWARD;
                }
                else if (irSensor[TOP_LEFT]) {  // irSensor[TOP_LEFT]
                    currentState = TURN_LEFT_45;
                }
                else if (irSensor[TOP_RIGHT]) {  // irSensor[TOP_RIGHT]
                    currentState = TURN_RIGHT_45;
                }

                irSensor[SIDE_LEFT] = !digitalRead(IR1);
                irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                if (irSensor[SIDE_LEFT]) {  // irSensor[SIDE_LEFT]
                    currentState = TURN_LEFT_90;
                }

                else if (irSensor[SIDE_RIGHT]) {  // irSensor[SIDE_RIGHT]
                    currentState = TURN_RIGHT_90;
                }
                else{
                    if (turkish){
                        currTurkish = xTaskGetTickCount();
                        if (currTurkish - lastTurkish >= TURKISH_TIME) {
                            #ifdef DEBUG
                            Serial.print("MOVING A LITTLE");
                            #endif
                            rightMotor.forward(FORWARD_40);
                            leftMotor.forward(FORWARD_40);
                            while(!elapsedTime(TURKISH_DELAY)){}
                            rightMotor.brake();
                            leftMotor.brake();
                            lastTurkish = xTaskGetTickCount();
                        }
                    }
                    else{
                        currentState = FORWARD;
                    }
                }
                break;

            case TURN_LEFT_45:
                currMove = xTaskGetTickCount();
                if (currMove - lastLeft45 >= LAST_LEFT_45_TIMER) {  // 2 * delay
                    rightMotor.forward(TURN_LEFT_SPEED);
                    leftMotor.backward(TURN_LEFT_SPEED);
                    while (!elapsedTime(TURN_LEFT_45_DELAY)) {
                        #ifdef DEBUG
                        Serial.println("Turning left");
                        #endif
                    };
                    lastLeft45 = xTaskGetTickCount();
                    leftMotor.brake();
                    rightMotor.brake();
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
                        #ifdef DEBUG
                        Serial.println("Turning right");
                        #endif
                    };
                    lastRight45 = xTaskGetTickCount();
                    leftMotor.brake();
                    rightMotor.brake();
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
                        #ifdef DEBUG
                        Serial.println("Turning left");
                        #endif
                    };
                    lastLeft90 = xTaskGetTickCount();
                    leftMotor.brake();
                    rightMotor.brake();
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
                        #ifdef DEBUG
                        Serial.println("Turning right");
                        #endif
                    };
                    lastRight90 = xTaskGetTickCount();
                    leftMotor.brake();
                    rightMotor.brake();
                }

                if (!startSignal) {
                    currentState = IDLE;
                }
                else {
                    currentState = BRAKE;
                }
                break;

            case BACKWARD:
                #ifdef DEBUG
                Serial.println("State backward");
                #endif
                rightMotor.backward(40);
                leftMotor.backward(40);

                if (!startSignal) {
                    currentState = IDLE;
                }
                currentState = BRAKE;
                break;

            case IDLE:
                leftMotor.brake();
                rightMotor.brake();
#ifdef DEBUG
                Serial.println(startSignal);
#endif
                if (startSignal) {
#ifdef DEBUG
                    Serial.println("Changing state");
#endif
                    if (digitalRead(DIPA)) {
                        snake = true;
                    }

                    if (digitalRead(DIPB)) {
                        turkish = true;
                        currentState = BRAKE;
                    }
                    else {
                        currentState = FORWARD;
                    }

                    if (digitalRead(DIPC) & !digitalRead(DIPD)) {
                        //currentState = HACER U
                        currentState = FORWARD;
                    }
                    else if (!digitalRead(DIPC) & digitalRead(DIPD)) {
                        currentState = MOVEMENT_45;
                    }
                    else if (digitalRead(DIPC) & digitalRead(DIPD)) {
                        currentState = TURN_180;
                    }
                }
                break;

            case MOVEMENT_45:
                rightMotor.backward(TURN_RIGHT_SPEED);
                leftMotor.forward(TURN_RIGHT_SPEED);
                while (!elapsedTime(TURN_RIGHT_45_DELAY)) {
#ifdef DEBUG
                    Serial.println("Turning right");
#endif
                };

                rightMotor.forward(FORWARD_40);  // AVANCE
                leftMotor.forward(FORWARD_40);
                while (!elapsedTime(
                    140)) {  // Ajustar, en asuncion 250 por ahi era
                }
                rightMotor.forward(TURN_LEFT_SPEED);
                leftMotor.backward(TURN_LEFT_SPEED);
                while (!elapsedTime(TURN_LEFT_90_DELAY)) {
                    #ifdef DEBUG
                    Serial.println("Turning left");
                    #endif
                };
                rightMotor.brake();
                leftMotor.brake();
                currentState = BRAKE;
                break;

            case TURN_180:
                rightMotor.forward(TURN_LEFT_SPEED);
                leftMotor.backward(TURN_LEFT_SPEED);
                while (!elapsedTime(TURN_LEFT_180_DELAY)) {
                    #ifdef DEBUG
                    Serial.println("Turning left");
                    #endif
                };
                leftMotor.brake();
                rightMotor.brake();
                currentState = BRAKE;
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void lineSensorTask(void *param) {
    while (true) {
        if (currentState == FORWARD) {
            lineSensor[0] = checkLineSensor(LINE_FRONT_LEFT);
            lineSensor[1] = checkLineSensor(LINE_FRONT_RIGHT);
        }
    }
}

#endif