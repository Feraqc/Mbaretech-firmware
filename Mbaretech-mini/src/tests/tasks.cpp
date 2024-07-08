#ifdef RUN_TASK_TEST
#include "globals.h"

bool snake = false;
bool turkish = false;

TickType_t currTurkish = 0;
TickType_t lastTurkish = 0;

void motorTask(void *param) {
    State currentState;
    currentState = IDLE;

    TickType_t lastLeft = 0;
    TickType_t lastRight = 0;
    TickType_t currMove;

    while (true) {
        if (currentState) {
            Serial.print(currentState);
            /*
            Serial.print("\t");
            Serial.print(irSensor[LEFT]);
            Serial.print("\t");
            Serial.print(irSensor[MID]);
            Serial.print("\t");
            Serial.print(irSensor[RIGHT]);
            */
            Serial.print("\n");
        }

        switch (currentState) {
            case FORWARD:
                Serial.println("FORWARD");
                if (!snake) {
                    rightMotor.forward(FORWARD_SPEED);
                    leftMotor.forward(FORWARD_SPEED);
                }
                else {
                    leftMotor.forward(FORWARD_SPEED);
                    rightMotor.forward(FORWARD_SPEED - 5);
                    while (!elapsedTime(TURN_LEFT_DELAY)) {
                    }
                    rightMotor.forward(FORWARD_SPEED);
                    leftMotor.forward(FORWARD_SPEED - 5);
                    while (!elapsedTime(TURN_LEFT_DELAY)) {
                    }
                }

                irSensor[MID] = !digitalRead(IR2);
                irSensor[LEFT] = !digitalRead(IR1);
                irSensor[RIGHT] = !digitalRead(IR3);

                if (!startSignal) {
                    currentState = IDLE;
                } /*
                 else if (elapsedTime(1000))
                 {
                     currentState = TURN_RIGHT;
                 }
                 */
                else if (irSensor[LEFT] & !irSensor[RIGHT]) {
                    leftMotor.forward(ALMOST_MAX_SPEED);
                    rightMotor.forward(MAX_SPEED);
                }
                else if (!irSensor[LEFT] & irSensor[RIGHT]) {
                    leftMotor.forward(MAX_SPEED);
                    rightMotor.forward(ALMOST_MAX_SPEED);
                }

                else if (irSensor[LEFT] & irSensor[RIGHT]) {
                    if (!turkish) {
                        leftMotor.forward(MAX_SPEED);
                        rightMotor.forward(MAX_SPEED);
                    }
                    else {
                        leftMotor.forward(MAX_SPEED);
                        rightMotor.forward(ALMOST_MAX_SPEED);
                        while (!elapsedTime(TURN_LEFT_DELAY)) {
                        }
                        rightMotor.forward(MAX_SPEED);
                        leftMotor.forward(ALMOST_MAX_SPEED);
                        while (!elapsedTime(TURN_LEFT_DELAY)) {
                        }
                    }
                }
                else if (!irSensor[MID]) {
                    currentState = BRAKE;
                }
                break;

            case BACKWARD:
                Serial.println("BACKWARD");
                rightMotor.backward(MIN_SPEED);
                leftMotor.backward(MIN_SPEED);

                if (!startSignal) {
                    currentState = IDLE;
                }
                else if (elapsedTime(1000)) {
                    currentState = FORWARD;
                }
                break;

            case IDLE:
                leftMotor.brake();
                rightMotor.brake();
                Serial.print(digitalRead(DIPA));
                Serial.print(digitalRead(DIPB));
                Serial.print(digitalRead(DIPC));
                Serial.println(digitalRead(DIPD));
                if (startSignal) {
                    Serial.println("Changing state");

                    if (digitalRead(DIPB)) {
                        turkish = true;
                    }

                    else if (digitalRead(DIPA)) {  // Ojo valido solo para mini
                                                   // las combinaciones
                        snake = true;
                    }
                    currentState = FORWARD;

                    if (!digitalRead(DIPD) & digitalRead(DIPC)) {
                        // currentState = TURN_180;
                    }
                    else if (digitalRead(DIPD) & !digitalRead(DIPC)) {
                        currentState = MOVEMENT_45;
                    }
                    else if (digitalRead(DIPD) & digitalRead(DIPC)) {
                        // currentState = ???;
                    }
                }
                break;

            case TURN_LEFT:
                Serial.println("TURN LEFT");
                currMove = xTaskGetTickCount();
                if (currMove - lastLeft >= LAST_LEFT_TIMER) {  // 2 * delay
                    rightMotor.brake();
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

            case TURN_RIGHT:
                Serial.println("TURN_RIGHT");
                currMove = xTaskGetTickCount();
                if (currMove - lastRight >= LAST_RIGHT_TIMER) {  // 2 * delay
                    rightMotor.forward(TURN_RIGHT_SPEED);
                    leftMotor.brake();
                    while (!elapsedTime(TURN_RIGHT_DELAY)) {
                        Serial.println("Turning left");
                    };
                    lastRight = xTaskGetTickCount();
                }

                if (!startSignal) {
                    currentState = IDLE;
                }
                currentState = BRAKE;

                // else if(checkRotation(desiredAngle)){
                //     currentState = BRAKE;
                // }
                break;

            case BRAKE:
                Serial.println("BRAKE");
                //leftMotor.brake();
                //rightMotor.brake();

                if (!startSignal) {
                    currentState = IDLE;
                }

                irSensor[MID] = !digitalRead(IR2);
                irSensor[LEFT] = !digitalRead(IR1);
                irSensor[RIGHT] = !digitalRead(IR3);
                if (irSensor[MID]) {
                    currentState = FORWARD;
                }
                else if (irSensor[LEFT]) {
                    currentState = TURN_LEFT;
                }

                else if (irSensor[RIGHT]) {
                    currentState = TURN_RIGHT;
                }
                else {
                    if (turkish) {
                        // Completar
                        currTurkish = xTaskGetTickCount();
                        if (currTurkish - lastTurkish >= TURKISH_TIME) {
                            currentState = FORWARD;
                            lastTurkish = xTaskGetTickCount();
                        }
                    }
                    else {
                        currentState = FORWARD;
                    }
                }
                break;

            case MOVEMENT_45:
                
                Serial.println("entro");
                // Serial.println("INITIAL MOVE");
                Serial.println("GIRO");
                rightMotor.forward(TURN_LEFT_SPEED + 40);  // GIRO
                leftMotor.backward(20);
                while (!elapsedTime(TURN_LEFT_DELAY)) {
                }
                Serial.println("AVANCE");
                rightMotor.forward(45);  // AVANCE
                leftMotor.forward(45);
                while (!elapsedTime(190)) {
                }
                Serial.println("GIRO");
                rightMotor.backward(70);  // GIRO
                leftMotor.forward(TURN_RIGHT_SPEED + 70);
                while (!elapsedTime(TURN_RIGHT_DELAY + 20)) {
                }
                currentState = BRAKE;

                break;
        }
        vTaskDelay(pdMS_TO_TICKS(TASK_TICKS));
    }
}

void loop() {};

#endif