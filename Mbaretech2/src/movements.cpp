#ifdef RUN_MOVEMENTS_TEST
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

void stateMachineTask(void *param) {

    State currentState = SNAKE;

    bool counter = 0;
    TickType_t lastLeft = 0;
    TickType_t lastRight = 0;
    TickType_t currMove;

    while (true) {
        if (startSignal) {
            switch (currentState) {
                case FORWARD:
                    Serial.println("State forward");
                    rightMotor.forward(40);
                    leftMotor.forward(40);

                    if (elapsedTime(
                            250)) {  // MAXIMO TIEMPO DE AVANCE  //75 es turco
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
                    else if (elapsedTime(2000)) {
                        counter = false;
                        // counter = !counter;
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

                case BACKWARD:
                    Serial.println("State backward");
                    rightMotor.backward(20);
                    leftMotor.backward(20);
                    // rightMotor.writePulse(rightMotor.minBackwarPulse);
                    // leftMotor.writePulse(leftMotor.minBackwarPulse);

                    if (!startSignal) {
                        currentState = IDLE;
                    }
                    else if (elapsedTime(200)) {
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
                        // currentState = TURN_LEFT;
                    }
                    break;

                case TURN_RIGHT_90:
                    Serial.println("TURN");
                    rightMotor.backward(TURN_RIGHT_SPEED);
                    leftMotor.forward(TURN_RIGHT_SPEED);
                    while (!elapsedTime(TURN_RIGHT_90_DELAY)) {
                    }
                    Serial.println("BRAKE");
                    rightMotor.brake();
                    leftMotor.brake();
                    while (!elapsedTime(1000)) {
                    }
                    break;

                case TURN_LEFT_90:
                    Serial.println("TURN");
                    rightMotor.forward(TURN_LEFT_SPEED);
                    leftMotor.backward(TURN_LEFT_SPEED);
                    while (!elapsedTime(TURN_LEFT_90_DELAY)) {
                    }
                    Serial.println("BRAKE");
                    rightMotor.brake();
                    leftMotor.brake();
                    while (!elapsedTime(1500)) {
                    }
                    break;

                case TURN_RIGHT_45:
                    Serial.println("TURN");
                    rightMotor.backward(TURN_RIGHT_SPEED);
                    leftMotor.forward(TURN_RIGHT_SPEED);
                    while (!elapsedTime(
                        TURN_RIGHT_45_DELAY)) {  // necesita un poco mas
                    }
                    Serial.println("BRAKE");
                    rightMotor.brake();
                    leftMotor.brake();
                    while (!elapsedTime(1000)) {
                    }
                    break;

                case TURN_LEFT_45:
                    Serial.println("TURN");
                    rightMotor.forward(TURN_LEFT_SPEED);
                    leftMotor.backward(TURN_LEFT_SPEED);
                    while (!elapsedTime(TURN_LEFT_45_DELAY)) {
                    }
                    Serial.println("BRAKE");
                    rightMotor.brake();
                    leftMotor.brake();
                    while (!elapsedTime(1000)) {
                    }
                    break;

                case FORWARD_RIGHT:
                    rightMotor.forward(FORWARD_42);
                    leftMotor.forward(FORWWARD_90);
                    while (!elapsedTime(SHORT_RIGHT_DELAY)) { //80 ms en mb2
                    }
                    Serial.println("BRAKE");
                    rightMotor.brake();
                    leftMotor.brake();
                    while (!elapsedTime(1000)) {
                    }
                    break;

                case FORWARD_LEFT:
                    rightMotor.forward(FORWARD_60);
                    leftMotor.forward(FORWARD_49);
                    while (!elapsedTime(SHORT_LEFT_DELAY)) { //80 ms en mb2
                    }
                    Serial.println("BRAKE");
                    rightMotor.brake();
                    leftMotor.brake();
                    while (!elapsedTime(1000)) {
                    }
                    break;

                case SNAKE:
                    leftMotor.forward(FORWARD_90);
                    rightMotor.forward(FORWARD_42);
                    while (!elapsedTime(SHORT_RIGHT_DELAY)) { //80 ms en mb2
                    }
                    rightMotor.forward(FORWARD_60);
                    leftMotor.forward(FORWARD_49);
                    while (!elapsedTime(SHORT_LEFT_DELAY)) { //80 ms en mb2
                    }
                    Serial.println("BRAKE");
                    rightMotor.brake();
                    leftMotor.brake();
                    while (!elapsedTime(1000)) {
                    }
                    break;
            }
        }
        else {
            Serial.println("Waiting signal");
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