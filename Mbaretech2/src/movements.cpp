#ifdef RUN_MOVEMENTS_TEST
#include "globals.h"


void stateMachineTask(void *param) {

    State currentState = BRAKE;

    bool counter = 0;
    TickType_t lastLeft = 0;
    TickType_t lastRight = 0;
    TickType_t currMove;

    int counterMov = 0;
    while (true) {
        if (startSignal) {
            switch (currentState) {
                case FORWARD:
                    Serial.println("State forward");
                    rightMotor.forward(100);
                    leftMotor.forward(100);
                    while (!elapsedTime(300)){}
                    rightMotor.brake();
                    leftMotor.brake();
                    while (!elapsedTime(2000)){}
                    currentState = BACKWARD;
                    break;

                case BRAKE:
                    Serial.println("State brake");
                    leftMotor.brake();
                    rightMotor.brake();

                    if (!startSignal) {
                        currentState = IDLE;
                    }
                    
                    if(digitalRead(DIPA)){
                        currentState = TURKISH;
                    }
                    else if(digitalRead(DIPB)){
                        // irSensor[TOP_LEFT] = !digitalRead(IR3);
                        // irSensor[TOP_RIGHT] = !digitalRead(IR5);
                        // if (irSensor[TOP_LEFT]) {  // irSensor[TOP_LEFT]
                        //     currentState = TURN_LEFT_45;
                        // }
                        // else if (irSensor[TOP_RIGHT]) {  // irSensor[TOP_RIGHT]
                        //     currentState = TURN_RIGHT_45;
                        // }

                        currentState = MOVEMENT_45;
                    }
                    else if(digitalRead(DIPC)){
                        irSensor[SHORT_LEFT] = !digitalRead(IR2);
                        irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                        if (irSensor[SHORT_LEFT] & !irSensor[SHORT_RIGHT]) {
                            currentState = FORWARD_LEFT;
                        }
                        else if (!irSensor[SHORT_LEFT] & irSensor[SHORT_RIGHT]) {
                            currentState = FORWARD_RIGHT;
                        }
                    }
                    else if(digitalRead(DIPD)){
                        currentState = SNAKE;
                    }

                    break;
                    // else if (elapsedTime(2000)) {
                    //     counter = false;
                    //     // counter = !counter;
                    //     if (counter) {
                    //         Serial.print("Counter is");
                    //         Serial.println(counter);
                    //         currentState = BACKWARD;
                    //     }
                    //     else {
                    //         Serial.print("Counter is");
                    //         Serial.println(counter);
                    //         currentState = FORWARD;
                    //     }
                case BACKWARD:
                    Serial.println("State backward");
                    rightMotor.backward(100);
                    leftMotor.backward(100);
                    while (!elapsedTime(300)){}
                    rightMotor.brake();
                    leftMotor.brake();
                    while (!elapsedTime(2000)){}
                    currentState = IDLE;
                    if (!startSignal) {
                        currentState = IDLE;
                    }

                    break;

                case IDLE:
                    Serial.println("IDLE");
                    leftMotor.brake();
                    rightMotor.brake();
                    break;

                case TURN_RIGHT_90:
                    Serial.println("TURN R 90");
                    rightMotor.backward(80);
                    leftMotor.forward(80);
                    while (!elapsedTime(95)) {
                    }
                    Serial.println("BRAKE");
                    rightMotor.brake();
                    leftMotor.brake();
                    currentState = BRAKE;
                    break;

                case TURN_LEFT_90:
                    Serial.println("TURN L 90");
                    rightMotor.forward(80);
                    leftMotor.backward(80);
                    while (!elapsedTime(95)) {
                    }
                    Serial.println("BRAKE");
                    rightMotor.brake();
                    leftMotor.brake();
                    currentState = BRAKE;
                    break;

                case TURN_RIGHT_45:
                    Serial.println("TURN R 45");
                    rightMotor.backward(80);
                    leftMotor.forward(80);
                    while (!elapsedTime(70)) {  // necesita un poco mas
                    }
                    Serial.println("BRAKE");
                    rightMotor.brake();
                    leftMotor.brake();
                    currentState = BRAKE;
                    break;

                case TURN_LEFT_45:
                    Serial.println("TURN L 45");
                    rightMotor.forward(80);
                    leftMotor.backward(80);
                    while (!elapsedTime(70)) {
                    }
                    Serial.println("BRAKE");
                    rightMotor.brake();
                    leftMotor.brake();
                    currentState = BRAKE;
                    break;

                case FORWARD_RIGHT:
                    rightMotor.forward(FORWARD_42);
                    leftMotor.forward(FORWARD_90);
                    while (!elapsedTime(80)) { //80 ms en mb2
                    }
                    Serial.println("BRAKE");
                    rightMotor.brake();
                    leftMotor.brake();
                    while (!elapsedTime(1000)) {
                    }
                    break;

                case FORWARD_LEFT:
                    rightMotor.forward(FORWARD_90);
                    leftMotor.forward(FORWARD_49);
                    while (!elapsedTime(80)) { //80 ms en mb2
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
                    while (!elapsedTime(80)) { //80 ms en mb2
                    }
                    rightMotor.forward(FORWARD_90);
                    leftMotor.forward(FORWARD_49);
                    while (!elapsedTime(80)) { //80 ms en mb2
                    }
                    Serial.println("BRAKE");
                    rightMotor.brake();
                    leftMotor.brake();
                    while (!elapsedTime(1500)) {
                    }
                    break;
                
                case TURN_180:
                    rightMotor.forward(80);
                    leftMotor.backward(80);
                    while (!elapsedTime(170)) {
                    }
                    Serial.println("BRAKE");
                    rightMotor.brake();
                    leftMotor.brake();
                    currentState = IDLE;

                break;

                case MOVEMENT_45:
                    Serial.println("TURN R 45");
                    rightMotor.backward(80);
                    leftMotor.forward(80);
                    while (!elapsedTime(70)) {
                        #ifdef DEBUG
                        Serial.println("Turning right");
                        #endif
                    }
                    
                    rightMotor.forward(FORWARD_40);  // AVANCE
                    leftMotor.forward(FORWARD_40);
                    while (!elapsedTime(140)) {  // Ajustar, en asuncion 250 por ahi era
                                            }

                    rightMotor.forward(80);
                    leftMotor.backward(80);
                    while (!elapsedTime(95)) {
                    }
                    while (!elapsedTime(TURN_LEFT_90_DELAY)) {
                        #ifdef DEBUG
                        Serial.println("TURN L 90");
                        #endif
                    };

                    rightMotor.brake();
                    leftMotor.brake();
                    currentState = BRAKE;
                }
            }
        else{
            Serial.println("Waiting signal");
            // Serial.print(digitalRead(DIPA));
            // Serial.print(digitalRead(DIPB));
            // Serial.print(digitalRead(DIPC));
            //Serial.println(digitalRead(DIPD));
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