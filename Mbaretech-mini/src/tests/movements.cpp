#ifdef RUN_MOVEMENTS_TEST
#include "globals.h"


// bool checkRotation(int rotationAngle) {
//     static int initialAngle = 0;
//     int currentAngleRotation = 0;
//     static bool firstCall = true;

//     if (xSemaphoreTake(gyroDataMutex, portMAX_DELAY) == pdTRUE) {
//         if (firstCall) {
//             firstCall = false;
//             initialAngle = currentAngle;  // Initialize initialAngle only once
//         }
//         currentAngleRotation = currentAngle;
//         xSemaphoreGive(gyroDataMutex);
//     }

//     if (abs(currentAngleRotation - initialAngle) >= rotationAngle) {
//         initialAngle = currentAngleRotation;
//         firstCall = true;
//         return true;
//     }
//     else {
//         return false;
//     }
// }

void motorTask(void *param) {

    State currentState;
    currentState = FORWARD;

    TickType_t lastLeft = 0;
    TickType_t lastRight = 0;
    TickType_t currMove;

     while (true) {
        if (startSignal) {
            switch (currentState){
                case FORWARD:
                    leftMotor.forward(FORWARD_SPEED);
                    rightMotor.forward(FORWARD_SPEED - 5);
                    while(!elapsedTime(500)){}
                    while(true){}
                break;
                case TURN_RIGHT:
                    Serial.println("Entering turn right");
                    
                    rightMotor.brake();
                    leftMotor.forward(10);
                    while(!elapsedTime(105)){};
                    rightMotor.brake();
                    leftMotor.brake();
                    while (!elapsedTime(1000)) {};
                    Serial.println("Getting out of turn right");
                    break;
                case TURN_LEFT:
                    rightMotor.forward(10);
                    leftMotor.brake();
                    while(!elapsedTime(105)){};
                    Serial.println("Stopping for a sec");
                    rightMotor.brake();
                    leftMotor.brake();
                    while (!elapsedTime(1000)) {};
                    Serial.println("Getting out of turn right");
                    break;

                // case MOVEMENT_U_LEFT:
                //     while(!elapsedTime(2500)){}
                //     rightMotor.forward(0); //AVANCE
                //     leftMotor.forward(45);
                //     while(!elapsedTime(950)){}
                //     currentState = IDLE;
                //     break;
                // case MOVEMENT_U_RIGHT:
                //     while(!elapsedTime(2500)){}
                //     rightMotor.forward(0); //AVANCE
                //     leftMotor.forward(95);
                //     while(!elapsedTime(950)){}
                //     currentState = IDLE;
                //     break;

                case MOVEMENT_45:
                    while(!elapsedTime(2500)){}
                    Serial.println("entro");
                    //Serial.println("INITIAL MOVE");
                        Serial.println("GIRO");
                            rightMotor.forward(TURN_LEFT_SPEED + 40); //GIRO
                            leftMotor.backward(20);
                    while(!elapsedTime(TURN_LEFT_DELAY)){}
                            Serial.println("AVANCE");
                            rightMotor.forward(45); //AVANCE
                            leftMotor.forward(45);
                    while(!elapsedTime(190)){}
                            Serial.println("GIRO");
                            rightMotor.backward(70); //GIRO
                            leftMotor.forward(TURN_RIGHT_SPEED + 70);
                    while(!elapsedTime(TURN_RIGHT_DELAY+20)){}
                            currentState = IDLE;

                    break;
                
                case IDLE:
                        Serial.println("IDLE");
                        rightMotor.brake();
                        leftMotor.brake();
                    break;
                }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void loop() {};

#endif