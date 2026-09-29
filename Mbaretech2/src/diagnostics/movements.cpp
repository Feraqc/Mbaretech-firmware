#include "firmwareConfig.h"
#if ENABLE_MOVEMENT_TEST && !ENABLE_LEGACY_MOVEMENTS
#include "globals.h"

// TO DO tener todo en macro y bien estrucutrado nuevamente

bool snake = false;
bool turkish = false;
bool retreat = false;
bool break_turn = false;

TickType_t currTurkish = 0;
TickType_t lastTurkish = 0;

TickType_t currMove;
TickType_t lastLeft45 = 0;
TickType_t lastRight45 = 0;
TickType_t lastLeft90 = 0;
TickType_t lastRight90 = 0;

TickType_t lastForward = 0;
TickType_t currForward = 0;

uint32_t delta = 0;

int line_left;
int line_right;
int tks;

void stateMachineTask(void *param) {

    changeState(BRAKE);

    bool counter = 0;
    rightMotor.begin();
    leftMotor.begin();
    TickType_t lastLeft = 0;
    TickType_t lastRight = 0;
    TickType_t currMove;

    int counterMov = 0;
    while (true) {
        line_left = readLineSensorFront(LINE_FRONT_LEFT);
        line_right = readLineSensorFront(LINE_FRONT_RIGHT);
        lineSensor[0] = checkLineSensora(line_left);
        lineSensor[1] = checkLineSensorb(line_right);
        if (startSignal) {
            switch (currentState) {

                case BRAKE:
                    #if ENABLE_DEBUG
                    Serial.println("BRAKE");
                    #endif
                    leftMotor.brake();
                    rightMotor.brake();

                    if (!startSignal) {
                        changeState(IDLE);
                    }
                    else if (parametros[0]==0){}
                    else if (parametros[0]==1){
                        if(parametros[1]==0){
                            retreat=false;
                            changeState(FORWARD);
                            break;
                        }
                        else if(parametros[1]==1){
                            changeState(BACKWARD);
                            break;
                        }
                        else if(parametros[1]==2){
                            retreat = true;
                            changeState(FORWARD);
                            break;
                        }
                    
                        else if(parametros[1]==3){
                            changeState(TURN_LEFT_45);
                            break;
                        }
                        else if(parametros[1]==4){
                            changeState(TURN_LEFT_90);
                            break;
                        }
                        else if(parametros[1]==5){
                            changeState(TURN_RIGHT_45);
                            break;
                        }
                        else if(parametros[1]==6){
                            changeState(TURN_RIGHT_90);
                            break;
                        }
                        else if(parametros[1]==7){
                            changeState(TURN_180);
                            break;
                        }
                        else if(parametros[1]==8){
                            tks=0;
                            changeState(TURKISH);
                            break;
                        }
                        else if(parametros[1]==9){
                            changeState(SNAKE);
                            break;
                        }
                        else if(parametros[1]==10){
                            changeState(SHORT_LEFT_MOVE);
                            break;
                        }
                        else if(parametros[1]==11){
                            changeState(SHORT_RIGHT_MOVE);
                            break;
                        }
                        else if(parametros[1]==12){
                            changeState(L_MOVEMENT_45);
                            break;
                        }
                        else if(parametros[1]==13){
                            changeState(R_MOVEMENT_45);
                            break;
                        }
                        else if(parametros[1]==14){
                            changeState(GIRO_U_L);
                            break;
                        }
                        else if(parametros[1]==15){
                            changeState(GIRO_U_R);
                            break;
                        }
                        else if(parametros[1]==16){
                            changeState(GIRO_U_L_LONG);
                            break;
                        }
                        else if(parametros[1]==17){
                            changeState(GIRO_U_R_LONG);
                            break;
                        }
                    }
                    else if (parametros[0]==2){
                        #ifdef MBARETECH_2
                            irSensor[SHORT_LEFT] = !digitalRead(IR2);
                            irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                            irSensor[TOP_MID] = !digitalRead(IR4);
                            irSensor[TOP_LEFT] = !digitalRead(IR3);
                            irSensor[TOP_RIGHT] = !digitalRead(IR5);
                            irSensor[SIDE_LEFT] = !digitalRead(IR1);
                            irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                        #endif
                        #if ENABLE_DEBUG
                            Serial.print(irSensor[SIDE_LEFT]);
                            Serial.print("\t");
                            Serial.print(irSensor[SHORT_LEFT]);
                            Serial.print("\t");
                            Serial.print(irSensor[TOP_LEFT]);
                            Serial.print("\t");
                            Serial.print(irSensor[TOP_MID]);
                            Serial.print("\t");
                            Serial.print(irSensor[TOP_RIGHT]);
                            Serial.print("\t");
                            Serial.print(irSensor[SHORT_RIGHT]);
                            Serial.print("\t");
                            Serial.print(irSensor[SIDE_RIGHT]);
                            Serial.print("\t\t");
                            Serial.print(line_left);
                            Serial.print("\t");
                            Serial.print(line_right);
                            Serial.print("\t");
                            Serial.print(lineSensor[0]);
                            Serial.print(lineSensor[1]);
                            Serial.print("\t\t");
                            Serial.print(digitalRead(DIPE));
                            Serial.print("\t");
                            Serial.print(digitalRead(DIPA));
                            Serial.print("\t");
                            Serial.print(digitalRead(DIPB));
                            Serial.print("\t");
                            Serial.print(digitalRead(DIPC));
                            Serial.print("\n"); 
                        #endif
                    }
                break;

                case IDLE:
                    #if ENABLE_DEBUG
                    Serial.println("IDLE");
                    #endif
                    leftMotor.brake();
                    rightMotor.brake();
                break;
                    
                case FORWARD:
                    #if ENABLE_DEBUG
                    Serial.println("FORWARD");
                    #endif
                    rightMotor.forward(parametros[2]);
                    leftMotor.forward(parametros[2]+parametros[18]);
                    
                    while (!elapsedTime(300)){
                        #if ENABLE_DEBUG
                        Serial.println("FORWARD LINE");
                        #endif
                        line_left = readLineSensorFront(LINE_FRONT_LEFT);
                        line_right = readLineSensorFront(LINE_FRONT_RIGHT);
                        lineSensor[0] = checkLineSensora(line_left);
                        lineSensor[1] = checkLineSensorb(line_right);
                        if(retreat && (lineSensor[0] || lineSensor[1])){
                            rightMotor.brake();
                            leftMotor.brake();
                            changeState(LINE_RETREAT);
                            break;
                        }
                    }
                    if(retreat){break;}
                    rightMotor.brake();
                    leftMotor.brake();
                    parametros[0]=0;
                    changeState(BRAKE);
                break;

                case BACKWARD:
                    #if ENABLE_DEBUG
                    Serial.println("BACKWARD");
                    #endif
                    rightMotor.backward(parametros[2]);
                    leftMotor.backward(parametros[2]+parametros[18]);
                    while (!elapsedTime(300)){}
                    rightMotor.brake();
                    leftMotor.brake();
                    parametros[0]=0;
                    changeState(BRAKE);
                break;
                
                case LINE_RETREAT:
                    #if ENABLE_DEBUG
                    Serial.println("LINE RETREAT");
                    #endif
                    rightMotor.backward(parametros[2]);
                    leftMotor.backward(parametros[2]+parametros[18]);
                    while(!elapsedTime(80)){}
                    rightMotor.brake();
                    leftMotor.brake();
                    parametros[0]=0;
                    irSensor[SHORT_LEFT] = !digitalRead(IR2);
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                    irSensor[SIDE_LEFT] = !digitalRead(IR1);
                    irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                    irSensor[TOP_LEFT] = !digitalRead(IR3);
                    irSensor[TOP_RIGHT] = !digitalRead(IR5);

                    if (irSensor[TOP_MID] || irSensor[SHORT_LEFT] || irSensor[SHORT_RIGHT] || irSensor[TOP_LEFT] || irSensor[TOP_RIGHT]){
                        changeState(BRAKE);
                        break;
                    }
                    else{
                        changeState(TURN_180);
                        break;
                    }
                
                break;

                case TURN_LEFT_45:
                    #if ENABLE_DEBUG
                    Serial.println("TURN L 45");
                    #endif
                    rightMotor.forward(parametros[3]);
                    leftMotor.backward(parametros[3]+parametros[18]);
                    while (!elapsedTime(parametros[4])){}
                    rightMotor.brake();
                    leftMotor.brake();
                    parametros[0]=0;
                    changeState(BRAKE);
                break;

                case TURN_LEFT_90:
                    #if ENABLE_DEBUG
                    Serial.println("TURN L 90");
                    #endif
                    rightMotor.forward(parametros[3]);
                    leftMotor.backward(parametros[3]+parametros[18]);
                    while (!elapsedTime(parametros[5])){}
                    rightMotor.brake();
                    leftMotor.brake();
                    parametros[0]=0;
                    changeState(BRAKE);
                break;

                case TURN_RIGHT_45:
                    #if ENABLE_DEBUG
                    Serial.println("TURN R 45");
                    #endif
                    leftMotor.forward(parametros[6]+parametros[18]);
                    rightMotor.backward(parametros[6]);
                    while (!elapsedTime(parametros[7])){}
                    rightMotor.brake();
                    leftMotor.brake();
                    parametros[0]=0;
                    changeState(BRAKE);
                break;

                case TURN_RIGHT_90:
                    #if ENABLE_DEBUG
                    Serial.println("TURN R 90");
                    #endif
                    leftMotor.forward(parametros[6]+parametros[18]);
                    rightMotor.backward(parametros[6]);
                    while (!elapsedTime(parametros[8])){
                    }
                    rightMotor.brake();
                    leftMotor.brake();
                    parametros[0]=0;
                    changeState(BRAKE);
                break;
       
                case TURN_180:
                    #if ENABLE_DEBUG
                    Serial.println("TURN 180");
                    #endif
                    rightMotor.forward(parametros[3]);
                    leftMotor.backward(parametros[3]+parametros[18]);
                    while (!elapsedTime(parametros[9])){}
                    rightMotor.brake();
                    leftMotor.brake();
                    parametros[0]=0;
                    changeState(BRAKE);
                break;

                case TURKISH:
                    currTurkish = xTaskGetTickCount();
                    if (currTurkish - lastTurkish >= parametros[13]){
                        tks++;
                        #if ENABLE_DEBUG
                        Serial.print("MOVING A LITTLE");
                        #endif
                        rightMotor.forward(parametros[15]);
                        leftMotor.forward(parametros[15]+parametros[18]);
                        while(!elapsedTime(parametros[14])){}
                        rightMotor.brake();
                        leftMotor.brake();
                        lastTurkish = xTaskGetTickCount();                      
                        }
                    if(tks>=3){
                        rightMotor.brake();
                        leftMotor.brake();
                        parametros[0]=0;
                        changeState(BRAKE);
                        break;
                    }
                    break;

                case SNAKE:
                    #if ENABLE_DEBUG
                    Serial.println("SNAKE");
                    #endif
                    leftMotor.forward(FORWARD_90);
                    rightMotor.forward(FORWARD_42);
                    while (!elapsedTime(80)) { //80 ms en mb2
                    }
                    rightMotor.forward(FORWARD_90);
                    leftMotor.forward(FORWARD_49);
                    while (!elapsedTime(80)) { //80 ms en mb2
                    }
                    rightMotor.brake();
                    leftMotor.brake();
                    parametros[0]=0;
                    changeState(BRAKE);
                    break;

                case SHORT_LEFT_MOVE:
                    rightMotor.forward(FORWARD_90);
                    leftMotor.forward(FORWARD_49);
                    while (!elapsedTime(80)) { //80 ms en mb2
                    }
                    leftMotor.brake();
                    rightMotor.brake();
                    parametros[0]=0;
                    changeState(BRAKE);
                    break;

                case SHORT_RIGHT_MOVE:
                    rightMotor.forward(FORWARD_42);
                    leftMotor.forward(FORWARD_90);
                    while (!elapsedTime(80)) { //80 ms en mb2
                    }
        
                    leftMotor.brake();
                    rightMotor.brake();
                    parametros[0]=0;
                    changeState(BRAKE);
                    break;

                case L_MOVEMENT_45:
                    #if ENABLE_DEBUG
                        Serial.println("L_MOVEMENT_45");
                    #endif
                    rightMotor.forward(parametros[3]);
                    leftMotor.backward(parametros[3]+parametros[18]);
                    while (!elapsedTime(parametros[4])){}   
                    rightMotor.forward(FORWARD_80);
                    leftMotor.forward(FORWARD_80);
                    while (!elapsedTime(150)){}
                    rightMotor.backward(parametros[6]);
                    leftMotor.forward(parametros[6]+parametros[18]);
                    while (!elapsedTime(parametros[8])){}
                    rightMotor.brake();
                    leftMotor.brake();
                    parametros[0]=0;
                    changeState(BRAKE);
                break;

                case R_MOVEMENT_45:
                    #if ENABLE_DEBUG
                        Serial.println("R_MOVEMENT_45");
                    #endif
                    rightMotor.backward(parametros[6]);
                    leftMotor.forward(parametros[6]+parametros[18]);
                    while (!elapsedTime(parametros[7])){}   
                    rightMotor.forward(FORWARD_80);
                    leftMotor.forward(FORWARD_80);
                    while (!elapsedTime(150)){}
                    rightMotor.forward(parametros[3]);
                    leftMotor.backward(parametros[3]+parametros[18]);
                    while (!elapsedTime(parametros[5])){}
                    rightMotor.brake();
                    leftMotor.brake();
                    parametros[0]=0;
                    changeState(BRAKE);
                break;

                case GIRO_U_L:
                    #if ENABLE_DEBUG
                    Serial.println("GIRO U IZQUIERDA CORTO");
                    #endif
                    rightMotor.forward(FORWARD_90);
                    leftMotor.forward(54);//80*2/3
                    while (!elapsedTime(parametros[16])) {
                        #if ENABLE_TURN_CANCEL
                        irSensor[TOP_MID] = !digitalRead(IR4);
                        irSensor[SHORT_LEFT] = !digitalRead(IR2);
                        irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                        irSensor[TOP_LEFT] = !digitalRead(IR3);
                        irSensor[TOP_RIGHT] = !digitalRead(IR5);
                        //irSensor[SIDE_LEFT] = !digitalRead(IR1);
                        //irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                        if (irSensor[TOP_MID]){
                            changeState(FORWARD);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_LEFT]){
                            changeState(SHORT_LEFT_MOVE);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_RIGHT]){
                            changeState(SHORT_RIGHT_MOVE);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_LEFT]){
                            changeState(TURN_LEFT_45);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_RIGHT]){
                            changeState(TURN_RIGHT_45);
                            break_turn = true;
                            break;
                        }/*
                        else if (irSensor[SIDE_LEFT]){
                            changeState(TURN_LEFT_90);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SIDE_RIGHT]){
                            changeState(TURN_RIGHT_90);
                            break_turn = true;
                            break;
                        }*/
                        #endif
                    }
                    leftMotor.brake();
                    rightMotor.brake();
                    parametros[0]=0;
                    changeState(BRAKE);
                break;

                case GIRO_U_R:
                    #if ENABLE_DEBUG
                    Serial.println("GIRO U DERECHA CORTO");
                    #endif
                    leftMotor.forward(FORWARD_90);
                    rightMotor.forward(54);//80*2/3
                    while (!elapsedTime(parametros[16])){
                        #if ENABLE_TURN_CANCEL
                        irSensor[TOP_MID] = !digitalRead(IR4);
                        irSensor[SHORT_LEFT] = !digitalRead(IR2);
                        irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                        irSensor[TOP_LEFT] = !digitalRead(IR3);
                        irSensor[TOP_RIGHT] = !digitalRead(IR5);
                        //irSensor[SIDE_LEFT] = !digitalRead(IR1);
                        //irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                        if (irSensor[TOP_MID]){
                            changeState(FORWARD);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_LEFT]){
                            changeState(SHORT_LEFT_MOVE);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_RIGHT]){
                            changeState(SHORT_RIGHT_MOVE);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_LEFT]){
                            changeState(TURN_LEFT_45);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_RIGHT]){
                            changeState(TURN_RIGHT_45);
                            break_turn = true;
                            break;
                        }/*
                        else if (irSensor[SIDE_LEFT]){
                            changeState(TURN_LEFT_90);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SIDE_RIGHT]){
                            changeState(TURN_RIGHT_90);
                            break_turn = true;
                            break;
                        }*/
                        #endif
                    }
                    leftMotor.brake();
                    rightMotor.brake();
                    parametros[0]=0;
                    changeState(BRAKE);
                break;

                case GIRO_U_L_LONG:
                    #if ENABLE_DEBUG
                    Serial.println("GIRO U IZQUIERDA LARGO");
                    #endif
                    rightMotor.forward(FORWARD_80);
                    leftMotor.forward(54);//80*2/3
                    while (!elapsedTime(parametros[17])) {
                        #if ENABLE_TURN_CANCEL
                        irSensor[TOP_MID] = !digitalRead(IR4);
                        irSensor[SHORT_LEFT] = !digitalRead(IR2);
                        irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                        irSensor[TOP_LEFT] = !digitalRead(IR3);
                        irSensor[TOP_RIGHT] = !digitalRead(IR5);
                        //irSensor[SIDE_LEFT] = !digitalRead(IR1);
                        //irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                        if (irSensor[TOP_MID]){
                            changeState(FORWARD);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_LEFT]){
                            changeState(SHORT_LEFT_MOVE);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_RIGHT]){
                            changeState(SHORT_RIGHT_MOVE);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_LEFT]){
                            changeState(TURN_LEFT_45);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_RIGHT]){
                            changeState(TURN_RIGHT_45);
                            break_turn = true;
                            break;
                        }/*
                        else if (irSensor[SIDE_LEFT]){
                            changeState(TURN_LEFT_90);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SIDE_RIGHT]){
                            changeState(TURN_RIGHT_90);
                            break_turn = true;
                            break;
                        }*/
                        #endif
                    }
                    leftMotor.brake();
                    rightMotor.brake();
                    parametros[0]=0;
                    changeState(BRAKE);
                break;

                case GIRO_U_R_LONG:
                    #if ENABLE_DEBUG
                    Serial.println("GIRO U DERECHA LARGO");
                    #endif
                    leftMotor.forward(FORWARD_80);
                    rightMotor.forward(54);//80*2/3
                    while (!elapsedTime(parametros[17])){
                        #if ENABLE_TURN_CANCEL
                        irSensor[TOP_MID] = !digitalRead(IR4);
                        irSensor[SHORT_LEFT] = !digitalRead(IR2);
                        irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                        irSensor[TOP_LEFT] = !digitalRead(IR3);
                        irSensor[TOP_RIGHT] = !digitalRead(IR5);
                        //irSensor[SIDE_LEFT] = !digitalRead(IR1);
                        //irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                        if (irSensor[TOP_MID]){
                            changeState(FORWARD);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_LEFT]){
                            changeState(SHORT_LEFT_MOVE);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_RIGHT]){
                            changeState(SHORT_RIGHT_MOVE);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_LEFT]){
                            changeState(TURN_LEFT_45);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_RIGHT]){
                            changeState(TURN_RIGHT_45);
                            break_turn = true;
                            break;
                        }/*
                        else if (irSensor[SIDE_LEFT]){
                            changeState(TURN_LEFT_90);
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SIDE_RIGHT]){
                            changeState(TURN_RIGHT_90);
                            break_turn = true;
                            break;
                        }*/
                        #endif
                    }
                    leftMotor.brake();
                    rightMotor.brake();
                    parametros[0]=0;
                    changeState(BRAKE);
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
                    changeState(BRAKE);
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
                    changeState(BRAKE);
                    break;

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
void loop(){};
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