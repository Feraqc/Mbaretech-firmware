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

TickType_t lastForward = 0;
TickType_t currForward = 0;

uint32_t delta = 0;

int line_left;
int line_right;

#ifdef MBARETECH_2
    uint32_t local_speed = FORWARD_80;
#endif

#ifdef MBARTETECH_1
    uint32_t local_speed = FORWARD_60;
#endif


bool break_turn = false;



void stateMachineTask(void *param) {
    bool counter = 0;
    currentState = IDLE;
    while (true) {
        line_left = readLineSensorFront(LINE_FRONT_LEFT);
        line_right = readLineSensorFront(LINE_FRONT_RIGHT);

        lineSensor[0] = checkLineSensor(line_left);
        lineSensor[1] = checkLineSensor(line_right);

        if ( (lineSensor[0] || lineSensor[1]) && !(irSensor[TOP_MID] || irSensor[SHORT_LEFT] || irSensor[SHORT_RIGHT])){
            if(startSignal){
                currentState = LINE_RETREAT;
            }
        }

        switch (currentState) {
            case FORWARD:
#ifdef DEBUG
                Serial.println("State forward");
#endif 
                currForward = xTaskGetTickCount();
                irSensor[TOP_MID] = !digitalRead(IR4);
                if ((currForward - lastForward) >= 500){
                    if (irSensor[TOP_MID]){
                        delta += 5;
                    }
                }
                #ifdef MBARETECH_2
                local_speed = FORWARD_80; // + delta
                #endif
                
                #ifdef MBARETECH_1
                local_speed = FORWARD_70; // + delta
                #endif
                if (local_speed > 95){
                    local_speed = 95;
                }

                if (!snake) {
                    rightMotor.forward(local_speed);  // 40% maso
                    leftMotor.forward(local_speed);
                }
                else {
                    #ifdef MBARETECH_2
                    leftMotor.forward(FORWARD_90);
                    rightMotor.forward(FORWARD_80);
                    #endif

                    #ifdef MBARETECH_1
                    leftMotor.forward(FORWARD_90);
                    rightMotor.forward(FORWARD_42);
                    #endif

                    while (!elapsedTime(SHORT_RIGHT_DELAY+10)) {  // 80 ms en mb2
                    }

                    #ifdef MBARETECH_2
                    rightMotor.forward(FORWARD_90);
                    leftMotor.forward(FORWARD_80);   
                    #endif

                    #ifdef MBARETECH_1
                    rightMotor.forward(FORWARD_90);
                    leftMotor.forward(FORWARD_49); 
                    #endif
                    //? ?         // probar mas
                    while (!elapsedTime(SHORT_LEFT_DELAY+10)) {  // 80 ms en mb2
                    }
                }

                #ifdef MBARETECH_2
                irSensor[SHORT_LEFT] = !digitalRead(IR2);
                irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                #endif

                #ifdef MBARETECH_1
                irSensor[SHORT_LEFT] = digitalRead(IR2);
                irSensor[SHORT_RIGHT] = digitalRead(IR6);
                #endif

                if (!startSignal) {  // KILLSWITCH
                    currentState = IDLE;
                }
                else if (irSensor[SHORT_LEFT] && irSensor[SHORT_RIGHT]) {
                    if (!snake) {
                        #ifdef DEBUG
                        Serial.print("MAX SPEED");
                        #endif
                        leftMotor.forward(MAX_SPEED);
                        rightMotor.forward(MAX_SPEED);
                    }
                    else {
                        #ifdef DEBUG
                        Serial.print("MAX SNAKE");
                        #endif
                        leftMotor.forward(FORWARD_90);
                        rightMotor.forward(FORWARD_80);
                        while (
                            !elapsedTime(SHORT_RIGHT_DELAY)) {  // 80 ms en mb2
                        }
                        rightMotor.forward(FORWARD_90);
                        leftMotor.forward(FORWARD_80);            // probar mas
                        while (!elapsedTime(SHORT_LEFT_DELAY)) {  // 80 ms en
                                                                  // mb2
                        }
                    }
                }

                else if (irSensor[SHORT_LEFT] && !irSensor[SHORT_RIGHT]) {
                    #ifdef DEBUG
                    Serial.print("SHORT LEFT");
                    #endif
                    leftMotor.forward(FORWARD_60);
                    rightMotor.forward(MAX_SPEED);
                    while (!elapsedTime(SHORT_LEFT_DELAY)) {  // 80 ms en mb2
                    }
                }
                else if (!irSensor[SHORT_LEFT] && irSensor[SHORT_RIGHT]) {
                    #ifdef DEBUG
                    Serial.print("SHORT_RIGHT");
                    #endif
                    leftMotor.forward(MAX_SPEED);
                    rightMotor.forward(52);
                    while (!elapsedTime(SHORT_RIGHT_DELAY)) {  // 80 ms en mb2
                    }
                }

                else if (!irSensor[TOP_MID]) {
                    #ifdef DEBUG
                    Serial.print("IR MID OFF");
                    #endif
                    if (turkish){
                        leftMotor.brake();
                        rightMotor.brake();
                    }
                    currentState = BRAKE;
                    lastForward = xTaskGetTickCount();
                    delta = 0;
                    #ifdef MBARETECH_2
                    local_speed = FORWARD_80;
                    #endif

                    #ifdef MBARTECH_1
                    local_speed = FORWARD_70;
                    #endif
                }
                else{
                    #ifdef DEBUG
                    Serial.print("IR MID ON");
                    #endif
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

                #ifdef MBARETECH_2
                irSensor[TOP_MID] = !digitalRead(IR4);
                irSensor[TOP_LEFT] = !digitalRead(IR3);
                irSensor[TOP_RIGHT] = !digitalRead(IR5);
                irSensor[SHORT_LEFT] = !digitalRead(IR2);
                irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                irSensor[SIDE_LEFT] = !digitalRead(IR1);
                irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                #endif

                #ifdef MBARETECH_1
                irSensor[TOP_MID] = !digitalRead(IR4);
                irSensor[TOP_LEFT] = digitalRead(IR3);
                irSensor[TOP_RIGHT] = digitalRead(IR5);
                irSensor[SHORT_LEFT] = digitalRead(IR2);
                irSensor[SHORT_RIGHT] = digitalRead(IR6);
                #endif

                if (irSensor[TOP_MID]) {  // irSensor[TOP_MID] leer sesnor medio
                    currentState = FORWARD;
                }
                else if (irSensor[SHORT_LEFT]){
                    currentState = SHORT_LEFT_MOVE;
                }
                else if (irSensor[SHORT_RIGHT]){
                    currentState = SHORT_RIGHT_MOVE;
                }
                else if (irSensor[TOP_LEFT]) {  // irSensor[TOP_LEFT]
                    currentState = TURN_LEFT_45;
                }
                else if (irSensor[TOP_RIGHT]) {  // irSensor[TOP_RIGHT]
                    currentState = TURN_RIGHT_45;
                }

                #ifdef MBARETECH_2
                else if (irSensor[SIDE_LEFT]) {  // irSensor[SIDE_LEFT]
                    currentState = TURN_LEFT_90;
                }

                else if (irSensor[SIDE_RIGHT]) {  // irSensor[SIDE_RIGHT]
                    currentState = TURN_RIGHT_90;
                }
                #endif
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
                        #ifdef DEBUG
                        Serial.print("Default forward");
                        #endif
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
                        Serial.println("Turning left 45");
                        #endif
                        #ifdef CANCEL_TURNS
                        irSensor[TOP_MID] = !digitalRead(IR4);
                        irSensor[SHORT_LEFT] = !digitalRead(IR2);
                        irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                        irSensor[TOP_LEFT] = !digitalRead(IR3);
                        irSensor[TOP_RIGHT] = !digitalRead(IR5);
                        if (irSensor[TOP_MID]){
                            currentState = FORWARD;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_LEFT]){
                            currentState = SHORT_LEFT_MOVE;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_RIGHT]){
                            currentState = SHORT_RIGHT_MOVE;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_LEFT]){
                            currentState = TURN_LEFT_45;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_RIGHT]){
                            currentState = TURN_RIGHT_45;
                            break_turn = true;
                            break;
                        }
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
                        Serial.println("Turning right 45");
                        #endif
                        #ifdef CANCEL_TURNS
                        irSensor[TOP_MID] = !digitalRead(IR4);
                        irSensor[SHORT_LEFT] = !digitalRead(IR2);
                        irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                        irSensor[TOP_LEFT] = !digitalRead(IR3);
                        irSensor[TOP_RIGHT] = !digitalRead(IR5);
                        if (irSensor[TOP_MID]){
                            currentState = FORWARD;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_LEFT]){
                            currentState = SHORT_LEFT_MOVE;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_RIGHT]){
                            currentState = SHORT_RIGHT_MOVE;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_LEFT]){
                            currentState = TURN_LEFT_45;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_RIGHT]){
                            currentState = TURN_RIGHT_45;
                            break_turn = true;
                            break;
                        }
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
                    if (!break_turn){
                    currentState = BRAKE;
                    }
                    break_turn = false;
                }
                break;

            case TURN_LEFT_90:
                currMove = xTaskGetTickCount();
                if (currMove - lastLeft90 >= LAST_LEFT_90_TIMER) {  // 2 * delay
                    rightMotor.forward(TURN_LEFT_SPEED);
                    leftMotor.backward(TURN_LEFT_SPEED);
                    while (!elapsedTime(TURN_LEFT_90_DELAY)) {
                        #ifdef DEBUG
                        Serial.println("Turning left 90");
                        #endif
                        #ifdef CANCEL_TURNS
                        irSensor[TOP_MID] = !digitalRead(IR4);
                        irSensor[SHORT_LEFT] = !digitalRead(IR2);
                        irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                        irSensor[TOP_LEFT] = !digitalRead(IR3);
                        irSensor[TOP_RIGHT] = !digitalRead(IR5);
                        if (irSensor[TOP_MID]){
                            currentState = FORWARD;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_LEFT]){
                            currentState = SHORT_LEFT_MOVE;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_RIGHT]){
                            currentState = SHORT_RIGHT_MOVE;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_LEFT]){
                            currentState = TURN_LEFT_45;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_RIGHT]){
                            currentState = TURN_RIGHT_45;
                            break_turn = true;
                            break;
                        }
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
                        Serial.println("Turning right 90");
                        #endif
                        #ifdef CANCEL_TURNS
                        irSensor[TOP_MID] = !digitalRead(IR4);
                        irSensor[SHORT_LEFT] = !digitalRead(IR2);
                        irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                        irSensor[TOP_LEFT] = !digitalRead(IR3);
                        irSensor[TOP_RIGHT] = !digitalRead(IR5);
                        if (irSensor[TOP_MID]){
                            currentState = FORWARD;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_LEFT]){
                            currentState = SHORT_LEFT_MOVE;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_RIGHT]){
                            currentState = SHORT_RIGHT_MOVE;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_LEFT]){
                            currentState = TURN_LEFT_45;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_RIGHT]){
                            currentState = TURN_RIGHT_45;
                            break_turn = true;
                            break;
                        }
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
                rightMotor.backward(80); //Tenia 100 en movements
                leftMotor.backward(80);
                // Le puso delay de 300 ms por algun motivo
                if (!startSignal) {
                    currentState = IDLE;
                }
                currentState = BRAKE;
                break;

            case IDLE:
                leftMotor.brake();
                rightMotor.brake();

                #ifdef MBARETECH_2
                irSensor[SHORT_LEFT] = !digitalRead(IR2);
                irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                irSensor[TOP_MID] = !digitalRead(IR4);
                irSensor[TOP_LEFT] = !digitalRead(IR3);
                irSensor[TOP_RIGHT] = !digitalRead(IR5);
                #endif

                #ifdef MBARETECH_1
                irSensor[SHORT_LEFT] = digitalRead(IR2); 
                irSensor[TOP_LEFT] = digitalRead(IR3);
                irSensor[TOP_MID] = !digitalRead(IR4);
                irSensor[TOP_RIGHT] = digitalRead(IR5);
                irSensor[SHORT_RIGHT] = digitalRead(IR6);
                #endif

#ifdef DEBUG
                //Serial.println(startSignal);
                #ifdef MBARETECH_2
                irSensor[SIDE_LEFT] = !digitalRead(IR1);
                irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                #endif

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
                Serial.print(digitalRead(DIPA));
                Serial.print("\t");
                Serial.print(digitalRead(DIPB));
                Serial.print("\t");
                Serial.print(digitalRead(DIPC));
                Serial.print("\t");
                Serial.print(digitalRead(DIPD));
                Serial.print("\n");


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
                        #define NUMBER_OF_READS 10 // no se usa
                        currentState = BRAKE;
                    }
                    else {
                        #define NUMBER_OF_READS 10
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

            case MOVEMENT_45: //ahora mismo es un giro 90 nomas
            /*
                rightMotor.backward(TURN_RIGHT_SPEED);
                leftMotor.forward(TURN_RIGHT_SPEED);
                //while(!elapsedTime(1)){Serial.println("Mini delay");}
                while (!elapsedTime(TURN_RIGHT_45_DELAY)) {
                //vTaskDelay(70);
#ifdef DEBUG
                    Serial.println("Turning right movement 45");
#endif
                };

                rightMotor.forward(FORWARD_60); 
                leftMotor.forward(FORWARD_60);   //PROBAR
                while (!elapsedTime(140)) {  // Ajustar, en asuncion 250 por ahi era
                    #ifdef DEBUG
                    Serial.println("Forward movement 45");
                    #endif
                }
                rightMotor.forward(TURN_LEFT_SPEED);
                leftMotor.backward(TURN_LEFT_SPEED);
                while (!elapsedTime(TURN_LEFT_90_DELAY)) {
                    #ifdef DEBUG
                    Serial.println("Turning left movement 45");
                    #endif
                };
                rightMotor.brake();
                leftMotor.brake();
                */
                rightMotor.forward(TURN_LEFT_SPEED);
                leftMotor.backward(TURN_LEFT_SPEED);
                while (!elapsedTime(TURN_LEFT_90_DELAY)) {
                    #ifdef DEBUG
                    Serial.println("Turning left 90");
                    #endif
                    #ifdef CANCEL_TURNS
                        irSensor[TOP_MID] = !digitalRead(IR4);
                        irSensor[SHORT_LEFT] = !digitalRead(IR2);
                        irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                        irSensor[TOP_LEFT] = !digitalRead(IR3);
                        irSensor[TOP_RIGHT] = !digitalRead(IR5);
                        if (irSensor[TOP_MID]){
                            currentState = FORWARD;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_LEFT]){
                            currentState = SHORT_LEFT_MOVE;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_RIGHT]){
                            currentState = SHORT_RIGHT_MOVE;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_LEFT]){
                            currentState = TURN_LEFT_45;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_RIGHT]){
                            currentState = TURN_RIGHT_45;
                            break_turn = true;
                            break;
                        }
                        #endif
                };
                leftMotor.brake();
                rightMotor.brake();

                currentState = BRAKE;
                break;

            case TURN_180:
                rightMotor.forward(TURN_LEFT_SPEED);
                leftMotor.backward(TURN_LEFT_SPEED);
                while (!elapsedTime(TURN_LEFT_180_DELAY)) {
                    #ifdef DEBUG
                    Serial.println("Turning left 180");
                    #endif
                    #ifdef CANCEL_TURNS
                        irSensor[TOP_MID] = !digitalRead(IR4);
                        irSensor[SHORT_LEFT] = !digitalRead(IR2);
                        irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                        irSensor[TOP_LEFT] = !digitalRead(IR3);
                        irSensor[TOP_RIGHT] = !digitalRead(IR5);
                        if (irSensor[TOP_MID]){
                            currentState = FORWARD;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_LEFT]){
                            currentState = SHORT_LEFT_MOVE;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SHORT_RIGHT]){
                            currentState = SHORT_RIGHT_MOVE;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_LEFT]){
                            currentState = TURN_LEFT_45;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[TOP_RIGHT]){
                            currentState = TURN_RIGHT_45;
                            break_turn = true;
                            break;
                        }
                        #endif
                };
                leftMotor.brake();
                rightMotor.brake();
                currentState = BRAKE;
                break;
            
            case LINE_RETREAT:
                rightMotor.backward(FORWARD_80);
                leftMotor.backward(FORWARD_80);
                while(!elapsedTime(80)){
                    #ifdef DEBUG
                    Serial.print("RETREAT");
                    #endif
                };
                irSensor[SHORT_LEFT] = !digitalRead(IR2);
                irSensor[TOP_MID] = !digitalRead(IR4);
                irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                irSensor[SIDE_LEFT] = !digitalRead(IR1);
                irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                irSensor[TOP_LEFT] = !digitalRead(IR3);
                irSensor[TOP_RIGHT] = !digitalRead(IR5);

                if (irSensor[TOP_MID] || irSensor[SHORT_LEFT] || irSensor[SHORT_RIGHT] || irSensor[TOP_LEFT] || irSensor[TOP_RIGHT]){
                    currentState = BRAKE;
                }
                else{
                    currentState = TURN_180;
                    turkish=true;
                }
                break;

            case SHORT_LEFT_MOVE:
                leftMotor.forward(FORWARD_60); //cambiar en forward
                rightMotor.forward(MAX_SPEED);
                irSensor[SHORT_LEFT] = !digitalRead(IR2);
                irSensor[TOP_MID] = !digitalRead(IR4);
                if (irSensor[TOP_MID]){
                    currentState = FORWARD;
                }
                else if (!irSensor[SHORT_LEFT]){
                    leftMotor.brake();
                    rightMotor.brake();
                    currentState = BRAKE;
                }
                break;

            case SHORT_RIGHT_MOVE:
                leftMotor.forward(MAX_SPEED);
                rightMotor.forward(52);
                irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                irSensor[TOP_MID] = !digitalRead(IR4);
                if (irSensor[TOP_MID]){
                    currentState = FORWARD;
                }
                else if(!irSensor[SHORT_RIGHT]){
                    leftMotor.brake();
                    rightMotor.brake();
                    currentState = BRAKE;
                }
                break;
                

        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


#endif