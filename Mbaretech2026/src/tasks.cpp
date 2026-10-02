#ifdef RUN_TASK_TEST
#include "globals.h"

bool snake = false;
bool turkish = false;
bool bandera = true;

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
    rightMotor.begin();
    leftMotor.begin();
    currentState = IDLE;
    while (true) {
        line_left = readLineSensorFront(LINE_FRONT_LEFT);
        line_right = readLineSensorFront(LINE_FRONT_RIGHT);
        lineSensor[0] = checkLineSensora(line_left);
        lineSensor[1] = checkLineSensorb(line_right);

        if (currentState != TEST_FORWARD && (lineSensor[0] || lineSensor[1]) && !(irSensor[TOP_MID] || irSensor[SHORT_LEFT] || irSensor[SHORT_RIGHT])){
            if(startSignal){
                currentState = LINE_RETREAT;
            }
        }

        switch (currentState){
            case IDLE:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                    leftMotor.brake();
                    rightMotor.brake();
                    #ifdef DEBUG
                        Serial.println("IDLE");
                    #endif

                    #ifdef MBARETECH_2
                    irSensor[SHORT_LEFT] = !digitalRead(IR2);
                    irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    irSensor[TOP_LEFT] = !digitalRead(IR3);
                    irSensor[TOP_RIGHT] = !digitalRead(IR5);
                    irSensor[SIDE_LEFT] = !digitalRead(IR1);
                    irSensor[SIDE_RIGHT] = !digitalRead(IR7);
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
                        if(!digitalRead(DIPE) && !digitalRead(DIPA) && !digitalRead(DIPB) && !digitalRead(DIPC)){
                            //0000 OK -- modo de prueba: ambos motores adelante a parametros[2]%, sin logica de combate
                            snake=false;
                            turkish=false;
                            currentState = TEST_FORWARD;
                            break;
                        }
                        else if(!digitalRead(DIPE) && !digitalRead(DIPA) && !digitalRead(DIPB) && digitalRead(DIPC)){
                            //0001 OK
                            snake=true;
                            turkish = false;
                            currentState = FORWARD;
                            break;
                        }
                        else if(!digitalRead(DIPE) && !digitalRead(DIPA) && digitalRead(DIPB) && !digitalRead(DIPC)){
                            //0010 OK
                            snake=false;
                            turkish = true;
                            currentState = BRAKE;
                            break;
                        }
                        else if(!digitalRead(DIPE) && !digitalRead(DIPA) && digitalRead(DIPB) && digitalRead(DIPC)){
                            //0011 OK
                            snake=true;
                            turkish = true;
                            bandera = true;
                            currentState = BRAKE;
                            break;
                        }
                        else if(!digitalRead(DIPE) && digitalRead(DIPA) && !digitalRead(DIPB) && !digitalRead(DIPC)){
                            //0100 OK
                            snake=false;
                            turkish=false;
                            currentState = TURN_LEFT_90;
                            break;
                        }
                        else if(!digitalRead(DIPE) && digitalRead(DIPA) && !digitalRead(DIPB) && digitalRead(DIPC)){
                            //0101 OK
                            snake=false;
                            turkish=false;
                            currentState = TURN_RIGHT_90;
                            break;
                        }
                        else if(!digitalRead(DIPE) && digitalRead(DIPA) && digitalRead(DIPB) && !digitalRead(DIPC)){
                            //0110 OK
                            snake=false;
                            turkish=false;
                            currentState = TURN_180;
                            break;
                        }
                        else if(!digitalRead(DIPE) && digitalRead(DIPA) && digitalRead(DIPB) && digitalRead(DIPC)){
                            //0111 OK
                            snake=false;
                            turkish=true;
                            currentState = L_MOVEMENT_45;
                            break;
                        }
                        //
                        else if(digitalRead(DIPE) && !digitalRead(DIPA) && !digitalRead(DIPB) && !digitalRead(DIPC)){
                            //1000 OK
                            snake=false;
                            turkish=true;
                            currentState = R_MOVEMENT_45;
                            break;
                        }
                        else if(digitalRead(DIPE) && !digitalRead(DIPA) && !digitalRead(DIPB) && digitalRead(DIPC)){
                            //1001 OK
                            snake=false;
                            turkish=true;
                            bandera = true;
                            currentState = GIRO_U_L;
                            break;
                        }
                        else if(digitalRead(DIPE) && !digitalRead(DIPA) && digitalRead(DIPB) && !digitalRead(DIPC)){
                            //1010
                            snake=false;
                            turkish=true;
                            bandera = true;
                            currentState = GIRO_U_R;
                            break;
                        }
                        else if(digitalRead(DIPE) && !digitalRead(DIPA) && digitalRead(DIPB) && digitalRead(DIPC)){
                            //1011
                            snake=false;
                            turkish=true;
                            bandera = true;
                            currentState = GIRO_U_L_LONG;
                            break;
                        }
                        else if(digitalRead(DIPE) && digitalRead(DIPA) && !digitalRead(DIPB) && !digitalRead(DIPC)){
                            //1100
                            snake=false;
                            turkish=true;
                            bandera = true;
                            currentState = GIRO_U_R_LONG;
                            break;
                        }
                        else if(digitalRead(DIPE) && digitalRead(DIPA) && !digitalRead(DIPB) && digitalRead(DIPC)){
                            //1101
                            snake=true;
                            turkish=true;
                            currentState = SHORT_LEFT_MOVE;
                            break;
                        }
                        else if(digitalRead(DIPE) && digitalRead(DIPA) && digitalRead(DIPB) && !digitalRead(DIPC)){
                            //1110
                            snake=true;
                            turkish=true;
                            currentState = SHORT_RIGHT_MOVE;
                            break;
                        }
                        else if(digitalRead(DIPE) && digitalRead(DIPA) && digitalRead(DIPB) && digitalRead(DIPC)){
                            //1111
                            snake=false;
                            turkish=false;
                            currentState = GIRO_U_L;
                            break;
                        }
                    }
                    break;
                #endif
            case FORWARD:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                #ifdef DEBUG
                    Serial.println("State forward");
                #endif 
                    currForward = xTaskGetTickCount();
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    #ifdef MBARETECH_2
                    irSensor[SHORT_LEFT] = !digitalRead(IR2);
                    irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                    #endif
                    if ((currForward - lastForward) >= 200){//500
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
                    //
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    irSensor[SHORT_LEFT] = !digitalRead(IR2);
                    irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                    line_left = readLineSensorFront(LINE_FRONT_LEFT);
                    line_right = readLineSensorFront(LINE_FRONT_RIGHT);
                    lineSensor[0] = checkLineSensora(line_left);
                    lineSensor[1] = checkLineSensorb(line_right);
                    if ( (lineSensor[0] || lineSensor[1]) && !(irSensor[TOP_MID]) ){
                        currentState = LINE_RETREAT;
                        break;    
                    }
                    //ACA ES EL DEFAULT FORWARD(IGUAL SI NO LEE SENSORES)
                    if (!snake) { //si no hay snake=forward
                        #ifdef FORWARDON
                        rightMotor.forward(local_speed);  // 40% maso
                        leftMotor.forward(local_speed);
                        #endif
                    }
                    else {       //snake EVITAR FORWARD+SNAKE
                        #if defined(MBARETECH_2) && defined(FORWARDON)
                        leftMotor.forward(95);
                        rightMotor.forward(85);
                        #endif

                        #if defined(MBARETECH_1) && defined(FORWARDON)
                        leftMotor.forward(FORWARD_90);
                        rightMotor.forward(FORWARD_42);
                        #endif

                        while (!elapsedTime(SHORT_RIGHT_DELAY+10)) {  // 80 ms en mb2
                        }

                        #if defined(MBARETECH_2) && defined(FORWARDON)
                        rightMotor.forward(95);//90
                        leftMotor.forward(85);//80 
                        #endif

                        #if defined(MBARETECH_1) && defined(FORWARDON)
                        rightMotor.forward(FORWARD_90);
                        leftMotor.forward(FORWARD_49); 
                        #endif
                        //? ?         // probar mas
                        while (!elapsedTime(SHORT_LEFT_DELAY+10)) {  // 80 ms en mb2
                        }
                    }
                    /*
                    #ifdef MBARETECH_2
                    irSensor[SHORT_LEFT] = !digitalRead(IR2);
                    irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                    #endif

                    #ifdef MBARETECH_1
                    irSensor[SHORT_LEFT] = digitalRead(IR2);
                    irSensor[SHORT_RIGHT] = digitalRead(IR6);
                    #endif
                    */
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    irSensor[SHORT_LEFT] = !digitalRead(IR2);
                    irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                    line_left = readLineSensorFront(LINE_FRONT_LEFT);
                    line_right = readLineSensorFront(LINE_FRONT_RIGHT);
                    lineSensor[0] = checkLineSensora(line_left);
                    lineSensor[1] = checkLineSensorb(line_right);
                    if ( (lineSensor[0] || lineSensor[1]) && !(irSensor[TOP_MID]) ){
                        currentState = LINE_RETREAT;
                        break;    
                    }
                    if (!startSignal) {  // KILLSWITCH
                        currentState = IDLE;
                    }
                    else if (irSensor[SHORT_LEFT] && irSensor[SHORT_RIGHT]) { //ACA NO IMPORTA LA LINEA
                        if (!snake) {
                            #ifdef DEBUG
                            Serial.print("MAX SPEED");
                            #endif
                            #ifdef FORWARDON
                            leftMotor.forward(MAX_SPEED);
                            rightMotor.forward(MAX_SPEED);
                            #endif
                        }
                        else {
                            #ifdef DEBUG
                            Serial.print("MAX SNAKE");
                            #endif
                            #ifdef FORWARDON
                            leftMotor.forward(95);
                            rightMotor.forward(85);
                            #endif
                            while (!elapsedTime(SHORT_RIGHT_DELAY)) {  // 80 ms en mb2
                            }
                            #ifdef FORWARDON
                            rightMotor.forward(95);
                            leftMotor.forward(85);          // probar mas
                            #endif
                            while (!elapsedTime(SHORT_LEFT_DELAY)) {  // 80 ms en mb2
                            }
                        }
                    }

                    else if ((lineSensor[0] || lineSensor[1]) && !(irSensor[TOP_MID]) ){ //AGREGADO EN EL BUS
                        currentState = LINE_RETREAT;
                        break;    
                    }
                    
                    else if (irSensor[SHORT_LEFT] && !irSensor[SHORT_RIGHT]) {
                        #ifdef DEBUG
                        Serial.print("SHORT LEFT");
                        #endif
                        leftMotor.forward(FORWARD_60+parametros[18]);
                        rightMotor.forward(MAX_SPEED);
                        while (!elapsedTime(SHORT_LEFT_DELAY)) { // 80 ms en mb2
                            line_left = readLineSensorFront(LINE_FRONT_LEFT); //AGREGADO EN BUS
                            line_right = readLineSensorFront(LINE_FRONT_RIGHT);
                            lineSensor[0] = checkLineSensora(line_left);
                            lineSensor[1] = checkLineSensorb(line_right);
                            if ( (lineSensor[0] || lineSensor[1]) ){
                                currentState = LINE_RETREAT;
                                break;    
                            }  
                        }
                    }
                    else if (!irSensor[SHORT_LEFT] && irSensor[SHORT_RIGHT]) {
                        #ifdef DEBUG
                        Serial.print("SHORT_RIGHT");
                        #endif
                        leftMotor.forward(MAX_SPEED);
                        rightMotor.forward(52);
                        while (!elapsedTime(SHORT_RIGHT_DELAY)) {  // 80 ms en mb2
                            line_left = readLineSensorFront(LINE_FRONT_LEFT); //AGREGADO EN BUS
                            line_right = readLineSensorFront(LINE_FRONT_RIGHT);
                            lineSensor[0] = checkLineSensora(line_left);
                            lineSensor[1] = checkLineSensorb(line_right);
                            if ( (lineSensor[0] || lineSensor[1]) ){
                                currentState = LINE_RETREAT;
                                break;    
                            }
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
                #endif
            case TEST_FORWARD:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                #ifdef DEBUG
                Serial.println("TEST FORWARD");
                #endif
                if (!startSignal) {  // KILLSWITCH
                    currentState = IDLE;
                    break;
                }
                #ifdef FORWARDON
                rightMotor.forward(parametros[2]);
                leftMotor.forward(parametros[2]);
                #endif
                break;
                #endif
            case BACKWARD:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                #ifdef DEBUG
                Serial.println("State backward");
                #endif
                rightMotor.backward(80); //Tenia 100 en movements
                leftMotor.backward(80+parametros[18]);
                // Le puso delay de 300 ms por algun motivo
                if (!startSignal) {
                    currentState = IDLE;
                }
                currentState = BRAKE;
                break;
                #endif
            case BRAKE:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
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
                    break;             
                }
                else if (irSensor[SHORT_LEFT]){
                    currentState = SHORT_LEFT_MOVE;
                    break;
                }
                else if (irSensor[SHORT_RIGHT]){
                    currentState = SHORT_RIGHT_MOVE;
                    break;
                }
                else if (irSensor[TOP_LEFT]) {  // irSensor[TOP_LEFT]
                    currentState = TURN_LEFT_45;
                    break;
                }
                else if (irSensor[TOP_RIGHT]) {  // irSensor[TOP_RIGHT]
                    currentState = TURN_RIGHT_45;
                    break;
                }

                #ifdef MBARETECH_2
                else if (irSensor[SIDE_LEFT]) {  // irSensor[SIDE_LEFT]
                    currentState = TURN_LEFT_90;
                    break;
                }

                else if (irSensor[SIDE_RIGHT]) {  // irSensor[SIDE_RIGHT]
                    currentState = TURN_RIGHT_90;
                    break;
                }
                #endif
                else{
                    if (turkish){
                        currTurkish = xTaskGetTickCount();
                        if (currTurkish - lastTurkish >= TURKISH_TIME) {
                            #ifdef DEBUG
                            Serial.print("MOVING A LITTLE");
                            #endif
                            rightMotor.forward(FORWARD_60);
                            leftMotor.forward(FORWARD_60+parametros[18]);
                            while(!elapsedTime(TURKISH_DELAY)){}
                            rightMotor.brake();
                            leftMotor.brake();
                            lastTurkish = xTaskGetTickCount();
                            //
                            line_left = readLineSensorFront(LINE_FRONT_LEFT); //AGREGADO EN BUS
                            line_right = readLineSensorFront(LINE_FRONT_RIGHT);
                            lineSensor[0] = checkLineSensora(line_left);
                            lineSensor[1] = checkLineSensorb(line_right);
                            if ( (lineSensor[0] || lineSensor[1]) ){
                                currentState = LINE_RETREAT;
                                break;    
                            }
                            //
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
                #endif
            case TURN_LEFT_45:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                #ifdef DEBUG
                Serial.println("TURN L 45");
                #endif
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3]+parametros[18]);
                while (!elapsedTime(parametros[4])){
                    if(!startSignal){break;}
                    #ifdef CANCEL_TURNS
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    if (irSensor[TOP_MID]){
                        currentState = FORWARD;
                        break_turn = true;
                        break;
                    }
                    #endif
                }
                rightMotor.brake();
                leftMotor.brake();
                currentState = BRAKE;
                break;
                #endif
            case TURN_LEFT_90:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                #ifdef DEBUG
                Serial.println("TURN L 90");
                #endif
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3]+parametros[18]);
                while (!elapsedTime(parametros[5])){
                    if(!startSignal){break;}
                    #ifdef CANCEL_TURNS
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    if (irSensor[TOP_MID]){
                        currentState = FORWARD;
                        break_turn = true;
                        break;
                    }
                    #endif
                }
                rightMotor.brake();
                leftMotor.brake();
                currentState = BRAKE;
                break;
                #endif
            case TURN_RIGHT_45:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                #ifdef DEBUG
                Serial.println("TURN R 45");
                #endif
                leftMotor.forward(parametros[6]+parametros[18]);
                rightMotor.backward(parametros[6]);
                while (!elapsedTime(parametros[7])){
                    if(!startSignal){break;}
                    #ifdef CANCEL_TURNS
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    if (irSensor[TOP_MID]){
                        currentState = FORWARD;
                        break_turn = true;
                        break;
                    }
                    #endif
                }
                rightMotor.brake();
                leftMotor.brake();
                parametros[0]=0;
                currentState = BRAKE;
                break;
                #endif
            case TURN_RIGHT_90:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                #ifdef DEBUG
                Serial.println("TURN R 90");
                #endif
                leftMotor.forward(parametros[6]+parametros[18]);
                rightMotor.backward(parametros[6]);
                while (!elapsedTime(parametros[8])){
                    if(!startSignal){break;}
                    #ifdef CANCEL_TURNS
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    if (irSensor[TOP_MID]){
                        currentState = FORWARD;
                        break_turn = true;
                        break;
                    }
                    #endif
                }
                rightMotor.brake();
                leftMotor.brake();
                currentState = BRAKE;
                break;
                #endif  
            case TURN_180:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3]+parametros[18]);
                while (!elapsedTime(parametros[9])){
                    if(!startSignal){break;}
                    #ifdef CANCEL_TURNS
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    irSensor[SIDE_LEFT] = !digitalRead(IR1);
                    if (irSensor[TOP_MID]){
                        currentState = FORWARD;
                        break_turn = true;
                        break;
                    }
                    else if (irSensor[SIDE_LEFT]){
                        currentState = TURN_LEFT_90;
                        break_turn = true;
                        break;
                    }
                    #endif
                }
                leftMotor.brake();
                rightMotor.brake();
                currentState = BRAKE;
                break;
                #endif
            case GIRO_U_L:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                rightMotor.forward(FORWARD_90);
                leftMotor.forward(FORWARD_60+parametros[18]);//80*2/3
                while (!elapsedTime(1000)){
                    if(!startSignal){break;}
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
                }
                leftMotor.brake();
                rightMotor.brake();
                currentState = BRAKE;
                break;
                #endif
            case GIRO_U_R:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                leftMotor.forward(FORWARD_90+parametros[18]);
                rightMotor.forward(FORWARD_60);//80*2/3
                while (!elapsedTime(1000)){
                    if(!startSignal){break;}
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
                }
                leftMotor.brake();
                rightMotor.brake();
                currentState = BRAKE;
                break;
                #endif
            case LINE_RETREAT:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                rightMotor.backward(FORWARD_90);
                leftMotor.backward(FORWARD_90);
                while(!elapsedTime(80)){
                    if(!startSignal){break;}
                    #ifdef DEBUG
                    Serial.print("RETREAT");
                    #endif
                }
                irSensor[SHORT_LEFT] = !digitalRead(IR2);
                irSensor[TOP_MID] = !digitalRead(IR4);
                irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                irSensor[SIDE_LEFT] = !digitalRead(IR1);
                irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                irSensor[TOP_LEFT] = !digitalRead(IR3);
                irSensor[TOP_RIGHT] = !digitalRead(IR5);

                if (irSensor[TOP_MID] || irSensor[SHORT_LEFT] || irSensor[SHORT_RIGHT] || irSensor[TOP_LEFT] || irSensor[TOP_RIGHT]){
                    currentState = BRAKE;
                    break;
                }
                else{
                    currentState = TURN_180;
                    break;
                }
                break;
                #endif
            case SHORT_LEFT_MOVE:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                rightMotor.forward(FORWARD_90);
                leftMotor.forward(FORWARD_42+parametros[18]);
                while (!elapsedTime(80)){
                    if(!startSignal){break;}
                }
                leftMotor.brake();
                rightMotor.brake();
                currentState = BRAKE;
                break;
                #endif
            case SHORT_RIGHT_MOVE:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                rightMotor.forward(FORWARD_42);
                leftMotor.forward(FORWARD_90+parametros[18]);
                while (!elapsedTime(80)) { //80 ms en mb2
                    if(!startSignal){break;}
                }
                leftMotor.brake();
                rightMotor.brake();
                currentState = BRAKE;
                break;
                #endif
            case L_MOVEMENT_45:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                #ifdef DEBUG
                    Serial.println("L_MOVEMENT_45");
                #endif
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3]+parametros[18]);
                while (!elapsedTime(parametros[4])){
                    if(!startSignal){break;}
                }   
                rightMotor.forward(FORWARD_90);
                leftMotor.forward(FORWARD_90+parametros[18]);
                while (!elapsedTime(150)){
                    if(!startSignal){break;}
                    #ifdef CANCEL_TURNS
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    //irSensor[SHORT_LEFT] = !digitalRead(IR2);
                    irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                    //irSensor[TOP_LEFT] = !digitalRead(IR3);
                    irSensor[TOP_RIGHT] = !digitalRead(IR5);
                    //irSensor[SIDE_LEFT] = !digitalRead(IR1);
                    irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                    if (irSensor[TOP_MID]){
                        currentState = FORWARD;
                        break_turn = true;
                        break;
                    }
                    else if (irSensor[SHORT_RIGHT]){
                        currentState = SHORT_RIGHT_MOVE;
                        break_turn = true;
                        break;
                    }
                    else if (irSensor[TOP_RIGHT]){
                        currentState = TURN_RIGHT_45;
                        break_turn = true;
                        break;
                    }
                    else if (irSensor[SIDE_RIGHT]){
                        currentState = TURN_RIGHT_90;
                        break_turn = true;
                        break;
                    }
                    #endif
                }
                if(break_turn){
                    currentState=BRAKE;
                    break;}
                rightMotor.backward(parametros[6]);
                leftMotor.forward(parametros[6]+parametros[18]);
                while (!elapsedTime(parametros[8])){
                    if(!startSignal){break;}
                    #ifdef CANCEL_TURNS
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    irSensor[SHORT_LEFT] = !digitalRead(IR2);
                    irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                    irSensor[TOP_LEFT] = !digitalRead(IR3);
                    irSensor[TOP_RIGHT] = !digitalRead(IR5);
                    irSensor[SIDE_LEFT] = !digitalRead(IR1);
                    irSensor[SIDE_RIGHT] = !digitalRead(IR7);
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
                    else if (irSensor[SIDE_LEFT]){
                        currentState = TURN_LEFT_90;
                        break_turn = true;
                        break;
                    }
                    else if (irSensor[SIDE_RIGHT]){
                        currentState = TURN_RIGHT_90;
                        break_turn = true;
                        break;
                    }
                    #endif
                }
                if(break_turn){
                    currentState=BRAKE;
                    break;}
                rightMotor.brake();
                leftMotor.brake();
                currentState = BRAKE;
            break;
                #endif
            case R_MOVEMENT_45:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                #ifdef DEBUG
                    Serial.println("R_MOVEMENT_45");
                #endif
                rightMotor.backward(parametros[6]);
                leftMotor.forward(parametros[6]+parametros[18]);
                while (!elapsedTime(parametros[7])){
                    if(!startSignal){break;}
                }   
                rightMotor.forward(FORWARD_90);
                leftMotor.forward(FORWARD_90);
                while (!elapsedTime(150)){
                    if(!startSignal){break;}
                    #ifdef CANCEL_TURNS
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    irSensor[SHORT_LEFT] = !digitalRead(IR2);
                    //irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                    irSensor[TOP_LEFT] = !digitalRead(IR3);
                    //irSensor[TOP_RIGHT] = !digitalRead(IR5);
                    irSensor[SIDE_LEFT] = !digitalRead(IR1);
                    //irSensor[SIDE_RIGHT] = !digitalRead(IR7);
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
                    else if (irSensor[TOP_LEFT]){
                        currentState = TURN_LEFT_45;
                        break_turn = true;
                        break;
                    }
                    else if (irSensor[SIDE_LEFT]){
                        currentState = TURN_LEFT_90;
                        break_turn = true;
                        break;
                    }
                    #endif
                }
                if(break_turn){
                    currentState=BRAKE;
                    break;}
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3]+parametros[18]);
                while (!elapsedTime(parametros[5])){
                    if(!startSignal){break;}
                    #ifdef CANCEL_TURNS
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    irSensor[SHORT_LEFT] = !digitalRead(IR2);
                    irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                    irSensor[TOP_LEFT] = !digitalRead(IR3);
                    irSensor[TOP_RIGHT] = !digitalRead(IR5);
                    irSensor[SIDE_LEFT] = !digitalRead(IR1);
                    irSensor[SIDE_RIGHT] = !digitalRead(IR7);
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
                    else if (irSensor[SIDE_LEFT]){
                        currentState = TURN_LEFT_90;
                        break_turn = true;
                        break;
                    }
                    else if (irSensor[SIDE_RIGHT]){
                        currentState = TURN_RIGHT_90;
                        break_turn = true;
                        break;
                    }
                    #endif
                }
                if(break_turn){
                    currentState=BRAKE;
                    break;}
                rightMotor.brake();
                leftMotor.brake();
                currentState = BRAKE;
                break;
                #endif
            case GIRO_U_L_LONG:
                break_turn=false;
                #ifdef ESTADOS_ORDEN
                #ifdef DEBUG
                Serial.println("GIRO U IZQUIERDA LARGO");
                #endif
                rightMotor.forward(FORWARD_90);
                leftMotor.forward(FORWARD_60+parametros[18]);//80*2/3
                while (!elapsedTime(2000)) {
                    if(!startSignal) break;
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
                }
                leftMotor.brake();
                rightMotor.brake();
                currentState = BRAKE;
                break;
                #endif
            case GIRO_U_R_LONG:
                break_turn=false;
            #ifdef ESTADOS_ORDEN
                #ifdef DEBUG
                Serial.println("GIRO U DERECHA LARGO");
                #endif
                leftMotor.forward(FORWARD_90+parametros[18]);
                rightMotor.forward(FORWARD_60);//80*2/3
                while (!elapsedTime(2000)){
                    if(!startSignal){break;}
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
                }
                leftMotor.brake();
                rightMotor.brake();

                currentState = BRAKE;
            break;
                #endif
            //UNUSED
            case FORWARD_RIGHT:
                #ifdef ESTADOS_ORDEN
                rightMotor.forward(FORWARD_42);
                leftMotor.forward(FORWARD_90);
                while (!elapsedTime(80)) { //80 ms en mb2
                }
                leftMotor.brake();
                rightMotor.brake();
                currentState = BRAKE;
                break;
                #endif
            case FORWARD_LEFT:
            #ifdef ESTADOS_ORDEN
                rightMotor.forward(FORWARD_90);
                leftMotor.forward(FORWARD_49);
                while (!elapsedTime(80)) { //80 ms en mb2
                }
                leftMotor.brake();
                rightMotor.brake();
                currentState = BRAKE;
                break;
                #endif
            case TURN_LEFT_45_IF:
                #ifdef ESTADOS_ORDEN
                currMove = xTaskGetTickCount();
                if (currMove - lastLeft45 >= LAST_LEFT_45_TIMER) {  // 2 * delay
                    rightMotor.forward(parametros[3]);
                    leftMotor.backward(parametros[3]+parametros[18]);
                    while (!elapsedTime(parametros[4])) {
                        #ifdef DEBUG
                        Serial.println("Turning left 45");
                        #endif
                        #ifdef CANCEL_TURNS
                        irSensor[TOP_MID] = !digitalRead(IR4);
                        irSensor[SHORT_LEFT] = !digitalRead(IR2);
                        irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                        //irSensor[TOP_LEFT] = !digitalRead(IR3);
                        irSensor[TOP_RIGHT] = !digitalRead(IR5);
                        irSensor[SIDE_LEFT] = !digitalRead(IR1);
                        irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                        if (irSensor[TOP_MID]){
                            currentState = FORWARD;
                            break_turn = true;
                            break;
                        }
                        /*else if (irSensor[SHORT_LEFT]){
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
                        else if (irSensor[SIDE_LEFT]){
                            currentState = TURN_LEFT_90;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SIDE_RIGHT]){
                            currentState = TURN_RIGHT_90;
                            break_turn = true;
                            break;
                        }*/
                        #endif
                    }
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
                #endif
            case TURN_RIGHT_45_IF:
            #ifdef ESTADOS_ORDEN
                currMove = xTaskGetTickCount();
                if (currMove - lastRight45 >= LAST_RIGHT_45_TIMER) {  // 2 * delay
                    rightMotor.backward(parametros[6]);
                    leftMotor.forward(parametros[6]+parametros[18]);
                    while (!elapsedTime(parametros[7])) {
                        #ifdef DEBUG
                        Serial.println("Turning right 45");
                        #endif
                        #ifdef CANCEL_TURNS
                        irSensor[TOP_MID] = !digitalRead(IR4);
                        irSensor[SHORT_LEFT] = !digitalRead(IR2);
                        irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                        irSensor[TOP_LEFT] = !digitalRead(IR3);
                        //irSensor[TOP_RIGHT] = !digitalRead(IR5);
                        irSensor[SIDE_LEFT] = !digitalRead(IR1);
                        irSensor[SIDE_RIGHT] = !digitalRead(IR7);

                        if (irSensor[TOP_MID]){
                            currentState = FORWARD;
                            break_turn = true;
                            break;
                        }
                        /*else if (irSensor[SHORT_LEFT]){
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
                        else if (irSensor[SIDE_LEFT]){
                            currentState = TURN_LEFT_90;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SIDE_RIGHT]){
                            currentState = TURN_RIGHT_90;
                            break_turn = true;
                            break;
                        }*/
                        #endif
                        
                    }
                    lastRight45 = xTaskGetTickCount();
                    leftMotor.brake();
                    rightMotor.brake();
                }

                if (!startSignal) {
                    currentState = IDLE;
                }
                else {
                    /*if (break_turn){
                    currentState = BRAKE;
                    }
                    break_turn = false;*/
                    currentState = BRAKE;
                }
                break;
                #endif
            case TURN_LEFT_90_IF: //corregir(copiar de 90)
                #ifdef ESTADOS_ORDEN
                currMove = xTaskGetTickCount();
                if (currMove - lastLeft90 >= LAST_LEFT_90_TIMER) {  // 2 * delay
                    rightMotor.forward(parametros[3]);
                    leftMotor.backward(parametros[3]+parametros[18]);
                    lastLeft90 = xTaskGetTickCount();
                    leftMotor.brake();
                    rightMotor.brake();
                }
                    #ifdef DEBUG
                    Serial.println("Turning left 90");
                    #endif
                    #ifdef CANCEL_TURNS
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    //irSensor[SHORT_LEFT] = !digitalRead(IR2);
                    //irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                    //irSensor[TOP_LEFT] = !digitalRead(IR3);
                    //irSensor[TOP_RIGHT] = !digitalRead(IR5);
                    //irSensor[SIDE_LEFT] = !digitalRead(IR1);
                    //irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                    if (irSensor[TOP_MID]){
                        currentState = FORWARD;
                        break_turn = true;
                        break;
                    }/*
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
                    else if (irSensor[SIDE_LEFT]){
                        currentState = TURN_LEFT_90;
                        break_turn = true;
                        break;
                    }
                    else if (irSensor[SIDE_RIGHT]){
                        currentState = TURN_RIGHT_90;
                        break_turn = true;
                        break;
                    }*/
                    #endif
                        

                if (!startSignal) {
                    currentState = IDLE;
                }
                else {
                    currentState = BRAKE;
                }
                break;
                
               /*
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3]+parametros[18]);
                while (!elapsedTime(parametros[5])) {
                    #ifdef DEBUG
                    Serial.println("Turning left 90");
                    #endif
                    #ifdef CANCEL_TURNS
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    irSensor[SHORT_LEFT] = !digitalRead(IR2);
                    irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                    irSensor[TOP_LEFT] = !digitalRead(IR3);
                    irSensor[TOP_RIGHT] = !digitalRead(IR5);
                    //irSensor[SIDE_LEFT] = !digitalRead(IR1);
                    irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                    if (irSensor[TOP_MID]){
                        currentState = FORWARD;
                        break_turn = true;
                        break;
                    }/*
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
                    else if (irSensor[SIDE_LEFT]){
                        currentState = TURN_LEFT_90;
                        break_turn = true;
                        break;
                    }
                    else if (irSensor[SIDE_RIGHT]){
                        currentState = TURN_RIGHT_90;
                        break_turn = true;
                        break;
                    }
                    #endif
                }
                //lastLeft90 = xTaskGetTickCount();
                leftMotor.brake();
                rightMotor.brake();
                
                if (!startSignal) {
                    currentState = IDLE;
                }
                else {
                    currentState = BRAKE;
                }
            break;*/
                #endif
            case TURN_RIGHT_90_IF:
                #ifdef ESTADOS_ORDEN
                currMove = xTaskGetTickCount();
                if (currMove - lastRight90 >= LAST_RIGHT_90_TIMER) {  // 2 * delay
                    rightMotor.backward(parametros[6]);
                    leftMotor.forward(parametros[6]+parametros[18]);
                    while (!elapsedTime(parametros[8])) {
                        #ifdef DEBUG
                        Serial.println("Turning right 90");
                        #endif
                        #ifdef CANCEL_TURNS
                        irSensor[TOP_MID] = !digitalRead(IR4);
                        //irSensor[SHORT_LEFT] = !digitalRead(IR2);
                        //irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                        //irSensor[TOP_LEFT] = !digitalRead(IR3);
                        //irSensor[TOP_RIGHT] = !digitalRead(IR5);
                        //irSensor[SIDE_LEFT] = !digitalRead(IR1);
                        //irSensor[SIDE_RIGHT] = !digitalRead(IR7);
                        if (irSensor[TOP_MID]){
                            currentState = FORWARD;
                            break_turn = true;
                            break;
                        }/*
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
                        else if (irSensor[SIDE_LEFT]){
                            currentState = TURN_LEFT_90;
                            break_turn = true;
                            break;
                        }
                        else if (irSensor[SIDE_RIGHT]){
                            currentState = TURN_RIGHT_90;
                            break_turn = true;
                            break;
                        }*/
                        #endif
                    }
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
                
               /*
                rightMotor.backward(parametros[6]);
                leftMotor.forward(parametros[6]+parametros[18]);
                while (!elapsedTime(parametros[8])) {
                    #ifdef DEBUG
                    Serial.println("Turning right 90");
                    #endif
                    #ifdef CANCEL_TURNS
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    irSensor[SHORT_LEFT] = !digitalRead(IR2);
                    irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                    irSensor[TOP_LEFT] = !digitalRead(IR3);
                    irSensor[TOP_RIGHT] = !digitalRead(IR5);
                    irSensor[SIDE_LEFT] = !digitalRead(IR1);
                    //irSensor[SIDE_RIGHT] = !digitalRead(IR7);
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
                    else if (irSensor[SIDE_LEFT]){
                        currentState = TURN_LEFT_90;
                        break_turn = true;
                        break;
                    }
                    else if (irSensor[SIDE_RIGHT]){
                        currentState = TURN_RIGHT_90;
                        break_turn = true;
                        break;
                    }
                    #endif
                    }
                    lastRight90 = xTaskGetTickCount();
                    leftMotor.brake();
                    rightMotor.brake();
                

                if (!startSignal) {
                    currentState = IDLE;
                }
                else {
                    currentState = BRAKE;
                }
                break;*/
                #endif
            case MOVEMENT_45: //ahora mismo es un giro 90 nomas
                #ifdef ESTADOS_ORDEN      
                /*rightMotor.backward(parametros[6]);
                leftMotor.forward(parametros[6]);
                //while(!elapsedTime(1)){Serial.println("Mini delay");}
                while (!elapsedTime(parametros[7])) {
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
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3]);
                while (!elapsedTime(parametros[5])) {
                    #ifdef DEBUG
                    Serial.println("Turning left movement 45");
                    #endif
                };
                rightMotor.brake();
                leftMotor.brake();
                */
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3]);
                while (!elapsedTime(parametros[5])) {
                    #ifdef DEBUG
                    Serial.println("Turning left 90");
                    #endif
                    #ifdef CANCEL_TURNS
                    irSensor[TOP_MID] = !digitalRead(IR4);
                    irSensor[SHORT_LEFT] = !digitalRead(IR2);
                    irSensor[SHORT_RIGHT] = !digitalRead(IR6);
                    irSensor[TOP_LEFT] = !digitalRead(IR3);
                    irSensor[TOP_RIGHT] = !digitalRead(IR5);
                    //irSensor[SIDE_LEFT] = !digitalRead(IR1);
                    irSensor[SIDE_RIGHT] = !digitalRead(IR7);
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
                    }/*
                    else if (irSensor[SIDE_LEFT]){
                        currentState = TURN_LEFT_90;
                        break_turn = true;
                        break;
                    }*/
                    else if (irSensor[SIDE_RIGHT]){
                        currentState = TURN_RIGHT_90;
                        break_turn = true;
                        break;
                    }
                    #endif
                }
                leftMotor.brake();
                rightMotor.brake();

                currentState = BRAKE;
                break;
                #endif
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


void loop(){}

#endif