#ifdef RUN_CALIBRACION
#include "globals.h"
#include "bluetoothComm.h"

// Modo de calibracion, separado del codigo de competencia (tasks.cpp).
// Lee los 7 IR, los 5 DIP y los sensores de linea siempre, y reporta
// por BLE/Serial mientras startSignal este activo (mismo comportamiento
// de kill switch que combate: primera señal prende, la siguiente apaga).
// El reporte SOLO se manda cuando algo cambio respecto al ultimo mensaje
// enviado -- evita saturar el canal BLE con lecturas repetidas idénticas.
// Formato: "MODO=0001 SIDE_LEFT=0/1 SHORT_LEFT=0/1 TOP_LEFT=0/1 TOP_MID=0/1
//           TOP_RIGHT=0/1 SHORT_RIGHT=0/1 SIDE_RIGHT=0/1 LINEA_DEL_IZQ=BLANCO/negro
//           LINEA_DEL_DER=BLANCO/negro"
// (los IR quedan en 0/1 -- "detectado/no" no tiene un equivalente natural a
// blanco/negro; los de linea usan texto porque es mas legible a simple vista,
// mismo criterio que ya usa src/tests/pruebaLinea.cpp)
// Los sensores de linea traseros (LINEA_TRAS_IZQ/DER) se suman al mensaje
// solo cuando LINE_BACK_INSTALLED este en 1 (globals.h) -- hoy en 0 porque
// LINE_BACK_LEFT/RIGHT todavia apuntan a pines en conflicto (GPIO19=DIPE,
// GPIO20=PIN_B1). Pendiente: actualizar esos pines y el sentinel una vez
// puenteados los traseros a pines libres.
// MODO=XXXX (agregado 2026-09-30) es el combo REALMENTE en ejecucion
// (comboCongelado, en binario de 4 bits DIPE-DIPA-DIPB-DIPC, ver modelo de
// 3 etapas mas abajo) -- se agrego para poder confirmar desde el celular por
// BLE, sin Serial, que combo quedo armado apenas arranca a correr. El DIP
// crudo en si sigue sin mandarse por BLE, solo se usa localmente y se
// imprime por Serial para depurar.
//
// Ademas, segun la combinacion de DIP (mismo orden DIPE-DIPA-DIPB-DIPC
// que usa tasks.cpp para elegir estrategia), ejecuta un movimiento
// puntual repetido, para poder calibrar velocidad/delay de cada giro
// en vivo por BLE sin tocar el firmware de competencia:
//
// MODELO DE 3 ETAPAS DEL KILLSWITCH (2026-09-30): el DIP ya NO se lee en
// cada vuelta del loop mientras el robot esta corriendo -- se lee UNA sola
// vez y se congela. Etapa 1 (startSignal=false, "preparar"): el DIP se lee
// libre cada vuelta, el usuario elige el combo. Etapa 2 (borde de subida de
// startSignal, "ejecutar"): el combo se congela en ese instante y ya no se
// vuelve a tocar el DIP mientras siga corriendo -- ni un switch flojo ni
// ruido electrico del motor acoplado a las lineas del DIP pueden cambiar el
// combo a mitad de una corrida. Etapa 3 (startSignal vuelve a false,
// "detener"): frena y vuelve a etapa 1, listo para congelar un combo nuevo
// la proxima vez. Antes de esto, `combo` se recalculaba del DIP en cada
// vuelta (~300ms) incluso con el motor corriendo, lo que dejaba abierta la
// puerta a que una sola lectura mala metiera un frenon de por medio.
//
//   0000 -> solo reporte de sensores, sin mover motores (modo por defecto)
//   0001 -> avance recto  (parametros[2] vel)
//   0010 -> giro 45 -- bilateral: TOP_LEFT gira izq (parametros[3]/[4]),
//           TOP_RIGHT gira der (parametros[6]/[7]), nada detectado frena
//   0011 -> giro 90 -- bilateral: SIDE_LEFT gira izq (parametros[3]/[5]),
//           SIDE_RIGHT gira der (parametros[6]/[8]), nada detectado frena
//   0100 -> junta 0010+0011: los 4 sensores de giro en un combo (TOP->45,
//           SIDE->90), nada detectado frena. Mismos parametros que arriba.
//   0101 -> libre (liberado al unificar 45/90 en bilateral)
//   0110 -> giro 180      (parametros[3] vel, parametros[9] delay)
//   0111 -> seguir sin atacar 1: gira para encarar (TOP/SIDE), SHORT se
//           omite por completo (frena, ni pivote ni empuje)
//   1000 -> seguir sin atacar 2: igual, pero SHORT dispara un pivote
//           asimetrico 90/42 (parametros[10]/[11] delay, sin correccion)
//   1001 -> seguir sin atacar 3: igual, pero SHORT reproduce el empuje+
//           curva real de combate (90/42+correccion, 80ms), con cooldown
//           de 10s por lado -- es el unico combo que empuja de verdad
//   1010-1111 -> libres, sin asignar todavia
//
// IMPORTANTE: parametros[] vive solo en RAM -- no sobrevive un reflash.
// Los valores buenos que se encuentren acá hay que (a) volver a
// mandarlos por BLE una vez en el build de combate, o (b) actualizar
// los #define correspondientes en globals.h para que queden de default.

void IRAM_ATTR CalibKS_ISR() { startSignal = digitalRead(START_PIN); }

// Mismo texto que usa src/tests/pruebaLinea.cpp para los sensores de linea
// (BLANCO/negro) en vez de 0/1 -- solo para linea, los IR se quedan en 0/1.
static const char* lineaTexto(bool detectada) { return detectada ? "BLANCO" : "negro"; }

// Combo (0-9) como texto binario de 4 bits (mismo orden DIPE-DIPA-DIPB-DIPC
// que usa el resto del archivo), para que el reporte por BLE muestre "0001"
// en vez de "1" -- comparable directo con la tabla de combos de los
// comentarios y con lo que el usuario arma fisicamente en el DIP.
static String comboTexto(int combo) {
    String s = "";
    s += (combo & 8) ? "1" : "0";
    s += (combo & 4) ? "1" : "0";
    s += (combo & 2) ? "1" : "0";
    s += (combo & 1) ? "1" : "0";
    return s;
}

void setup() {
    #ifdef DEBUG
    Serial.begin(115200);
    #endif

    // SKIP_BLE: apaga BLE por completo (init + advertising) para aislar si
    // el radio (siempre activo advertisement, no solo notify()) es la causa
    // del giro intermitente en el combo 0001. Prueba temporal, no tocar el
    // resto de la logica -- agregar -DSKIP_BLE en el bloque CALIBRACION de
    // platformio.ini para activarla.
    #ifndef SKIP_BLE
    BLE_UART_Init("MBARETECH");
    #endif

    #ifdef RUN_LINE_SENSOR
    lineSensorsInit();
    #endif

    #ifdef MBARETECH_2
    pinMode(IR1, INPUT);
    pinMode(IR7, INPUT);
    #endif
    pinMode(IR2, INPUT);
    pinMode(IR3, INPUT);
    pinMode(IR4, INPUT);
    pinMode(IR5, INPUT);
    pinMode(IR6, INPUT);

    pinMode(DIPA, INPUT);
    pinMode(DIPB, INPUT);
    pinMode(DIPC, INPUT);
    pinMode(DIPD, INPUT);
    pinMode(DIPE, INPUT);

    pinMode(START_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(START_PIN), CalibKS_ISR, CHANGE);

    rightMotor.begin();
    leftMotor.begin();

    // Nota: no corre el eFuse write de main.cpp (esp_efuse_write_field_cnt)
    // -- este modo no lo necesita.
}

void loop() {
    #ifdef MBARETECH_2
    irSensor[SIDE_LEFT]   = !digitalRead(IR1);
    irSensor[SHORT_LEFT]  = !digitalRead(IR2);
    irSensor[TOP_LEFT]    = !digitalRead(IR3);
    irSensor[TOP_MID]     = !digitalRead(IR4);
    irSensor[TOP_RIGHT]   = !digitalRead(IR5);
    irSensor[SHORT_RIGHT] = !digitalRead(IR6);
    irSensor[SIDE_RIGHT]  = !digitalRead(IR7);
    #endif

    #ifdef MBARETECH_1
    irSensor[SHORT_LEFT]  = digitalRead(IR2);
    irSensor[TOP_LEFT]    = digitalRead(IR3);
    irSensor[TOP_MID]     = !digitalRead(IR4);
    irSensor[TOP_RIGHT]   = digitalRead(IR5);
    irSensor[SHORT_RIGHT] = digitalRead(IR6);
    #endif

    bool dA = digitalRead(DIPA);
    bool dB = digitalRead(DIPB);
    bool dC = digitalRead(DIPC);
    bool dD = digitalRead(DIPD);
    bool dE = digitalRead(DIPE);
    int comboLive = (dE ? 8 : 0) | (dA ? 4 : 0) | (dB ? 2 : 0) | (dC ? 1 : 0);

    int line_left  = readLineSensorFront(LINE_FRONT_LEFT);
    int line_right = readLineSensorFront(LINE_FRONT_RIGHT);
    lineSensor[0] = checkLineSensora(line_left);
    lineSensor[1] = checkLineSensorb(line_right);

    #if LINE_BACK_INSTALLED
    int line_back_left  = readLineSensorBack(LINE_BACK_LEFT);
    int line_back_right = readLineSensorBack(LINE_BACK_RIGHT);
    // readLineSensorBack devuelve -1 si adc2_get_raw() falla (ESP32 bloquea
    // ADC2 mientras el radio Wi-Fi/BT esta activo -- BLE cuenta). -1 es
    // SIEMPRE <= THRESHOLD, asi que antes de este chequeo un fallo de
    // lectura se malinterpretaba como "ve blanco" y ensuciaba el filtro de
    // 7 lecturas. Ahora, si falla, se ignora el ciclo y se mantiene el
    // ultimo estado debounced en vez de contaminar el contador.
    if (line_back_left  != -1) lineSensor[2] = checkLineSensorc(line_back_left);
    if (line_back_right != -1) lineSensor[3] = checkLineSensord(line_back_right);

    #ifdef DEBUG
    static unsigned long ultimoPrintLinea = 0;
    if (millis() - ultimoPrintLinea >= 1000) {
        ultimoPrintLinea = millis();
        Serial.print("LINEA cruda (THRESHOLD="); Serial.print(THRESHOLD);
        Serial.print("): DEL_IZQ="); Serial.print(line_left);
        Serial.print(" DEL_DER="); Serial.print(line_right);
        Serial.print(" TRAS_IZQ="); Serial.print(line_back_left);
        Serial.print(line_back_left == -1 ? "(ERROR lectura)" : "");
        Serial.print(" TRAS_DER="); Serial.print(line_back_right);
        Serial.println(line_back_right == -1 ? "(ERROR lectura)" : "");
    }
    #endif
    #endif

    // --- Modelo de 3 etapas del killswitch ---
    // Etapa 1 (startSignal=false, "preparar"): el DIP se lee libremente cada
    // vuelta (arriba, comboLive) para que el usuario vea en vivo que combo
    // va a quedar armado, pero no se ejecuta ningun movimiento.
    // Etapa 2 (startSignal=true, "ejecutar"): en el instante exacto en que
    // se activa (borde de subida, detectado con "corriendo"), el combo se
    // CONGELA en comboCongelado a partir del comboLive de ESE instante, y
    // desde ahi no se vuelve a tocar el DIP para nada -- ni ruido del motor
    // ni un switch flojo pueden cambiar el combo mientras esta corriendo.
    // Etapa 3 (siguiente vez que startSignal vuelve a false, "detener"):
    // frena y vuelve a etapa 1 (corriendo=false), listo para congelar un
    // combo nuevo la proxima vez que se active.
    static bool corriendo = false;
    static int comboCongelado = 0;

    #ifdef DEBUG
    // Imprime SIEMPRE, una vez por vuelta, con timestamp: combo_vivo es lo
    // que el DIP dice ahora mismo, combo_ejecutando es lo que realmente se
    // esta corriendo (congelado, -1 si todavia estamos en etapa 1). Si
    // combo_vivo cambia en medio de una corrida pero combo_ejecutando se
    // mantiene fijo, el DIP puede tener ruido/flojera pero ya no afecta el
    // movimiento -- justamente lo que este cambio busca garantizar.
    Serial.print(millis());
    Serial.print(" DIP crudo E,A,B,C,D = ");
    Serial.print(dE); Serial.print(dA); Serial.print(dB); Serial.print(dC); Serial.print(dD);
    Serial.print("  -> combo_vivo="); Serial.print(comboLive);
    Serial.print(" combo_ejecutando="); Serial.println(corriendo ? comboCongelado : -1);
    #endif

    if (!startSignal) {
        rightMotor.brake();
        leftMotor.brake();
        corriendo = false; // etapa 3 -> vuelve a etapa 1, se podra congelar de nuevo
        delay(50);
        return;
    }

    if (!corriendo) {
        // Borde etapa 1 -> etapa 2: se congela el combo una sola vez.
        comboCongelado = comboLive;
        corriendo = true;
    }

    int combo = comboCongelado;

    String msg = "MODO=" + comboTexto(combo)
               + " SIDE_LEFT=" + String(irSensor[SIDE_LEFT])
               + " SHORT_LEFT=" + String(irSensor[SHORT_LEFT])
               + " TOP_LEFT=" + String(irSensor[TOP_LEFT])
               + " TOP_MID=" + String(irSensor[TOP_MID])
               + " TOP_RIGHT=" + String(irSensor[TOP_RIGHT])
               + " SHORT_RIGHT=" + String(irSensor[SHORT_RIGHT])
               + " SIDE_RIGHT=" + String(irSensor[SIDE_RIGHT])
               + " LINEA_DEL_IZQ=" + String(lineaTexto(lineSensor[0]))
               + " LINEA_DEL_DER=" + String(lineaTexto(lineSensor[1]))
               #if LINE_BACK_INSTALLED
               + " LINEA_TRAS_IZQ=" + String(lineaTexto(lineSensor[2]))
               + " LINEA_TRAS_DER=" + String(lineaTexto(lineSensor[3]))
               #endif
               ;

    static String ultimoMsg = "";
    if (msg != ultimoMsg) {
        #ifdef DEBUG
        Serial.println(msg);
        #endif
        #ifndef SKIP_BLE
        sendData(msg);
        #endif
        ultimoMsg = msg;
    }

    // --- Movimiento puntual segun combo de DIP ---
    switch (combo) {
        case 0: // 0000: solo reporte, no mueve nada
            break;

        case 1: // 0001: avance recto
            rightMotor.forward(parametros[2]);
            leftMotor.forward(parametros[2]);
            break;

        case 2: // 0010: giro 45 -- bilateral, el sensor decide el lado.
                // TOP_LEFT -> izquierda, TOP_RIGHT -> derecha, nada -> frena.
            if (irSensor[TOP_LEFT]) {
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3] + parametros[18]);
                while (!elapsedTime(parametros[4])) { if (!startSignal) break; }
            }
            else if (irSensor[TOP_RIGHT]) {
                leftMotor.forward(parametros[6] + parametros[18]);
                rightMotor.backward(parametros[6]);
                while (!elapsedTime(parametros[7])) { if (!startSignal) break; }
            }
            rightMotor.brake();
            leftMotor.brake();
            break;

        case 3: // 0011: giro 90 -- bilateral, el sensor decide el lado.
                // SIDE_LEFT -> izquierda, SIDE_RIGHT -> derecha, nada -> frena.
            #ifdef DEBUG
            Serial.print("case3 combo="); Serial.print(combo);
            Serial.print(" SIDE_LEFT="); Serial.print(irSensor[SIDE_LEFT]);
            Serial.print(" SIDE_RIGHT="); Serial.println(irSensor[SIDE_RIGHT]);
            #endif
            if (irSensor[SIDE_LEFT]) {
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3] + parametros[18]);
                while (!elapsedTime(parametros[5])) { if (!startSignal) break; }
            }
            else if (irSensor[SIDE_RIGHT]) {
                leftMotor.forward(parametros[6] + parametros[18]);
                rightMotor.backward(parametros[6]);
                while (!elapsedTime(parametros[8])) { if (!startSignal) break; }
            }
            rightMotor.brake();
            leftMotor.brake();
            break;

        case 4: // 0100: junta 0010 + 0011 -- los 4 sensores de giro en un
                // solo combo: TOP_LEFT/TOP_RIGHT -> 45, SIDE_LEFT/SIDE_RIGHT
                // -> 90, nada detectado -> frena. Mismos parametros[] que
                // los combos individuales (no se duplica ningun valor nuevo).
            if (irSensor[TOP_LEFT]) {
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3] + parametros[18]);
                while (!elapsedTime(parametros[4])) { if (!startSignal) break; }
            }
            else if (irSensor[TOP_RIGHT]) {
                leftMotor.forward(parametros[6] + parametros[18]);
                rightMotor.backward(parametros[6]);
                while (!elapsedTime(parametros[7])) { if (!startSignal) break; }
            }
            else if (irSensor[SIDE_LEFT]) {
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3] + parametros[18]);
                while (!elapsedTime(parametros[5])) { if (!startSignal) break; }
            }
            else if (irSensor[SIDE_RIGHT]) {
                leftMotor.forward(parametros[6] + parametros[18]);
                rightMotor.backward(parametros[6]);
                while (!elapsedTime(parametros[8])) { if (!startSignal) break; }
            }
            rightMotor.brake();
            leftMotor.brake();
            break;

        case 6: // 0110: giro 180
            rightMotor.forward(parametros[3]);
            leftMotor.backward(parametros[3] + parametros[18]);
            while (!elapsedTime(parametros[9])) { if (!startSignal) break; }
            rightMotor.brake();
            leftMotor.brake();
            break;

        case 7: // 0111: seguir sin atacar 1 -- gira para encarar al objetivo,
                // igual prioridad que BRAKE en tasks.cpp para 45/90, pero
                // omite el short por completo (ni pivote ni empuje): solo
                // TOP_LEFT/TOP_RIGHT/SIDE_LEFT/SIDE_RIGHT giran. TOP_MID,
                // SHORT_LEFT, SHORT_RIGHT y "nada detectado" quedan en freno.
            if (irSensor[TOP_LEFT]) {
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3] + parametros[18]);
                while (!elapsedTime(parametros[4])) { if (!startSignal) break; }
            }
            else if (irSensor[TOP_RIGHT]) {
                leftMotor.forward(parametros[6] + parametros[18]);
                rightMotor.backward(parametros[6]);
                while (!elapsedTime(parametros[7])) { if (!startSignal) break; }
            }
            else if (irSensor[SIDE_LEFT]) {
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3] + parametros[18]);
                while (!elapsedTime(parametros[5])) { if (!startSignal) break; }
            }
            else if (irSensor[SIDE_RIGHT]) {
                leftMotor.forward(parametros[6] + parametros[18]);
                rightMotor.backward(parametros[6]);
                while (!elapsedTime(parametros[8])) { if (!startSignal) break; }
            }
            rightMotor.brake();
            leftMotor.brake();
            break;

        case 8: // 1000: seguir sin atacar 2 -- igual que el anterior, pero
                // SHORT_LEFT/SHORT_RIGHT ahora si disparan, como un PIVOTE
                // asimetrico (una rueda 90% adelante, la otra 42% atras --
                // no es la mezcla de empuje+curva real de combate) con
                // duracion ajustable en vivo por BLE via parametros[10]/[11]
                // (SHORT_LEFT_DELAY/SHORT_RIGHT_DELAY, sin uso en tasks.cpp).
            if (irSensor[SHORT_LEFT]) {
                rightMotor.forward(90);
                leftMotor.backward(42);
                while (!elapsedTime(parametros[10])) { if (!startSignal) break; }
            }
            else if (irSensor[SHORT_RIGHT]) {
                leftMotor.forward(90);
                rightMotor.backward(42);
                while (!elapsedTime(parametros[11])) { if (!startSignal) break; }
            }
            else if (irSensor[TOP_LEFT]) {
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3] + parametros[18]);
                while (!elapsedTime(parametros[4])) { if (!startSignal) break; }
            }
            else if (irSensor[TOP_RIGHT]) {
                leftMotor.forward(parametros[6] + parametros[18]);
                rightMotor.backward(parametros[6]);
                while (!elapsedTime(parametros[7])) { if (!startSignal) break; }
            }
            else if (irSensor[SIDE_LEFT]) {
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3] + parametros[18]);
                while (!elapsedTime(parametros[5])) { if (!startSignal) break; }
            }
            else if (irSensor[SIDE_RIGHT]) {
                leftMotor.forward(parametros[6] + parametros[18]);
                rightMotor.backward(parametros[6]);
                while (!elapsedTime(parametros[8])) { if (!startSignal) break; }
            }
            rightMotor.brake();
            leftMotor.brake();
            break;

        case 9: { // 1001: seguir sin atacar 3 -- SHORT_LEFT/SHORT_RIGHT
                  // reproducen el movimiento REAL de combate (empuje+curva,
                  // ambos adelante, 90/42+correccion, 80ms fijos como
                  // SHORT_LEFT_MOVE/SHORT_RIGHT_MOVE de tasks.cpp) pero con
                  // un cooldown de 10s por lado -- unico movimiento de este
                  // archivo que empuja de verdad, asi que se limita por
                  // seguridad cuanto puede repetirse. El resto de acciones
                  // (los pivotes 45/90) no tienen esa restriccion.
            static unsigned long ultimoShortLeft = 0;
            static unsigned long ultimoShortRight = 0;
            const unsigned long COOLDOWN_MS = 10000;

            if (irSensor[SHORT_LEFT]) {
                if (millis() - ultimoShortLeft >= COOLDOWN_MS) {
                    rightMotor.forward(90);
                    leftMotor.forward(42 + parametros[18]);
                    while (!elapsedTime(80)) { if (!startSignal) break; }
                    ultimoShortLeft = millis();
                }
            }
            else if (irSensor[SHORT_RIGHT]) {
                if (millis() - ultimoShortRight >= COOLDOWN_MS) {
                    rightMotor.forward(42);
                    leftMotor.forward(90 + parametros[18]);
                    while (!elapsedTime(80)) { if (!startSignal) break; }
                    ultimoShortRight = millis();
                }
            }
            else if (irSensor[TOP_LEFT]) {
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3] + parametros[18]);
                while (!elapsedTime(parametros[4])) { if (!startSignal) break; }
            }
            else if (irSensor[TOP_RIGHT]) {
                leftMotor.forward(parametros[6] + parametros[18]);
                rightMotor.backward(parametros[6]);
                while (!elapsedTime(parametros[7])) { if (!startSignal) break; }
            }
            else if (irSensor[SIDE_LEFT]) {
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3] + parametros[18]);
                while (!elapsedTime(parametros[5])) { if (!startSignal) break; }
            }
            else if (irSensor[SIDE_RIGHT]) {
                leftMotor.forward(parametros[6] + parametros[18]);
                rightMotor.backward(parametros[6]);
                while (!elapsedTime(parametros[8])) { if (!startSignal) break; }
            }
            rightMotor.brake();
            leftMotor.brake();
            break;
        }

        default: // 0101 (liberado al unificar 45/90 en bilateral) y
                 // 1010-1111: libres, sin asignar todavia
            rightMotor.brake();
            leftMotor.brake();
            break;
    }

    delay(300);
}
#endif // RUN_CALIBRACION
