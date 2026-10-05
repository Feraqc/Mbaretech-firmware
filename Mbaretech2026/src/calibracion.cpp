#ifdef RUN_CALIBRACION
#include "globals.h"
#include "bluetoothComm.h"
#include <Wire.h>
#include <esp_timer.h>

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
//   0101 -> shorts de combate: SHORT_LEFT/SHORT_RIGHT reproducen
//           SHORT_LEFT_MOVE/SHORT_RIGHT_MOVE de tasks.cpp (ambos motores
//           adelante 90/42+correccion, 80ms), UNA vez, y despues ignora
//           los sensores 10s (cooldown unico, para los dos lados)
//   0110 -> giro 180 bilateral: SIDE_LEFT gira 180 a la izq ([3]/[9]),
//           SIDE_RIGHT gira 180 a la der ([6]/[19]), nada detectado frena
//   0111 -> seguir sin atacar 1: gira para encarar (TOP/SIDE), SHORT se
//           omite por completo (frena, ni pivote ni empuje)
//   1000 -> seguir sin atacar 2: igual, pero SHORT hace los shorts de
//           combate de 0101 (90/42+correccion, 80ms) con cooldown unico
//           de 5s solo para los shorts; TOP/SIDE siguen girando siempre.
//           Linea delantera (5 lecturas) -> reversa 80ms + giro 180 de 0110
//   1001 -> autocalibracion del giro de 45 con el IMU (de autoCalGiro.cpp):
//           gira IZQ/DER alternado, mide el angulo con el giroscopio y
//           corrige el tiempo (arranca de parametros[4]/[7]) hasta 45 +-2;
//           una vez por activacion, despues repite el resumen FIN cada 2s
//   1010 -> autocalibracion del giro de 90 (el de SIDE_LEFT/SIDE_RIGHT),
//           igual que 1001 pero con objetivo 90 y parametros[5]/[8]
//   1011-1111 -> libres, sin asignar todavia
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

// ===================== Autocalibracion de giro con IMU =====================
// Traida de src/tests/autoCalGiro.cpp (mismo algoritmo y mismas constantes),
// para usarla desde un combo del DIP sin cambiar de build. El robot gira con
// los mismos comandos de motor que 0010/tasks.cpp, mide con el eje Z del
// giroscopio el angulo real (incluida la inercia despues de frenar) y corrige
// el tiempo de forma proporcional hasta quedar en objetivo +- AC_TOLERANCIA,
// alternando IZQ/DER con AC_ESPERA_MS de pausa. El freno lo da un esp_timer,
// para que una traba del I2C no estire el giro; un intento con un hueco
// entre lecturas > AC_GAP_MAX_MS se descarta y se repite.
// Los tiempos encontrados NO se guardan solos en parametros[]: copiarlos por
// BLE o a los #define TURN_*_DELAY de globals.h.
#define AC_TOLERANCIA      2.0f
#define AC_ESPERA_MS       1000
#define AC_MAX_INTENTOS    15     // por lado
#define AC_GAP_MAX_MS      10.0f
#define AC_QUIETO_DPS      5.0f
#define AC_QUIETO_MS       30
#define AC_ASENTAR_MAX_MS  600
#define AC_TIEMPO_MIN_MS   10
#define AC_TIEMPO_MAX_MS   400

#define MPU_ADDR_A       0x68
#define MPU_ADDR_B       0x69
#define REG_CONFIG       0x1A
#define REG_GYRO_CFG     0x1B
#define REG_GYRO_ZOUT    0x47
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75
#define GYRO_LSB_POR_DPS 16.4f   // +-2000 dps

static uint8_t mpuAddr = 0;
static bool imuOk = false;
static float biasGz = 0;

// Estado de los combos 1001/1010: autocalPendiente se pone en true en cada flanco de
// subida del killswitch (cuando se congela el combo), asi corre una sola vez
// por activacion.
static bool autocalPendiente = false;
static String autocalResumen = "";
static unsigned long autocalUltimoResumen = 0;

static bool escribirReg(uint8_t reg, uint8_t valor) {
    Wire.beginTransmission(mpuAddr);
    Wire.write(reg);
    Wire.write(valor);
    return Wire.endTransmission() == 0;
}

static bool leerRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
    Wire.beginTransmission(mpuAddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(mpuAddr, n) != n) return false;
    for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
}

static bool detectarIMU() {
    uint8_t candidatos[2] = {MPU_ADDR_A, MPU_ADDR_B};
    for (uint8_t i = 0; i < 2; i++) {
        mpuAddr = candidatos[i];
        uint8_t who;
        if (leerRegs(REG_WHO_AM_I, &who, 1)) return true;
    }
    mpuAddr = 0;
    return false;
}

static bool configurarIMU() {
    bool ok = true;
    ok &= escribirReg(REG_PWR_MGMT_1, 0x01);  // despertar, reloj = PLL giro X
    delay(100);
    ok &= escribirReg(REG_CONFIG, 0x01);      // DLPF ~184Hz: poca demora en giros cortos
    ok &= escribirReg(REG_GYRO_CFG, 0x18);    // +-2000 dps
    return ok;
}

static bool leerGz(int16_t *gz) {
    uint8_t b[2];
    if (!leerRegs(REG_GYRO_ZOUT, b, 2)) return false;
    *gz = (int16_t)(b[0] << 8 | b[1]);
    return true;
}

static bool calibrarBias() {
    const int N = 300;
    long suma = 0;
    int validas = 0;
    for (int i = 0; i < N; i++) {
        int16_t gz;
        if (leerGz(&gz)) { suma += gz; validas++; }
        delay(2);
    }
    if (validas < N / 2) return false;
    biasGz = (float)suma / validas;
    return true;
}

static void autocalReportar(const String &msg) {
    #ifdef DEBUG
    Serial.println(msg);
    #endif
    #ifndef SKIP_BLE
    sendData(msg + "\n");  // salto de linea: si no, la app del celular los pega
    #endif
}

static esp_timer_handle_t timerFreno;
static volatile bool frenado = false;

static void frenarCallback(void *) {
    rightMotor.brake();
    leftMotor.brake();
    frenado = true;
}

enum Lado { IZQ = 0, DER = 1 };

static void arrancarGiro(Lado lado) {
    if (lado == IZQ) {
        rightMotor.forward(parametros[3]);
        leftMotor.backward(parametros[3] + parametros[18]);
    } else {
        leftMotor.forward(parametros[6] + parametros[18]);
        rightMotor.backward(parametros[6]);
    }
}

struct Medicion {
    float grados;    // con signo (el signo depende del montaje del IMU)
    float gapMaxMs;  // mayor hueco entre lecturas validas
    int   fallos;    // lecturas I2C fallidas durante el giro
    bool  abortado;  // se solto el killswitch
};

static Medicion medirGiro(Lado lado, int tiempoMs) {
    Medicion m = {0, 0, 0, false};
    frenado = false;

    unsigned long ultimo = micros();
    arrancarGiro(lado);
    esp_timer_start_once(timerFreno, (uint64_t)tiempoMs * 1000);

    unsigned long tFreno = 0;
    unsigned long quietoDesde = 0;
    while (true) {
        if (!startSignal) {
            esp_timer_stop(timerFreno);
            frenarCallback(nullptr);
            m.abortado = true;
            return m;
        }

        // Tope de espera despues de frenar, aunque el IMU deje de responder.
        if (frenado) {
            if (tFreno == 0) tFreno = millis();
            if (millis() - tFreno >= AC_ASENTAR_MAX_MS) break;
        }

        int16_t gz;
        if (!leerGz(&gz)) { m.fallos++; continue; }  // el hueco queda medido en dt
        unsigned long ahora = micros();
        float dt = (ahora - ultimo) / 1e6f;
        ultimo = ahora;
        if (dt * 1000.0f > m.gapMaxMs) m.gapMaxMs = dt * 1000.0f;

        float dps = (gz - biasGz) / GYRO_LSB_POR_DPS;
        m.grados += dps * dt;

        if (frenado) {
            unsigned long ms = millis();
            if (fabsf(dps) < AC_QUIETO_DPS) {
                if (quietoDesde == 0) quietoDesde = ms;
                if (ms - quietoDesde >= AC_QUIETO_MS) break;
            } else {
                quietoDesde = 0;
            }
        }
    }
    return m;
}

static int corregirTiempo(int tiempo, float medido, float objetivo) {
    float nuevo;
    if (medido < 5.0f) {
        nuevo = tiempo * 1.3f;  // casi no giro: subir lo maximo permitido
    } else {
        nuevo = tiempo * objetivo / medido;
        nuevo = constrain(nuevo, tiempo * 0.7f, tiempo * 1.3f);
    }
    int n = (int)lroundf(nuevo);
    if (n == tiempo) n += (medido > objetivo) ? -1 : 1;
    return constrain(n, AC_TIEMPO_MIN_MS, AC_TIEMPO_MAX_MS);
}

static bool esperarConKillswitch(unsigned long ms) {
    unsigned long t0 = millis();
    while (millis() - t0 < ms) {
        if (!startSignal) return false;
        delay(10);
    }
    return true;
}

// Corre la autocalibracion completa (bloqueante) y devuelve el resumen
// "FIN ..." (o un mensaje de error/aborto). objetivo en grados; idxIzq/idxDer
// son los indices de parametros[] de donde sale el tiempo inicial de cada
// lado ([4]/[7] para 45, [5]/[8] para 90).
static String autocalGiro(float objetivo, int idxIzq, int idxDer) {
    if (!imuOk) {
        String e = "ERROR: IMU no responde -- revisar cableado. No se gira.";
        autocalReportar(e);
        return e;
    }

    autocalReportar("Autocal " + String(objetivo, 0) + ": quieto 1s y calibrando giroscopio, no tocar...");
    if (!esperarConKillswitch(1000)) return "Abortado (killswitch).";
    if (!calibrarBias()) {
        String e = "ERROR: lecturas del IMU fallando al calibrar. No se gira.";
        autocalReportar(e);
        return e;
    }

    int tiempo[2]   = {parametros[idxIzq], parametros[idxDer]};
    float ultimo[2] = {0, 0};   // ultimo angulo valido medido
    int probado[2]  = {0, 0};   // tiempo con el que se midio ese angulo
    int intentos[2] = {0, 0};
    bool listo[2]   = {false, false};
    const char *nombre[2] = {"IZQ", "DER"};

    while (!(listo[IZQ] && listo[DER])) {
        for (int l = IZQ; l <= DER; l++) {
            if (listo[l]) continue;
            if (intentos[l] >= AC_MAX_INTENTOS) { listo[l] = true; continue; }

            intentos[l]++;
            Medicion m = medirGiro((Lado)l, tiempo[l]);
            if (m.abortado) { autocalReportar("Abortado (killswitch)."); return "Abortado (killswitch)."; }

            float grados = fabsf(m.grados);
            String linea = String(nombre[l]) + " #" + String(intentos[l]) + " "
                         + String(tiempo[l]) + "ms -> " + String(m.grados, 1) + "g"
                         + " gap " + String(m.gapMaxMs, 1) + "ms fallos " + String(m.fallos);

            if (m.gapMaxMs > AC_GAP_MAX_MS) {
                linea += " DESCARTADO, repite";
            } else {
                float error = grados - objetivo;
                ultimo[l] = grados;
                probado[l] = tiempo[l];
                linea += " err " + String(error, 1);
                if (fabsf(error) <= AC_TOLERANCIA) {
                    listo[l] = true;
                    linea += " OK";
                } else {
                    tiempo[l] = corregirTiempo(tiempo[l], grados, objetivo);
                    linea += " -> " + String(tiempo[l]) + "ms";
                }
            }
            autocalReportar(linea);

            if (!esperarConKillswitch(AC_ESPERA_MS)) { autocalReportar("Abortado (killswitch)."); return "Abortado (killswitch)."; }
        }
    }

    String resumen = "FIN " + String(objetivo, 0) + "g:";
    for (int l = IZQ; l <= DER; l++) {
        bool ok = fabsf(ultimo[l] - objetivo) <= AC_TOLERANCIA;
        resumen += String(" ") + nombre[l] + "=" + String(probado[l]) + "ms("
                 + String(ultimo[l], 1) + "g" + (ok ? ")" : " NO CONVERGIO)");
    }
    autocalReportar(resumen);
    return resumen;
}
// ===========================================================================

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

    // IMU para la autocalibracion de 1001. No bloquea: si no responde, se
    // reintenta cada 1s desde loop() con el killswitch apagado.
    esp_timer_create_args_t args = {};
    args.callback = frenarCallback;
    args.name = "frenoGiro";
    esp_timer_create(&args, &timerFreno);
    Wire.begin(SDA_PIN, SCL_PIN, 100000);
    Wire.setTimeOut(20);
    imuOk = detectarIMU() && configurarIMU();
    #ifdef DEBUG
    Serial.println(imuOk ? "IMU detectado." : "IMU NO detectado, reintentando cada 1s...");
    #endif

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
        static unsigned long ultimoIntentoIMU = 0;
        if (!imuOk && millis() - ultimoIntentoIMU >= 1000) {
            ultimoIntentoIMU = millis();
            imuOk = detectarIMU() && configurarIMU();
            #ifdef DEBUG
            if (imuOk) Serial.println("IMU detectado.");
            #endif
        }
        delay(50);
        return;
    }

    if (!corriendo) {
        // Borde etapa 1 -> etapa 2: se congela el combo una sola vez.
        comboCongelado = comboLive;
        corriendo = true;
        autocalPendiente = true;  // 1001/1010: una autocalibracion por activacion
        autocalResumen = "";
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

        case 5: { // 0101: shorts de combate -- mismo orden, PWM y tiempo que
                  // BRAKE -> SHORT_LEFT_MOVE/SHORT_RIGHT_MOVE en tasks.cpp
                  // (ambos motores ADELANTE, 90/42 + parametros[18], 80ms).
                  // Por seguridad hace el movimiento una sola vez y despues
                  // ignora los sensores durante COOLDOWN_MS, sin importar el
                  // lado (un unico cooldown para los dos). La primera vez no
                  // espera: se usa un flag y no "ultimo = 0", porque con 0 el
                  // primer movimiento quedaria bloqueado los primeros 10s
                  // despues de encender el robot.
            static bool hizoShort = false;
            static unsigned long ultimoShort = 0;
            const unsigned long COOLDOWN_MS = 10000;

            if (hizoShort && millis() - ultimoShort < COOLDOWN_MS) {
                rightMotor.brake();
                leftMotor.brake();
                break; // en cooldown: no lee los shorts
            }

            if (irSensor[SHORT_LEFT]) {
                rightMotor.forward(FORWARD_90);
                leftMotor.forward(FORWARD_42 + parametros[18]);
                while (!elapsedTime(80)) { if (!startSignal) break; }
                hizoShort = true;
                ultimoShort = millis();
                #ifndef SKIP_BLE
                sendData("SHORT_LEFT_MOVE hecho, proximo en 10s");
                #endif
                #ifdef DEBUG
                Serial.println("SHORT_LEFT_MOVE hecho, proximo en 10s");
                #endif
            }
            else if (irSensor[SHORT_RIGHT]) {
                rightMotor.forward(FORWARD_42);
                leftMotor.forward(FORWARD_90 + parametros[18]);
                while (!elapsedTime(80)) { if (!startSignal) break; }
                hizoShort = true;
                ultimoShort = millis();
                #ifndef SKIP_BLE
                sendData("SHORT_RIGHT_MOVE hecho, proximo en 10s");
                #endif
                #ifdef DEBUG
                Serial.println("SHORT_RIGHT_MOVE hecho, proximo en 10s");
                #endif
            }
            rightMotor.brake();
            leftMotor.brake();
            break;
        }

        case 6: // 0110: giro 180 -- bilateral, el sensor decide el lado
                // (igual que 0011 pero con 180). SIDE_LEFT -> izquierda, mismo
                // giro que TURN_180 de tasks.cpp ([3]/[9]); SIDE_RIGHT ->
                // derecha ([6]/[19], tiempo propio porque los giros a la
                // derecha no duran lo mismo -- tasks.cpp no tiene 180 der).
                // Nada detectado -> frena (antes giraba sin parar).
            if (irSensor[SIDE_LEFT]) {
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3] + parametros[18]);
                while (!elapsedTime(parametros[9])) { if (!startSignal) break; }
            }
            else if (irSensor[SIDE_RIGHT]) {
                leftMotor.forward(parametros[6] + parametros[18]);
                rightMotor.backward(parametros[6]);
                while (!elapsedTime(parametros[19])) { if (!startSignal) break; }
            }
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

        case 8: { // 1000: seguir sin atacar 2 -- igual que 0111 (TOP->45,
                  // SIDE->90), pero SHORT_LEFT/SHORT_RIGHT hacen los shorts de
                  // combate de 0101 (SHORT_LEFT_MOVE/SHORT_RIGHT_MOVE de
                  // tasks.cpp: ambos motores adelante, 90/42 + parametros[18],
                  // 80ms). Solo los shorts tienen restriccion: despues de uno,
                  // los dos shorts quedan bloqueados COOLDOWN_MS (cooldown
                  // unico). Mientras tanto un SHORT se ignora y se siguen
                  // evaluando TOP/SIDE, que giran normalmente sin espera.
                  // Primer short sin espera (flag, igual que en 0101).
            static bool hizoShort = false;
            static unsigned long ultimoShort = 0;
            const unsigned long COOLDOWN_MS = 5000;
            bool shortListo = !hizoShort || millis() - ultimoShort >= COOLDOWN_MS;

            // Linea delantera (prioridad sobre todo, como LINE_RETREAT en
            // tasks.cpp): reversa 90% x 80ms y giro 180 con los valores de
            // 0110 (parametros[3]/[9]/[18]); despues frena y espera el
            // siguiente sensor. lineSensor[0]/[1] es una sola lectura (arriba
            // del loop), asi que se confirma con 4 lecturas mas seguidas
            // (5 en total, a pedido del usuario; la prueba de frenado validada
            // usaba 3) para no girar por un pico de ruido de los motores.
            bool lineaIzq = lineSensor[0], lineaDer = lineSensor[1];
            for (int i = 0; i < 4 && (lineaIzq || lineaDer); i++) {
                delay(1);
                lineaIzq = lineaIzq && checkLineSensora(readLineSensorFront(LINE_FRONT_LEFT));
                lineaDer = lineaDer && checkLineSensorb(readLineSensorFront(LINE_FRONT_RIGHT));
            }

            if (lineaIzq || lineaDer) {
                rightMotor.backward(FORWARD_90);
                leftMotor.backward(FORWARD_90);
                while (!elapsedTime(80)) { if (!startSignal) break; }
                rightMotor.forward(parametros[3]);
                leftMotor.backward(parametros[3] + parametros[18]);
                while (!elapsedTime(parametros[9])) { if (!startSignal) break; }
                String aviso = String("LINEA ") + (lineaIzq && lineaDer ? "AMBOS" : (lineaIzq ? "IZQ" : "DER"))
                             + " -> reversa 80ms + giro 180";
                #ifndef SKIP_BLE
                sendData(aviso);
                #endif
                #ifdef DEBUG
                Serial.println(aviso);
                #endif
            }
            else if (shortListo && irSensor[SHORT_LEFT]) {
                rightMotor.forward(FORWARD_90);
                leftMotor.forward(FORWARD_42 + parametros[18]);
                while (!elapsedTime(80)) { if (!startSignal) break; }
                hizoShort = true;
                ultimoShort = millis();
                #ifndef SKIP_BLE
                sendData("SHORT_LEFT_MOVE hecho, proximo short en 5s");
                #endif
                #ifdef DEBUG
                Serial.println("SHORT_LEFT_MOVE hecho, proximo short en 5s");
                #endif
            }
            else if (shortListo && irSensor[SHORT_RIGHT]) {
                rightMotor.forward(FORWARD_42);
                leftMotor.forward(FORWARD_90 + parametros[18]);
                while (!elapsedTime(80)) { if (!startSignal) break; }
                hizoShort = true;
                ultimoShort = millis();
                #ifndef SKIP_BLE
                sendData("SHORT_RIGHT_MOVE hecho, proximo short en 5s");
                #endif
                #ifdef DEBUG
                Serial.println("SHORT_RIGHT_MOVE hecho, proximo short en 5s");
                #endif
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

        case 9: // 1001: autocalibracion del giro de 45 con el IMU (traida
                // de src/tests/autoCalGiro.cpp). Corre UNA vez por
                // activacion del killswitch; despues repite el resumen cada
                // 2s hasta soltarlo. Bloquea el loop mientras calibra (no hay
                // reporte de sensores en ese tiempo).
            if (autocalPendiente) {
                autocalPendiente = false;
                autocalResumen = autocalGiro(45.0f, 4, 7);
                autocalUltimoResumen = millis();
            } else if (autocalResumen.length() && millis() - autocalUltimoResumen >= 2000) {
                autocalUltimoResumen = millis();
                autocalReportar(autocalResumen);
            }
            rightMotor.brake();
            leftMotor.brake();
            break;

        case 10: // 1010: autocalibracion del giro de 90 -- el que usan
                 // SIDE_LEFT/SIDE_RIGHT (TURN_LEFT_90/TURN_RIGHT_90). Misma
                 // logica que 1001, con objetivo 90 y tiempos iniciales de
                 // parametros[5] (IZQ) / [8] (DER).
            if (autocalPendiente) {
                autocalPendiente = false;
                autocalResumen = autocalGiro(90.0f, 5, 8);
                autocalUltimoResumen = millis();
            } else if (autocalResumen.length() && millis() - autocalUltimoResumen >= 2000) {
                autocalUltimoResumen = millis();
                autocalReportar(autocalResumen);
            }
            rightMotor.brake();
            leftMotor.brake();
            break;

        default: // 1011-1111: libres, sin asignar todavia
            rightMotor.brake();
            leftMotor.brake();
            break;
    }

    delay(300);
}
#endif // RUN_CALIBRACION
