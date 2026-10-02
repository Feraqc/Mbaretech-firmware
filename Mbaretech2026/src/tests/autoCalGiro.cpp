#ifdef RUN_AUTOCAL_GIRO
#include "globals.h"
#include "bluetoothComm.h"
#include <Wire.h>
#include <esp_timer.h>

// Autocalibracion de giros con el IMU: en vez de ajustar a ojo el tiempo de
// giro (modos 0010/0011 de calibracion.cpp), el robot gira, mide con el
// giroscopio cuantos grados giro de verdad, corrige el tiempo y repite hasta
// quedar dentro de OBJETIVO_GRADOS +- TOLERANCIA_GRADOS.
//
// El giro es exactamente el de calibracion.cpp/tasks.cpp:
//   IZQ: derecho forward(parametros[3]), izquierdo backward(parametros[3]+parametros[18])
//   DER: izquierdo forward(parametros[6]+parametros[18]), derecho backward(parametros[6])
// y lo unico que se modifica es el tiempo. Arranca desde los valores actuales
// (parametros[4]/[7] para 45, que salen de TURN_*_45_DELAY en globals.h).
//
// Se alternan los lados (IZQ, DER, IZQ, ...) para que el robot vuelva mas o
// menos a la misma orientacion y no se vaya girando siempre para el mismo
// lado. Cada lado deja de intentar cuando ya convergio. Entre intento e
// intento hay ESPERA_MS de pausa (robot quieto).
//
// Medicion: se integra el eje Z del giroscopio desde que arrancan los motores
// hasta que el robot queda quieto DESPUES de frenar -- asi el angulo incluye
// lo que sigue girando por inercia, que es lo que importa en combate. El
// freno lo da un timer (esp_timer) y no el lazo, para que una demora del I2C
// no estire el giro. Si el I2C se traba mas de GAP_MAX_MS en un intento, la
// medicion no es confiable: se descarta y se repite con el mismo tiempo.
//
// Correccion del tiempo: proporcional (tiempo * objetivo / medido), limitada
// a +-30% por intento y al menos 1ms de cambio, para converger en pocos
// intentos sin pegar saltos grandes.
//
// Uso: robot quieto en el piso, activar el killswitch. Espera 1s, calibra el
// bias del giroscopio (~0.6s, NO tocarlo) y empieza. Al terminar repite el
// resumen cada 2s por Serial y BLE hasta soltar el killswitch. Soltarlo en
// cualquier momento frena y aborta.
//
// Los tiempos encontrados NO se guardan solos: copiarlos a parametros[] por
// BLE o a los #define TURN_*_DELAY de globals.h.

#define OBJETIVO_GRADOS    45.0f
#define TOLERANCIA_GRADOS  2.0f  // antes 5 (2026-10-01): con 5 los dos lados quedaban en 42-43
#define ESPERA_MS          1000  // pausa entre intentos
#define MAX_INTENTOS       15    // por lado
#define GAP_MAX_MS         10.0f // hueco maximo aceptable entre lecturas del IMU
#define QUIETO_DPS         5.0f  // por debajo de esto se considera quieto
#define QUIETO_MS          30    // tiempo quieto seguido para dar el giro por terminado
#define ASENTAR_MAX_MS     600   // tope de espera despues de frenar
#define TIEMPO_MIN_MS      10
#define TIEMPO_MAX_MS      400

// De donde sale el tiempo inicial de cada lado: [4]/[7] = 45 grados.
// Para 90 grados usar [5]/[8] (y cambiar OBJETIVO_GRADOS).
#define IDX_TIEMPO_IZQ 4
#define IDX_TIEMPO_DER 7

// MPU6050
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

// Solo el eje Z del giroscopio (2 bytes): lectura mas corta = mas muestras.
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

static void reportar(const String &msg) {
    Serial.println(msg);
    sendData(msg + "\n");  // salto de linea: si no, la app del celular los pega
}

// --- Freno por timer, independiente del lazo de lectura del IMU ---
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
            if (millis() - tFreno >= ASENTAR_MAX_MS) break;
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
            if (fabsf(dps) < QUIETO_DPS) {
                if (quietoDesde == 0) quietoDesde = ms;
                if (ms - quietoDesde >= QUIETO_MS) break;
            } else {
                quietoDesde = 0;
            }
        }
    }
    return m;
}

static int corregirTiempo(int tiempo, float medido) {
    float nuevo;
    if (medido < 5.0f) {
        nuevo = tiempo * 1.3f;  // casi no giro: subir lo maximo permitido
    } else {
        nuevo = tiempo * OBJETIVO_GRADOS / medido;
        nuevo = constrain(nuevo, tiempo * 0.7f, tiempo * 1.3f);
    }
    int n = (int)lroundf(nuevo);
    if (n == tiempo) n += (medido > OBJETIVO_GRADOS) ? -1 : 1;
    return constrain(n, TIEMPO_MIN_MS, TIEMPO_MAX_MS);
}

static bool esperarConKillswitch(unsigned long ms) {
    unsigned long t0 = millis();
    while (millis() - t0 < ms) {
        if (!startSignal) return false;
        delay(10);
    }
    return true;
}

void IRAM_ATTR AutoCalKS_ISR() { startSignal = digitalRead(START_PIN); }

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n=== AUTOCALIBRACION DE GIRO CON IMU ===");

    // Killswitch y motores primero, sin depender del IMU (leccion de pruebaMartillo.cpp)
    pinMode(START_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(START_PIN), AutoCalKS_ISR, CHANGE);
    rightMotor.begin();
    leftMotor.begin();

    esp_timer_create_args_t args = {};
    args.callback = frenarCallback;
    args.name = "frenoGiro";
    esp_timer_create(&args, &timerFreno);

    BLE_UART_Init("MBARETECH");

    Wire.begin(SDA_PIN, SCL_PIN, 100000);
    Wire.setTimeOut(20);
    imuOk = detectarIMU() && configurarIMU();
    Serial.println(imuOk ? "IMU detectado." : "IMU NO detectado, reintentando cada 1s...");

    Serial.printf("Objetivo %.0f +- %.0f grados, arranque IZQ=%dms DER=%dms, pausa %dms\n",
                  OBJETIVO_GRADOS, TOLERANCIA_GRADOS,
                  parametros[IDX_TIEMPO_IZQ], parametros[IDX_TIEMPO_DER], ESPERA_MS);
}

void loop() {
    static bool corriendo = false;
    static String resumen = "";
    static unsigned long ultimoResumen = 0;
    static unsigned long ultimoIntentoIMU = 0;

    if (!startSignal) {
        rightMotor.brake();
        leftMotor.brake();
        corriendo = false;
        if (!imuOk && millis() - ultimoIntentoIMU >= 1000) {
            ultimoIntentoIMU = millis();
            imuOk = detectarIMU() && configurarIMU();
            if (imuOk) Serial.println("IMU detectado.");
        }
        delay(50);
        return;
    }

    if (corriendo) {
        // Ya termino esta activacion: repetir el resumen hasta soltar el killswitch.
        if (resumen.length() && millis() - ultimoResumen >= 2000) {
            ultimoResumen = millis();
            reportar(resumen);
        }
        delay(10);
        return;
    }
    corriendo = true;
    ultimoResumen = millis();

    if (!imuOk) {
        resumen = "ERROR: IMU no responde -- revisar cableado. No se gira.";
        reportar(resumen);
        return;
    }

    reportar("Autocal: quieto 1s y calibrando giroscopio, no tocar...");
    if (!esperarConKillswitch(1000)) return;
    if (!calibrarBias()) {
        resumen = "ERROR: lecturas del IMU fallando al calibrar. No se gira.";
        reportar(resumen);
        return;
    }

    int tiempo[2]   = {parametros[IDX_TIEMPO_IZQ], parametros[IDX_TIEMPO_DER]};
    float ultimo[2] = {0, 0};   // ultimo angulo valido medido
    int probado[2]  = {0, 0};   // tiempo con el que se midio ese angulo
    int intentos[2] = {0, 0};
    bool listo[2]   = {false, false};
    const char *nombre[2] = {"IZQ", "DER"};

    while (!(listo[IZQ] && listo[DER])) {
        for (int l = IZQ; l <= DER; l++) {
            if (listo[l]) continue;
            if (intentos[l] >= MAX_INTENTOS) { listo[l] = true; continue; }

            intentos[l]++;
            Medicion m = medirGiro((Lado)l, tiempo[l]);
            if (m.abortado) { reportar("Abortado (killswitch)."); return; }

            float grados = fabsf(m.grados);
            String linea = String(nombre[l]) + " #" + String(intentos[l]) + " "
                         + String(tiempo[l]) + "ms -> " + String(m.grados, 1) + "g"
                         + " gap " + String(m.gapMaxMs, 1) + "ms fallos " + String(m.fallos);

            if (m.gapMaxMs > GAP_MAX_MS) {
                linea += " DESCARTADO, repite";
            } else {
                float error = grados - OBJETIVO_GRADOS;
                ultimo[l] = grados;
                probado[l] = tiempo[l];
                linea += " err " + String(error, 1);
                if (fabsf(error) <= TOLERANCIA_GRADOS) {
                    listo[l] = true;
                    linea += " OK";
                } else {
                    tiempo[l] = corregirTiempo(tiempo[l], grados);
                    linea += " -> " + String(tiempo[l]) + "ms";
                }
            }
            reportar(linea);

            if (!esperarConKillswitch(ESPERA_MS)) { reportar("Abortado (killswitch)."); return; }
        }
    }

    resumen = "FIN " + String(OBJETIVO_GRADOS, 0) + "g:";
    for (int l = IZQ; l <= DER; l++) {
        bool ok = fabsf(ultimo[l] - OBJETIVO_GRADOS) <= TOLERANCIA_GRADOS;
        resumen += String(" ") + nombre[l] + "=" + String(probado[l]) + "ms("
                 + String(ultimo[l], 1) + "g" + (ok ? ")" : " NO CONVERGIO)");
    }
    reportar(resumen);
}

#endif // RUN_AUTOCAL_GIRO
