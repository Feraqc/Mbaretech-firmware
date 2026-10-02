#ifdef RUN_GIRAR45_LINEA
#include "globals.h"
#include "bluetoothComm.h"

// Prueba: el robot avanza y rebota en el borde del dohyo con giros de 45.
// Mientras el killswitch este activo:
//   - avanza a FORWARD_80 (velocidad de ataque de tasks.cpp)
//   - linea con el sensor IZQUIERDO -> reversa 80ms, gira 45 a la DERECHA, sigue
//   - linea con el sensor DERECHO   -> reversa 80ms, gira 45 a la IZQUIERDA, sigue
//   - linea con LOS DOS a la vez (borde de frente) -> reversa 80ms, gira ~180
//     a la IZQUIERDA (un giro de 45 lo dejaria todavia mirando al borde). El
//     180 todavia no esta calibrado: GIRO_180_MS fijo para ver que pasa.
//
// Todos los movimientos son los de combate (tasks.cpp), con PWM de competencia:
//   reversa:  backward(FORWARD_90) x 80ms                (LINE_RETREAT)
//   45 izq:   der forward([3]), izq backward([3]+[18]) x [4]   (TURN_LEFT_45)
//   45 der:   izq forward([6]+[18]), der backward([6]) x [7]   (TURN_RIGHT_45)
//   ~180:     der forward([3]), izq backward([3]+[18]) x GIRO_180_MS (sin calibrar)
// Los tiempos de giro se pueden ajustar en vivo por BLE ("4 69", "7 58", ...).
//
// La linea se confirma con 3 lecturas seguidas en blanco por sensor (igual que
// la prueba de frenado validada), para no reaccionar a picos de ruido de los
// motores. Solo sensores delanteros. Cada reaccion se informa por Serial y BLE.
// Soltar el killswitch frena de inmediato, incluso a mitad de un giro.

#define VEL_AVANCE      FORWARD_80
#define VEL_REVERSA     FORWARD_90
#define TIEMPO_REVERSA  80
#define LECTURAS_BLANCO 3
#define GIRO_180_MS     120   // provisorio, sin calibrar (2026-10-01)

void IRAM_ATTR Girar45KS_ISR() { startSignal = digitalRead(START_PIN); }

// Espera ms milisegundos; devuelve false si se solto el killswitch.
static bool esperar(unsigned long ms) {
    unsigned long t0 = millis();
    while (millis() - t0 < ms) {
        if (!startSignal) return false;
        delay(1);
    }
    return true;
}

static void frenar() {
    rightMotor.brake();
    leftMotor.brake();
}

static void avanzar() {
    rightMotor.forward(VEL_AVANCE);
    leftMotor.forward(VEL_AVANCE);
}

static void reportar(const String &msg) {
    Serial.println(msg);
    sendData(msg + "\n");
}

void setup() {
    Serial.begin(115200);

    pinMode(START_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(START_PIN), Girar45KS_ISR, CHANGE);
    rightMotor.begin();
    leftMotor.begin();

    lineSensorsInit();
    BLE_UART_Init("MBARETECH");
}

void loop() {
    static bool corriendo = false;
    static int seguidasIzq = 0, seguidasDer = 0;
    static int reacciones = 0;

    if (!startSignal) {
        frenar();
        corriendo = false;
        delay(20);
        return;
    }

    if (!corriendo) {
        corriendo = true;
        seguidasIzq = seguidasDer = 0;
        reacciones = 0;
        reportar("En marcha. Giro 45: IZQ=" + String(parametros[4]) + "ms DER="
                 + String(parametros[7]) + "ms, ~180 IZQ=" + String(GIRO_180_MS) + "ms");
        avanzar();
    }

    seguidasIzq = checkLineSensora(readLineSensorFront(LINE_FRONT_LEFT))  ? seguidasIzq + 1 : 0;
    seguidasDer = checkLineSensorb(readLineSensorFront(LINE_FRONT_RIGHT)) ? seguidasDer + 1 : 0;
    bool izq = seguidasIzq >= LECTURAS_BLANCO;
    bool der = seguidasDer >= LECTURAS_BLANCO;
    if (!izq && !der) return;  // sigue avanzando

    // Reversa (igual que LINE_RETREAT)
    rightMotor.backward(VEL_REVERSA);
    leftMotor.backward(VEL_REVERSA);
    if (!esperar(TIEMPO_REVERSA)) { frenar(); return; }

    // Giro alejandose de la linea
    reacciones++;
    unsigned long tiempoGiro;
    if (izq && der) {
        rightMotor.forward(parametros[3]);
        leftMotor.backward(parametros[3] + parametros[18]);
        tiempoGiro = GIRO_180_MS;
        reportar("#" + String(reacciones) + " linea AMBOS -> giro ~180 IZQ");
    } else if (izq) {
        leftMotor.forward(parametros[6] + parametros[18]);
        rightMotor.backward(parametros[6]);
        tiempoGiro = parametros[7];
        reportar("#" + String(reacciones) + " linea IZQ -> giro 45 DER");
    } else {
        rightMotor.forward(parametros[3]);
        leftMotor.backward(parametros[3] + parametros[18]);
        tiempoGiro = parametros[4];
        reportar("#" + String(reacciones) + " linea DER -> giro 45 IZQ");
    }
    if (!esperar(tiempoGiro)) { frenar(); return; }

    // Sigue adelante con los contadores en cero
    seguidasIzq = seguidasDer = 0;
    avanzar();
}

#endif // RUN_GIRAR45_LINEA
