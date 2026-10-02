#ifdef RUN_BORRAR_ATRAS_ADELANTE
#include "globals.h"

// Prueba temporal (se puede borrar): mueve UN motor a la vez. Mientras el
// killswitch este activo repite en bucle:
//   derecho adelante  TIEMPO_MS -> frena PAUSA_MS
//   derecho atras     TIEMPO_MS -> frena PAUSA_MS
//   izquierdo adelante TIEMPO_MS -> frena PAUSA_MS
//   izquierdo atras    TIEMPO_MS -> frena PAUSA_MS
// El motor que no se prueba queda frenado todo el tiempo. Mismo PWM en todos
// los pasos (parametros[2], igual que pruebaMotores.cpp). Soltar el
// killswitch frena de inmediato, en cualquier punto del ciclo; al
// reactivarlo arranca de nuevo por "derecho adelante". Sin sensores ni BLE.

#define TIEMPO_MS 300
#define PAUSA_MS  1000

void IRAM_ATTR AtrasAdelanteKS_ISR() { startSignal = digitalRead(START_PIN); }

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

// Mueve solo un motor TIEMPO_MS y despues frena PAUSA_MS.
// Devuelve false si se solto el killswitch.
static bool paso(Motor &motor, bool adelante, const char *nombre) {
    #ifdef DEBUG
    Serial.println(nombre);
    #endif
    frenar();  // el otro motor queda frenado
    if (adelante) motor.forward(parametros[2]);
    else          motor.backward(parametros[2]);
    bool ok = esperar(TIEMPO_MS);
    frenar();
    return ok && esperar(PAUSA_MS);
}

void setup() {
    #ifdef DEBUG
    Serial.begin(115200);
    #endif

    pinMode(START_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(START_PIN), AtrasAdelanteKS_ISR, CHANGE);

    rightMotor.begin();
    leftMotor.begin();
}

void loop() {
    if (!startSignal) {
        frenar();
        delay(20);
        return;
    }

    if (!paso(rightMotor, true,  ">> derecho adelante"))   return;
    if (!paso(rightMotor, false, ">> derecho atras"))      return;
    if (!paso(leftMotor,  true,  ">> izquierdo adelante")) return;
    paso(leftMotor, false, ">> izquierdo atras");
}

#endif // RUN_BORRAR_ATRAS_ADELANTE
