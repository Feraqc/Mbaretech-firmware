#ifdef RUN_PRUEBA_MOTORES
#include "globals.h"

// Prueba minima de motores: avanza los dos motores hacia adelante mientras
// el killswitch (startSignal) este activo, frena en cuanto se corta --
// mismo comportamiento que el kill switch de combate (primera señal
// prende, la siguiente apaga). Tiene su propio setup()/loop(); no toca
// sensores ni BLE. Velocidad: parametros[2] (mismo indice que usa
// TEST_FORWARD en tasks.cpp y el avance recto de calibracion.cpp).

void IRAM_ATTR PruebaMotoresKS_ISR() { startSignal = digitalRead(START_PIN); }

void setup() {
    #ifdef DEBUG
    Serial.begin(115200);
    #endif

    pinMode(START_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(START_PIN), PruebaMotoresKS_ISR, CHANGE);

    rightMotor.begin();
    leftMotor.begin();
}

void loop() {
    if (startSignal) {
        rightMotor.forward(parametros[2]);
        leftMotor.forward(parametros[2]);
    } else {
        rightMotor.brake();
        leftMotor.brake();
    }
    delay(50);
}

#endif // RUN_PRUEBA_MOTORES
