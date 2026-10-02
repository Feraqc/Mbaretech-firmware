#ifdef RUN_PRUEBA_SWITCH
#include "globals.h"

// Prueba minima de los 5 DIP switches: solo lee y reporta, no mueve nada.
// Muestra el estado individual de DIPA-E y el combo calculado (mismo orden
// de bits DIPE-DIPA-DIPB-DIPC que usa tasks.cpp/calibracion.cpp para elegir
// estrategia/accion -- DIPD no entra en esa cuenta, pero se muestra igual
// para poder verificarlo). Imprime solo cuando algo cambia, moviendo un
// solo switch a la vez para confirmar que cada uno responde en el
// caracter que le corresponde. Sin motores, sin BLE, sin startSignal --
// no hace falta nada de eso para esta verificacion.

void setup() {
    Serial.begin(115200);
    delay(1500);  // tiempo para abrir el monitor serie
    Serial.println("\n=== PRUEBA DE DIP SWITCHES ===");
    Serial.println("Mové un switch a la vez y confirmá que cambia el caracter correcto.");

    pinMode(DIPA, INPUT);
    pinMode(DIPB, INPUT);
    pinMode(DIPC, INPUT);
    pinMode(DIPD, INPUT);
    pinMode(DIPE, INPUT);
}

void loop() {
    bool dA = digitalRead(DIPA);
    bool dB = digitalRead(DIPB);
    bool dC = digitalRead(DIPC);
    bool dD = digitalRead(DIPD);
    bool dE = digitalRead(DIPE);

    int combo = (dE ? 8 : 0) | (dA ? 4 : 0) | (dB ? 2 : 0) | (dC ? 1 : 0);

    static int estadoAnterior = -1;
    int estadoActual = (dA << 0) | (dB << 1) | (dC << 2) | (dD << 3) | (dE << 4);
    if (estadoActual != estadoAnterior) {
        Serial.print("DIPA="); Serial.print(dA);
        Serial.print(" DIPB="); Serial.print(dB);
        Serial.print(" DIPC="); Serial.print(dC);
        Serial.print(" DIPD="); Serial.print(dD);
        Serial.print(" DIPE="); Serial.print(dE);
        Serial.print("   ->   combo="); Serial.println(combo);
        estadoAnterior = estadoActual;
    }

    delay(50);
}

#endif // RUN_PRUEBA_SWITCH
