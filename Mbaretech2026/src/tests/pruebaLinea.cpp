#ifdef RUN_PRUEBA_LINEA
#include "globals.h"

// Prueba de los sensores de linea. Tiene setup() propio (no usa el de
// combate de main.cpp). No mueve motores. Necesita RUN_LINE_SENSOR para
// compilar lineSensor.cpp (lectura del ADC y el filtro de combate).
//
// Cada 10ms lee los dos sensores delanteros: LINE_FRONT_LEFT (ADC1 canal 2,
// GPIO3) y LINE_FRONT_RIGHT (ADC1 canal 7, GPIO8). Los traseros
// (LINE_BACK_*) se omiten: no estan instalados en el robot por ahora.
// y cada 1 segundo imprime por Serial una tabla con, por sensor:
//   valor crudo, min/max desde el ultimo reset, medio=(min+max)/2 y si el
//   crudo esta por debajo de THRESHOLD -> "BLANCO"/"negro" (mismo criterio
//   que combate: en este robot la linea blanca da lecturas BAJAS).
// Tambien imprime el booleano filtrado de combate
// (checkLineSensora/b: 7 lecturas seguidas <= THRESHOLD), tanto el valor
// actual como si se activo en algun momento del ultimo segundo (para no
// perder un cruce rapido de la linea entre dos impresiones).
//
// Uso para elegir THRESHOLD: 'r' para resetear min/max, pasar cada sensor
// sobre el negro y sobre el blanco del dohyo, y mirar el "medio".
//
// Comandos por Serial (115200):
//   r -> resetea min/max

#define N_LINEA 2
static const char *nombres[N_LINEA] = {"DEL_IZQ", "DEL_DER"};
static int minimo[N_LINEA];
static int maximo[N_LINEA];
static unsigned long ultimoPrint = 0;
static bool vioLineaIzq = false;  // filtro activo en algun momento desde la ultima impresion
static bool vioLineaDer = false;

static void resetearMinMax() {
    for (int i = 0; i < N_LINEA; i++) {
        minimo[i] = 99999;
        maximo[i] = -1;
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);  // tiempo para abrir el monitor serie
    Serial.println("\n=== PRUEBA SENSORES DE LINEA ===");
    Serial.printf("THRESHOLD actual = %d (linea detectada si crudo <= THRESHOLD)\n", THRESHOLD);

    lineSensorsInit();  // ADC1 canales 2 y 7

    resetearMinMax();
    Serial.println("Comando: r = resetear min/max");
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'r') { resetearMinMax(); Serial.println(">> min/max reseteados"); }
    }

    int crudo[N_LINEA];
    crudo[0] = readLineSensorFront(LINE_FRONT_LEFT);
    crudo[1] = readLineSensorFront(LINE_FRONT_RIGHT);

    // Mismo filtro que combate, llamado en cada ciclo para que el contador
    // de 7 lecturas seguidas avance igual que en tasks.cpp
    bool filtIzq = checkLineSensora(crudo[0]);
    bool filtDer = checkLineSensorb(crudo[1]);
    if (filtIzq) vioLineaIzq = true;
    if (filtDer) vioLineaDer = true;

    for (int i = 0; i < N_LINEA; i++) {
        if (crudo[i] < minimo[i]) minimo[i] = crudo[i];
        if (crudo[i] > maximo[i]) maximo[i] = crudo[i];
    }

    // Se sigue muestreando cada 10ms (el filtro de combate y el min/max lo
    // necesitan), pero solo se imprime una tabla por segundo
    if (millis() - ultimoPrint >= 1000) {
        ultimoPrint = millis();
        Serial.println();
        Serial.printf("---------------- t=%lus  (THRESHOLD=%d) ----------------\n",
                      millis() / 1000, THRESHOLD);
        Serial.println("  Sensor     Crudo    Min    Max  Medio   Ve");
        Serial.println("  ---------  -----  -----  -----  -----   ------");
        for (int i = 0; i < N_LINEA; i++) {
            Serial.printf("  %-9s %6d %6d %6d %6d   %s\n", nombres[i], crudo[i],
                          minimo[i], maximo[i], (minimo[i] + maximo[i]) / 2,
                          crudo[i] <= THRESHOLD ? "BLANCO" : "negro");
        }
        Serial.printf("  Filtro de combate (ahora):        IZQ=%-5s  DER=%s\n",
                      filtIzq ? "LINEA" : "-", filtDer ? "LINEA" : "-");
        Serial.printf("  Filtro de combate (ultimo seg.):  IZQ=%-5s  DER=%s\n",
                      vioLineaIzq ? "LINEA" : "-", vioLineaDer ? "LINEA" : "-");
        vioLineaIzq = false;
        vioLineaDer = false;
    }

    delay(10);
}

#endif // RUN_PRUEBA_LINEA
