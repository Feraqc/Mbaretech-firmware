#ifdef RUN_BORRAR
// Prueba mínima del ESP32: imprime "Hola mundo" por Serial cada 1 segundo.
// Tiene su propio setup()/loop(); el setup() de combate de main.cpp no
// compila en este modo. Archivo temporal, se puede borrar.

#include <Arduino.h>

void setup() {
    Serial.begin(115200);
}

void loop() {
    Serial.println("Hola mundo");
    delay(1000);
}

#endif // RUN_BORRAR
