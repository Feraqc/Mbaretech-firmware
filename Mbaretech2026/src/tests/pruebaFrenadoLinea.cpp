#ifdef RUN_PRUEBA_FRENADO
#include "globals.h"
#include "bluetoothComm.h"

// Prueba de frenado con sensor de linea, en dos variantes (flag FRENADO_ATRAS
// en platformio.ini):
//
// - ADELANTE (por defecto): avanza a FORWARD_80 (la velocidad real de ataque
//   de tasks.cpp) y apenas un sensor de linea DELANTERO (LS1/LS2) ve blanco,
//   reproduce exactamente la reaccion de combate (LINE_RETREAT: reversa al
//   90% durante 80ms) y frena.
// - ATRAS (FRENADO_ATRAS): lo mismo en espejo -- retrocede a FORWARD_80 y
//   apenas un sensor de linea TRASERO (LS3/LS4) ve blanco, empuja hacia
//   adelante al 90% durante 80ms y frena. El combate todavia no tiene esta
//   reaccion (tasks.cpp no lee los traseros): sirve para medirla antes.
//
// Sirve para ver en el dohyo si esa reaccion alcanza para no caerse, y
// ajustar 80ms/90% si no.
//
// Una prueba por activacion del killswitch (mismo patron que
// calibracion.cpp/pruebaMartillo.cpp): activar -> se mueve -> ve linea ->
// contragolpe -> frena -> reporta y espera a que se suelte el killswitch.
//
// Seguridad: si en TIMEOUT_MS no ve la linea (sensor roto, mal umbral),
// frena igual para no seguir de largo fuera del dohyo.
//
// El resultado se manda por Serial y BLE y se repite cada 1s (para poder
// tener las manos en el killswitch y leerlo despues en el celular).

#define VEL_AVANCE      FORWARD_80   // ataque real en tasks.cpp
#define VEL_REVERSA     FORWARD_90   // igual que LINE_RETREAT
#define TIEMPO_REVERSA  80           // ms, igual que LINE_RETREAT
#define TIMEOUT_MS      1500  // vuelto a 1500 para el dohyo (en banco se habia subido a 5000)
#define LECTURAS_BLANCO 3     // lecturas seguidas <= THRESHOLD para confirmar linea (filtra
                              // picos de ruido de los motores al arrancar). Solo en esta prueba.

#ifdef FRENADO_ATRAS
  #define NOMBRE_PRUEBA "ATRAS (sensores traseros LS3/LS4)"
  // Traseros por ADC2: readLineSensorBack devuelve -1 si la lectura falla
  // (el radio BLE puede bloquear ADC2). -1 se trata como "sin dato", nunca
  // como blanco (ver esBlanco/actualizar).
  static int leerIzq() { return readLineSensorBack(LINE_BACK_LEFT); }
  static int leerDer() { return readLineSensorBack(LINE_BACK_RIGHT); }
  static void moverAvance()      { rightMotor.backward(VEL_AVANCE);  leftMotor.backward(VEL_AVANCE); }
  static void moverContragolpe() { rightMotor.forward(VEL_REVERSA);  leftMotor.forward(VEL_REVERSA); }
#else
  #define NOMBRE_PRUEBA "ADELANTE (sensores delanteros LS1/LS2)"
  static int leerIzq() { return readLineSensorFront(LINE_FRONT_LEFT); }
  static int leerDer() { return readLineSensorFront(LINE_FRONT_RIGHT); }
  static void moverAvance()      { rightMotor.forward(VEL_AVANCE);   leftMotor.forward(VEL_AVANCE); }
  static void moverContragolpe() { rightMotor.backward(VEL_REVERSA); leftMotor.backward(VEL_REVERSA); }
#endif

static bool esBlanco(int crudo) { return crudo >= 0 && crudo <= THRESHOLD; }

// Cuenta lecturas seguidas en blanco; una sola en negro la vuelve a 0. Una
// lectura fallida (-1) no suma ni resetea, solo se cuenta como error.
static void actualizar(int crudo, int &seguidas, int &errores) {
    if (crudo < 0) { errores++; return; }
    seguidas = esBlanco(crudo) ? seguidas + 1 : 0;
}

void IRAM_ATTR FrenadoKS_ISR() { startSignal = digitalRead(START_PIN); }

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n=== PRUEBA FRENADO CON SENSOR DE LINEA -- " NOMBRE_PRUEBA " ===");

    BLE_UART_Init("MBARETECH");

    pinMode(START_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(START_PIN), FrenadoKS_ISR, CHANGE);
    rightMotor.begin();
    leftMotor.begin();

    lineSensorsInit();

    Serial.printf("Movimiento %d%%, contragolpe %d%% x %dms, timeout %dms, THRESHOLD=%d, %d lecturas seguidas\n",
                  VEL_AVANCE, VEL_REVERSA, TIEMPO_REVERSA, TIMEOUT_MS, THRESHOLD, LECTURAS_BLANCO);
    Serial.println("Listo. Activa el killswitch apuntando al borde del dohyo.");
}

void loop() {
    static bool corriendo = false;
    static bool terminado = false;
    static unsigned long tInicio = 0;
    static unsigned long ultimoMicros = 0;
    static unsigned long ultimoRepeat = 0;
    static String resultado = "";
    static float lazoMaxMs = 0;
    static int seguidasIzq = 0;
    static int seguidasDer = 0;
    static int erroresIzq = 0;
    static int erroresDer = 0;

    if (!startSignal) {
        rightMotor.brake();
        leftMotor.brake();
        corriendo = false;
        terminado = false;
        delay(50);
        return;
    }

    if (!corriendo) {
        corriendo = true;
        terminado = false;
        lazoMaxMs = 0;
        seguidasIzq = 0;
        seguidasDer = 0;
        erroresIzq = 0;
        erroresDer = 0;

        // Si ya ve blanco antes de arrancar (ej. en banco, la base clara o muy
        // cerca de los sensores), no moverse: si no, detecta "linea" a los 0ms,
        // hace el contragolpe y frena sin haberse movido nunca.
        int crudoIzq0 = leerIzq();
        int crudoDer0 = leerDer();
        if (esBlanco(crudoIzq0) || esBlanco(crudoDer0)) {
            terminado = true;
            ultimoRepeat = 0;
            resultado = "ARRANCA SOBRE BLANCO, no se mueve (crudo izq=" + String(crudoIzq0)
                      + " der=" + String(crudoDer0) + ", THRESHOLD=" + String(THRESHOLD)
                      + ") -- poner negro debajo de los sensores";
            return;
        }

        Serial.println(">> En marcha...");
        sendData("En marcha...");
        moverAvance();
        tInicio = millis();
        ultimoMicros = micros();
    }

    if (!terminado) {
        unsigned long ahora = micros();
        float lazoMs = (ahora - ultimoMicros) / 1000.0f;
        ultimoMicros = ahora;
        if (lazoMs > lazoMaxMs) lazoMaxMs = lazoMs;

        int crudoIzq = leerIzq();
        int crudoDer = leerDer();
        // Recien con LECTURAS_BLANCO seguidas en blanco se toma como linea.
        actualizar(crudoIzq, seguidasIzq, erroresIzq);
        actualizar(crudoDer, seguidasDer, erroresDer);
        bool izq = seguidasIzq >= LECTURAS_BLANCO;
        bool der = seguidasDer >= LECTURAS_BLANCO;
        unsigned long transcurrido = millis() - tInicio;

        if (izq || der) {
            moverContragolpe();
            delay(TIEMPO_REVERSA);
            rightMotor.brake();
            leftMotor.brake();
            terminado = true;
            ultimoRepeat = 0;
            resultado = "RESULTADO: linea a los " + String(transcurrido) + "ms con "
                      + String(izq && der ? "AMBOS" : (izq ? "IZQ" : "DER"))
                      + " (crudo izq=" + String(crudoIzq) + " der=" + String(crudoDer)
                      + ", THRESHOLD=" + String(THRESHOLD) + ") lazoMax="
                      + String(lazoMaxMs, 2) + "ms errores=" + String(erroresIzq) + "/"
                      + String(erroresDer) + " -- contragolpe " + String(VEL_REVERSA)
                      + "% x " + String(TIEMPO_REVERSA) + "ms hecho";
        } else if (transcurrido >= TIMEOUT_MS) {
            rightMotor.brake();
            leftMotor.brake();
            terminado = true;
            ultimoRepeat = 0;
            resultado = "RESULTADO: NO vio la linea en " + String(TIMEOUT_MS)
                      + "ms, frenado por seguridad (ultimo crudo izq=" + String(crudoIzq)
                      + " der=" + String(crudoDer) + ", errores=" + String(erroresIzq)
                      + "/" + String(erroresDer) + ")";
        }
    } else {
        if (millis() - ultimoRepeat >= 1000) {
            ultimoRepeat = millis();
            Serial.println(">> " + resultado);
            sendData(resultado);
        }
        delay(10);
    }
}

#endif // RUN_PRUEBA_FRENADO
