#ifdef RUN_PRUEBA_MARTILLO
#include "globals.h"
#include "bluetoothComm.h"
#include <Wire.h>

// Prueba para el futuro "modo martillo" (tasks.cpp, todavia no implementado):
// la idea de combate es que si el robot empuja al sumo y NO logra avanzar
// (trabado en un empuje parejo), convenga retroceder un poco y volver a
// embestir en vez de seguir empujando en el lugar. El problema fisico: la
// aceleracion es ~0 tanto si esta trabado como si se mueve a velocidad
// constante, asi que no alcanza con mirar |a|. Lo que si sirve es integrar
// la aceleracion hacia adelante durante una ventana corta y acotada al
// arrancar el empuje, para estimar cuanta velocidad gano el robot en ese
// lapso -- si gano poca, esta trabado.
//
// Este archivo SOLO mide, no implementa el martillo (no retrocede ni
// reintenta). Uso: correrlo una vez empujando contra algo pesado/trabado
// y otra vez libre (nada adelante), anotar los resultados impresos (ahora
// una curva completa, ver checkpoints abajo), y con eso definir la ventana
// y el umbral real para tasks.cpp.
//
// Eje: ay es adelante/atras. Signo CORREGIDO 2026-10-01 con empujes reales
// de motor: motor.forward() da ay NEGATIVO (el empuje libre de 300ms da
// ~-0.23 g*s ~ 2.3 m/s, coherente con lo que recorre en el dohyo de 154cm).
// La prueba a mano con pruebaIMU.cpp habia dado lo contrario -- el "adelante"
// del usuario (la pala) no coincide con el sentido de motor.forward().
// Entonces: ay negativo = acelera hacia donde empuja el motor, ay positivo =
// frena (ej. el golpe contra el rival/pared). IMU montado plano (az ~ -1g).
//
// Uso: activar el killswitch -> empuja a FORWARD_80% durante VENTANA_MS
// (= CHECKPOINT_MS * NUM_CHECKPOINTS) integrando ay*dt cada vuelta,
// guardando la velocidad acumulada en cada checkpoint (cada CHECKPOINT_MS
// ms) -> al terminar, frena e imprime TODA la curva (una velocidad por
// checkpoint, no un solo numero) -> no vuelve a empujar hasta soltar y
// volver a activar el killswitch (mismo modelo de "una medicion por
// activacion" que el de 3 etapas de calibracion.cpp, para no seguir
// chocando contra la pared en loop). Se cambio de "una ventana fija" a
// "una curva de checkpoints" el 2026-09-30 porque una prueba trabada contra
// la pared dio IGUAL que libre con una sola ventana de 300ms -- no
// concluyente, posiblemente porque esa ventana era muy corta para que
// libre y trabado diverjan. Con la curva completa se puede ver en que
// punto (si alguno) empiezan a separarse, sin tener que reflashear con
// cada duracion distinta para probar.
//
// Velocidad de empuje = FORWARD_80 (2026-09-30, antes usaba parametros[2]):
// el umbral que salga de esta prueba solo sirve si se mide en las mismas
// condiciones que el ataque real de tasks.cpp -- ahi el estado FORWARD
// empuja a local_speed=FORWARD_80 (80%), NO a parametros[2] (ese indice no
// lo lee tasks.cpp para nada, solo lo usaban movements.cpp/TEST_FORWARD).
// Nota aparte: si ademas los dos sensores cortos (SHORT_LEFT y SHORT_RIGHT)
// detectan al mismo tiempo, tasks.cpp asume contacto solido y sube a
// MAX_SPEED (100%) -- esta prueba mide el caso general de empuje a 80%, no
// ese caso de contacto confirmado a full potencia.
//
// Registros/funciones MPU6050 duplicados de pruebaIMU.cpp a proposito --
// mismo criterio que el resto de los archivos de prueba de este proyecto:
// cada uno autocontenido, sin compartir codigo entre tests.
//
// BLE (agregado 2026-09-30): el empuje real hay que hacerlo con las manos
// libres para poder soltar el killswitch rapido si el robot se va del
// dohyo -- no da tiempo a mirar un cable/laptop. El resultado ahora TAMBIEN
// se manda por BLE (mismo servicio/UART que calibracion.cpp, sendData()),
// asi que con el celular emparejado y una app de terminal BLE conectada de
// antemano, el usuario puede tener las manos en el killswitch y el
// historial de resultados va quedando en el log de la app del celular.

#define MPU_ADDR_A      0x68
#define MPU_ADDR_B      0x69
#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define REG_ACCEL_XOUT  0x3B   // 14 bytes: acel XYZ, temp, giro XYZ
#define REG_PWR_MGMT_1  0x6B
#define REG_WHO_AM_I    0x75

// +-16g (antes +-8g): un choque a ~2 m/s que frena en pocos ms son 30-40g,
// con +-8g el sensor saturaba y la integral perdia casi todo el frenazo --
// por eso la prueba contra la pared (con espacio, choque incluido) daba
// igual que libre (2026-10-01).
#define ACCEL_LSB_POR_G   2048.0f

// En vez de una sola ventana fija (300/500ms probados antes, un resultado
// trabado contra pared dio IGUAL que libre -- no concluyente, posiblemente
// porque la ventana era muy corta para que se note la diferencia), se
// reportan varios puntos en el tiempo de UN mismo empuje: asi se ve toda la
// curva de "velocidad estimada" sin tener que reflashear con cada ventana
// distinta para probar.
// Total 300ms (6 x 50ms): con 300ms el robot ya cruza mas de medio dohyo,
// 600ms (6 x 100ms, version anterior) lo sacaba afuera.
#define CHECKPOINT_MS 50
#define NUM_CHECKPOINTS 6
#define VENTANA_MS (CHECKPOINT_MS * NUM_CHECKPOINTS)  // duracion total del empuje de prueba

static uint8_t mpuAddr = 0;

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
        if (leerRegs(REG_WHO_AM_I, &who, 1)) {
            Serial.printf("IMU en 0x%02X, WHO_AM_I=0x%02X\n", mpuAddr, who);
            return true;
        }
    }
    mpuAddr = 0;
    return false;
}

static bool configurarIMU() {
    bool ok = true;
    ok &= escribirReg(REG_PWR_MGMT_1, 0x01);  // despertar, reloj = PLL giro X
    delay(100);
    ok &= escribirReg(REG_SMPLRT_DIV, 0x00);
    ok &= escribirReg(REG_CONFIG, 0x01);      // DLPF ~184Hz (antes 44Hz, aplastaba el pico del golpe)
    ok &= escribirReg(REG_GYRO_CFG, 0x18);    // +-2000 dps (no se usa aca, pero deja el sensor en un estado conocido)
    ok &= escribirReg(REG_ACCEL_CFG, 0x18);   // +-16g
    return ok;
}

static bool leerAy(float *ayg) {
    uint8_t b[14];
    if (!leerRegs(REG_ACCEL_XOUT, b, 14)) return false;
    int16_t ay = (int16_t)(b[2] << 8 | b[3]);
    *ayg = ay / ACCEL_LSB_POR_G;
    return true;
}

void IRAM_ATTR MartilloKS_ISR() { startSignal = digitalRead(START_PIN); }

static bool imuOk = false;

// Intenta detectar+configurar el IMU UNA vez, sin bloquear. Se llama en
// setup() y, si falla ahi, se reintenta solo desde loop() -- asi un IMU que
// tarda en responder (o que no esta conectado) nunca impide que el
// killswitch/motores funcionen, que es lo critico de esta prueba.
static void intentarIMU() {
    if (detectarIMU()) {
        if (!configurarIMU()) {
            Serial.println("ERROR: fallo la escritura de configuracion del IMU");
        }
        imuOk = true;
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n=== PRUEBA MARTILLO (empuje + medicion de velocidad estimada) ===");

    BLE_UART_Init("MBARETECH");

    // Killswitch y motores PRIMERO, sin depender de que el IMU responda --
    // antes este bloque estaba despues de un "while (!detectarIMU())" sin
    // limite de reintentos, asi que si el IMU no se detectaba a la primera
    // (ruido I2C, timing al arrancar) el setup() se quedaba trabado ahi para
    // siempre y el killswitch/motores NUNCA se llegaban a inicializar --
    // bug encontrado 2026-09-30, el robot no respondia a nada.
    pinMode(START_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(START_PIN), MartilloKS_ISR, CHANGE);
    rightMotor.begin();
    leftMotor.begin();

    Wire.begin(SDA_PIN, SCL_PIN, 100000);
    Wire.setTimeOut(20);
    intentarIMU();
    if (!imuOk) {
        Serial.println("No se encontro el IMU todavia -- se sigue reintentando en loop(),");
        Serial.println("el killswitch y los motores ya funcionan igual.");
    }

    Serial.println("Listo. Activa el killswitch para correr UN empuje de prueba.");
    Serial.printf("Empuje: FORWARD_80=%d%% (velocidad real de ataque en tasks.cpp), duracion total=%dms, checkpoints cada %dms\n", FORWARD_80, VENTANA_MS, CHECKPOINT_MS);
    Serial.println("Podes desconectar el USB para hacer el empuje (corre a bateria) --");
    Serial.println("el resultado se repite cada 1s despues del empuje, asi que aparece");
    Serial.println("apenas reconectes y reabras el monitor.");
    Serial.println("O conecta el celular por BLE (app de terminal UART) ANTES de");
    Serial.println("empujar -- el resultado tambien se manda por ahi, para tener las");
    Serial.println("manos libres en el killswitch.");
}

void loop() {
    static bool corriendo = false;   // etapa 2 activa (empujando o ya midio, esperando soltar)
    static bool yaMidio = false;     // ya se corto el empuje y se imprimio el resultado
    static float velEstimada = 0;
    static unsigned long tInicio = 0;
    static unsigned long ultimoMicros = 0;
    static unsigned long ultimoRepeat = 0;
    static unsigned long ultimoIntentoIMU = 0;
    static float checkpoints[NUM_CHECKPOINTS];
    static int proximoCheckpoint = 0;
    static float picoFrenada = 0;   // ay mas positivo visto = golpe/frenazo
    static float picoEmpuje = 0;    // ay mas negativo visto = arranque
    static int muestras = 0;
    static int saltos = 0;          // vueltas con dt > 10ms, no integradas
    static float dtMaxMs = 0;

    if (!imuOk && millis() - ultimoIntentoIMU >= 1000) {
        ultimoIntentoIMU = millis();
        intentarIMU();
        if (imuOk) Serial.println("IMU detectado.");
    }

    if (!startSignal) {
        rightMotor.brake();
        leftMotor.brake();
        corriendo = false;
        yaMidio = false;
        delay(50);
        return;
    }

    if (!corriendo) {
        // Flanco de subida: arranca un empuje de prueba nuevo.
        corriendo = true;
        yaMidio = false;
        velEstimada = 0;
        proximoCheckpoint = 0;
        picoFrenada = 0;
        picoEmpuje = 0;
        muestras = 0;
        saltos = 0;
        dtMaxMs = 0;
        // Los avisos van ANTES de arrancar el reloj: el sendData() por BLE
        // tarda varios ms y antes quedaba metido en el primer dt, que se
        // multiplicaba por el sacudon del arranque y dominaba toda la
        // integral (bug 2026-10-01: dos corridas libres dieron +0.18 y -0.18).
        Serial.println(">> Empuje iniciado...");
        sendData("Empuje iniciado...");
        rightMotor.forward(FORWARD_80);
        leftMotor.forward(FORWARD_80);
        tInicio = millis();
        ultimoMicros = micros();
    }

    if (!yaMidio) {
        float ayg;
        unsigned long ahora = micros();
        float dt = (ahora - ultimoMicros) / 1e6f;
        ultimoMicros = ahora;

        if (dt * 1000.0f > dtMaxMs) dtMaxMs = dt * 1000.0f;

        if (leerAy(&ayg)) {
            // Una vuelta lenta (I2C trabado, etc.) no se integra: multiplicar
            // una sola lectura por un dt largo inventa velocidad.
            if (dt <= 0.010f) velEstimada += ayg * dt;
            else saltos++;
            if (ayg > picoFrenada) picoFrenada = ayg;
            if (ayg < picoEmpuje) picoEmpuje = ayg;
            muestras++;
            // No imprime nada durante el empuje -- ver nota abajo sobre por
            // que el resultado se repite DESPUES en vez de mostrarse en vivo.
        }

        unsigned long transcurrido = millis() - tInicio;
        while (proximoCheckpoint < NUM_CHECKPOINTS &&
               transcurrido >= (unsigned long)(proximoCheckpoint + 1) * CHECKPOINT_MS) {
            checkpoints[proximoCheckpoint] = velEstimada;
            proximoCheckpoint++;
        }

        if (proximoCheckpoint >= NUM_CHECKPOINTS) {
            rightMotor.brake();
            leftMotor.brake();
            yaMidio = true;
            ultimoRepeat = 0; // fuerza el primer print del resultado ya mismo
        }
    } else {
        // El empuje real es demasiado rapido/violento para leerlo en vivo
        // con el cable puesto -- la idea es hacer la prueba con el robot
        // suelto (a bateria, sin USB) y reconectar el Serial DESPUES para
        // ver el resultado. Como el ESP32-S3 sigue corriendo con la
        // bateria aunque se desconecte el USB, el resultado no se pierde:
        // se repite cada 1s mientras se espera que se suelte el killswitch,
        // asi que aparece apenas se reabre el monitor, sin apurar el timing.
        if (millis() - ultimoRepeat >= 1000) {
            ultimoRepeat = millis();
            String resultado = "RESULTADO vel.estimada: ";
            for (int i = 0; i < NUM_CHECKPOINTS; i++) {
                resultado += String((i + 1) * CHECKPOINT_MS) + "ms=" + String(checkpoints[i], 3) + " ";
            }
            resultado += "| picoFrenada=" + String(picoFrenada, 2) + "g picoEmpuje=" + String(picoEmpuje, 2)
                       + "g muestras=" + String(muestras)
                       + " saltos=" + String(saltos) + " dtMax=" + String(dtMaxMs, 1) + "ms"
                       + " -- solta y reactiva el killswitch para otra medicion";
            Serial.println(">> " + resultado);
            sendData(resultado);
        }
    }

    // Sin delay mientras empuja: un golpe dura pocos ms y con delay(10)
    // (~11ms por lectura) se lo salteaba.
    if (yaMidio) delay(10);
}

#endif // RUN_PRUEBA_MARTILLO
