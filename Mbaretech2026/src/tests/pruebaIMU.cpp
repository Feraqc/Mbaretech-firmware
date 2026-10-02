#ifdef RUN_PRUEBA_IMU
#include "globals.h"
#include <Wire.h>

// Prueba basica del IMU (MPU6050 o compatible) por registros directos,
// SIN librerias externas (ni I2Cdev ni MPU6050/DMP) -- asi, si algo falla,
// se sabe que es cableado/hardware y no la libreria.
//
// Pasos:
//   1. Escanea el bus I2C en SDA_PIN/SCL_PIN (15/16, ver globals.h) y lista
//      todas las direcciones que responden.
//   2. Busca el IMU en 0x68 (AD0=LOW) o 0x69 (AD0=HIGH) y lee WHO_AM_I
//      para identificar el chip.
//   3. Lo despierta y configura: giro +-2000 dps, acel +-8g, filtro 44Hz.
//   4. Calibra el bias del giroscopio (ROBOT QUIETO durante ~2s).
//   5. Lee a ~100Hz e imprime cada 100ms: aceleracion (g), modulo |a|,
//      pico de |a| desde la ultima impresion (util para ver choques),
//      velocidad angular (dps), yaw integrado desde gz (grados) y temperatura.
//
// Comandos por Serial (115200):
//   z -> pone el yaw en 0
//   c -> recalibra el bias del giroscopio (dejar el robot quieto)
//
// Nota: el yaw se integra desde el eje Z del giroscopio, asumiendo el IMU
// montado plano. Si al girar el robot sobre el piso el que cambia es gx o
// gy, el IMU esta montado de canto y hay que integrar ese eje en su lugar.
// Sin magnetometro el yaw deriva lentamente con el tiempo -- es normal.
//
// Motores: no se tocan en esta prueba.

// Registros del MPU6050
#define MPU_ADDR_A      0x68
#define MPU_ADDR_B      0x69
#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define REG_ACCEL_XOUT  0x3B   // 14 bytes: acel XYZ, temp, giro XYZ
#define REG_PWR_MGMT_1  0x6B
#define REG_WHO_AM_I    0x75

#define ACCEL_LSB_POR_G   4096.0f  // +-8g
#define GYRO_LSB_POR_DPS  16.4f    // +-2000 dps

static uint8_t mpuAddr = 0;
static float biasGx = 0, biasGy = 0, biasGz = 0;
static float yaw = 0;
static float picoAcel = 0;
static unsigned long ultimoMicros = 0;
static unsigned long ultimoPrint = 0;

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

// Lee el nivel de SDA/SCL ANTES de arrancar Wire. En reposo, un bus I2C
// sano esta en HIGH (por las pull-up). Si sin pull-up interna leen LOW,
// no hay pull-up externa (o el IMU no esta alimentado); si leen LOW aun
// con pull-up interna, la linea esta en corto a GND o algo la retiene.
static void diagnosticarLineas() {
    pinMode(SDA_PIN, INPUT);
    pinMode(SCL_PIN, INPUT);
    delay(5);
    int sdaSin = digitalRead(SDA_PIN), sclSin = digitalRead(SCL_PIN);
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
    delay(5);
    int sdaCon = digitalRead(SDA_PIN), sclCon = digitalRead(SCL_PIN);
    Serial.printf("Lineas en reposo -- sin pull-up interna: SDA=%d SCL=%d | "
                  "con pull-up interna: SDA=%d SCL=%d\n", sdaSin, sclSin, sdaCon, sclCon);
    if (sdaCon == 0 || sclCon == 0) {
        Serial.println("  ATENCION: linea en LOW aun con pull-up -> corto a GND o dispositivo trabando el bus");
    } else if (sdaSin == 0 || sclSin == 0) {
        Serial.println("  ATENCION: sin pull-up externa (o IMU sin alimentacion) -- se usan las internas, debiles");
    }
}

// Recuperacion de bus trabado: si un esclavo quedo a mitad de un byte
// (p.ej. el ESP se reseteo pero el IMU no), sigue reteniendo SDA en LOW.
// Se le dan hasta 9 pulsos de SCL para que termine el byte y suelte SDA,
// y luego una condicion STOP manual.
static void recuperarBus() {
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, OUTPUT_OPEN_DRAIN);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(10);
    int pulsos = 0;
    while (digitalRead(SDA_PIN) == LOW && pulsos < 9) {
        digitalWrite(SCL_PIN, LOW);
        delayMicroseconds(10);
        digitalWrite(SCL_PIN, HIGH);
        delayMicroseconds(10);
        pulsos++;
    }
    // STOP: SDA sube mientras SCL esta en HIGH
    pinMode(SDA_PIN, OUTPUT_OPEN_DRAIN);
    digitalWrite(SDA_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(SDA_PIN, HIGH);
    delayMicroseconds(10);
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
    delay(1);
    Serial.printf("Recuperacion de bus: %d pulsos de SCL -> ahora SDA=%d SCL=%d\n",
                  pulsos, digitalRead(SDA_PIN), digitalRead(SCL_PIN));
}

static void escanearI2C() {
    Serial.printf("Escaneando I2C (SDA=%d, SCL=%d)...\n", SDA_PIN, SCL_PIN);
    int encontrados = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.printf("  dispositivo en 0x%02X\n", addr);
            encontrados++;
        } else if (err != 2) {
            // 2 = NACK de direccion (normal: no hay nada ahi). Otro codigo
            // (4/5 = error de bus/timeout) indica un problema electrico.
            Serial.printf("  0x%02X: error de bus %d\n", addr, err);
        }
        if (addr % 32 == 0) Serial.printf("  ...hasta 0x%02X\n", addr);
    }
    if (encontrados == 0) {
        Serial.println("  NINGUN dispositivo -- revisar alimentacion, SDA/SCL invertidos, pull-ups");
    }
}

static bool detectarIMU() {
    uint8_t candidatos[2] = {MPU_ADDR_A, MPU_ADDR_B};
    for (uint8_t i = 0; i < 2; i++) {
        mpuAddr = candidatos[i];
        uint8_t who;
        if (leerRegs(REG_WHO_AM_I, &who, 1)) {
            const char *chip = "desconocido/clon";
            if (who == 0x68) chip = "MPU6050";
            else if (who == 0x70) chip = "MPU6500";
            else if (who == 0x71) chip = "MPU9250";
            else if (who == 0x73) chip = "MPU9255";
            Serial.printf("IMU en 0x%02X, WHO_AM_I=0x%02X (%s)\n", mpuAddr, who, chip);
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
    ok &= escribirReg(REG_CONFIG, 0x03);      // DLPF ~44Hz
    ok &= escribirReg(REG_GYRO_CFG, 0x18);    // +-2000 dps
    ok &= escribirReg(REG_ACCEL_CFG, 0x10);   // +-8g
    return ok;
}

static bool leerCrudo(int16_t *ax, int16_t *ay, int16_t *az,
                      int16_t *temp, int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t b[14];
    if (!leerRegs(REG_ACCEL_XOUT, b, 14)) return false;
    *ax   = (int16_t)(b[0] << 8 | b[1]);
    *ay   = (int16_t)(b[2] << 8 | b[3]);
    *az   = (int16_t)(b[4] << 8 | b[5]);
    *temp = (int16_t)(b[6] << 8 | b[7]);
    *gx   = (int16_t)(b[8] << 8 | b[9]);
    *gy   = (int16_t)(b[10] << 8 | b[11]);
    *gz   = (int16_t)(b[12] << 8 | b[13]);
    return true;
}

static void calibrarGiro() {
    Serial.println("Calibrando giroscopio -- NO mover el robot...");
    const int N = 1000;
    long sx = 0, sy = 0, sz = 0;
    int validas = 0;
    for (int i = 0; i < N; i++) {
        int16_t ax, ay, az, t, gx, gy, gz;
        if (leerCrudo(&ax, &ay, &az, &t, &gx, &gy, &gz)) {
            sx += gx; sy += gy; sz += gz;
            validas++;
        }
        delay(2);
    }
    if (validas == 0) {
        Serial.println("ERROR: ninguna lectura valida durante la calibracion");
        return;
    }
    biasGx = (float)sx / validas;
    biasGy = (float)sy / validas;
    biasGz = (float)sz / validas;
    yaw = 0;
    ultimoMicros = micros();
    Serial.printf("Bias giro (crudo): gx=%.1f gy=%.1f gz=%.1f  (%d muestras)\n",
                  biasGx, biasGy, biasGz, validas);
}

void setup() {
    Serial.begin(115200);
    delay(1500);  // tiempo para abrir el monitor serie
    Serial.println("\n=== PRUEBA IMU ===");

    diagnosticarLineas();
    recuperarBus();
    // 100kHz para el diagnostico (mas tolerante a pull-ups debiles)
    Wire.begin(SDA_PIN, SCL_PIN, 100000);
    Wire.setTimeOut(20);  // ms por transaccion -- que un bus trabado no cuelgue todo
    escanearI2C();

    while (!detectarIMU()) {
        Serial.println("No se encontro IMU en 0x68/0x69. Reintentando en 2s...");
        delay(2000);
        escanearI2C();
    }

    if (!configurarIMU()) {
        Serial.println("ERROR: fallo la escritura de configuracion");
    }
    calibrarGiro();
    Serial.println("Comandos: z = yaw a 0, c = recalibrar giro");
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'z') { yaw = 0; Serial.println(">> yaw = 0"); }
        if (c == 'c') calibrarGiro();
    }

    int16_t ax, ay, az, t, gx, gy, gz;
    if (!leerCrudo(&ax, &ay, &az, &t, &gx, &gy, &gz)) {
        Serial.println("ERROR de lectura I2C");
        delay(500);
        return;
    }

    unsigned long ahora = micros();
    float dt = (ahora - ultimoMicros) / 1e6f;
    ultimoMicros = ahora;

    float axg = ax / ACCEL_LSB_POR_G;
    float ayg = ay / ACCEL_LSB_POR_G;
    float azg = az / ACCEL_LSB_POR_G;
    float modulo = sqrtf(axg * axg + ayg * ayg + azg * azg);
    if (modulo > picoAcel) picoAcel = modulo;

    float gxd = (gx - biasGx) / GYRO_LSB_POR_DPS;
    float gyd = (gy - biasGy) / GYRO_LSB_POR_DPS;
    float gzd = (gz - biasGz) / GYRO_LSB_POR_DPS;
    yaw += gzd * dt;

    float tempC = t / 340.0f + 36.53f;  // formula del MPU6050

    if (millis() - ultimoPrint >= 100) {
        ultimoPrint = millis();
        Serial.printf("ax=%6.2f ay=%6.2f az=%6.2f |a|=%5.2f pico=%5.2f  "
                      "gx=%8.1f gy=%8.1f gz=%8.1f  yaw=%8.1f  T=%.1fC\n",
                      axg, ayg, azg, modulo, picoAcel, gxd, gyd, gzd, yaw, tempC);
        picoAcel = 0;
    }

    delay(10);  // ~100Hz
}

#endif // RUN_PRUEBA_IMU
