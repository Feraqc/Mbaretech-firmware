#include "firmwareConfig.h"
#if ENABLE_GYRO_TEST
#include "globals.h"

void loop() {
    static uint32_t lastSample = 0, lastPacket = millis();
    static bool reportedError = false;
    const uint32_t now = millis();
    if (!imu.isReady()) {
        if (!reportedError) {
            Serial.print("IMU,ERROR_INICIALIZACION,");
            Serial.println(imu.getInitError());
            reportedError = true;
        }
    } else {
        if (imu.getData()) {
            lastPacket = now;
            if (uint32_t(now - lastSample) >= 100) {
                lastSample = now;
                Serial.print("YAW,");
                Serial.print(now);
                Serial.print(",");
                Serial.println(imu.currentAngle);
            }
        }
        if (uint32_t(now - lastPacket) >= 1000 && uint32_t(now - lastSample) >= 1000) {
            lastSample = now;
            Serial.println("IMU,SIN_PAQUETES_RECIENTES_DMP");
        }
    }
    delay(10);
}
#endif
