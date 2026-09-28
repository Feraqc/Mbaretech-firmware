#ifdef RUN_GYRO_TEST
#include "globals.h"

void loop() {
    if(imu.getData()){
        Serial.println(imu.currentAngle);
    }

    delay(10);
}

#endif  // RUN_GYRO_TEST