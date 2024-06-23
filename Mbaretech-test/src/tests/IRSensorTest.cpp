#ifdef RUN_IR_SENSOR_TEST
#include "globals.h"  // Include the header file

void loop() {
    for (int i = 0; i < 6; i++) {
        Serial.print(sensorReadings[i]);
        Serial.print("\t");
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);

}
#endif  // RUN_IR_SENSOR_TEST