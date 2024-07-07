#ifdef RUN_IR_SENSOR_TEST
#include "globals.h"  // Include the header file

void loop() {
    irSensor[SIDE_LEFT] = !digitalRead(IR1); // Side left en short left
    irSensor[SHORT_LEFT] = !digitalRead(IR2);  // Short left no lee
    irSensor[TOP_LEFT] = !digitalRead(IR3); // Ok
    irSensor[TOP_MID] = !digitalRead(IR4); //Short right con top mid swap
    irSensor[TOP_RIGHT] = !digitalRead(IR5); // Top right en side right
    irSensor[SHORT_RIGHT] = !digitalRead(IR6); // En Top mid
    irSensor[SIDE_RIGHT] = !digitalRead(IR7); // Side right en top right

    for (int i = 0; i < 7; i++) {
        Serial.print(irSensor[i]);
        Serial.print("\t");
    }
    Serial.print("\n");
    vTaskDelay(1);

}
#endif  // RUN_IR_SENSOR_TEST