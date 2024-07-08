#ifdef RUN_SENSORS_TEST
#include "globals.h"  // Include the header file

void loop() {
    irSensor[LEFT] = !digitalRead(IR1); // Side left en short left
    irSensor[MID] = !digitalRead(IR2);  // Short left no lee
    irSensor[RIGHT] = !digitalRead(IR3); // Ok

    dipSwitch[0] = digitalRead(DIPA);
    dipSwitch[1] = digitalRead(DIPB);
    dipSwitch[2] = digitalRead(DIPC);
    dipSwitch[3] = digitalRead(DIPD);

    int line_left = readLineSensor(LINE_FRONT_LEFT);
    int line_right = readLineSensor(LINE_FRONT_RIGHT);

    lineSensor[0] = checkLineSensor(line_left);
    lineSensor[1] = checkLineSensor(line_right);
    

    for (int i = 0; i < 3; i++) {
        Serial.print(irSensor[i]);
        Serial.print("\t");
    }

    Serial.print("\t");
    Serial.print("\t");

    for (int i = 0; i < 4; i++) {
        Serial.print(dipSwitch[i]);
        Serial.print("\t");
    }

    Serial.print("\t");
    Serial.print("\t");

    Serial.print(line_left);
    Serial.print("\t");
    Serial.print(line_right);
    Serial.print("\t");
    Serial.print(lineSensor[0]);
    Serial.print(lineSensor[1]);

    Serial.print("\n");
    vTaskDelay(200);

}
#endif  // RUN_IR_SENSOR_TEST