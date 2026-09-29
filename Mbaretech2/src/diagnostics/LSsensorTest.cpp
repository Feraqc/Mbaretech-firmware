#include "firmwareConfig.h"
#if ENABLE_LINE_TEST
#include "globals.h"


void loop(){
    static int reading1 ;
    static int reading2;

    Serial.print(readLineSensorFront(LINE_FRONT_LEFT));
    Serial.print("\t");
    Serial.print(readLineSensorFront(LINE_FRONT_RIGHT));
    Serial.print("\t");
    Serial.print(readLineSensorBack(LINE_BACK_LEFT));
    Serial.print("\t");
    Serial.print(readLineSensorBack(LINE_BACK_RIGHT));
    Serial.print("\t");
    Serial.print("\n");

    delay(10);

}

#endif