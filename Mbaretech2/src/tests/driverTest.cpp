#ifdef RUN_DRIVER_TEST
#include <Arduino.h>
#include <math.h>
#include "globals.h"

void loop() {

    //leftMotor.setSpeed(0.9*1024);

    Serial.println("Setting motors to minimum pulse width");
    leftMotor.reverse(75);
    rightMotor.reverse(75);

    delay(2000); // Wait for 2 seconds

    // Test the motors with middle pulse width
    Serial.println("Setting motors to middle pulse width");
    leftMotor.brake();
    rightMotor.brake();

    delay(2000); // Wait for 2 seconds

    // Test the motors with maximum pulse width
    Serial.println("Setting motors to maximum pulse width");
    leftMotor.forward(70);
    rightMotor.forward(70);

    delay(2000); // Wait for 2 seconds
}

#endif //RUN_DRIVER_TEST