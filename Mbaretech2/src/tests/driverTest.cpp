#ifdef RUN_DRIVER_TEST
#include <Arduino.h>
#include <math.h>
#include "globals.h"

void loop() {

    //leftMotor.setSpeed(0.9*1024);

    Serial.println("Setting motors to 20 backward");
    leftMotor.backward(20);
    rightMotor.backward(20);

    delay(2000); // Wait for 2 seconds

    // Test the motors with middle pulse width
    Serial.println("Setting motors to brake");
    leftMotor.brake();
    rightMotor.brake();

    delay(2000); // Wait for 2 seconds

    // Test the motors with maximum pulse width
    Serial.println("Setting motors to 20 forward");
    leftMotor.forward(20);
    rightMotor.forward(20);

    delay(2000); // Wait for 2 seconds

    // Test the motors with middle pulse width
    Serial.println("Setting motors to brake");
    leftMotor.brake();
    rightMotor.brake();

    delay(2000); // Wait for 2 seconds
}

#endif //RUN_DRIVER_TEST