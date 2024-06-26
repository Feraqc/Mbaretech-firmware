#ifdef RUN_DRIVER_TEST
#include "globals.h"  // Include the header file

void loop() {
    // Test the motors with minimum pulse width
    Serial.println("Setting motors to minimum pulse width");
    ledcWrite(leftMotorChannel, usToDutyCycle(MIN_PULSE_WIDTH));
    ledcWrite(rightMotorChannel, usToDutyCycle(MIN_PULSE_WIDTH));
    delay(2000); // Wait for 2 seconds

    // Test the motors with middle pulse width
    Serial.println("Setting motors to middle pulse width");
    ledcWrite(leftMotorChannel, usToDutyCycle(MID_PULSE_WIDTH));
    ledcWrite(rightMotorChannel, usToDutyCycle(MID_PULSE_WIDTH));
    delay(2000); // Wait for 2 seconds

    // Test the motors with maximum pulse width
    Serial.println("Setting motors to maximum pulse width");
    ledcWrite(leftMotorChannel, usToDutyCycle(MAX_PULSE_WIDTH));
    ledcWrite(rightMotorChannel, usToDutyCycle(MAX_PULSE_WIDTH));
    delay(2000); // Wait for 2 seconds
}


#endif  // RUN_GYRO_TEST
