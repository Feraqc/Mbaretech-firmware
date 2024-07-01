#ifdef RUN_MOVEMENTS_TEST
#include <Arduino.h>
#include "globals.h"

#define WAIT_TIME 2000

// Define movement options
enum Movement {
    FORWARD,
    LEFT_TURN_30,
    LEFT_TURN_45,
    LEFT_TURN_90,
    LEFT_TURN_180,
    RIGHT_TURN_30,
    RIGHT_TURN_45,
    RIGHT_TURN_90,
    RIGHT_TURN_180,
    BRAKE
};

// Define the movement to test
Movement testMovement = FORWARD;


void loop() {
    // Empty loop since we are using setup for testing
    // Perform the test movement
    switch (testMovement) {
        case FORWARD:
            startForward(30); // Move forward at 50% speed
            vTaskDelay(pdMS_TO_TICKS(500));
            startForward(0); // APAGAR RAPIDO SI SE HACE ASI
            vTaskDelay(pdMS_TO_TICKS(WAIT_TIME));
            break;
        case LEFT_TURN_30:
            startLeftTurn(TICKS_FOR_30_DEGREES); // 30-degree left turn
            vTaskDelay(pdMS_TO_TICKS(WAIT_TIME));
            break;
        case LEFT_TURN_45:
            startLeftTurn(TICKS_FOR_45_DEGREES); // 45-degree left turn
            vTaskDelay(pdMS_TO_TICKS(WAIT_TIME));
            break;
        case LEFT_TURN_90:
            startLeftTurn(TICKS_FOR_90_DEGREES); // 90-degree left turn
            vTaskDelay(pdMS_TO_TICKS(WAIT_TIME));
            break;
        case LEFT_TURN_180:
            startLeftTurn(TICKS_FOR_180_DEGREES); // 180-degree left turn
            vTaskDelay(pdMS_TO_TICKS(WAIT_TIME));
            break;
        case RIGHT_TURN_30:
            startRightTurn(TICKS_FOR_30_DEGREES); // 30-degree right turn
            vTaskDelay(pdMS_TO_TICKS(WAIT_TIME));
            break;
        case RIGHT_TURN_45:
            startRightTurn(TICKS_FOR_45_DEGREES); // 45-degree right turn
            vTaskDelay(pdMS_TO_TICKS(WAIT_TIME));
            break;
        case RIGHT_TURN_90:
            startRightTurn(TICKS_FOR_90_DEGREES); // 90-degree right turn
            vTaskDelay(pdMS_TO_TICKS(WAIT_TIME));
            break;
        case RIGHT_TURN_180:
            startRightTurn(TICKS_FOR_180_DEGREES); // 180-degree right turn
            vTaskDelay(pdMS_TO_TICKS(WAIT_TIME));
            break;
        case BRAKE:
            leftMotor.brake();
            rightMotor.brake();
            break;
    }
}

#endif   // RUN_MOVEMENTS_TEST
