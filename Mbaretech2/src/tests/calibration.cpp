#ifdef RUN_TURN_CALIBRATION

#include <Arduino.h>

#include "globals.h"
#include "IMU.h"
#include "motor.h"
#include "bluetoothComm.h"

// ============================================================
// TURN CALIBRATION CONFIGURATION
// ============================================================

// Initial delay used at the beginning of every calibration routine.
static constexpr uint32_t BASE_TURN_DELAY_MS = 300;

// Delay correction gain:
// newDelay = currentDelay + KP_DELAY * angleError
//
// Example:
// Target   = 90 deg
// Measured = 80 deg
// Error    = +10 deg
// KP       = 2 ms/deg
// New delay = old delay + 20 ms
static constexpr float KP_DELAY = 2.0f;

// Safety limits for automatic delay adjustment.
static constexpr uint32_t MIN_TURN_DELAY_MS = 10;
static constexpr uint32_t MAX_TURN_DELAY_MS = 1500;

// Time allowed for the robot to mechanically settle after braking.
static constexpr uint32_t SETTLING_TIME_MS = 250;

// Time between calibration turns.
static constexpr uint32_t BETWEEN_TURNS_MS = 1500;

// Number of iterations performed for each calibration command.
static constexpr uint8_t CALIBRATION_TURNS = 8;

// Turn speed.
// Replace these with the production turn-speed constants if appropriate.
static constexpr int TURN_SPEED_LEFT  = 94;
static constexpr int TURN_SPEED_RIGHT = 94;


// ============================================================
// TURN DIRECTION
// ============================================================

enum class TurnDirection {
    LEFT,
    RIGHT
};


// ============================================================
// ANGLE UTILITIES
// ============================================================

/**
 * @brief Calculate shortest signed angular difference.
 *
 * Result is normalized to [-180, 180].
 *
 * Example:
 * before = 170
 * after  = -100
 *
 * Raw difference = -270
 * Normalized     = +90
 */
static float angleDifference(float after, float before)
{
    float difference = after - before;

    while (difference > 180.0f) {
        difference -= 360.0f;
    }

    while (difference < -180.0f) {
        difference += 360.0f;
    }

    return difference;
}


/**
 * @brief Wait until the IMU provides a valid yaw measurement.
 */
static float readYaw(IMU &imu)
{
    while (!imu.getData()) {
        delay(5);
    }

    return imu.currentAngle;
}


// ============================================================
// MOTOR CONTROL
// ============================================================

/**
 * @brief Execute a turn using the requested direction and delay.
 */
static void executeTurn(
    TurnDirection direction,
    uint32_t turnDelayMs
)
{
    if (direction == TurnDirection::LEFT) {

        leftMotor.backward(TURN_SPEED_LEFT);
        rightMotor.forward(TURN_SPEED_RIGHT);

    } else {

        leftMotor.forward(TURN_SPEED_LEFT);
        rightMotor.backward(TURN_SPEED_RIGHT);
    }

    delay(turnDelayMs);

    leftMotor.brake();
    rightMotor.brake();
}


// ============================================================
// CALIBRATION ROUTINE
// ============================================================

static void calibrateTurnDelay(
    TurnDirection direction,
    float desiredAngle
)
{
    IMU imu;

    uint32_t turnDelayMs = BASE_TURN_DELAY_MS;

    const char *directionText =
        (direction == TurnDirection::LEFT)
            ? "LEFT"
            : "RIGHT";


    // --------------------------------------------------------
    // INITIAL REPORT
    // --------------------------------------------------------

    String message;

    message += "\n=== TURN CALIBRATION ===\n";

    message += "Direction: ";
    message += directionText;

    message += "\nTarget angle: ";
    message += String(desiredAngle, 2);
    message += " deg";

    message += "\nBase delay: ";
    message += String(turnDelayMs);
    message += " ms";

    message += "\nIterations: ";
    message += String(CALIBRATION_TURNS);

    message += "\n========================\n";

    sendData(message);


    // --------------------------------------------------------
    // INITIALIZE IMU
    // --------------------------------------------------------

    imu.begin();

    // Allow IMU/DMP to start producing measurements.
    delay(1000);


    // ========================================================
    // CALIBRATION LOOP
    // ========================================================

    for (uint8_t i = 0; i < CALIBRATION_TURNS; i++) {

        // ----------------------------------------------------
        // READ YAW BEFORE TURN
        // ----------------------------------------------------

        float beforeTurn = readYaw(imu);


        // ----------------------------------------------------
        // EXECUTE TURN
        // ----------------------------------------------------

        executeTurn(
            direction,
            turnDelayMs
        );


        // ----------------------------------------------------
        // WAIT FOR ROBOT TO STOP
        // ----------------------------------------------------

        delay(SETTLING_TIME_MS);


        // ----------------------------------------------------
        // READ YAW AFTER TURN
        // ----------------------------------------------------

        float afterTurn = readYaw(imu);


        // ----------------------------------------------------
        // CALCULATE TURN ANGLE
        // ----------------------------------------------------

        float signedTurn =
            angleDifference(
                afterTurn,
                beforeTurn
            );

        float measuredAngle =
            fabsf(signedTurn);


        // ----------------------------------------------------
        // CALCULATE ERROR
        // ----------------------------------------------------

        float error =
            desiredAngle - measuredAngle;


        // ----------------------------------------------------
        // CALCULATE DELAY CORRECTION
        // ----------------------------------------------------

        float correction =
            KP_DELAY * error;

        int32_t newDelay =
            static_cast<int32_t>(turnDelayMs) +
            static_cast<int32_t>(roundf(correction));


        // Prevent calibration from generating unsafe or
        // unreasonable delay values.
        newDelay = constrain(
            newDelay,
            static_cast<int32_t>(MIN_TURN_DELAY_MS),
            static_cast<int32_t>(MAX_TURN_DELAY_MS)
        );


        // ----------------------------------------------------
        // REPORT ITERATION
        // ----------------------------------------------------

        String report;

        report += "\n--- Iteration ";
        report += String(i + 1);
        report += "/";
        report += String(CALIBRATION_TURNS);
        report += " ---";

        report += "\nDirection: ";
        report += directionText;

        report += "\nTarget: ";
        report += String(desiredAngle, 2);
        report += " deg";

        report += "\nBefore: ";
        report += String(beforeTurn, 2);
        report += " deg";

        report += "\nAfter: ";
        report += String(afterTurn, 2);
        report += " deg";

        report += "\nMeasured: ";
        report += String(measuredAngle, 2);
        report += " deg";

        report += "\nError: ";
        report += String(error, 2);
        report += " deg";

        report += "\nCurrent delay: ";
        report += String(turnDelayMs);
        report += " ms";

        report += "\nCorrection: ";
        report += String(correction, 2);
        report += " ms";

        report += "\nNext delay: ";
        report += String(newDelay);
        report += " ms";

        report += "\n------------------------\n";

        sendData(report);


        // ----------------------------------------------------
        // APPLY CORRECTION
        // ----------------------------------------------------

        turnDelayMs =
            static_cast<uint32_t>(newDelay);


        // ----------------------------------------------------
        // WAIT BEFORE NEXT TURN
        // ----------------------------------------------------

        delay(BETWEEN_TURNS_MS);
    }


    // ========================================================
    // FINAL REPORT
    // ========================================================

    String result;

    result += "\n=== CALIBRATION COMPLETE ===";

    result += "\nDirection: ";
    result += directionText;

    result += "\nTarget: ";
    result += String(desiredAngle, 2);
    result += " deg";

    result += "\nFinal delay: ";
    result += String(turnDelayMs);
    result += " ms";

    result += "\n============================\n";

    sendData(result);
}


// ============================================================
// BLUETOOTH COMMAND PARSER
// ============================================================

/**
 * Expected commands:
 *
 * CAL TURN LEFT 90
 * CAL TURN RIGHT 90
 * CAL TURN LEFT 180
 * CAL TURN RIGHT 180
 *
 * Returns true when the command belongs to this calibration
 * module, even if the command contains invalid parameters.
 */
bool handleTurnCalibrationCommand(String command)
{
    command.trim();
    command.toUpperCase();

    // --------------------------------------------------------
    // Check command prefix
    // --------------------------------------------------------

    if (!command.startsWith("CAL TURN ")) {
        return false;
    }


    // --------------------------------------------------------
    // Remove prefix
    //
    // "CAL TURN LEFT 90"
    //          ↓
    // "LEFT 90"
    // --------------------------------------------------------

    String parameters =
        command.substring(9);

    parameters.trim();


    // --------------------------------------------------------
    // Separate direction and angle
    // --------------------------------------------------------

    int separator =
        parameters.indexOf(' ');

    if (separator < 0) {

        sendData(
            "ERROR: Use CAL TURN <LEFT|RIGHT> <ANGLE>\n"
        );

        return true;
    }


    String directionText =
        parameters.substring(
            0,
            separator
        );

    String angleText =
        parameters.substring(
            separator + 1
        );

    directionText.trim();
    angleText.trim();


    // --------------------------------------------------------
    // Parse direction
    // --------------------------------------------------------

    TurnDirection direction;

    if (directionText == "LEFT") {

        direction = TurnDirection::LEFT;

    } else if (directionText == "RIGHT") {

        direction = TurnDirection::RIGHT;

    } else {

        sendData(
            "ERROR: Direction must be LEFT or RIGHT\n"
        );

        return true;
    }


    // --------------------------------------------------------
    // Parse desired angle
    // --------------------------------------------------------

    float desiredAngle =
        angleText.toFloat();

    if (desiredAngle <= 0.0f ||
        desiredAngle > 180.0f) {

        sendData(
            "ERROR: Angle must be > 0 and <= 180 deg\n"
        );

        return true;
    }


    // --------------------------------------------------------
    // Start calibration
    // --------------------------------------------------------

    calibrateTurnDelay(
        direction,
        desiredAngle
    );

    return true;
}

#endif // RUN_TURN_CALIBRATION