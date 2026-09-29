#pragma once

// PlatformIO loads the one editable build configuration through a normal include,
// so edits are tracked as source dependencies. Host tests omit this marker and
// retain their explicit synthetic feature combinations.
#if defined(MBARETECH_USE_BUILD_CONFIG) && MBARETECH_USE_BUILD_CONFIG
#include "buildConfig.h"
#endif

// Build choices live in buildConfig.h.
// This header owns validation and fallback defaults for standalone/host builds.
// All feature/program switches use ENABLE_*=0/1. Omitted means disabled here.
#ifndef ENABLE_FSM
#define ENABLE_FSM 0
#endif
// Generic recipe runtime is opt-in; the existing combat program is preserved.
#ifndef ENABLE_RECIPE_FSM
#define ENABLE_RECIPE_FSM 0
#endif
#ifndef ENABLE_SENSOR_TASK
#define ENABLE_SENSOR_TASK 0
#endif
#ifndef ENABLE_BLE
#define ENABLE_BLE 0
#endif
#ifndef ENABLE_SERIAL
#define ENABLE_SERIAL 0
#endif
#ifndef ENABLE_LOGGING
#define ENABLE_LOGGING 0
#endif
#ifndef ENABLE_DEBUG
#define ENABLE_DEBUG 0
#endif
#ifndef ENABLE_MOTORS
#define ENABLE_MOTORS 0
#endif
#ifndef ENABLE_LINE_SENSORS
#define ENABLE_LINE_SENSORS 0
#endif
#ifndef ENABLE_IR_SENSORS
#define ENABLE_IR_SENSORS 0
#endif
#ifndef ENABLE_DIP_SWITCHES
#define ENABLE_DIP_SWITCHES 0
#endif
#ifndef ENABLE_GYRO
#define ENABLE_GYRO 0
#endif
#ifndef ENABLE_TASK_TIMING
#define ENABLE_TASK_TIMING 0
#endif
#ifndef ENABLE_TURN_CANCEL
#define ENABLE_TURN_CANCEL 0
#endif
#ifndef ENABLE_GYRO_TEST
#define ENABLE_GYRO_TEST 0
#endif
#ifndef ENABLE_MOVEMENT_TEST
#define ENABLE_MOVEMENT_TEST 0
#endif
#ifndef ENABLE_LEGACY_MOVEMENTS
#define ENABLE_LEGACY_MOVEMENTS 0
#endif
#ifndef ENABLE_MOTOR_TEST
#define ENABLE_MOTOR_TEST 0
#endif
#ifndef ENABLE_LINE_TEST
#define ENABLE_LINE_TEST 0
#endif
#ifndef ENABLE_TURN_CALIBRATION
#define ENABLE_TURN_CALIBRATION 0
#endif

#if (ENABLE_FSM != 0 && ENABLE_FSM != 1) || \
    (ENABLE_RECIPE_FSM != 0 && ENABLE_RECIPE_FSM != 1) || \
    (ENABLE_SENSOR_TASK != 0 && ENABLE_SENSOR_TASK != 1) || \
    (ENABLE_BLE != 0 && ENABLE_BLE != 1) || \
    (ENABLE_SERIAL != 0 && ENABLE_SERIAL != 1) || \
    (ENABLE_LOGGING != 0 && ENABLE_LOGGING != 1) || \
    (ENABLE_DEBUG != 0 && ENABLE_DEBUG != 1) || \
    (ENABLE_MOTORS != 0 && ENABLE_MOTORS != 1) || \
    (ENABLE_LINE_SENSORS != 0 && ENABLE_LINE_SENSORS != 1) || \
    (ENABLE_IR_SENSORS != 0 && ENABLE_IR_SENSORS != 1) || \
    (ENABLE_DIP_SWITCHES != 0 && ENABLE_DIP_SWITCHES != 1) || \
    (ENABLE_GYRO != 0 && ENABLE_GYRO != 1) || \
    (ENABLE_TASK_TIMING != 0 && ENABLE_TASK_TIMING != 1) || \
    (ENABLE_TURN_CANCEL != 0 && ENABLE_TURN_CANCEL != 1) || \
    (ENABLE_GYRO_TEST != 0 && ENABLE_GYRO_TEST != 1) || \
    (ENABLE_MOVEMENT_TEST != 0 && ENABLE_MOVEMENT_TEST != 1) || \
    (ENABLE_LEGACY_MOVEMENTS != 0 && ENABLE_LEGACY_MOVEMENTS != 1) || \
    (ENABLE_MOTOR_TEST != 0 && ENABLE_MOTOR_TEST != 1) || \
    (ENABLE_LINE_TEST != 0 && ENABLE_LINE_TEST != 1) || \
    (ENABLE_TURN_CALIBRATION != 0 && ENABLE_TURN_CALIBRATION != 1)
#error "All ENABLE_* switches must be 0 or 1."
#endif

// Reject obsolete switches instead of silently ignoring an old build profile.
#if defined(RUN_TASK_TEST) || defined(RUN_SENSORS_TEST) || defined(RUN_GYRO_TEST) || \
    defined(RUN_MOVEMENTS_TEST) || defined(RUN_DRIVER_TEST) || defined(RUN_LS_SENSOR_TEST) || \
    defined(RUN_TURN_CALIBRATION) || defined(RUN_LINE_SENSOR) || defined(FORWARDON) || \
    defined(CANCEL_TURNS) || defined(DEBUG) || defined(OLD) || defined(ESTADOS_ORDEN) || \
    defined(RUN_MOVEMENT_SENSOR_CALIBRATION)
#error "Obsolete build flags: use ENABLE_* switches from firmwareConfig.h."
#endif
#if (ENABLE_FSM + ENABLE_RECIPE_FSM + ENABLE_GYRO_TEST + ENABLE_MOVEMENT_TEST + ENABLE_MOTOR_TEST + ENABLE_LINE_TEST + ENABLE_TURN_CALIBRATION) > 1
#error "Select at most one control/diagnostic program."
#endif
#if ENABLE_RECIPE_FSM && !ENABLE_SENSOR_TASK
#error "Recipe FSM requires ENABLE_SENSOR_TASK."
#endif
#if ENABLE_RECIPE_FSM && !(ENABLE_SERIAL || ENABLE_LOGGING)
#error "Recipe FSM requires Serial or logging to report recipe validation errors."
#endif
#if ENABLE_FSM && (!ENABLE_SENSOR_TASK || !ENABLE_LINE_SENSORS || !ENABLE_IR_SENSORS)
#error "FSM requires ENABLE_SENSOR_TASK, ENABLE_LINE_SENSORS and ENABLE_IR_SENSORS."
#endif
#if ENABLE_SENSOR_TASK && (ENABLE_MOVEMENT_TEST || ENABLE_LINE_TEST || ENABLE_TURN_CALIBRATION)
#error "Sensor task and direct-reading diagnostics cannot own sensors simultaneously."
#endif
#if ENABLE_MOVEMENT_TEST && (!ENABLE_SERIAL || !ENABLE_LINE_SENSORS || !ENABLE_IR_SENSORS || !ENABLE_DIP_SWITCHES)
#error "Movement tests require Serial, line, IR and DIP acquisition."
#endif
#if ENABLE_LEGACY_MOVEMENTS && !ENABLE_MOVEMENT_TEST
#error "ENABLE_LEGACY_MOVEMENTS requires ENABLE_MOVEMENT_TEST."
#endif
#if ENABLE_GYRO_TEST && (!ENABLE_GYRO || !ENABLE_SERIAL || ENABLE_LOGGING)
#error "Gyro diagnostic requires gyro + Serial and excludes the logging IMU owner."
#endif
#if ENABLE_MOTOR_TEST && (!ENABLE_MOTORS || !ENABLE_SERIAL)
#error "Motor diagnostic requires motors and Serial."
#endif
#if ENABLE_LINE_TEST && (!ENABLE_LINE_SENSORS || !ENABLE_SERIAL)
#error "Line diagnostic requires line sensors and Serial."
#endif
#if ENABLE_TURN_CALIBRATION
#error "Turn calibration has no standalone entry point; use the supported diagnostics."
#endif
#if ENABLE_LOGGING && !(ENABLE_SERIAL || ENABLE_BLE)
#error "Logging requires at least one transport: ENABLE_SERIAL or ENABLE_BLE."
#endif
#if ENABLE_BLE && !ENABLE_LOGGING
#error "BLE UART requires ENABLE_LOGGING."
#endif
#if ENABLE_DEBUG && !ENABLE_SERIAL
#error "Debug output requires ENABLE_SERIAL."
#endif
#if ENABLE_TASK_TIMING && !ENABLE_SENSOR_TASK
#error "Task timing requires ENABLE_SENSOR_TASK."
#endif

#ifndef SENSOR_READ_PERIOD_TICKS
#define SENSOR_READ_PERIOD_TICKS 1
#endif
#ifndef FSM_STEP_PERIOD_TICKS
#define FSM_STEP_PERIOD_TICKS 1
#endif
#ifndef FSM_DEFAULT_OPENING
#define FSM_DEFAULT_OPENING 0
#endif
#if SENSOR_READ_PERIOD_TICKS < 1 || FSM_STEP_PERIOD_TICKS < 1
#error "Task periods must be at least one tick."
#endif
#if FSM_DEFAULT_OPENING < 0 || FSM_DEFAULT_OPENING > 15
#error "FSM_DEFAULT_OPENING must be 0..15 (E/A/B/C)."
#endif
