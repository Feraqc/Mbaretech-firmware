#pragma once

// ============================================================
// EDIT THIS FILE TO CONFIGURE THE FIRMWARE BUILD
// ============================================================
// PlatformIO selects this header through firmwareConfig.h. Normal include
// dependencies ensure changes rebuild the affected application sources.
// Build with `pio run`; no environment or command-line feature selection needed.
// firmwareConfig.h checks dependencies and supplies defaults for host tests,
// which intentionally do not load this board configuration.

// Board wiring: define exactly one model, without a numeric value.
#define MBARETECH_2
// #define MBARETECH_1

// ============================================================
// PROGRAM SELECTION (at most one exclusive program may be 1)
// ============================================================
// Preserve the previous default: isolated gyro diagnostic, no motor output.
// For sensor-only operation, set every program below to 0 and enable acquisition.
#define ENABLE_GYRO_TEST          1
#define ENABLE_FSM                0 // Existing combat controller; not migrated.
#define ENABLE_RECIPE_FSM         0 // Generic recipe runtime selected below.
#define ENABLE_MOVEMENT_TEST      0
#define ENABLE_MOTOR_TEST         0
#define ENABLE_LINE_TEST          0
#define ENABLE_LEGACY_MOVEMENTS   0 // Requires ENABLE_MOVEMENT_TEST.
#define ENABLE_TURN_CALIBRATION   0 // Unsupported old diagnostic; use a recipe.

// ============================================================
// HARDWARE AND ACQUISITION (0 = excluded, 1 = enabled)
// ============================================================
// Recipe/legacy combat need ENABLE_SENSOR_TASK. Individual sensor groups remain
// independent; enable every input used by the chosen controller or recipe.
#define ENABLE_MOTORS             0 // Master gate for GPIO/PWM motor writes.
#define ENABLE_SENSOR_TASK        0
#define ENABLE_LINE_SENSORS       0
#define ENABLE_IR_SENSORS         0
#define ENABLE_DIP_SWITCHES       0
#define ENABLE_GYRO               1

// ============================================================
// COMMUNICATION AND DIAGNOSTICS
// ============================================================
// BLE requires logging. The isolated gyro diagnostic excludes logging because
// it owns the IMU itself. ENABLE_DEBUG does not enable Serial or logging.
#define ENABLE_SERIAL             1
#define ENABLE_BLE                0
#define ENABLE_LOGGING            0
#define ENABLE_DEBUG              0
#define ENABLE_TASK_TIMING        0
#define ENABLE_TURN_CANCEL        0 // Existing combat maneuver cancellation.

// Periods are FreeRTOS ticks (>=1), not milliseconds. Recipe motor calibration
// and generic runtime settings remain in fsm/FSMDefinitions.h.
#define SENSOR_READ_PERIOD_TICKS  1
#define FSM_STEP_PERIOD_TICKS     1
#define FSM_DEFAULT_OPENING       0 // Legacy combat: 0..15 when DIP is disabled.

// ============================================================
// GENERIC RECIPE SELECTION (only used with ENABLE_RECIPE_FSM=1)
// ============================================================
// Uncomment exactly one. Normal recipes need no additional START condition:
// their task lifecycle owns START. Recipe combat is still unavailable.
#if ENABLE_RECIPE_FSM
#define FSM_ACTIVE_RECIPE_MOTOR_TEST
// #define FSM_ACTIVE_RECIPE_TURN_CALIBRATION
// #define FSM_ACTIVE_RECIPE_COMBAT
#endif

#if (defined(MBARETECH_1) + defined(MBARETECH_2)) != 1
#error "Select exactly one board model in buildConfig.h"
#endif

// firmwareConfig.h checks all feature dependencies after including this file.
