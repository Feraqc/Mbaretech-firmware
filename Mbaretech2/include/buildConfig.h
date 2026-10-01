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
// Choose the program independently of motor output permission below.
// For sensor-only operation, set every program below to 0 and enable acquisition.
#define ENABLE_GYRO_TEST          0
#define ENABLE_FSM                0 // Existing combat controller; not migrated.
#define ENABLE_RECIPE_FSM         1 // Generic recipe runtime selected below.
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
// DEBUG ONLY: 0 = physical START_PIN; 1 = force recipe motor permission active.
// With ENABLE_MOTORS=1 the robot may move immediately after boot on valid data.
// Does not alter the ISR latch, sensor validity, recipe timers or legacy combat.
#define FORCE_START_ACTIVE        0
#define ENABLE_SENSOR_TASK        1
#define ENABLE_LINE_SENSORS       0
#define ENABLE_IR_SENSORS         0
#define ENABLE_DIP_SWITCHES       0
#define ENABLE_GYRO               0

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
#define ENABLE_TELEMETRY          1 // Canonical capture for Serial/BLE/WiFi.

// Optional read-only Live Telemetry for the generic recipe FSM.
// STA mode joins Wokwi-GUEST so Wokwi can forward localhost:8180 to port 80.
// AP mode is for a physical robot; connect the editor to ws://192.168.4.1/ws.
#define ENABLE_WIFI_TELEMETRY      1
#define WIFI_TELEMETRY_USE_STA     1
#define WIFI_TELEMETRY_STA_SSID    "Wokwi-GUEST"
#define WIFI_TELEMETRY_STA_PASSWORD ""
#define WIFI_TELEMETRY_AP_SSID     "MBARETECH-FSM"
#define WIFI_TELEMETRY_AP_PASSWORD ""

// Generic FSM Serial presentation.
// 0 = full CSV/structured log
// 1 = compact human-readable console
#define FSM_CONSOLE_COMPACT       1

// Periods are FreeRTOS ticks (>=1), not milliseconds. Recipe motor calibration
// and generic runtime settings remain in fsm/FSMDefinitions.h.
#define SENSOR_READ_PERIOD_TICKS  1
#define FSM_STEP_PERIOD_TICKS     1
#define FSM_DEFAULT_OPENING       0 // Legacy combat: 0..15 when DIP is disabled.

// ============================================================
// GENERIC RECIPE SELECTION (only used with ENABLE_RECIPE_FSM=1)
// ============================================================
// Uncomment exactly one. Normal recipes need no additional START condition:
// Drive owns START output permission; the FSM runs on valid/fresh sensor data.
// Recipe combat is still unavailable.
#if ENABLE_RECIPE_FSM
//#define FSM_ACTIVE_RECIPE_EDITOR_TEST
// #define FSM_ACTIVE_RECIPE_TEST
//#define FSM_ACTIVE_RECIPE_TURN_CALIBRATION
// #define FSM_ACTIVE_RECIPE_COMBAT
#define FSM_ACTIVE_RECIPE_STATE_TEST
#endif

#if (defined(MBARETECH_1) + defined(MBARETECH_2)) != 1
#error "Select exactly one board model in buildConfig.h"
#endif

// firmwareConfig.h checks all feature dependencies after including this file.
