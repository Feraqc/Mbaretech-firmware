#pragma once
#include <stdint.h>
#include "firmwareConfig.h"

// Human-edited catalog for the generic runtime only. Legacy combat calibration
// remains in globals.h; changing these values must not retune CombatFsm.
namespace fsm_defs {

// ============================================================
// SYSTEM / SCHEMA METADATA
// ============================================================
// Version 2 adds counted condition expressions and explicit completion policy.
static constexpr uint16_t SCHEMA_VERSION = 2;

// ============================================================
// RUNTIME CONFIGURATION
// ============================================================
namespace runtime {
// Acquisition age in milliseconds; START has its own observation timestamp.
static constexpr uint32_t SENSOR_MAX_AGE_MS = 50;
// ESP32 FreeRTOS stack units are bytes. Period remains build-configurable ticks.
static constexpr unsigned TASK_STACK_BYTES = 4096;
static constexpr unsigned TASK_PRIORITY = 3;
static constexpr uint32_t TASK_PERIOD_TICKS = FSM_STEP_PERIOD_TICKS;
static constexpr uint32_t IDLE_LOOP_DELAY_MS = 10;
// Bounded debug queue: newest event is dropped when full, never stalls control.
static constexpr bool TRANSITION_LOGGING = true;
static constexpr unsigned LOG_QUEUE_CAPACITY = 32;
static constexpr unsigned LOG_EVENTS_PER_POLL = 4;
static constexpr unsigned LOG_LINE_BYTES = 512;
static constexpr unsigned LOG_EXPRESSION_BYTES = 192;
static_assert(LOG_QUEUE_CAPACITY > 0, "Transition queue must have storage");
static_assert(LOG_LINE_BYTES >= 256 && LOG_EXPRESSION_BYTES >= 64, "Debug buffers too small");
}

// ============================================================
// MOTOR / MOVEMENT PARAMETERS
// ============================================================
namespace motor {
// Signed wheel command limits, used by Drive and recipe validation.
static constexpr int MIN_PERCENT = -100;
static constexpr int MAX_PERCENT = 100;
// Calibration magnitudes (0..100), not direction; recipes supply the signs.
static constexpr int8_t TEST_SPEED_PCT = 30;
static constexpr int8_t TURN_CALIBRATION_SPEED_PCT = 40;
static_assert(TEST_SPEED_PCT >= 0 && TEST_SPEED_PCT <= MAX_PERCENT, "Motor test speed range");
static_assert(TURN_CALIBRATION_SPEED_PCT >= 0 && TURN_CALIBRATION_SPEED_PCT <= MAX_PERCENT, "Turn speed range");
}

// ============================================================
// TIMER CALIBRATION
// ============================================================
namespace timers {
// Milliseconds, consumed by the motor-test recipe. Separate values allow tuning
// each direction independently without editing transition topology.
static constexpr uint32_t MOTOR_TEST_FORWARD_MS = 400;
static constexpr uint32_t MOTOR_TEST_STOP_MS = 200;
static constexpr uint32_t MOTOR_TEST_BACKWARD_MS = 400;
// Open-loop turn calibration; these durations do not guarantee a physical angle.
static constexpr uint32_t TURN_LEFT_MS = 350;
static constexpr uint32_t TURN_PAUSE_MS = 500;
static constexpr uint32_t TURN_RIGHT_MS = 350;
}

// ============================================================
// CONDITION CATALOG
// ============================================================
enum class ConditionId : uint8_t {
    NONE = 0,
    START_ACTIVE, // Available for explicit START diagnostics, not normal gating.
    IR1_DETECTED, IR2_DETECTED, IR3_DETECTED, IR4_DETECTED,
    IR5_DETECTED, IR6_DETECTED, IR7_DETECTED,
    LINE_LEFT_DETECTED, LINE_RIGHT_DETECTED,
    COUNT
};
struct ConditionMetadata {
    ConditionId id;
    const char* name;
    const char* description;
};
static constexpr ConditionMetadata CONDITIONS[] = {
    {ConditionId::NONE, "NONE", "Always false; no sensor condition"},
    {ConditionId::START_ACTIVE, "START_ACTIVE", "START latch was active at observation time"},
    {ConditionId::IR1_DETECTED, "IR1_DETECTED", "Side-left rival sensor detects an opponent"},
    {ConditionId::IR2_DETECTED, "IR2_DETECTED", "Short-left rival sensor detects an opponent"},
    {ConditionId::IR3_DETECTED, "IR3_DETECTED", "Upper-left rival sensor detects an opponent"},
    {ConditionId::IR4_DETECTED, "IR4_DETECTED", "Upper-center rival sensor detects an opponent"},
    {ConditionId::IR5_DETECTED, "IR5_DETECTED", "Upper-right rival sensor detects an opponent"},
    {ConditionId::IR6_DETECTED, "IR6_DETECTED", "Short-right rival sensor detects an opponent"},
    {ConditionId::IR7_DETECTED, "IR7_DETECTED", "Side-right rival sensor detects an opponent"},
    {ConditionId::LINE_LEFT_DETECTED, "LINE_LEFT_DETECTED", "Front-left sensor detects the ring boundary"},
    {ConditionId::LINE_RIGHT_DETECTED, "LINE_RIGHT_DETECTED", "Front-right sensor detects the ring boundary"}
};
static_assert(sizeof(CONDITIONS) / sizeof(CONDITIONS[0]) == static_cast<unsigned>(ConditionId::COUNT), "Complete condition catalog required");
inline const ConditionMetadata* conditionMetadata(ConditionId id) {
    for (const auto& entry : CONDITIONS) if (entry.id == id) return &entry;
    return nullptr;
}
inline const char* conditionName(ConditionId id) {
    const auto* entry = conditionMetadata(id);
    return entry ? entry->name : "UNKNOWN_CONDITION";
}

// ============================================================
// STATE CATALOG / METADATA
// ============================================================
// IDs describe identity only. Recipes own commands, destinations and order.
enum class StateId : uint8_t { IDLE, MOTOR_SEQUENCE, TURN_SEQUENCE, DONE, COUNT };
struct StateMetadata {
    StateId id;
    const char* name;
    const char* description;
};
static constexpr StateMetadata STATES[] = {
    {StateId::IDLE, "IDLE", "Stopped entry state after lifecycle permits execution"},
    {StateId::MOTOR_SEQUENCE, "MOTOR_SEQUENCE", "Forward, stop and backward motor validation"},
    {StateId::TURN_SEQUENCE, "TURN_SEQUENCE", "Left turn, pause and right turn calibration"},
    {StateId::DONE, "DONE", "Stopped terminal state until lifecycle restarts the machine"}
};
static_assert(sizeof(STATES) / sizeof(STATES[0]) == static_cast<unsigned>(StateId::COUNT), "Complete state catalog required");
inline const StateMetadata* stateMetadata(StateId id) {
    for (const auto& entry : STATES) if (entry.id == id) return &entry;
    return nullptr;
}
inline const char* stateName(StateId id) {
    const auto* entry = stateMetadata(id);
    return entry ? entry->name : "UNKNOWN_STATE";
}
} // namespace fsm_defs
