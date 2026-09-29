#pragma once
#include <stdint.h>
#include <stddef.h>
#include "FSMDefinitions.h"

namespace fsm {
// Schema version 2: catalogs/configuration live in FSMDefinitions.h. Exporters
// must emit counted expressions; version-1 flat trigger initializers are obsolete.
enum class MotionId : uint8_t { NONE = 0, STOP, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT };
using fsm_defs::ConditionId;
using fsm_defs::StateId;
enum class TriggerType : uint8_t { TIMER, SENSOR, COMPLETION };
enum class LogicOp : uint8_t { AND, OR };
static constexpr int16_t STEP_COMPLETE = -1;

struct MotorCommand {
    int8_t left_pct;
    int8_t right_pct;
};
// Explicit lengths let validation reject mismatched AND/OR expressions before
// reading them. Array storage must still be at least as long as its declared count.
struct ConditionExpression {
    const ConditionId* terms;
    uint8_t termCount;
    const LogicOp* operators;
    uint8_t operatorCount;
};
struct TriggerRecipe {
    TriggerType type;
    uint32_t timerMs;              // TIMER only, milliseconds from state/step entry.
    ConditionExpression expression; // SENSOR only; evaluated left to right.
};
struct StateTransitionRecipe {
    TriggerRecipe trigger;
    StateId next_state;
};
struct StepTransitionRecipe {
    TriggerRecipe trigger;
    int16_t next_step; // Step index, or STEP_COMPLETE; never a top-level StateId.
};
struct StepRecipe {
    MotionId motion;
    MotorCommand motor;
    const StepTransitionRecipe* out_transitions;
    uint8_t out_count;
};
struct SubFsmRecipe {
    const StepRecipe* steps;
    uint8_t step_count;
    // Opt-in per sequence: otherwise reachable STEP_COMPLETE needs a completion
    // exit. This is a validation policy, never an implicit transition or brake.
    bool allowHoldOnCompletion;
};
enum class StateKind : uint8_t { MOTOR, SUBFSM };
struct StateRecipe {
    StateId id;
    StateKind kind;
    MotionId motion;
    MotorCommand motor; // MOTOR states only; SUBFSM uses its active step command.
    const StateTransitionRecipe* out_transitions;
    uint8_t out_count;
    const SubFsmRecipe* subfsm; // nullptr for MOTOR states.
};
struct MachineRecipe {
    StateId initial_state;
    const StateRecipe* states;
    uint8_t state_count;
    const char* name; // Required static recipe identity used by debug events.
};
}  // namespace fsm
