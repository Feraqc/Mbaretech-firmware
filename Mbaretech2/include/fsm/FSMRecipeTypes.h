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
using ParameterId = uint16_t;
static constexpr ParameterId NO_PARAMETER = 0;
enum class ParameterType : uint8_t { Integer };
enum class ParameterUnit : uint8_t { Percent, Milliseconds };
enum class ParameterPolicy : uint8_t { Immediate, NextStateEntry, NextStepEntry, NextMachineStart, StoppedOnly };
enum class ParameterAccess : uint8_t { Writable, ReadOnly };
// IDs and keys belong to the recipe, independently of table order.
struct ParameterDefinition {
    ParameterId id;
    const char* key;
    const char* name;
    ParameterType type;
    ParameterUnit unit;
    int32_t defaultValue;
    int32_t minimum;
    int32_t maximum;
    int32_t step;
    ParameterPolicy policy;
    ParameterAccess access;
};

struct MotorCommand {
    int8_t left_pct;
    int8_t right_pct;
    // 0 usa el literal; otro ID resuelve el parámetro y conserva el literal
    // como valor de respaldo/documentación del header.
    ParameterId leftParameter;
    ParameterId rightParameter;
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
    // TIMER solamente; 0 conserva timerMs como literal.
    ParameterId timerParameter;
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
    const ParameterDefinition* parameters;
    uint16_t parameterCount;
};
}  // namespace fsm
