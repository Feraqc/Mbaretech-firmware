#include "firmwareConfig.h"
#if ENABLE_RECIPE_FSM
#include "fsm/StateMachine.h"
#include <string.h>

namespace fsm {
namespace {
bool validMotor(const MotorCommand& motor) {
    return motor.left_pct >= fsm_defs::motor::MIN_PERCENT && motor.left_pct <= fsm_defs::motor::MAX_PERCENT &&
           motor.right_pct >= fsm_defs::motor::MIN_PERCENT && motor.right_pct <= fsm_defs::motor::MAX_PERCENT;
}
bool validMotion(MotionId motion) {
    return static_cast<unsigned>(motion) <= static_cast<unsigned>(MotionId::TURN_RIGHT);
}
bool hasState(const MachineRecipe& machine, StateId id) {
    for (unsigned i = 0; i < machine.state_count; ++i)
        if (machine.states[i].id == id) return true;
    return false;
}
const ParameterDefinition* parameter(const MachineRecipe& machine, ParameterId id) {
    if (id == NO_PARAMETER) return nullptr;
    for (unsigned i = 0; i < machine.parameterCount; ++i)
        if (machine.parameters[i].id == id) return &machine.parameters[i];
    return nullptr;
}
bool validParameterKey(const char* key) {
    if (!key || *key < 'a' || *key > 'z') return false;
    for (const char* p = key + 1; *p; ++p)
        if (!(*p >= 'a' && *p <= 'z') && !(*p >= '0' && *p <= '9') && *p != '_')
            return false;
    return true;
}
bool validReference(const MachineRecipe& machine, ParameterId id, ParameterUnit unit,
                    ParameterPolicy entryPolicy) {
    if (id == NO_PARAMETER) return true;
    const auto* definition = parameter(machine, id);
    return definition && definition->unit == unit && definition->type == ParameterType::Integer &&
           (definition->policy == entryPolicy || definition->policy == ParameterPolicy::NextMachineStart ||
            definition->policy == ParameterPolicy::StoppedOnly) &&
           (unit != ParameterUnit::Percent ||
            (definition->minimum >= -100 && definition->maximum <= 100)) &&
           (unit != ParameterUnit::Milliseconds || definition->minimum >= 0);
}
bool validMotorReferences(const MachineRecipe& machine, const MotorCommand& motor,
                          ParameterPolicy entryPolicy) {
    return validReference(machine, motor.leftParameter, ParameterUnit::Percent, entryPolicy) &&
           validReference(machine, motor.rightParameter, ParameterUnit::Percent, entryPolicy);
}
const char* validateTrigger(const TriggerRecipe& trigger, bool allowCompletion) {

    switch (trigger.type) {
    case TriggerType::TIMER: return nullptr;
    case TriggerType::COMPLETION:
        return allowCompletion ? nullptr : "COMPLETION requires a SubFSM-level transition";
    case TriggerType::SENSOR:
        const auto& expression = trigger.expression;
        if (!expression.termCount || !expression.terms) return "Sensor condition requires at least one term";
        // Check counts before indexing either table, including single-term cases.
        if (expression.operatorCount != expression.termCount - 1)
            return "Condition operatorCount must equal termCount - 1";
        if (expression.operatorCount && !expression.operators) return "Missing condition operators";
        for (unsigned i = 0; i < expression.termCount; ++i) {
            if (!fsm_defs::conditionMetadata(expression.terms[i]))
                return "Unknown ConditionId";
            if (i && expression.operators[i - 1] != LogicOp::AND &&
                     expression.operators[i - 1] != LogicOp::OR)
                return "Unknown LogicOp";
        }
        return nullptr;
    }
    return "Unknown TriggerType";
}

bool reachesCompletion(const SubFsmRecipe& sequence) {
    // Structural reachability from entry step 0. Conditions may be true in some
    // input history, so every declared edge is considered. Each step is queued
    // once; bounded arrays avoid recursion/heap and handle cycles deterministically.
    bool visited[UINT8_MAX] = {};
    uint8_t pending[UINT8_MAX] = {};
    unsigned head = 0, tail = 1;
    visited[0] = true;
    while (head < tail) {
        const auto& step = sequence.steps[pending[head++]];
        for (unsigned i = 0; i < step.out_count; ++i) {
            const int16_t target = step.out_transitions[i].next_step;
            if (target == STEP_COMPLETE) return true;
            if (!visited[target]) {
                visited[target] = true;
                pending[tail++] = static_cast<uint8_t>(target);
            }
        }
    }
    return false;
}
}

const char* validateRecipe(const MachineRecipe& machine) {
    if (!machine.name || !machine.name[0]) return "Machine name is required for logging";
    if (!machine.states || !machine.state_count) return "Machine has no states";
    if (machine.parameterCount > MAX_RUNTIME_PARAMETERS) return "Too many recipe parameters";
    if (machine.parameterCount && !machine.parameters) return "Missing recipe parameter table";
    for (unsigned i = 0; i < machine.parameterCount; ++i) {
        const auto& entry = machine.parameters[i];
        if (entry.id == NO_PARAMETER || !validParameterKey(entry.key) || !entry.name ||
            entry.type != ParameterType::Integer ||
            (entry.unit != ParameterUnit::Percent && entry.unit != ParameterUnit::Milliseconds) ||
            static_cast<unsigned>(entry.policy) > static_cast<unsigned>(ParameterPolicy::StoppedOnly) ||
            entry.step <= 0 || entry.minimum > entry.maximum ||
            entry.defaultValue < entry.minimum || entry.defaultValue > entry.maximum ||
            (int64_t(entry.defaultValue) - entry.minimum) % entry.step ||
            (entry.access != ParameterAccess::Writable && entry.access != ParameterAccess::ReadOnly))
            return "Invalid recipe parameter";
        for (unsigned j = 0; j < i; ++j)
            if (machine.parameters[j].id == entry.id ||
                strcmp(machine.parameters[j].key, entry.key) == 0)
                return "Duplicate recipe parameter";
    }
    if (!hasState(machine, machine.initial_state)) return "Initial state does not exist";
    for (unsigned i = 0; i < machine.state_count; ++i) {
        const auto& state = machine.states[i];
        if (!fsm_defs::stateMetadata(state.id)) return "Unknown StateId";
        for (unsigned j = 0; j < i; ++j)
            if (machine.states[j].id == state.id) return "Duplicate StateId";
        if (!validMotor(state.motor)) return "State motor outside -100..100";
        if (!validMotorReferences(machine, state.motor, ParameterPolicy::NextStateEntry)) return "Invalid state motor parameter";
        if (!validMotion(state.motion)) return "Unknown MotionId";
        const bool subfsm = state.kind == StateKind::SUBFSM;
        if (!subfsm && state.kind != StateKind::MOTOR) return "Unknown StateKind";
        if (!subfsm && state.subfsm) return "MOTOR state must not contain SubFSM data";
        if (state.out_count && !state.out_transitions) return "Missing state transition table";
        bool hasCompletionExit = false;
        for (unsigned t = 0; t < state.out_count; ++t) {
            const auto& transition = state.out_transitions[t];
            if (!hasState(machine, transition.next_state)) return "State transition target does not exist";
            if (const char* error = validateTrigger(transition.trigger, subfsm)) return error;
            if (transition.trigger.timerParameter != NO_PARAMETER &&
                (transition.trigger.type != TriggerType::TIMER ||
                 !validReference(machine, transition.trigger.timerParameter, ParameterUnit::Milliseconds,
                                 ParameterPolicy::NextStateEntry)))
                return "Unknown state timer parameter";
            if (transition.trigger.type == TriggerType::COMPLETION)
                hasCompletionExit = true;
        }
        if (!subfsm) continue;
        if (!state.subfsm || !state.subfsm->steps || !state.subfsm->step_count)
            return "SubFSM requires at least one step";
        for (unsigned s = 0; s < state.subfsm->step_count; ++s) {
            const auto& step = state.subfsm->steps[s];
            if (!validMotor(step.motor)) return "Step motor outside -100..100";
            if (!validMotorReferences(machine, step.motor, ParameterPolicy::NextStepEntry)) return "Invalid step motor parameter";
            if (!validMotion(step.motion)) return "Unknown step MotionId";
            if (step.out_count && !step.out_transitions) return "Missing step transition table";
            for (unsigned t = 0; t < step.out_count; ++t) {
                const auto& transition = step.out_transitions[t];
                if (transition.next_step != STEP_COMPLETE &&
                    (transition.next_step < 0 || transition.next_step >= state.subfsm->step_count))
                    return "Step transition target does not exist";
                if (const char* error = validateTrigger(transition.trigger, false)) return error;
                if (transition.trigger.timerParameter != NO_PARAMETER &&
                    (transition.trigger.type != TriggerType::TIMER ||
                     !validReference(machine, transition.trigger.timerParameter, ParameterUnit::Milliseconds,
                                     ParameterPolicy::NextStepEntry)))
                    return "Unknown step timer parameter";
            }
        }
        // Do not invent a transition. Require a completion exit or explicit
        // per-sequence acknowledgement that retaining the last command is intended.
        if (!hasCompletionExit && !state.subfsm->allowHoldOnCompletion && reachesCompletion(*state.subfsm))
            return "Reachable STEP_COMPLETE requires completion exit or allowHoldOnCompletion";
    }
    return nullptr;
}
}
#endif
