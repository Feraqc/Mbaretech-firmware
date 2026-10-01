#pragma once
#include "../FSMRecipeTypes.h"

namespace fsm_recipe_turn_calibration {
using namespace fsm;
// Calibration examples, not replacements for the existing combat constants.
static const ParameterDefinition PARAMETERS[] = {
    {1, "left_turn_duration", "Left turn duration", ParameterType::Integer, ParameterUnit::Milliseconds,
     fsm_defs::timers::TURN_LEFT_MS, 0, 60000, 1, ParameterPolicy::NextStepEntry},
    {2, "right_turn_duration", "Right turn duration", ParameterType::Integer, ParameterUnit::Milliseconds,
     fsm_defs::timers::TURN_RIGHT_MS, 0, 60000, 1, ParameterPolicy::NextStepEntry}
};
static constexpr ConditionId EDGE_TERMS[] = {ConditionId::LINE_LEFT_DETECTED, ConditionId::LINE_RIGHT_DETECTED};
static constexpr LogicOp EDGE_OPS[] = {LogicOp::OR};
static_assert(sizeof(EDGE_OPS) / sizeof(EDGE_OPS[0]) + 1 == sizeof(EDGE_TERMS) / sizeof(EDGE_TERMS[0]), "Condition operator count");
static constexpr TriggerRecipe EDGE_TRIGGER = {TriggerType::SENSOR, 0, {EDGE_TERMS, 2, EDGE_OPS, 1}};
static const StepTransitionRecipe LEFT_OUT[] = {
    {{TriggerType::TIMER, fsm_defs::timers::TURN_LEFT_MS, {nullptr, 0, nullptr, 0}, 1}, 1}
};
static const StepTransitionRecipe PAUSE_OUT[] = {
    {{TriggerType::TIMER, fsm_defs::timers::TURN_PAUSE_MS, {nullptr, 0, nullptr, 0}}, 2}
};
static const StepTransitionRecipe RIGHT_OUT[] = {
    {{TriggerType::TIMER, fsm_defs::timers::TURN_RIGHT_MS, {nullptr, 0, nullptr, 0}, 2}, STEP_COMPLETE}
};
static const StepRecipe STEPS[] = {
    {MotionId::TURN_LEFT, {-fsm_defs::motor::TURN_CALIBRATION_SPEED_PCT, fsm_defs::motor::TURN_CALIBRATION_SPEED_PCT}, LEFT_OUT, 1},
    {MotionId::STOP, {0, 0}, PAUSE_OUT, 1},
    {MotionId::TURN_RIGHT, {fsm_defs::motor::TURN_CALIBRATION_SPEED_PCT, -fsm_defs::motor::TURN_CALIBRATION_SPEED_PCT}, RIGHT_OUT, 1}
};
static const SubFsmRecipe SEQUENCE = {STEPS, 3, false};
// Sensor health permits execution. This zero-time edge does not wait for START.
static const StateTransitionRecipe IDLE_OUT[] = {{{TriggerType::TIMER, 0, {nullptr, 0, nullptr, 0}}, StateId::TURN_SEQUENCE}};
static const StateTransitionRecipe SEQUENCE_OUT[] = {
    {EDGE_TRIGGER, StateId::DONE},
    {{TriggerType::COMPLETION, 0, {nullptr, 0, nullptr, 0}}, StateId::DONE}
};
static const StateRecipe STATES[] = {
    {StateId::IDLE, StateKind::MOTOR, MotionId::STOP, {0, 0}, IDLE_OUT, 1, nullptr},
    {StateId::TURN_SEQUENCE, StateKind::SUBFSM, MotionId::NONE, {0, 0}, SEQUENCE_OUT, 2, &SEQUENCE},
    {StateId::DONE, StateKind::MOTOR, MotionId::STOP, {0, 0}, nullptr, 0, nullptr}
};
static const MachineRecipe MACHINE = {StateId::IDLE, STATES, 3, "TURN_CALIBRATION", PARAMETERS, 2};
}  // namespace fsm_recipe_turn_calibration
