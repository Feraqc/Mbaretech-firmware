#pragma once

#include "../FSMRecipeTypes.h"

namespace fsm_recipe_state_test {
using namespace fsm;

// Los tipos y catálogos pertenecen al firmware; aquí solo se define la receta.
static const StateTransitionRecipe tabla_0[] = {
    {{TriggerType::TIMER, 500, {nullptr, 0, nullptr, 0}, 1}, StateId::FORWARD_LEFT_45}
};
static const StateTransitionRecipe tabla_1[] = {
    {{TriggerType::TIMER, 500, {nullptr, 0, nullptr, 0}, 3}, StateId::BACKWARD}
};
static const StateTransitionRecipe tabla_2[] = {
    {{TriggerType::TIMER, 200, {nullptr, 0, nullptr, 0}, 5}, StateId::FORWARD}
};
static const StepTransitionRecipe tabla_3[] = {
    {{TriggerType::TIMER, 2000, {nullptr, 0, nullptr, 0}, 8}, 1}
};
static const StepTransitionRecipe tabla_4[] = {
    {{TriggerType::TIMER, 3000, {nullptr, 0, nullptr, 0}, 9}, STEP_COMPLETE}
};
static const StepRecipe tabla_5[] = {
    {MotionId::FORWARD, {50, 100, 6, 7}, tabla_3, 1},
    {MotionId::TURN_RIGHT, {100, -100, 0, 0}, tabla_4, 1}
};
static const SubFsmRecipe secuencia_6 = {tabla_5, 2, false};
static const StateTransitionRecipe tabla_7[] = {
    {{TriggerType::COMPLETION, 0, {nullptr, 0, nullptr, 0}, 0}, StateId::FORWARD}
};
static const StateRecipe tabla_8[] = {
    {StateId::IDLE, StateKind::MOTOR, MotionId::STOP, {0, 0, 0, 0}, tabla_0, 1, nullptr},
    {StateId::FORWARD, StateKind::MOTOR, MotionId::STOP, {100, 100, 2, 2}, tabla_1, 1, nullptr},
    {StateId::BACKWARD, StateKind::MOTOR, MotionId::STOP, {-100, -100, 4, 4}, tabla_2, 1, nullptr},
    {StateId::FORWARD_LEFT_45, StateKind::SUBFSM, MotionId::STOP, {100, 100, 0, 0}, tabla_7, 1, &secuencia_6}
};
static const ParameterDefinition tabla_9[] = {
    {1, "idle_duration", "Idle duration", ParameterType::Integer, ParameterUnit::Milliseconds, 500, 0, 60000, 1, ParameterPolicy::NextStateEntry, ParameterAccess::Writable},
    {2, "forward_speed", "Forward speed", ParameterType::Integer, ParameterUnit::Percent, 100, -100, 100, 1, ParameterPolicy::NextStateEntry, ParameterAccess::Writable},
    {3, "forward_duration", "Forward duration", ParameterType::Integer, ParameterUnit::Milliseconds, 500, 0, 60000, 1, ParameterPolicy::NextStateEntry, ParameterAccess::Writable},
    {4, "backward_speed", "Backward speed", ParameterType::Integer, ParameterUnit::Percent, -100, -100, 100, 1, ParameterPolicy::NextStateEntry, ParameterAccess::Writable},
    {5, "backward_duration", "Backward duration", ParameterType::Integer, ParameterUnit::Milliseconds, 200, 0, 60000, 1, ParameterPolicy::NextStateEntry, ParameterAccess::Writable},
    {6, "first_step_left", "First step left", ParameterType::Integer, ParameterUnit::Percent, 50, -100, 100, 1, ParameterPolicy::NextStepEntry, ParameterAccess::Writable},
    {7, "first_step_right", "First step right", ParameterType::Integer, ParameterUnit::Percent, 100, -100, 100, 1, ParameterPolicy::NextStepEntry, ParameterAccess::Writable},
    {8, "first_step_duration", "First step duration", ParameterType::Integer, ParameterUnit::Milliseconds, 2000, 0, 60000, 1, ParameterPolicy::NextStepEntry, ParameterAccess::Writable},
    {9, "second_step_duration", "Second step duration", ParameterType::Integer, ParameterUnit::Milliseconds, 3000, 0, 60000, 1, ParameterPolicy::NextStepEntry, ParameterAccess::Writable}
};
static const MachineRecipe MACHINE = {StateId::IDLE, tabla_8, 4, "state_test", tabla_9, 9};

} // namespace fsm_recipe_state_test
