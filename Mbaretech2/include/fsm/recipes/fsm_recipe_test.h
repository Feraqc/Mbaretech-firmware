#pragma once

#include "../FSMRecipeTypes.h"

namespace fsm_recipe_test {
using namespace fsm;

// Sólo los valores elegidos para pruebas en vivo se exponen como parámetros.
static const ParameterDefinition parameters[] = {
    {1, "forward_speed", "Forward speed", ParameterType::Integer, ParameterUnit::Percent,
     100, -100, 100, 1, ParameterPolicy::NextStateEntry},
    {2, "forward_duration", "Forward duration", ParameterType::Integer, ParameterUnit::Milliseconds,
     2500, 0, 60000, 1, ParameterPolicy::NextStateEntry}
};

// Los tipos y catálogos pertenecen al firmware; aquí solo se define la receta.
static const StateTransitionRecipe tabla_0[] = {
    {{TriggerType::TIMER, 5000, {nullptr, 0, nullptr, 0}}, StateId::MOTOR_SEQUENCE}
};
static const StateTransitionRecipe tabla_1[] = {
    {{TriggerType::TIMER, 2500, {nullptr, 0, nullptr, 0}, 2}, StateId::MOTOR_SEQUENCE_1}
};
static const StateTransitionRecipe tabla_2[] = {
    {{TriggerType::TIMER, 2500, {nullptr, 0, nullptr, 0}}, StateId::MOTOR_SEQUENCE}
};
static const StateRecipe tabla_3[] = {
    {StateId::IDLE, StateKind::MOTOR, MotionId::STOP, {0, 0}, tabla_0, 1, nullptr},
    {StateId::MOTOR_SEQUENCE, StateKind::MOTOR, MotionId::STOP, {100, 100, 1, 1}, tabla_1, 1, nullptr},
    {StateId::MOTOR_SEQUENCE_1, StateKind::MOTOR, MotionId::STOP, {-100, -100}, tabla_2, 1, nullptr}
};
static const MachineRecipe MACHINE = {StateId::IDLE, tabla_3, 3, "TURN_CALIBRATION", parameters, 2};

} // namespace fsm_recipe_test
