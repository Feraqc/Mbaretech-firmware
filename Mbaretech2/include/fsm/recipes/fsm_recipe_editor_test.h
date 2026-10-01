#pragma once

#include "../FSMRecipeTypes.h"

namespace fsm_recipe_editor_test {
using namespace fsm;

// Los tipos y catálogos pertenecen al firmware; aquí solo se define la receta.
static const StateRecipe tabla_0[] = {
    {StateId::IDLE, StateKind::MOTOR, MotionId::STOP, {0, 0}, nullptr, 0, nullptr}
};
static const MachineRecipe MACHINE = {StateId::IDLE, tabla_0, 1, "EDITOR_TEST"};

} // namespace fsm_recipe_editor_test
