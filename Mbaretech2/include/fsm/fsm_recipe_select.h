#pragma once

// Selection is exclusively a source/build choice. Reject ambiguous selections.
#if (defined(FSM_ACTIVE_RECIPE_MOTOR_TEST) + defined(FSM_ACTIVE_RECIPE_TURN_CALIBRATION) + defined(FSM_ACTIVE_RECIPE_COMBAT)) != 1
#error "Select exactly one FSM_ACTIVE_RECIPE_* header"
#endif

#if defined(FSM_ACTIVE_RECIPE_MOTOR_TEST)
#include "recipes/fsm_recipe_motor_test.h"
namespace active_fsm_recipe = fsm_recipe_motor_test;
#elif defined(FSM_ACTIVE_RECIPE_TURN_CALIBRATION)
#include "recipes/fsm_recipe_turn_calibration.h"
namespace active_fsm_recipe = fsm_recipe_turn_calibration;
#elif defined(FSM_ACTIVE_RECIPE_COMBAT)
#include "recipes/fsm_recipe_combat.h"
namespace active_fsm_recipe = fsm_recipe_combat;
#endif
