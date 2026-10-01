#pragma once


#if defined(FSM_ACTIVE_RECIPE_TEST)
#include "recipes/fsm_recipe_test.h"
namespace active_fsm_recipe = fsm_recipe_test;
#elif defined(FSM_ACTIVE_RECIPE_TURN_CALIBRATION)
#include "recipes/fsm_recipe_turn_calibration.h"
namespace active_fsm_recipe = fsm_recipe_turn_calibration;
#elif defined(FSM_ACTIVE_RECIPE_COMBAT)
#include "recipes/fsm_recipe_combat.h"
namespace active_fsm_recipe = fsm_recipe_combat;
#elif defined(FSM_ACTIVE_RECIPE_EDITOR_TEST)
#include "recipes/fsm_recipe_editor_test.h"
namespace active_fsm_recipe = fsm_recipe_editor_test;
#elif defined(FSM_ACTIVE_RECIPE_STATE_TEST)
#include "recipes/fsm_recipe_state_test.h"
namespace active_fsm_recipe = fsm_recipe_state_test;
#endif
