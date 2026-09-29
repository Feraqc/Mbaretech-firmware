#pragma once
#include "../FSMRecipeTypes.h"

// Do not substitute a simplified strategy for CombatFsm. The fixed GUI schema
// cannot encode DIP openings, negative sensor tests, or persistent Turkish time.
// See docs/firmware.md for the migration blockers. Existing combat builds continue
// to use CombatFsm; selecting this unimplemented translation fails explicitly.
#error "Combat recipe unavailable: fixed schema cannot preserve existing combat behavior"
namespace fsm_recipe_combat { using namespace fsm; }
