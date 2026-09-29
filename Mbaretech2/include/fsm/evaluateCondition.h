#pragma once
#include "fsm/FSMDefinitions.h"
#include "sensorSnapshot.h"
namespace fsm {
// Maps exactly one condition. AND/OR is evaluated by State in recipe order.
// Snapshot values must already be normalized. NONE and unknown IDs are false;
// recipe validation rejects unregistered IDs before execution.
bool evaluateCondition(fsm_defs::ConditionId id, const SensorSnapshot& sensors);
}
