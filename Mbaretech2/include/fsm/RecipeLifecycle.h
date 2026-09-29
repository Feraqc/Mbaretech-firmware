#pragma once
#include "FSMDefinitions.h"
#include "sensorSnapshot.h"

namespace fsm {
// Normal recipe ownership: START permits the lifecycle, not an extra recipe edge.
// sampledAtMs describes acquisition; startObservedAtMs describes the live latch.
inline bool canRunRecipe(const SensorSnapshot& sensors, uint32_t nowMs) {
    return sensors.startActive && sensors.valid &&
           uint32_t(nowMs - sensors.sampledAtMs) <= fsm_defs::runtime::SENSOR_MAX_AGE_MS;
}
}
