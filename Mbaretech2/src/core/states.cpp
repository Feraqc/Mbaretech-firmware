#include "firmwareConfig.h"
#include "states.h"
#include "globals.h"
#include "telemetry/TelemetryService.h"
#include <cstring>
#if ENABLE_LOGGING
#include "dataLogging.h"
#endif

volatile State currentState = IDLE;
static uint32_t stateEnteredAtMs = 0;

uint32_t combatStateElapsedMs(uint32_t nowMs) {
    return uint32_t(nowMs - stateEnteredAtMs);
}

const char* stateName(State state) {
    switch (state) {
#define STATE_NAME(name) case name: return #name;
        FIRMWARE_STATE_LIST(STATE_NAME)
#undef STATE_NAME
    }
    return "DESCONOCIDO";
}

void changeState(State next) {
    const State previous = currentState;
    if (previous == next) return;
    currentState = next;
    stateEnteredAtMs = millis();
    TelemetryEvent event{};
    event.type = TelemetryType::FsmTransition;
    strncpy(event.machine, "COMBAT", sizeof(event.machine) - 1);
    strncpy(event.state, stateName(previous), sizeof(event.state) - 1);
    strncpy(event.next, stateName(next), sizeof(event.next) - 1);
    strncpy(event.condition, "STATE_CHANGE", sizeof(event.condition) - 1);
    telemetryPublish(event);
    resetElapsedTime();
#if ENABLE_LOGGING && !ENABLE_TELEMETRY
    loggingStateChanged(previous, next);
#endif
}
