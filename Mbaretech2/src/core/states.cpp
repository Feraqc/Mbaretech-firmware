#include "firmwareConfig.h"
#include "states.h"
#include "globals.h"
#if ENABLE_LOGGING
#include "dataLogging.h"
#endif

volatile State currentState = IDLE;

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
    resetElapsedTime();
#if ENABLE_LOGGING
    loggingStateChanged(previous, next);
#endif
}
