#include "firmwareConfig.h"
#if ENABLE_RECIPE_FSM
#include "fsm/TransitionLog.h"
#include "bluetoothComm.h"
#include <stdio.h>

namespace fsm {
namespace {
TransitionQueue events;
portMUX_TYPE eventMux = portMUX_INITIALIZER_UNLOCKED;
void transmit(const char* line) {
#if ENABLE_LOGGING
    sendData(String(line));
#else
    Serial.print(line);
#endif
}
}
void enqueueTransition(const TransitionEvent& event) {
    if (!fsm_defs::runtime::TRANSITION_LOGGING) return;
    portENTER_CRITICAL(&eventMux);
    events.push(event);
    portEXIT_CRITICAL(&eventMux);
}
void pollRecipeTransitions() {
    if (!fsm_defs::runtime::TRANSITION_LOGGING) return;
    char line[fsm_defs::runtime::LOG_LINE_BYTES];
    for (unsigned i = 0; i < fsm_defs::runtime::LOG_EVENTS_PER_POLL; ++i) {
        TransitionEvent event{};
        portENTER_CRITICAL(&eventMux);
        const bool available = events.pop(event);
        portEXIT_CRITICAL(&eventMux);
        if (!available) break;
        const bool complete = formatTransition(event, line, sizeof(line));
        // Guard line framing even if a future machine name exceeds the buffer.
        if (!complete) {
            line[sizeof(line) - 2] = '\n';
            line[sizeof(line) - 1] = '\0';
        }
        transmit(line);
        if (!complete) transmit("FSM_LOG_TRUNCATED\n");
    }
    portENTER_CRITICAL(&eventMux);
    const uint32_t dropped = events.takeDropped();
    portEXIT_CRITICAL(&eventMux);
    if (dropped) {
        snprintf(line, sizeof(line), "FSM_LOG_DROPPED,%lu\n", static_cast<unsigned long>(dropped));
        transmit(line);
    }
}
}
#endif
