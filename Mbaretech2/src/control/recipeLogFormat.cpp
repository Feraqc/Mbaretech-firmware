#include "firmwareConfig.h"
#if ENABLE_RECIPE_FSM
#include "fsm/TransitionLog.h"
#include <stdio.h>
#include <stdarg.h>

namespace fsm {
namespace {
// Append without allocating. A truncated expression is explicitly marked below.
bool append(char* buffer, size_t capacity, size_t& used, const char* format, ...) {
    if (used >= capacity) return false;
    va_list args;
    va_start(args, format);
    const int written = vsnprintf(buffer + used, capacity - used, format, args);
    va_end(args);
    if (written < 0 || static_cast<size_t>(written) >= capacity - used) {
        used = capacity;
        return false;
    }
    used += static_cast<size_t>(written);
    return true;
}
}
bool formatTransition(const TransitionEvent& event, char* output, size_t capacity) {
    char detail[fsm_defs::runtime::LOG_EXPRESSION_BYTES]{};
    size_t used = 0;
    bool complete = true;
    const char* type = "UNKNOWN";
    switch (event.condition.type) {
    case TriggerType::TIMER:
        type = "TIMER";
        complete = append(detail, sizeof(detail), used, "%lu_ms", static_cast<unsigned long>(event.condition.timerMs));
        break;
    case TriggerType::COMPLETION:
        type = "COMPLETION";
        complete = append(detail, sizeof(detail), used, "STEP_COMPLETE");
        break;
    case TriggerType::SENSOR:
        type = "SENSOR";
        for (unsigned i = 0; i < event.condition.expression.termCount && complete; ++i) {
            if (i) complete = append(detail, sizeof(detail), used, "_%s_",
                event.condition.expression.operators[i - 1] == LogicOp::AND ? "AND" : "OR");
            if (complete) complete = append(detail, sizeof(detail), used, "%s",
                fsm_defs::conditionName(event.condition.expression.terms[i]));
        }
        break;
    }
    // Includes source command and step; STEP -1 means a basic state. For a STEP
    // event NEXT_STEP -1 means completion. For STATE events it means state exit.
    const int written = snprintf(output, capacity,
        "FSM,%s,AT_MS=%lu,SCOPE=%s,STATE=%s,CONDITION=%s,DETAIL=%s%s,"
        "ELAPSED_MS=%lu,NEXT=%s,MOTOR_L=%d,MOTOR_R=%d,STEP=%d,NEXT_STEP=%d\n",
        event.machine, static_cast<unsigned long>(event.atMs),
        event.scope == TransitionScope::STATE ? "STATE" : "STEP",
        fsm_defs::stateName(event.state), type, detail, complete ? "" : "[TRUNCATED]",
        static_cast<unsigned long>(event.elapsedMs), fsm_defs::stateName(event.nextState),
        event.motor.left_pct, event.motor.right_pct, event.step, event.nextStep);
    return complete && written >= 0 && static_cast<size_t>(written) < capacity;
}
}
#endif
