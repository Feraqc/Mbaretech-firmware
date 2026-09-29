#include "firmwareConfig.h"
#if ENABLE_RECIPE_FSM
#include "fsm/StateMachine.h"

namespace fsm {
const StateRecipe* StateMachine::findState(StateId id) const {
    for (unsigned i = 0; i < machine_.state_count; ++i)
        if (machine_.states[i].id == id) return &machine_.states[i];
    return nullptr;
}
bool StateMachine::begin(uint32_t nowMs, RecipeErrorReporter report) {
    stop();
    error_ = validateRecipe(machine_);
    if (error_) {
        if (report) report(error_);
        return false;
    }
    running_ = true;
    transitionTo(machine_.initial_state, nowMs);
    return true;
}
void StateMachine::transitionTo(StateId id, uint32_t nowMs) {
    const auto* next = findState(id);
    if (!next) {
        error_ = "State transition target does not exist";
        stop();
        return;
    }
    active_.exit();
    active_.bind(next, &drive_, machine_.name, observer_);
    active_.enter(nowMs);
}
void StateMachine::update(const SensorSnapshot& sensors, uint32_t nowMs) {
    if (!running_) return;
    StateId next{};
    if (active_.update(sensors, nowMs, next)) transitionTo(next, nowMs);
}
void StateMachine::stop() {
    active_.exit();
    running_ = false;
    drive_.stop();
}
StateId StateMachine::currentState() const {
    return active_.id();
}
}
#endif
