#include "firmwareConfig.h"
#if ENABLE_RECIPE_FSM
#include "fsm/StateMachine.h"

namespace fsm {
void State::bind(const StateRecipe* recipe, Drive* drive,
                 const char* machineName, TransitionObserver observer) {
    recipe_ = recipe;
    drive_ = drive;
    machineName_ = machineName;
    observer_ = observer;
}
void State::recordTransition(const TriggerRecipe& condition,
                             TransitionScope scope, StateId nextState,
                             int16_t nextStep, uint32_t nowMs, uint32_t elapsedMs) const {
    if (!observer_) return;
    const bool sequence = recipe_->kind == StateKind::SUBFSM;
    const int16_t step = sequence ? activeStep_ : -1;
    const auto motor = sequence ? recipe_->subfsm->steps[activeStep_].motor : recipe_->motor;
    observer_({machineName_, scope, recipe_->id, nextState, condition,
               nowMs, elapsedMs, motor, step, nextStep});
}
void State::enter(uint32_t nowMs) {
    enteredAtMs_ = stepEnteredAtMs_ = nowMs;
    activeStep_ = 0;
    subfsmComplete_ = false;
    if (!recipe_ || !drive_) return;
    // StateMachine validates all pointers and counts before binding a recipe.
    drive_->apply(recipe_->kind == StateKind::MOTOR
                      ? recipe_->motor : recipe_->subfsm->steps[0].motor);
}
bool State::evaluateTrigger(const TriggerRecipe& trigger,
                            const SensorSnapshot& sensors, uint32_t elapsedMs,
                            bool completion) const {

    switch (trigger.type) {
    case TriggerType::TIMER: return elapsedMs >= trigger.timerMs;
    case TriggerType::COMPLETION: return completion;
    case TriggerType::SENSOR: {
        bool result = evaluateCondition(trigger.expression.terms[0], sensors);
        // Left fold, not C++ precedence: A OR B AND C means (A OR B) AND C.
        // Resolve every term even when an intermediate result is already true.
        for (unsigned i = 1; i < trigger.expression.termCount; ++i) {
            const bool term = evaluateCondition(trigger.expression.terms[i], sensors);
            result = trigger.expression.operators[i - 1] == LogicOp::AND
                         ? result && term : result || term;
        }
        return result;
    }
    }
    return false;
}
bool State::updateMotorState(const SensorSnapshot& sensors, uint32_t nowMs,
                             StateId& nextState) {
    drive_->apply(recipe_->motor);
    for (unsigned i = 0; i < recipe_->out_count; ++i) {
        const auto& transition = recipe_->out_transitions[i];
        if (evaluateTrigger(transition.trigger, sensors, uint32_t(nowMs - enteredAtMs_), false)) {
            nextState = transition.next_state;
            recordTransition(transition.trigger, TransitionScope::STATE, nextState, -1,
                             nowMs, uint32_t(nowMs - enteredAtMs_));
            return true;
        }
    }
    return false; // No implicit transition, even for a state with no outputs.
}
bool State::updateSubFsm(const SensorSnapshot& sensors, uint32_t nowMs,
                         StateId& nextState) {

    const uint32_t stateElapsed = uint32_t(nowMs - enteredAtMs_);
    // Global sensor interrupts preempt both step timers and completion.
    for (unsigned i = 0; i < recipe_->out_count; ++i) {
        const auto& transition = recipe_->out_transitions[i];
        if (transition.trigger.type == TriggerType::SENSOR &&
            evaluateTrigger(transition.trigger, sensors, stateElapsed, subfsmComplete_)) {
            nextState = transition.next_state;
            recordTransition(transition.trigger, TransitionScope::STATE, nextState, -1, nowMs, stateElapsed);
            return true;
        }
    }
    const auto& step = recipe_->subfsm->steps[activeStep_];
    drive_->apply(step.motor);
    if (!subfsmComplete_) {
        for (unsigned i = 0; i < step.out_count; ++i) {
            const auto& transition = step.out_transitions[i];
            if (!evaluateTrigger(transition.trigger, sensors, uint32_t(nowMs - stepEnteredAtMs_), false)) continue;
            recordTransition(transition.trigger, TransitionScope::STEP, recipe_->id,
                             transition.next_step, nowMs, uint32_t(nowMs - stepEnteredAtMs_));
            if (transition.next_step == STEP_COMPLETE) {
                subfsmComplete_ = true;
            } else {
                activeStep_ = transition.next_step;
                stepEnteredAtMs_ = nowMs;
                drive_->apply(recipe_->subfsm->steps[activeStep_].motor);
            }
            break; // At most one internal transition per update; no catch-up loop.
        }
    }
    // Completion has priority over remaining top-level timers regardless of
    // array position. Order within each priority group remains recipe order.
    for (unsigned pass = 0; pass < 2; ++pass) {
        const TriggerType type = pass == 0 ? TriggerType::COMPLETION : TriggerType::TIMER;
        for (unsigned i = 0; i < recipe_->out_count; ++i) {
            const auto& transition = recipe_->out_transitions[i];
            if (transition.trigger.type == type &&
                evaluateTrigger(transition.trigger, sensors, stateElapsed, subfsmComplete_)) {
                nextState = transition.next_state;
                recordTransition(transition.trigger, TransitionScope::STATE, nextState, -1, nowMs, stateElapsed);
                return true;
            }
        }
    }
    // A completed SubFSM holds its last command until an explicit exit fires.
    return false;
}
bool State::update(const SensorSnapshot& sensors, uint32_t nowMs,
                    StateId& nextState) {
    if (!recipe_ || !drive_) return false;
    return recipe_->kind == StateKind::MOTOR
               ? updateMotorState(sensors, nowMs, nextState)
               : updateSubFsm(sensors, nowMs, nextState);
}
void State::exit() {
    // Next enter() applies the replacement command. No implicit brake pulse.
    recipe_ = nullptr;
    drive_ = nullptr;
}
StateId State::id() const {
    return recipe_ ? recipe_->id : StateId{};
}
}
#endif
