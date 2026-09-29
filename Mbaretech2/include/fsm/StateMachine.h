#pragma once
#include "fsm/Drive.h"
#include "fsm/evaluateCondition.h"
#include "fsm/TransitionLog.h"

namespace fsm {
using RecipeErrorReporter = void (*)(const char* message);
// Returns the first error (static text), or nullptr. No allocation or IO here.
const char* validateRecipe(const MachineRecipe& machine);

// One runtime object handles every recipe state; tables are never copied.
// Namespace avoids colliding with the legacy global enum State.
class State {
public:
    // Precondition: tables were accepted by validateRecipe and outlive this state.
    // Observer must be bounded/nonblocking; firmware uses enqueueTransition.
    void bind(const StateRecipe* recipe, Drive* drive,
              const char* machineName = "UNNAMED", TransitionObserver observer = nullptr);
    // Apply entry command immediately and reset both timing domains.
    void enter(uint32_t nowMs);
    // Return a requested top-level transition. Internal step changes stay here.
    bool update(const SensorSnapshot& sensors, uint32_t nowMs,
                StateId& nextState);
    // Releases the bound table without inserting a brake pulse between states.
    void exit();
    StateId id() const;
private:
    const StateRecipe* recipe_ = nullptr;
    Drive* drive_ = nullptr;
    uint32_t enteredAtMs_ = 0;
    int16_t activeStep_ = 0;
    uint32_t stepEnteredAtMs_ = 0;
    bool subfsmComplete_ = false;
    const char* machineName_ = nullptr;
    TransitionObserver observer_ = nullptr;
    void recordTransition(const TriggerRecipe& condition,
                          TransitionScope scope, StateId nextState,
                          int16_t nextStep, uint32_t nowMs, uint32_t elapsedMs) const;
    bool evaluateTrigger(const TriggerRecipe& trigger,
                         const SensorSnapshot& sensors, uint32_t elapsedMs,
                         bool completion) const;
    bool updateMotorState(const SensorSnapshot& sensors, uint32_t nowMs,
                          StateId& nextState);
    bool updateSubFsm(const SensorSnapshot& sensors, uint32_t nowMs,
                      StateId& nextState);
};

class StateMachine {
public:
    // References only; neither construction nor observer installation starts IO.
    StateMachine(const MachineRecipe& recipe, Drive& drive,
                 TransitionObserver observer = nullptr)
        : machine_(recipe), drive_(drive), observer_(observer) {}
    // Validation precedes entry, so a malformed recipe never commands movement.
    bool begin(uint32_t nowMs, RecipeErrorReporter report = nullptr);
    // One bounded update, no sensor IO. Caller owns START and freshness policy.
    void update(const SensorSnapshot& sensors, uint32_t nowMs);
    // Stop is a lifecycle decision, not an implicit recipe transition.
    void stop();
    bool isRunning() const { return running_; }
    // Returns zero-valued ID when stopped; use isRunning() to distinguish that.
    StateId currentState() const;
    // Error text has static lifetime and is cleared by the next begin().
    const char* error() const { return error_; }
private:
    const MachineRecipe& machine_;
    Drive& drive_;
    State active_;
    bool running_ = false;
    const char* error_ = nullptr;
    TransitionObserver observer_ = nullptr;
    const StateRecipe* findState(StateId id) const;
    void transitionTo(StateId id, uint32_t nowMs);
};
}
