#pragma once
#include "FSMRecipeTypes.h"

namespace fsm {
enum class TransitionScope : uint8_t { STATE, STEP };

// Immutable event copied to a bounded queue. Recipe/name/expression pointers must
// outlive queued events (normal recipes are static const). Motor is the source
// command when the condition fired; elapsed time uses the condition's domain.
struct TransitionEvent {
    const char* machine;
    TransitionScope scope;
    StateId state;
    StateId nextState;
    TriggerRecipe condition;
    uint32_t atMs;
    uint32_t elapsedMs;
    MotorCommand motor;
    int16_t step;      // -1 for a basic state.
    int16_t nextStep;  // STEP_COMPLETE for internal completion; -1 for state exit.
};
using TransitionObserver = void (*)(const TransitionEvent& event);

// Storage is allocation-free. Firmware callers synchronize access outside this
// class so formatting and Serial/BLE transmission never hold the queue lock.
class TransitionQueue {
public:
    bool push(const TransitionEvent& event) {
        if (count_ == fsm_defs::runtime::LOG_QUEUE_CAPACITY) {
            ++dropped_;
            return false;
        }
        events_[(head_ + count_) % fsm_defs::runtime::LOG_QUEUE_CAPACITY] = event;
        ++count_;
        return true;
    }
    bool pop(TransitionEvent& event) {
        if (!count_) return false;
        event = events_[head_];
        head_ = (head_ + 1) % fsm_defs::runtime::LOG_QUEUE_CAPACITY;
        --count_;
        return true;
    }
    uint32_t takeDropped() {
        const uint32_t result = dropped_;
        dropped_ = 0;
        return result;
    }
private:
    TransitionEvent events_[fsm_defs::runtime::LOG_QUEUE_CAPACITY]{};
    unsigned head_ = 0;
    unsigned count_ = 0;
    uint32_t dropped_ = 0;
};

// Bounded formatter, independent of Arduino. Returns false if output truncates.
bool formatTransition(const TransitionEvent& event, char* output, size_t capacity);
// Firmware producer callback: copies only, never formats, blocks or allocates.
void enqueueTransition(const TransitionEvent& event);
// Called by communications (or Arduino idle loop for Serial-only builds).
void pollRecipeTransitions();
}
