#pragma once
#include "sensorTasks.h"

// One bounded step, with no IO, delay, or busy wait. Signed percentages:
// positive = forward, negative = backward, zero = brake.
struct MotorCommand {
    int left, right;
    constexpr MotorCommand(int leftSpeed = 0, int rightSpeed = 0)
        : left(leftSpeed), right(rightSpeed) {}
};

class CombatFsm {
public:
    MotorCommand step(const SensorSnapshot& sensors, bool started, uint32_t nowMs);
    State state() const { return state_; }
private:
    State state_ = IDLE;
    uint8_t phase_ = 0;
    uint32_t phaseStarted_ = 0, lastTurkish_ = 0;
    bool snake_ = false, turkish_ = false;
    void enter(State next, uint32_t nowMs);
    void selectOpening(const SensorSnapshot& sensors, uint32_t nowMs);
    MotorCommand stepForward(const SensorSnapshot& sensors, uint32_t nowMs);
    MotorCommand stepRetreat(const SensorSnapshot& sensors, uint32_t nowMs);
    MotorCommand stepSearch(const SensorSnapshot& sensors, uint32_t nowMs);
    MotorCommand stepManeuver(const SensorSnapshot& sensors, uint32_t nowMs);
};
