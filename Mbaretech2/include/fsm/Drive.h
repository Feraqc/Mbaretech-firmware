#pragma once
#include "fsm/FSMRecipeTypes.h"
class Motor;
namespace fsm {
// The only bridge from signed recipe commands to the single-wheel driver.
class Drive {
public:
    // Driver references must outlive this adapter; construction performs no IO.
    Drive(Motor& left, Motor& right) : left_(left), right_(right) {}
    // Initialize both drivers and establish a stopped output before any command.
    void begin();
    // Percentages are signed direction commands, not measured wheel speed.
    void apply(const MotorCommand& command);
    // May be called repeatedly by lifecycle handling; both wheels brake.
    void stop();
private:
    Motor& left_;
    Motor& right_;
    static void applySigned(Motor& motor, int8_t percent);
};
}
