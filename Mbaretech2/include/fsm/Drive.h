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
    // Retain the latest signed request even when motion permission is disabled.
    void apply(const MotorCommand& command);
    // False brakes immediately without discarding the request. True immediately
    // applies that request, without requiring another FSM update or transition.
    // ENABLE_MOTORS remains the independent compile-time gate in Motor.
    void setMotionEnabled(bool enabled);
    // A control stop brakes and clears the request; cannot replay an old command.
    // Does not change permission, so begin()/enter() can use the caller's gate.
    void stop();
    // Effective output for diagnostics; START permission masks a retained request.
    MotorCommand outputCommand() const { return motionEnabled_ ? requested_ : MotorCommand{0, 0}; }
private:
    Motor& left_;
    Motor& right_;
    MotorCommand requested_{0, 0};
    bool motionEnabled_ = false;
    void applyRequested();
    static void applySigned(Motor& motor, int8_t percent);
};
}
