#include "firmwareConfig.h"
#if ENABLE_RECIPE_FSM
#include "fsm/Drive.h"
#include "motor.h"
namespace fsm {
void Drive::begin() {
    motionEnabled_ = false;
    left_.begin();
    right_.begin();
    stop();
}
void Drive::applySigned(Motor& motor, int8_t percent) {
    // Widen before negation so int8_t minimum (-128) is handled safely.
    int speed = percent;
    if (speed > fsm_defs::motor::MAX_PERCENT) speed = fsm_defs::motor::MAX_PERCENT;
    if (speed < fsm_defs::motor::MIN_PERCENT) speed = fsm_defs::motor::MIN_PERCENT;
    if (speed > 0) motor.forward(static_cast<uint32_t>(speed));
    else if (speed < 0) motor.backward(static_cast<uint32_t>(-speed));
    else motor.brake();
}
void Drive::apply(const MotorCommand& command) {
    requested_ = command;
    applyRequested();
}
void Drive::setMotionEnabled(bool enabled) {
    motionEnabled_ = enabled;
    applyRequested();
}
void Drive::applyRequested() {
    if (!motionEnabled_) {
        left_.brake();
        right_.brake();
        return;
    }
    applySigned(left_, requested_.left_pct);
    applySigned(right_, requested_.right_pct);
}
void Drive::stop() {
    requested_ = {0, 0};
    left_.brake();
    right_.brake();
}
}
#endif
