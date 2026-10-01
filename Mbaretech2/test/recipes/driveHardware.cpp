#include <cassert>
#include <cstdio>
#include "motor.h"
#include "fsm/RecipeLifecycle.h"

int main() {
    // Use the production Motor and Drive; only the GPIO/LEDC boundary is mocked.
    Motor left(1, 2, 3, 0), right(4, 5, 6, 1);
    fsm::Drive drive(left, right);
    drive.begin();
    drive.apply({30, -30});
    assert(appliedDuty[0] == 0 && appliedDuty[1] == 0);
    assert(pins[2] == 0 && pins[3] == 0 && pins[5] == 0 && pins[6] == 0);
    drive.setMotionEnabled(true);
#if ENABLE_MOTORS
    assert(appliedDuty[0] == 306 && appliedDuty[1] == 306);
    assert(pins[2] == 0 && pins[3] == 1 && pins[5] == 1 && pins[6] == 0);
#endif
    drive.setMotionEnabled(false);
    assert(appliedDuty[0] == 0 && appliedDuty[1] == 0);
    assert(pins[2] == 0 && pins[3] == 0 && pins[5] == 0 && pins[6] == 0);
    drive.apply({-40, 40}); // The newest request replaces the old one while gated.
    SensorSnapshot snapshot{};
    snapshot.startActive = false;
    drive.setMotionEnabled(fsm::effectiveStartActive(snapshot));
#if ENABLE_MOTORS && FORCE_START_ACTIVE
    assert(appliedDuty[0] == 409 && appliedDuty[1] == 409);
#else
    assert(appliedDuty[0] == 0 && appliedDuty[1] == 0);
#endif
    snapshot.startActive = true;
    drive.setMotionEnabled(fsm::effectiveStartActive(snapshot));
#if ENABLE_MOTORS
    assert(appliedDuty[0] == 409 && appliedDuty[1] == 409);
    assert(pins[2] == 1 && pins[3] == 0 && pins[5] == 0 && pins[6] == 1);
#endif
    drive.stop(); // A real stop clears the retained command.
    drive.setMotionEnabled(false);
    drive.setMotionEnabled(true);
    assert(appliedDuty[0] == 0 && appliedDuty[1] == 0);
#if !ENABLE_MOTORS
    assert(ioWrites == 0); // Override must never bypass the master hardware gate.
#endif
    std::puts("Drive GPIO/PWM permission tests passed");
}
