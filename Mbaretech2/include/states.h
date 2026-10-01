#pragma once
#include <stdint.h>

// One ordered catalog keeps legacy numeric state IDs and telemetry names in sync.
#define FIRMWARE_STATE_LIST(X) \
    X(IDLE) \
    X(FORWARD) \
    X(BACKWARD) \
    X(TURN_RIGHT) \
    X(TURN_LEFT_45) \
    X(TURN_RIGHT_45) \
    X(TURN_RIGHT_90) \
    X(TURN_LEFT_90) \
    X(TURN_LEFT_45_IF) \
    X(TURN_RIGHT_45_IF) \
    X(TURN_RIGHT_90_IF) \
    X(TURN_LEFT_90_IF) \
    X(FORWARD_LEFT) \
    X(FORWARD_RIGHT) \
    X(MOVEMENT_45) \
    X(L_MOVEMENT_45) \
    X(R_MOVEMENT_45) \
    X(TURN_180) \
    X(BRAKE) \
    X(SHORT_LEFT_MOVE) \
    X(SHORT_RIGHT_MOVE) \
    X(LINE_RETREAT) \
    X(INITIAL_MOVEMENT) \
    X(SNAKE) \
    X(TURKISH) \
    X(GIRO_U_L) \
    X(GIRO_U_R) \
    X(GIRO_U_L_LONG) \
    X(GIRO_U_R_LONG)

enum State {
#define DECLARE_STATE(name) name,
    FIRMWARE_STATE_LIST(DECLARE_STATE)
#undef DECLARE_STATE
};

extern volatile State currentState;
const char* stateName(State state);
void changeState(State next);
uint32_t combatStateElapsedMs(uint32_t nowMs);
