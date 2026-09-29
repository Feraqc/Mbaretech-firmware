#include "firmwareConfig.h"
#if ENABLE_FSM || defined(COMBAT_FSM_HOST_TEST)
#include "combatFsm.h"

namespace {
constexpr uint8_t sensorBit(Sensor sensor) { return 1u << sensor; }
constexpr uint32_t RETREAT_MINIMUM_MS = 80;
constexpr uint32_t SHORT_MOVE_MS = 80;
constexpr uint32_t COMPOUND_STRAIGHT_MS = 150;
constexpr uint32_t U_TURN_MS = 1000;
constexpr uint32_t LONG_U_TURN_MS = 2000;
constexpr uint8_t CENTER = sensorBit(TOP_MID);
constexpr uint8_t FRONT = CENTER | sensorBit(SHORT_LEFT) | sensorBit(SHORT_RIGHT) | sensorBit(TOP_LEFT) | sensorBit(TOP_RIGHT);
constexpr uint8_t ALL_IR = FRONT | sensorBit(SIDE_LEFT) | sensorBit(SIDE_RIGHT);
constexpr uint8_t LEFT_IR = CENTER | sensorBit(SHORT_LEFT) | sensorBit(TOP_LEFT) | sensorBit(SIDE_LEFT);
constexpr uint8_t RIGHT_IR = CENTER | sensorBit(SHORT_RIGHT) | sensorBit(TOP_RIGHT) | sensorBit(SIDE_RIGHT);

State selectTargetState(const SensorSnapshot& sensors, uint8_t mask) {
    const Sensor order[] = {TOP_MID, SHORT_LEFT, SHORT_RIGHT, TOP_LEFT, TOP_RIGHT, SIDE_LEFT, SIDE_RIGHT};
    const State states[] = {FORWARD, SHORT_LEFT_MOVE, SHORT_RIGHT_MOVE, TURN_LEFT_45,
                            TURN_RIGHT_45, TURN_LEFT_90, TURN_RIGHT_90};
    for (unsigned i = 0; i < 7; ++i) {
        if ((mask & sensorBit(order[i])) && sensors.ir[order[i]]) {
            return states[i];
        }
    }
    return BRAKE;
}

struct Phase {
    MotorCommand motors;
    uint32_t durationMs;
    uint8_t cancelSensorMask;
    bool isFinalPhase;
};
Phase leftTurn(uint32_t durationMs, uint8_t mask = CENTER, bool isFinalPhase = true) {
    return {{-(TURN_LEFT_SPEED + CORRECT_SPEED), TURN_LEFT_SPEED}, durationMs, mask, isFinalPhase};
}
Phase rightTurn(uint32_t durationMs, uint8_t mask = CENTER, bool isFinalPhase = true) {
    return {{TURN_RIGHT_SPEED + CORRECT_SPEED, -TURN_RIGHT_SPEED}, durationMs, mask, isFinalPhase};
}
bool describeManeuverPhase(State state, uint8_t phaseIndex, Phase& description) {
    switch (state) {
        case TURN_LEFT_45:
            description = leftTurn(TURN_LEFT_45_DELAY);
            break;
        case TURN_LEFT_90:
            description = leftTurn(TURN_LEFT_90_DELAY);
            break;
        case TURN_RIGHT_45:
            description = rightTurn(TURN_RIGHT_45_DELAY);
            break;
        case TURN_RIGHT_90:
            description = rightTurn(TURN_RIGHT_90_DELAY);
            break;
        case TURN_180:
            description = leftTurn(TURN_LEFT_180_DELAY, CENTER | sensorBit(SIDE_LEFT));
            break;
        case SHORT_LEFT_MOVE:
            description = {{FORWARD_42 + CORRECT_SPEED, FORWARD_90}, SHORT_MOVE_MS, 0, true};
            break;
        case SHORT_RIGHT_MOVE:
            description = {{FORWARD_90 + CORRECT_SPEED, FORWARD_42}, SHORT_MOVE_MS, 0, true};
            break;
        case FORWARD_LEFT:
            description = {{FORWARD_49, FORWARD_90}, SHORT_MOVE_MS, 0, true};
            break;
        case FORWARD_RIGHT:
            description = {{FORWARD_90, FORWARD_42}, SHORT_MOVE_MS, 0, true};
            break;
        case GIRO_U_L:
        case GIRO_U_L_LONG:
            description = {{FORWARD_60 + CORRECT_SPEED, FORWARD_90},
                 state == GIRO_U_L ? U_TURN_MS : LONG_U_TURN_MS, FRONT, true};
            break;
        case GIRO_U_R:
        case GIRO_U_R_LONG:
            description = {{FORWARD_90 + CORRECT_SPEED, FORWARD_60},
                 state == GIRO_U_R ? U_TURN_MS : LONG_U_TURN_MS, FRONT, true};
            break;
        case L_MOVEMENT_45:
            if (phaseIndex == 0) description = leftTurn(TURN_LEFT_45_DELAY, 0, false);
            else if (phaseIndex == 1) description = {{FORWARD_90 + CORRECT_SPEED, FORWARD_90}, COMPOUND_STRAIGHT_MS, RIGHT_IR, false};
            else description = rightTurn(TURN_RIGHT_90_DELAY, ALL_IR);
            break;
        case R_MOVEMENT_45:
            if (phaseIndex == 0) description = rightTurn(TURN_RIGHT_45_DELAY, 0, false);
            else if (phaseIndex == 1) description = {{FORWARD_90, FORWARD_90}, COMPOUND_STRAIGHT_MS, LEFT_IR, false};
            else description = leftTurn(TURN_LEFT_90_DELAY, ALL_IR);
            break;
        case MOVEMENT_45:
            description = {{-TURN_LEFT_SPEED, TURN_LEFT_SPEED}, TURN_LEFT_90_DELAY, FRONT | sensorBit(SIDE_RIGHT), true};
            break;
        default: return false;
    }
    return true;
}
}

void CombatFsm::enter(State next, uint32_t nowMs) {
    state_ = next;
    phase_ = 0;
    phaseStarted_ = nowMs;
}

void CombatFsm::selectOpening(const SensorSnapshot& sensors, uint32_t nowMs) {
    // Preserve the existing E/A/B/C opening table; DIP D is sampled but unused.
#if ENABLE_DIP_SWITCHES
    const unsigned selection = (sensors.dip[DIP_E] << 3) | (sensors.dip[DIP_A] << 2) |
                               (sensors.dip[DIP_B] << 1) | unsigned(sensors.dip[DIP_C]);
#else
    const unsigned selection = FSM_DEFAULT_OPENING;
#endif
    static const State openings[16] = {
        FORWARD, FORWARD, BRAKE, BRAKE, TURN_LEFT_90, TURN_RIGHT_90, TURN_180,
        L_MOVEMENT_45, R_MOVEMENT_45, GIRO_U_L, GIRO_U_R, GIRO_U_L_LONG,
        GIRO_U_R_LONG, SHORT_LEFT_MOVE, SHORT_RIGHT_MOVE, GIRO_U_L
    };
    snake_ = selection == 1 || selection == 3 || selection == 13 || selection == 14;
    turkish_ = selection == 2 || selection == 3 || (selection >= 7 && selection <= 14);
    lastTurkish_ = nowMs;
    enter(openings[selection], nowMs);
}

MotorCommand CombatFsm::stepForward(const SensorSnapshot& sensors, uint32_t nowMs) {
    const bool both = sensors.ir[SHORT_LEFT] && sensors.ir[SHORT_RIGHT];
    if (sensors.ir[SHORT_LEFT] && !sensors.ir[SHORT_RIGHT]) return {FORWARD_60 + CORRECT_SPEED, MAX_SPEED};
    if (sensors.ir[SHORT_RIGHT] && !sensors.ir[SHORT_LEFT]) return {MAX_SPEED, 52};
    if (!sensors.ir[TOP_MID] && !both) {
        // Keep searching forward unless the DIP-selected strategy waits for a target.
        const State detected = selectTargetState(sensors, ALL_IR);
        if (detected != BRAKE || turkish_) {
            enter(detected, nowMs);
            return {};
        }
    }
    if (snake_) {
        const uint32_t durationMs = (phase_ == 0 ? SHORT_RIGHT_DELAY : SHORT_LEFT_DELAY) + (both ? 0 : 10);
        if (uint32_t(nowMs - phaseStarted_) >= durationMs) {
            phase_ ^= 1;
            phaseStarted_ = nowMs;
        }
#ifdef MBARETECH_2
        return phase_ == 0 ? MotorCommand{95, 85} : MotorCommand{85, 95};
#else
        if (both) return phase_ == 0 ? MotorCommand{95, 85} : MotorCommand{85, 95};
        return phase_ == 0 ? MotorCommand{FORWARD_90, FORWARD_42} : MotorCommand{FORWARD_49, FORWARD_90};
#endif
    }
#ifdef MBARETECH_2
    const int speed = both ? MAX_SPEED : FORWARD_80;
#else
    const int speed = both ? MAX_SPEED : FORWARD_70;
#endif
    return {speed, speed};
}

MotorCommand CombatFsm::step(const SensorSnapshot& sensors, bool started, uint32_t nowMs) {
    // 1. Stop and acquisition health override every strategy and phase.
    if (!started || !sensors.valid || uint32_t(nowMs - sensors.sampledAtMs) > SENSOR_MAX_AGE_MS) {
        enter(IDLE, nowMs);
        snake_ = turkish_ = false;
        return {};
    }
    // 2. Latch the opening only when leaving IDLE.
    if (state_ == IDLE) selectOpening(sensors, nowMs);

    // Border has priority over targets in every maneuver. Do not restart the
    // retreat deadline on repeated samples; remain reversing while on the border.
    if ((sensors.line[0] || sensors.line[1]) && state_ != LINE_RETREAT) enter(LINE_RETREAT, nowMs);
    switch (state_) {
        case LINE_RETREAT: return stepRetreat(sensors, nowMs);
        case FORWARD: return stepForward(sensors, nowMs);
        case BRAKE: return stepSearch(sensors, nowMs);
        default: return stepManeuver(sensors, nowMs);
    }
}

MotorCommand CombatFsm::stepRetreat(const SensorSnapshot& sensors, uint32_t nowMs) {
    if (uint32_t(nowMs - phaseStarted_) >= RETREAT_MINIMUM_MS && !sensors.line[0] && !sensors.line[1]) {
        enter(selectTargetState(sensors, FRONT) != BRAKE ? BRAKE : TURN_180, nowMs);
        return {};
    }
    return {-FORWARD_90, -FORWARD_90};
}

MotorCommand CombatFsm::stepSearch(const SensorSnapshot& sensors, uint32_t nowMs) {
    const State detected = selectTargetState(sensors, ALL_IR);
    if (detected != BRAKE) {
        enter(detected, nowMs);
        return {};
    }
    if (!turkish_) {
        enter(FORWARD, nowMs);
        return {};
    }
    if (phase_ == 0 && uint32_t(nowMs - lastTurkish_) >= TURKISH_TIME) {
        phase_ = 1;
        phaseStarted_ = nowMs;
    }
    if (phase_ == 1) {
        if (uint32_t(nowMs - phaseStarted_) < TURKISH_DELAY)
            return {FORWARD_60 + CORRECT_SPEED, FORWARD_60};
        phase_ = 0;
        lastTurkish_ = nowMs;
    }
    return {};
}

MotorCommand CombatFsm::stepManeuver(const SensorSnapshot& sensors, uint32_t nowMs) {
    Phase description;
    if (!describeManeuverPhase(state_, phase_, description)) {
        enter(IDLE, nowMs);
        return {};
    }
#if ENABLE_TURN_CANCEL
    const State detected = selectTargetState(sensors, description.cancelSensorMask);
    if (detected != BRAKE) {
        enter(detected, nowMs);
        return {};
    }
#endif
    if (uint32_t(nowMs - phaseStarted_) >= description.durationMs) {
        if (description.isFinalPhase) {
            enter(BRAKE, nowMs);
            return {};
        }
        ++phase_;
        phaseStarted_ = nowMs; // Each physical phase gets its full duration after a late tick.
        if (!describeManeuverPhase(state_, phase_, description)) {
            enter(IDLE, nowMs);
            return {};
        }
    }
    return description.motors;
}
#endif
