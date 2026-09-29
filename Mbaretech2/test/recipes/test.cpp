#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <new>
#include <cstring>
#include "fsm/RecipeLifecycle.h"
#include "fsm/fsm_recipe_select.h"
#include "motor.h"
#include "fsm/StateMachine.h"

// The update loop must not allocate. Include recipe begin/stop in this check.
static unsigned allocations = 0;
void* operator new(size_t size) {
    ++allocations;
    if (void* memory = std::malloc(size ? size : 1)) return memory;
    throw std::bad_alloc();
}
void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, size_t) noexcept { std::free(memory); }
using namespace fsm;

static constexpr StateId A = StateId::IDLE;
static constexpr StateId B = StateId::MOTOR_SEQUENCE;
static constexpr StateId C = StateId::DONE;
static constexpr ConditionId CENTER_TERMS[] = {ConditionId::IR4_DETECTED};
static constexpr TriggerRecipe CENTER = {TriggerType::SENSOR, 0, {CENTER_TERMS, 1, nullptr, 0}};
static constexpr TriggerRecipe COMPLETE = {TriggerType::COMPLETION, 0, {nullptr, 0, nullptr, 0}};
static TriggerRecipe timer(uint32_t ms) { return {TriggerType::TIMER, ms, {nullptr, 0, nullptr, 0}}; }
static StateRecipe motorState(StateId id, int8_t speed, const StateTransitionRecipe* out = nullptr, uint8_t count = 0) {
    return {id, StateKind::MOTOR, MotionId::FORWARD, {speed, speed}, out, count, nullptr};
}
static bool reported = false;
static void report(const char* error) { assert(error); reported = true; }
static TransitionEvent recorded[16]{};
static unsigned eventCount = 0;
static void record(const TransitionEvent& event) {
    assert(eventCount < 16);
    recorded[eventCount++] = event;
}

static void testDriveAndConditions() {
    Motor left, right;
    Drive drive(left, right);
    drive.begin();
    drive.apply({127, -128});
    assert(left.command == 100 && right.command == -100);
    drive.apply({0, 1});
    assert(left.command == 0 && right.command == 1);
    drive.apply({-1, 0});
    assert(left.command == -1 && right.command == 0);
    drive.stop();
    assert(left.command == 0 && right.command == 0);
    SensorSnapshot sample;
    assert(!evaluateCondition(ConditionId::NONE, sample));
    sample.startActive = true;
    assert(evaluateCondition(ConditionId::START_ACTIVE, sample));
    for (unsigned i = 0; i < 7; ++i) {
        sample.ir[i] = true;
        const auto id = static_cast<ConditionId>(static_cast<unsigned>(ConditionId::IR1_DETECTED) + i);
        assert(evaluateCondition(id, sample));
        sample.ir[i] = false;
        assert(!evaluateCondition(id, sample));
    }
    sample.line[0] = true;
    assert(evaluateCondition(ConditionId::LINE_LEFT_DETECTED, sample));
    assert(!evaluateCondition(ConditionId::LINE_RIGHT_DETECTED, sample));
    sample.line[1] = true;
    assert(evaluateCondition(ConditionId::LINE_RIGHT_DETECTED, sample));
}

static void testMotorTransitions() {
    Motor left, right;
    Drive drive(left, right);
    // Two equal deadlines: the first transition must win, including wraparound.
    const StateTransitionRecipe out[] = {{timer(10), B}, {timer(10), C}};
    const StateRecipe states[] = {motorState(A, 10, out, 2), motorState(B, -20), motorState(C, 30)};
    const MachineRecipe recipe = {A, states, 3, "TEST"};
    StateMachine machine(recipe, drive);
    assert(machine.begin(UINT32_MAX - 4));
    assert(left.command == 10);
    machine.update({}, 4);
    assert(machine.currentState() == A);
    machine.update({}, 5);
    assert(machine.currentState() == B && left.command == -20);
    machine.update({}, 1000);
    assert(machine.currentState() == B); // No implicit transition.
    machine.stop();
    machine.update({}, 1001);
    assert(left.command == 0 && right.command == 0);

    // A OR B AND C must be folded left-to-right, not using C++ precedence.
    const ConditionId terms[] = {ConditionId::IR1_DETECTED, ConditionId::IR2_DETECTED, ConditionId::IR3_DETECTED};
    const LogicOp ops[] = {LogicOp::OR, LogicOp::AND};
    const StateTransitionRecipe conditionOut[] = {{{TriggerType::SENSOR, 0, {terms, 3, ops, 2}}, B}};
    const StateRecipe conditionStates[] = {motorState(A, 0, conditionOut, 1), motorState(B, 50)};
    const MachineRecipe conditionRecipe = {A, conditionStates, 2, "TEST"};
    StateMachine conditions(conditionRecipe, drive);
    assert(conditions.begin(0));
    SensorSnapshot sample;
    sample.ir[0] = true;
    conditions.update(sample, 0);
    assert(conditions.currentState() == A);
    sample.ir[2] = true;
    conditions.update(sample, 0);
    assert(conditions.currentState() == B && left.command == 50);
}

static void testSubFsm() {
    Motor left, right;
    Drive drive(left, right);
    const StepTransitionRecipe firstOut[] = {{timer(10), 1}};
    const StepTransitionRecipe lastOut[] = {{timer(20), STEP_COMPLETE}};
    const StepRecipe steps[] = {
        {MotionId::FORWARD, {10, 10}, firstOut, 1},
        {MotionId::BACKWARD, {-20, -20}, lastOut, 1}
    };
    SubFsmRecipe subfsm = {steps, 2, false};
    // Timer appears first, but condition interrupts and completion outrank it.
    const StateTransitionRecipe out[] = {{timer(100), C}, {COMPLETE, B}, {CENTER, C}};
    StateRecipe states[] = {
        {A, StateKind::SUBFSM, MotionId::NONE, {0, 0}, out, 3, &subfsm},
        motorState(B, 0), motorState(C, 30)
    };
    const MachineRecipe recipe = {A, states, 3, "TEST"};
    StateMachine machine(recipe, drive);
    assert(machine.begin(0) && left.command == 10);
    machine.update({}, 50); // Late update starts the second step at 50, not 10.
    assert(machine.currentState() == A && left.command == -20);
    machine.update({}, 69);
    assert(machine.currentState() == A);
    machine.update({}, 100); // Completion wins over the simultaneous state timer.
    assert(machine.currentState() == B && left.command == 0);

    assert(machine.begin(0));
    SensorSnapshot detected;
    detected.ir[3] = true;
    machine.update(detected, 10); // Interrupt preempts internal step change.
    assert(machine.currentState() == C && left.command == 30);
    assert(machine.begin(0));
    machine.update({}, 10);
    machine.update(detected, 30); // Interrupt also preempts completion.
    assert(machine.currentState() == C);

    states[0].out_count = 0;
    states[0].out_transitions = nullptr;
    assert(!machine.begin(0)); // Unsafe implicit hold is rejected.
    subfsm.allowHoldOnCompletion = true;
    assert(machine.begin(0));
    machine.update({}, 10);
    machine.update({}, 30);
    machine.update({}, 10000);
    assert(machine.currentState() == A && left.command == -20); // Completed but no exit.

    // Internal conditions use the same ordering rule as timers. The first matching
    // output must win even when a later output would complete the sequence.
    const StepTransitionRecipe sensorOut[] = {{CENTER, 1}, {timer(0), STEP_COMPLETE}};
    const StepRecipe sensorSteps[] = {
        {MotionId::FORWARD, {12, 12}, sensorOut, 2},
        {MotionId::BACKWARD, {-15, -15}, nullptr, 0}
    };
    const SubFsmRecipe sensorSequence = {sensorSteps, 2, true};
    states[0].subfsm = &sensorSequence;
    assert(machine.begin(0));
    machine.update(detected, 0);
    assert(left.command == -15 && machine.currentState() == A);

    // Zero-duration self-loop is bounded to one step transition per update.
    const StepTransitionRecipe loopOut[] = {{timer(0), 0}};
    const StepRecipe loopSteps[] = {{MotionId::FORWARD, {5, 5}, loopOut, 1}};
    const SubFsmRecipe loop = {loopSteps, 1, false};
    states[0].subfsm = &loop;
    assert(machine.begin(0));
    machine.update({}, 0);
    assert(machine.currentState() == A && left.command == 5);
}

static void testValidation() {
    Motor left, right;
    Drive drive(left, right);
    StateRecipe states[] = {motorState(A, 0), motorState(B, 0)};
    MachineRecipe recipe = {A, states, 2, "TEST"};
    assert(!validateRecipe(recipe));
    recipe.initial_state = C;
    assert(validateRecipe(recipe));
    recipe.initial_state = A;
    recipe.state_count = 0;
    assert(validateRecipe(recipe));
    recipe.state_count = 2;
    states[1].id = A;
    assert(validateRecipe(recipe));
    states[1].id = B;
    states[0].motor.left_pct = 101;
    StateMachine invalid(recipe, drive);
    assert(!invalid.begin(0, report) && reported && invalid.error());
    invalid.update({}, 10);
    assert(left.movementCalls == 0 && right.movementCalls == 0);
    states[0].motor.left_pct = 0;
    StateTransitionRecipe out[] = {{timer(1), C}};
    states[0].out_transitions = out;
    states[0].out_count = 1;
    assert(validateRecipe(recipe)); // Missing target.
    out[0].next_state = B;
    out[0].trigger = COMPLETE;
    assert(validateRecipe(recipe)); // Basic completion is unsupported.
    out[0].trigger = {TriggerType::SENSOR, 0, {nullptr, 0, nullptr, 0}};
    assert(validateRecipe(recipe)); // Empty expression.
    const ConditionId terms[] = {ConditionId::START_ACTIVE, ConditionId::IR1_DETECTED};
    out[0].trigger = {TriggerType::SENSOR, 0, {terms, 2, nullptr, 1}};
    assert(validateRecipe(recipe)); // Missing operator table.
    const LogicOp badOps[] = {static_cast<LogicOp>(99)};
    out[0].trigger.expression.operators = badOps;
    assert(validateRecipe(recipe));
    const LogicOp validOps[] = {LogicOp::OR};
    out[0].trigger.expression.operators = validOps;
    out[0].trigger.expression.operatorCount = 0;
    assert(validateRecipe(recipe)); // Too few operators, checked before reading.
    out[0].trigger.expression.operatorCount = 2;
    assert(validateRecipe(recipe)); // Too many; array only holds one, never indexed.
    out[0].trigger.expression.operatorCount = 1;
    assert(!validateRecipe(recipe));
    out[0].trigger.expression.termCount = 1;
    assert(validateRecipe(recipe)); // A single term requires zero operators.
    const ConditionId badTerms[] = {static_cast<ConditionId>(99)};
    out[0].trigger = {TriggerType::SENSOR, 0, {badTerms, 1, nullptr, 0}};
    assert(validateRecipe(recipe));
    out[0].trigger = {static_cast<TriggerType>(99), 0, {nullptr, 0, nullptr, 0}};
    assert(validateRecipe(recipe));
    states[0].out_count = 0;
    states[0].kind = StateKind::SUBFSM;
    assert(validateRecipe(recipe)); // Missing SubFSM.
    StepTransitionRecipe stepOut[] = {{timer(0), 1}};
    StepRecipe steps[] = {{MotionId::STOP, {0, 0}, stepOut, 1}};
    SubFsmRecipe subfsm = {steps, 0, false};
    states[0].subfsm = &subfsm;
    assert(validateRecipe(recipe)); // Empty SubFSM.
    subfsm.step_count = 1;
    assert(validateRecipe(recipe)); // Step index 1 is out of range.
    stepOut[0].next_step = -2;
    assert(validateRecipe(recipe));
    stepOut[0].next_step = STEP_COMPLETE;
    stepOut[0].trigger = COMPLETE;
    assert(validateRecipe(recipe)); // No completion source inside a step.
    stepOut[0].trigger = timer(0);
    steps[0].motor.right_pct = -101;
    assert(validateRecipe(recipe));
    steps[0].motor.right_pct = 0;
    assert(validateRecipe(recipe)); // Reachable completion needs an exit.
    subfsm.allowHoldOnCompletion = true;
    assert(!validateRecipe(recipe));

    // A disconnected completion step does not require hold permission.
    const StepRecipe disconnected[] = {
        {MotionId::STOP, {0, 0}, nullptr, 0},
        {MotionId::STOP, {0, 0}, stepOut, 1}
    };
    subfsm = {disconnected, 2, false};
    assert(!validateRecipe(recipe));
    states[1].id = StateId::COUNT;
    assert(validateRecipe(recipe)); // IDs must have catalog metadata.
}

static void testSelectedRecipe() {
    Motor left, right;
    Drive drive(left, right);
    eventCount = 0;
    StateMachine machine(active_fsm_recipe::MACHINE, drive, record);
    assert(machine.begin(0));
    SensorSnapshot sample;
    // Lifecycle has already permitted begin(); the recipe needs no second START.
    sample.startActive = false;
    machine.update(sample, 0);
#if defined(FSM_ACTIVE_RECIPE_MOTOR_TEST)
    assert(left.command == 30 && right.command == 30);
    machine.update(sample, 400);
    assert(left.command == 0);
    machine.update(sample, 600);
    assert(left.command == -30);
    machine.update(sample, 1000);
#else
    assert(left.command == -40 && right.command == 40);
    machine.update(sample, 350);
    assert(left.command == 0);
    machine.update(sample, 850);
    assert(left.command == 40 && right.command == -40);
    machine.update(sample, 1200);
#endif
    assert(machine.currentState() == StateId::DONE && left.command == 0);
    assert(eventCount == 5);
    assert(recorded[0].scope == TransitionScope::STATE && recorded[0].state == StateId::IDLE);
    assert(recorded[1].scope == TransitionScope::STEP && recorded[1].step == 0 && recorded[1].nextStep == 1);
    assert(recorded[3].nextStep == STEP_COMPLETE && recorded[3].step == 2);
    assert(recorded[4].condition.type == TriggerType::COMPLETION && recorded[4].nextState == StateId::DONE);
    assert(recorded[1].elapsedMs == recorded[1].condition.timerMs);
    char line[fsm_defs::runtime::LOG_LINE_BYTES];
    assert(formatTransition(recorded[1], line, sizeof(line)));
    assert(std::strstr(line, "CONDITION=TIMER") && std::strstr(line, "STEP=0,NEXT_STEP=1"));
    assert(std::strstr(line, active_fsm_recipe::MACHINE.name));
    const char* sourceState = fsm_defs::stateName(recorded[1].state);
    assert(std::strstr(line, sourceState));
    char shortLine[8];
    assert(!formatTransition(recorded[1], shortLine, sizeof(shortLine)));
}

static void testLoggingAndLifecycle() {
    TransitionEvent event = {"TEST", TransitionScope::STATE, A, B, CENTER, 20, 10, {15, -15}, -1, -1};
    char line[fsm_defs::runtime::LOG_LINE_BYTES];
    assert(formatTransition(event, line, sizeof(line)));
    assert(std::strstr(line, "DETAIL=IR4_DETECTED"));
    assert(std::strstr(line, "ELAPSED_MS=10"));
    assert(std::strstr(line, "MOTOR_L=15,MOTOR_R=-15"));
    TransitionQueue queue;
    for (unsigned i = 0; i < fsm_defs::runtime::LOG_QUEUE_CAPACITY; ++i) {
        event.atMs = i;
        assert(queue.push(event));
    }
    assert(!queue.push(event) && !queue.push(event));
    assert(queue.takeDropped() == 2 && queue.takeDropped() == 0);
    for (unsigned i = 0; i < fsm_defs::runtime::LOG_QUEUE_CAPACITY; ++i) {
        assert(queue.pop(event) && event.atMs == i);
    }
    assert(!queue.pop(event));
    assert(queue.push(event) && queue.pop(event)); // Reusable after wrapping.

    SensorSnapshot sample;
    sample.valid = true;
    sample.sampledAtMs = UINT32_MAX - 9;
    sample.startObservedAtMs = 40;
    assert(!canRunRecipe(sample, 40)); // START off always stops the lifecycle.
    sample.startActive = true;
    assert(canRunRecipe(sample, 40)); // Exactly 50 ms old, across wraparound.
    assert(!canRunRecipe(sample, 41)); // A fresh START read cannot hide stale ADC.
    sample.valid = false;
    assert(!canRunRecipe(sample, 40));
    assert(!fsm_defs::conditionMetadata(ConditionId::COUNT));
    assert(!fsm_defs::stateMetadata(StateId::COUNT));
}

int main() {
    const unsigned before = allocations;
    testDriveAndConditions();
    testMotorTransitions();
    testSubFsm();
    testValidation();
    testSelectedRecipe();
    testLoggingAndLifecycle();
    assert(allocations == before);
    std::puts("PASS: Drive, conditions, timers, priorities, SubFSM, validation, selected recipe, no allocations");
}
