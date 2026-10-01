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
    drive.setMotionEnabled(true);
    drive.begin();
    drive.setMotionEnabled(true);
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
    drive.setMotionEnabled(true);
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
    drive.setMotionEnabled(true);
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
    drive.setMotionEnabled(true);
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
    drive.setMotionEnabled(true);
    eventCount = 0;
    StateMachine machine(active_fsm_recipe::MACHINE, drive, record);
    assert(machine.begin(0));
    SensorSnapshot sample;
    // Direct runtime test: START is not part of recipe topology or timer logic.
    sample.startActive = false;
    machine.update(sample, 0);
#if defined(FSM_ACTIVE_RECIPE_TEST)
    // La receta TEST histórica recorre cuatro estados y vuelve a IDLE.
    assert(machine.currentState() == StateId::IDLE && left.command == 0);
    machine.update(sample, 2000);
    assert(machine.currentState() == StateId::TEST_FORWARD && left.command == 100 && right.command == 100);
    machine.update(sample, 4000);
    assert(machine.currentState() == StateId::TEST_STOP && left.command == 0);
    machine.update(sample, 9000);
    assert(machine.currentState() == StateId::TEST_BACKWARD && left.command == -100 && right.command == -100);
    machine.update(sample, 11000);
    assert(machine.currentState() == StateId::IDLE && left.command == 0 && right.command == 0);
    assert(eventCount == 4);
    assert(recorded[0].state == StateId::IDLE && recorded[0].nextState == StateId::TEST_FORWARD);
    assert(recorded[3].state == StateId::TEST_BACKWARD && recorded[3].nextState == StateId::IDLE);
#else
    assert(left.command == -40 && right.command == 40);
    machine.update(sample, 350);
    assert(left.command == 0);
    machine.update(sample, 850);
    assert(left.command == 40 && right.command == -40);
    machine.update(sample, 1200);
    assert(machine.currentState() == StateId::DONE && left.command == 0);
    assert(eventCount == 5);
    assert(recorded[0].scope == TransitionScope::STATE && recorded[0].state == StateId::IDLE);
    assert(recorded[1].scope == TransitionScope::STEP && recorded[1].step == 0 && recorded[1].nextStep == 1);
    assert(recorded[3].nextStep == STEP_COMPLETE && recorded[3].step == 2);
    assert(recorded[4].condition.type == TriggerType::COMPLETION && recorded[4].nextState == StateId::DONE);
    assert(recorded[1].elapsedMs == recorded[1].condition.timerMs);
#endif
    char line[fsm_defs::runtime::LOG_LINE_BYTES];
    assert(formatTransition(recorded[1], line, sizeof(line)));
#if FSM_CONSOLE_COMPACT
#if defined(FSM_ACTIVE_RECIPE_TEST)
    assert(std::strstr(line, "[FSM]") && std::strstr(line, "TEST_FORWARD -> TEST_STOP"));
#else
    assert(std::strstr(line, "[STEP]") && std::strstr(line, "0->1"));
#endif
#else
#if defined(FSM_ACTIVE_RECIPE_TEST)
    assert(std::strstr(line, "SCOPE=STATE") && std::strstr(line, "NEXT=TEST_STOP"));
#else
    assert(std::strstr(line, "CONDITION=TIMER") && std::strstr(line, "STEP=0,NEXT_STEP=1"));
#endif
    assert(std::strstr(line, active_fsm_recipe::MACHINE.name));
#endif
    const char* sourceState = fsm_defs::stateName(recorded[1].state);
    assert(std::strstr(line, sourceState));
    char shortLine[8];
    assert(!formatTransition(recorded[1], shortLine, sizeof(shortLine)));
}

static void testLoggingAndLifecycle() {
    TransitionEvent event = {"TEST", TransitionScope::STATE, A, B, CENTER, 20, 10, {15, -15}, -1, -1};
    char line[fsm_defs::runtime::LOG_LINE_BYTES];
    assert(formatTransition(event, line, sizeof(line)));
#if FSM_CONSOLE_COMPACT
    assert(std::strcmp(line, "[FSM]    20 | IDLE -> MOTOR_SEQUENCE | IR4_DETECTED\n") == 0);
#else
    assert(std::strstr(line, "DETAIL=IR4_DETECTED"));
    assert(std::strstr(line, "ELAPSED_MS=10"));
    assert(std::strstr(line, "MOTOR_L=15,MOTOR_R=-15"));
#endif
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
    assert(isControlDataValid(sample, 40)); // START off does not invalidate acquisition.
    assert(effectiveStartActive(sample) == bool(FORCE_START_ACTIVE));
    assert(!evaluateCondition(ConditionId::START_ACTIVE, sample)); // Override is output-only.
    sample.startActive = true;
    assert(isControlDataValid(sample, 40)); // Exactly 50 ms old, across wraparound.
    assert(!isControlDataValid(sample, 41)); // A fresh START read cannot hide stale ADC.
    sample.valid = false;
    assert(!isControlDataValid(sample, 40));
    assert(!fsm_defs::conditionMetadata(ConditionId::COUNT));
    assert(!fsm_defs::stateMetadata(StateId::COUNT));
}

static void testStartOutputPermission() {
    Motor left, right;
    Drive drive(left, right);
    drive.begin(); // Default permission is off.
    drive.apply({30, 30});
    assert(left.command == 0 && right.command == 0);
    drive.setMotionEnabled(true);
    assert(left.command == 30 && right.command == 30);
    drive.setMotionEnabled(false);
    assert(left.command == 0 && right.command == 0);
    drive.apply({-20, -25}); // Latest request replaces the old one while disabled.
    drive.setMotionEnabled(true);
    assert(left.command == -20 && right.command == -25);
    drive.stop();
    drive.setMotionEnabled(false);
    drive.setMotionEnabled(true);
    assert(left.command == 0 && right.command == 0); // A real stop cleared the request.

    // Use the real task-control helpers with a short sequence. Timers, sensor
    // interrupts and logging must run independently of physical START.
    const StepTransitionRecipe forwardOut[] = {{timer(400), 1}};
    const StepTransitionRecipe pauseOut[] = {{timer(200), 2}};
    const StepTransitionRecipe backwardOut[] = {{timer(400), STEP_COMPLETE}};
    const StepRecipe steps[] = {
        {MotionId::FORWARD, {30, 30}, forwardOut, 1},
        {MotionId::STOP, {0, 0}, pauseOut, 1},
        {MotionId::BACKWARD, {-30, -30}, backwardOut, 1}
    };
    const SubFsmRecipe sequence = {steps, 3, false};
    const StateTransitionRecipe sequenceOut[] = {{CENTER, C}, {COMPLETE, C}};
    const StateRecipe states[] = {
        {B, StateKind::SUBFSM, MotionId::NONE, {0, 0}, sequenceOut, 2, &sequence},
        motorState(C, 0)
    };
    const MachineRecipe recipe = {B, states, 2, "START_GATE_TEST"};
    eventCount = 0;
    StateMachine machine(recipe, drive, record);
    SensorSnapshot sample;
    sample.valid = true;
    sample.startActive = false;
    updateRecipeControl(machine, drive, sample, 0);
    assert(machine.isRunning() && machine.currentState() == B);
    assert(left.command == (FORCE_START_ACTIVE ? 30 : 0));
    sample.sampledAtMs = 400;
    updateRecipeControl(machine, drive, sample, 400);
    assert(left.command == 0 && eventCount == 1);
    sample.sampledAtMs = 600;
    updateRecipeControl(machine, drive, sample, 600);
    assert(left.command == (FORCE_START_ACTIVE ? -30 : 0) && eventCount == 2);

    // No state update/entry here: START alone replays the current backward request.
    sample.startActive = true;
    assert(refreshRecipeMotorPermission(machine, drive, sample, 600));
    assert(left.command == -30 && right.command == -30 && eventCount == 2);
    sample.startActive = false;
    assert(refreshRecipeMotorPermission(machine, drive, sample, 601));
    assert(left.command == (FORCE_START_ACTIVE ? -30 : 0));
    assert(machine.currentState() == B && eventCount == 2);
    sample.sampledAtMs = 999;
    updateRecipeControl(machine, drive, sample, 999);
    assert(machine.currentState() == B);
    sample.sampledAtMs = 1000;
    updateRecipeControl(machine, drive, sample, 1000);
    assert(machine.currentState() == C && eventCount == 4); // Original deadline, no reset.
    assert(left.command == 0 && right.command == 0);
    sample.startActive = true;
    refreshRecipeMotorPermission(machine, drive, sample, 1000);
    assert(machine.currentState() == C && left.command == 0); // START does not restart DONE.

    // Invalid data resets even with override; recovery restarts on a fresh sample.
    sample.valid = false;
    updateRecipeControl(machine, drive, sample, 1001);
    assert(!machine.isRunning() && left.command == 0);
    sample.valid = true;
    sample.startActive = false;
    sample.sampledAtMs = 1100;
    updateRecipeControl(machine, drive, sample, 1100);
    assert(machine.currentState() == B && machine.isRunning());
    sample.ir[3] = true;
    updateRecipeControl(machine, drive, sample, 1101);
    assert(machine.currentState() == C); // Condition evaluation also runs with START low.

    sample.ir[3] = false;
    sample.valid = false;
    updateRecipeControl(machine, drive, sample, 1102);
    sample.valid = true;
    sample.sampledAtMs = 1200;
    updateRecipeControl(machine, drive, sample, 1200);
    sample.startActive = true;
    const unsigned movementsBeforeStale = left.movementCalls;
    refreshRecipeMotorPermission(machine, drive, sample, 1251);
    assert(!machine.isRunning() && left.command == 0);
    assert(left.movementCalls == movementsBeforeStale); // No stale-command pulse on START rise.
    drive.setMotionEnabled(true);
    assert(left.command == 0); // The stale-data stop cleared the retained request.

    // El modo remoto exige un START nuevo después de OFF y reinicia en el paso 0.
    sample.valid = true;
    sample.sampledAtMs = 1300;
    sample.startRemoteControlled = true;
    sample.startActive = false;
    updateRecipeControl(machine, drive, sample, 1300);
    assert(!machine.isRunning() && left.command == 0);
    sample.startActive = true;
    updateRecipeControl(machine, drive, sample, 1301);
    assert(machine.isRunning() && machine.currentState() == B && machine.currentStep() == 0);
    assert(left.command == 30);
    sample.startActive = false;
    refreshRecipeMotorPermission(machine, drive, sample, 1302);
    assert(!machine.isRunning() && left.command == 0);
    updateRecipeControl(machine, drive, sample, 1303);
    assert(!machine.isRunning());
    sample.startActive = true;
    updateRecipeControl(machine, drive, sample, 1304);
    assert(machine.isRunning() && machine.currentStep() == 0 && left.command == 30);
}

// Golden lines protect structured byte compatibility and compact presentation.
// This test runs inside the existing allocation counter in both build modes.
static void testConsoleFormats() {
    char line[fsm_defs::runtime::LOG_LINE_BYTES];
    TransitionEvent event = {"MOTOR_TEST", TransitionScope::STATE, A, B, timer(0), 80, 0, {0, 0}, -1, -1};
    assert(formatTransition(event, line, sizeof(line)));
#if FSM_CONSOLE_COMPACT
    assert(std::strcmp(line, "[FSM]    80 | IDLE -> MOTOR_SEQUENCE | TIMER 0ms\n") == 0);
#else
    assert(std::strcmp(line, "FSM,MOTOR_TEST,AT_MS=80,SCOPE=STATE,STATE=IDLE,CONDITION=TIMER,DETAIL=0_ms,ELAPSED_MS=0,NEXT=MOTOR_SEQUENCE,MOTOR_L=0,MOTOR_R=0,STEP=-1,NEXT_STEP=-1\n") == 0);
#endif
    event = {"MOTOR_TEST", TransitionScope::STEP, B, B, timer(400), 480, 400, {30, 30}, 0, 1};
    assert(formatTransition(event, line, sizeof(line)));
#if FSM_CONSOLE_COMPACT
    assert(std::strcmp(line, "[STEP]  480 | MOTOR_SEQUENCE 0->1 | 400ms | L=30 R=30\n") == 0);
#else
    assert(std::strcmp(line, "FSM,MOTOR_TEST,AT_MS=480,SCOPE=STEP,STATE=MOTOR_SEQUENCE,CONDITION=TIMER,DETAIL=400_ms,ELAPSED_MS=400,NEXT=MOTOR_SEQUENCE,MOTOR_L=30,MOTOR_R=30,STEP=0,NEXT_STEP=1\n") == 0);
#endif
    event.atMs = 1080;
    event.step = 2;
    event.nextStep = STEP_COMPLETE;
    event.motor = {-30, -30};
    assert(formatTransition(event, line, sizeof(line)));
#if FSM_CONSOLE_COMPACT
    assert(std::strcmp(line, "[STEP] 1080 | MOTOR_SEQUENCE 2->END | 400ms | L=-30 R=-30\n") == 0);
#else
    assert(std::strcmp(line, "FSM,MOTOR_TEST,AT_MS=1080,SCOPE=STEP,STATE=MOTOR_SEQUENCE,CONDITION=TIMER,DETAIL=400_ms,ELAPSED_MS=400,NEXT=MOTOR_SEQUENCE,MOTOR_L=-30,MOTOR_R=-30,STEP=2,NEXT_STEP=-1\n") == 0);
#endif
    event = {"MOTOR_TEST", TransitionScope::STATE, B, C, COMPLETE, 1080, 1000, {-30, -30}, 2, -1};
    assert(formatTransition(event, line, sizeof(line)));
#if FSM_CONSOLE_COMPACT
    assert(std::strcmp(line, "[FSM]  1080 | MOTOR_SEQUENCE -> DONE | COMPLETE\n") == 0);
#else
    assert(std::strcmp(line, "FSM,MOTOR_TEST,AT_MS=1080,SCOPE=STATE,STATE=MOTOR_SEQUENCE,CONDITION=COMPLETION,DETAIL=STEP_COMPLETE,ELAPSED_MS=1000,NEXT=DONE,MOTOR_L=-30,MOTOR_R=-30,STEP=2,NEXT_STEP=-1\n") == 0);
#endif
    const ConditionId terms[] = {ConditionId::IR3_DETECTED, ConditionId::IR4_DETECTED, ConditionId::LINE_LEFT_DETECTED};
    const LogicOp operators[] = {LogicOp::OR, LogicOp::AND};
    event.condition = {TriggerType::SENSOR, 0, {terms, 3, operators, 2}};
    assert(formatTransition(event, line, sizeof(line)));
#if FSM_CONSOLE_COMPACT
    assert(std::strcmp(line, "[FSM]  1080 | MOTOR_SEQUENCE -> DONE | IR3_DETECTED OR IR4_DETECTED AND LINE_LEFT_DETECTED\n") == 0);
#else
    assert(std::strcmp(line, "FSM,MOTOR_TEST,AT_MS=1080,SCOPE=STATE,STATE=MOTOR_SEQUENCE,CONDITION=SENSOR,DETAIL=IR3_DETECTED_OR_IR4_DETECTED_AND_LINE_LEFT_DETECTED,ELAPSED_MS=1000,NEXT=DONE,MOTOR_L=-30,MOTOR_R=-30,STEP=2,NEXT_STEP=-1\n") == 0);
#endif
    event = {"TEST", TransitionScope::STEP, StateId::TURN_SEQUENCE, StateId::TURN_SEQUENCE,
             {TriggerType::SENSOR, 0, {terms + 2, 1, nullptr, 0}}, 730, 50, {-40, 40}, 1, 2};
    assert(formatTransition(event, line, sizeof(line)));
#if FSM_CONSOLE_COMPACT
    assert(std::strcmp(line, "[STEP]  730 | TURN_SEQUENCE 1->2 | LINE_LEFT_DETECTED | L=-40 R=40\n") == 0);
#else
    assert(std::strcmp(line, "FSM,TEST,AT_MS=730,SCOPE=STEP,STATE=TURN_SEQUENCE,CONDITION=SENSOR,DETAIL=LINE_LEFT_DETECTED,ELAPSED_MS=50,NEXT=TURN_SEQUENCE,MOTOR_L=-40,MOTOR_R=40,STEP=1,NEXT_STEP=2\n") == 0);
#endif
    // Boundary behavior: exact fit succeeds; missing terminator space fails.
    const size_t length = std::strlen(line);
    assert(formatTransition(event, line, length + 1));
    assert(!formatTransition(event, line, length));
    assert(line[length - 1] == '\0');
    assert(!formatTransition(event, nullptr, 0));
    char single[1] = {'x'};
    assert(!formatTransition(event, single, sizeof(single)) && single[0] == '\0');

    // Overflow the expression buffer while keeping its source arrays valid.
    ConditionId longTerms[32];
    LogicOp longOperators[31];
    for (auto& term : longTerms) term = ConditionId::LINE_LEFT_DETECTED;
    for (auto& op : longOperators) op = LogicOp::AND;
    event.condition.expression = {longTerms, 32, longOperators, 31};
    assert(!formatTransition(event, line, sizeof(line)));
    assert(std::strstr(line, "[TRUNCATED]"));
    assert(line[std::strlen(line) - 1] == '\n');
}

int main() {
    const unsigned before = allocations;
    testDriveAndConditions();
    testMotorTransitions();
    testSubFsm();
    testValidation();
    testSelectedRecipe();
    testLoggingAndLifecycle();
    testStartOutputPermission();
    testConsoleFormats();
    assert(allocations == before);
    std::puts("PASS: Drive, conditions, timers, priorities, SubFSM, validation, selected recipe, no allocations");
}
