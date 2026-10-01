#include <cassert>
#include <cstring>
#include "fsm/ParameterCommand.h"
#include "fsm/StateMachine.h"
#include "fsm/RecipeLifecycle.h"
#include "motor.h"
using namespace fsm;

static ParameterChange edit(const char* id, int32_t value) {
    ParameterChange result{};
    std::strncpy(result.id, id, sizeof(result.id) - 1);
    result.value = value;
    return result;
}
int main() {
    const ParameterDefinition definitions[] = {
        {1, "idle_duration", "Idle duration", ParameterType::Integer, ParameterUnit::Milliseconds,
         350, 0, 60000, 1, ParameterPolicy::NextStateEntry},
        {2, "drive_left", "Drive left", ParameterType::Integer, ParameterUnit::Percent,
         40, -100, 100, 1, ParameterPolicy::NextStateEntry}
    };
    const StateTransitionRecipe outA[] = {{{TriggerType::TIMER, 350, {}, 1}, StateId::MOTOR_SEQUENCE}};
    const StateTransitionRecipe outB[] = {{{TriggerType::TIMER, 100, {}}, StateId::IDLE}};
    const StateRecipe states[] = {
        {StateId::IDLE, StateKind::MOTOR, MotionId::STOP, {0,0}, outA, 1, nullptr},
        {StateId::MOTOR_SEQUENCE, StateKind::MOTOR, MotionId::FORWARD, {40,40,2,0}, outB, 1, nullptr}
    };
    const MachineRecipe recipe = {StateId::IDLE, states, 2, "PARAM_TEST", definitions, 2};
    RuntimeParameters params;
    assert(params.initialize(recipe));
    assert(params.count() == 2);
    ParameterSnapshot first{}; params.snapshot(first);
    assert(first.revision == 0);
    assert(params.timer(outA[0].trigger, first) == 350);
    assert(params.motor(states[1].motor, first).left_pct == 40);

    ParameterChange good[] = {edit("idle_duration", 370), edit("drive_left", 45)};
    auto result = params.set(0, good, 2, true);
    assert(result.accepted && result.revision == 1);
    ParameterSnapshot updated{}; params.snapshot(updated);
    assert(params.timer(outA[0].trigger, first) == 350); // Entrada activa congelada.
    assert(params.timer(outA[0].trigger, updated) == 370);
    assert(params.motor(states[1].motor, updated).left_pct == 45);
    assert(params.motor(states[1].motor, updated).right_pct == 40);

    const char* invalidIds[] = {"missing", "drive_left"};
    for (unsigned i = 0; i < 2; ++i) {
        ParameterChange pair[] = {edit("idle_duration", 400), edit(invalidIds[i], 101)};
        assert(!params.set(1, pair, 2, true).accepted);
    }
    ParameterChange below = edit("idle_duration", -1);
    ParameterChange above = edit("idle_duration", 60001);
    assert(!params.set(1, &below, 1, true).accepted);
    assert(!params.set(1, &above, 1, true).accepted);
    params.snapshot(updated);
    assert(updated.revision == 1 && params.timer(outA[0].trigger, updated) == 370);
    assert(!params.set(0, good, 2, true).accepted); // Revisión obsoleta.

    ParameterCommand command{}; const char* error = nullptr;
    assert(decodeParameterCommand("{\"type\":\"param_set\",\"machine\":\"PARAM_TEST\",\"transaction\":42,\"baseRevision\":1,\"changes\":[{\"id\":\"idle_duration\",\"value\":390}]}", command, error));
    assert(command.count == 1 && command.changes[0].value == 390);
    assert(!decodeParameterCommand("{\"type\":\"param_set\",\"machine\":\"PARAM_TEST\",\"transaction\":42,\"baseRevision\":1,\"changes\":[{\"id\":\"idle_duration\",\"value\":\"390\"}]}", command, error));
    assert(decodeParameterCommand("{\"type\":\"param_set\",\"machine\":\"PARAM_TEST\",\"transaction\":42,\"baseRevision\":1,\"changes\":[{\"id\":\"idle_duration\",\"value\":390},{\"id\":\"idle_duration\",\"value\":400}]}", command, error));
    assert(!params.set(1, command.changes, command.count, true).accepted);
    assert(decodeParameterCommand("{\"type\":\"param_reset\",\"machine\":\"PARAM_TEST\",\"transaction\":43,\"baseRevision\":1}", command, error));
    assert(decodeParameterCommand("{\"type\":\"start_set\",\"transaction\":44,\"active\":false}", command, error));
    assert(command.operation == ParameterOperation::StartSet && !command.startActive);
    assert(decodeParameterCommand("{\"type\":\"start_set\",\"transaction\":45,\"active\":true}", command, error));
    assert(command.startActive);
    assert(!decodeParameterCommand("{\"type\":\"start_set\",\"transaction\":46,\"active\":1}", command, error));
    assert(!decodeParameterCommand("{\"type\":\"start_set\",\"transaction\":46,\"active\":true,\"revision\":1}", command, error));
    assert(params.reset(1, false).accepted);
    params.snapshot(updated);
    assert(updated.revision == 2 && params.timer(outA[0].trigger, updated) == 350);

    // NEXT_MACHINE_START publica el nuevo overlay sin afectar entradas previas.
    const ParameterDefinition restartDefinitions[] = {
        {3, "restart_speed", "Restart speed", ParameterType::Integer, ParameterUnit::Percent,
         20, -100, 100, 1, ParameterPolicy::NextMachineStart}
    };
    const MachineRecipe restartRecipe = {StateId::IDLE, states, 2, "RESTART_TEST",
                                         restartDefinitions, 1};
    RuntimeParameters restartParams;
    assert(restartParams.initialize(restartRecipe));
    restartParams.beginMachine();
    const MotorCommand restartMotor{20, 0, 3, 0};
    ParameterChange nextStart = edit("restart_speed", 60);
    assert(restartParams.set(0, &nextStart, 1, true).accepted);
    ParameterSnapshot overlay{}, latched{};
    restartParams.snapshot(overlay);
    restartParams.entrySnapshot(latched);
    assert(restartParams.motor(restartMotor, overlay).left_pct == 60);
    assert(restartParams.motor(restartMotor, latched).left_pct == 20);
    restartParams.beginMachine();
    restartParams.entrySnapshot(latched);
    assert(restartParams.motor(restartMotor, latched).left_pct == 60);

    Motor left, right; Drive drive(left,right); drive.begin(); drive.setMotionEnabled(true);
    StateMachine machine(recipe, drive, nullptr, &params);
    assert(machine.begin(0));
    ParameterChange timerChange = edit("idle_duration", 500);
    assert(params.set(2, &timerChange, 1, true).accepted);
    machine.update({}, 349); assert(machine.currentState() == StateId::IDLE);
    machine.update({}, 350); assert(machine.currentState() == StateId::MOTOR_SEQUENCE);
    machine.update({}, 450); assert(machine.currentState() == StateId::IDLE);
    machine.update({}, 949); assert(machine.currentState() == StateId::IDLE);
    machine.update({}, 950); assert(machine.currentState() == StateId::MOTOR_SEQUENCE);

    // START remoto OFF vacía el comando y exige una entrada nueva desde IDLE.
    SensorSnapshot sample{};
    sample.valid = true;
    sample.sampledAtMs = 1000;
    sample.startRemoteControlled = true;
    sample.startActive = false;
    updateRecipeControl(machine, drive, sample, 1000);
    assert(!machine.isRunning() && drive.outputCommand().left_pct == 0);
    sample.startActive = true;
    updateRecipeControl(machine, drive, sample, 1001);
    assert(machine.isRunning() && machine.currentState() == StateId::IDLE);
    sample.startActive = false;
    updateRecipeControl(machine, drive, sample, 1002);
    assert(!machine.isRunning());
    sample.startActive = true;
    updateRecipeControl(machine, drive, sample, 1003);
    assert(machine.isRunning() && machine.currentState() == StateId::IDLE);
}
