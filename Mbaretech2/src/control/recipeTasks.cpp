#include "firmwareConfig.h"
#if ENABLE_RECIPE_FSM
#include "sensorTasks.h"
#include "bluetoothComm.h"
#include "fsm/StateMachine.h"
#include "fsm/fsm_recipe_select.h"
#include "fsm/RecipeLifecycle.h"
#include "fsm/WifiTelemetry.h"
#include "fsm/ParameterService.h"
#include "telemetry/TelemetryService.h"
#include <cstring>

namespace {
void reportRecipeError(const char* error) {
    // Startup/fault path only: no String allocation or transport IO per step.
#if ENABLE_TELEMETRY
    telemetryPublishText(TelemetryType::Error, error);
#else
#if ENABLE_LOGGING
    sendData(String("FSM_RECIPE_ERROR,") + error + "\n");
#elif ENABLE_SERIAL
    Serial.print("FSM_RECIPE_ERROR,");
    Serial.println(error);
#endif
#endif
}
void recordRecipeTransition(const fsm::TransitionEvent& event) {
#if ENABLE_TELEMETRY
    TelemetryEvent record{};
    record.type = event.scope == fsm::TransitionScope::STEP
        ? TelemetryType::FsmStepTransition : TelemetryType::FsmTransition;
    strncpy(record.machine, event.machine, sizeof(record.machine) - 1);
    strncpy(record.state, fsm_defs::stateIdName(event.state), sizeof(record.state) - 1);
    strncpy(record.next, fsm_defs::stateIdName(event.nextState), sizeof(record.next) - 1);
    record.step = event.step;
    record.nextStep = event.nextStep;
    record.elapsedMs = event.elapsedMs;
    record.timerMs = event.condition.timerMs;
    record.paramRevision = event.parameterRevision;
    if (event.condition.type == fsm::TriggerType::TIMER)
        strncpy(record.condition, "TIMER", sizeof(record.condition) - 1);
    else if (event.condition.type == fsm::TriggerType::COMPLETION)
        strncpy(record.condition, "COMPLETION", sizeof(record.condition) - 1);
    else {
        size_t used = 0;
        for (unsigned i = 0; i < event.condition.expression.termCount; ++i) {
            const char* term = fsm_defs::conditionName(event.condition.expression.terms[i]);
            const char* op = i ? (event.condition.expression.operators[i - 1] == fsm::LogicOp::AND ? " AND " : " OR ") : "";
            const int n = snprintf(record.condition + used, sizeof(record.condition) - used,
                                   "%s%s", op, term);
            if (n < 0 || size_t(n) >= sizeof(record.condition) - used) break;
            used += size_t(n);
        }
    }
    telemetryPublish(record);
#else
    fsm::enqueueTransition(event);
#endif
}
}

void stateMachineTask(void*) {
    fsm::Drive drive(leftMotor, rightMotor);
    fsm::StateMachine machine(active_fsm_recipe::MACHINE, drive, recordRecipeTransition,
                              &fsm::parameterServiceStore());
    drive.begin();
    // Reject malformed recipes before any command, regardless of START.
    const char* error = fsm::validateRecipe(active_fsm_recipe::MACHINE);
    if (error) {
        reportRecipeError(error);
        drive.stop();
        vTaskDelete(nullptr);
        return;
    }
    TickType_t wake = xTaskGetTickCount();
    uint32_t observedStartGeneration = fsm::parameterServiceStartGeneration();
#if ENABLE_TASK_TIMING
    uint32_t previousStartUs = 0;
    bool hasPreviousStart = false;
#endif
    for (;;) {
#if ENABLE_TASK_TIMING
        const uint32_t startUs = micros();
#endif
        SensorSnapshot sensors = readSensorSnapshot();
        const uint32_t nowMs = millis();
        const uint32_t startGeneration = fsm::parameterServiceStartGeneration();
        if (startGeneration != observedStartGeneration) {
            // ON y OFF invalidan tiempos y comandos retenidos antes de evaluar.
            machine.stop();
            sensors = readSensorSnapshot();
            observedStartGeneration = startGeneration;
        }
        fsm::updateRecipeControl(machine, drive, sensors, nowMs);
        fsm::parameterServiceRunning(machine.isRunning());
        // Catch a falling START edge during evaluation without resetting the FSM.
        // Data that became stale/invalid meanwhile still causes a real stop.
        const SensorSnapshot latest = readSensorSnapshot();
        fsm::refreshRecipeMotorPermission(machine, drive, latest, millis());
        telemetryPublishState(active_fsm_recipe::MACHINE.name,
                              fsm_defs::stateIdName(machine.currentState()), machine.currentStep(),
                              machine.stateElapsedMs(nowMs), machine.stepElapsedMs(nowMs),
                              machine.isRunning(),
                              machine.error() ? "ERROR" : !latest.valid ? "SNAPSHOT_INVALID" :
                              !fsm::isControlDataValid(latest, millis()) ? "SNAPSHOT_STALE" :
                              !fsm::effectiveStartActive(latest) ? "START_INACTIVE" : "RUNNING",
                              machine.activeRevision());
        const fsm::MotorCommand output = drive.outputCommand();
        telemetryPublishMotor(output.left_pct, output.right_pct,
                              fsm_defs::stateIdName(machine.currentState()), machine.activeRevision());
        if (machine.error()) {
            reportRecipeError(machine.error());
            machine.stop();
            vTaskDelete(nullptr);
            return;
        }
#if ENABLE_TASK_TIMING
        recordTaskTiming(TimedTask::Fsm, uint32_t(micros() - startUs),
                         hasPreviousStart ? uint32_t(startUs - previousStartUs) : 0);
        previousStartUs = startUs;
        hasPreviousStart = true;
#endif
        // Match the existing scheduling policy: yield instead of burst catch-up.
        if (xTaskGetTickCount() - wake >= fsm_defs::runtime::TASK_PERIOD_TICKS) wake = xTaskGetTickCount();
        vTaskDelayUntil(&wake, fsm_defs::runtime::TASK_PERIOD_TICKS);
    }
}

void loop() {
#if !ENABLE_LOGGING
    // Serial-only builds have no communications task. Drain from Arduino's idle
    // loop, never the real-time control task. Exactly one consumer in every mode.
    fsm::pollRecipeTransitions();
#endif
    delay(fsm_defs::runtime::IDLE_LOOP_DELAY_MS);
}
#endif
