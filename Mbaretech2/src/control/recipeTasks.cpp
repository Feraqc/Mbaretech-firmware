#include "firmwareConfig.h"
#if ENABLE_RECIPE_FSM
#include "sensorTasks.h"
#include "bluetoothComm.h"
#include "fsm/StateMachine.h"
#include "fsm/fsm_recipe_select.h"
#include "fsm/RecipeLifecycle.h"

namespace {
void reportRecipeError(const char* error) {
    // Startup/fault path only: no String allocation or transport IO per step.
#if ENABLE_LOGGING
    sendData(String("FSM_RECIPE_ERROR,") + error + "\n");
#elif ENABLE_SERIAL
    Serial.print("FSM_RECIPE_ERROR,");
    Serial.println(error);
#endif
}
}

void stateMachineTask(void*) {
    fsm::Drive drive(leftMotor, rightMotor);
    fsm::StateMachine machine(active_fsm_recipe::MACHINE, drive, fsm::enqueueTransition);
    drive.begin();
    // Reject even an idle malformed recipe immediately, before waiting for START.
    const char* error = fsm::validateRecipe(active_fsm_recipe::MACHINE);
    if (error) {
        reportRecipeError(error);
        drive.stop();
        vTaskDelete(nullptr);
        return;
    }
    bool running = false;
    TickType_t wake = xTaskGetTickCount();
#if ENABLE_TASK_TIMING
    uint32_t previousStartUs = 0;
    bool hasPreviousStart = false;
#endif
    for (;;) {
#if ENABLE_TASK_TIMING
        const uint32_t startUs = micros();
#endif
        const SensorSnapshot sensors = readSensorSnapshot();
        const uint32_t nowMs = millis();
        if (!fsm::canRunRecipe(sensors, nowMs)) {
            machine.stop();
            running = false;
        } else {
            // Restart the selected recipe after STOP or stale/invalid acquisition.
            if (!running) running = machine.begin(nowMs);
            if (running) machine.update(sensors, nowMs);
            // Catch a STOP edge received during evaluation via the same interface.
            if (!fsm::canRunRecipe(readSensorSnapshot(), millis())) {
                machine.stop();
                running = false;
            }
        }
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
