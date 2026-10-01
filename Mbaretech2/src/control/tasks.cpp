#include "firmwareConfig.h"
#if ENABLE_FSM
#include "combatFsm.h"
#include "telemetry/TelemetryService.h"

namespace {
void applyMotor(Motor& motor, int speed) {
    if (speed > 0) motor.forward(speed);
    else if (speed < 0) motor.backward(-speed);
    else motor.brake();
}
}

void stateMachineTask(void*) {
    CombatFsm fsm;
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
        MotorCommand command = fsm.step(sensors, startSignal, millis());
        // Catch a stop edge that arrived while calculating this step.
        if (!startSignal) command = fsm.step(sensors, false, millis());
        changeState(fsm.state());
        telemetryPublishMotor(command.left, command.right, stateName(fsm.state()));
        telemetryPublishState("COMBAT", stateName(fsm.state()), -1,
                              combatStateElapsedMs(millis()), 0, startSignal,
                              startSignal ? "RUNNING" : "START_INACTIVE");
        applyMotor(leftMotor, command.left);
        applyMotor(rightMotor, command.right);
#if ENABLE_TASK_TIMING
        recordTaskTiming(TimedTask::Fsm, uint32_t(micros() - startUs),
                         hasPreviousStart ? uint32_t(startUs - previousStartUs) : 0);
        previousStartUs = startUs;
        hasPreviousStart = true;
#endif
        if (xTaskGetTickCount() - wake >= FSM_PERIOD_TICKS) wake = xTaskGetTickCount();
        vTaskDelayUntil(&wake, FSM_PERIOD_TICKS);
    }
}

void loop() { delay(10); }
#endif
