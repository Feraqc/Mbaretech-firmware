#pragma once
#include "FSMDefinitions.h"
#include "sensorSnapshot.h"
#include "StateMachine.h"

namespace fsm {
// La salud de adquisición y START gobiernan todo el ciclo genérico.
// A recent START observation cannot make an old acquisition sample fresh.
inline bool isControlDataValid(const SensorSnapshot& sensors, uint32_t nowMs) {
    return sensors.valid &&
           uint32_t(nowMs - sensors.sampledAtMs) <= fsm_defs::runtime::SENSOR_MAX_AGE_MS;
}
// The bench override changes motor permission only. Diagnostic conditions and
// the ISR-owned physical latch retain the real START value.
inline bool effectiveStartActive(const SensorSnapshot& sensors) {
    if (sensors.startRemoteControlled) return sensors.startActive;
#if FORCE_START_ACTIVE
    (void)sensors;
    return true;
#else
    return sensors.startActive;
#endif
}

// Shared by task and host tests. Check health before enabling a retained command:
// a START rising edge accompanied by stale data must not briefly energize motors.
inline bool refreshRecipeMotorPermission(StateMachine& machine, Drive& drive,
                                        const SensorSnapshot& sensors, uint32_t nowMs) {
    if (!isControlDataValid(sensors, nowMs)) {
        drive.setMotionEnabled(false);
        machine.stop();
        return false;
    }
    drive.setMotionEnabled(effectiveStartActive(sensors));
    // Un START OFF, físico o remoto, descarta estado, paso y temporizadores.
    if (!effectiveStartActive(sensors)) machine.stop();
    return true;
}

// Sólo ON inicia la receta desde el estado inicial; OFF espera sin ejecutar FSM.
inline void updateRecipeControl(StateMachine& machine, Drive& drive,
                                const SensorSnapshot& sensors, uint32_t nowMs) {
    if (!refreshRecipeMotorPermission(machine, drive, sensors, nowMs)) return;
    if (!effectiveStartActive(sensors)) return;
    if (!machine.isRunning() && !machine.begin(nowMs)) return;
    machine.update(sensors, nowMs);
}
}
