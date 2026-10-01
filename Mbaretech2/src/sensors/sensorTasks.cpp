#include "firmwareConfig.h"
#if ENABLE_SENSOR_TASK
#include "sensorTasks.h"
#include "telemetry/TelemetryService.h"
#if ENABLE_RECIPE_FSM
#include "fsm/ParameterService.h"
#endif

namespace {
portMUX_TYPE snapshotMux = portMUX_INITIALIZER_UNLOCKED;
SensorSnapshot latest;
#if ENABLE_TASK_TIMING
TaskTiming timings[2];
#endif
TaskHandle_t sensorTaskHandle = nullptr;

void readLineSensors(SensorSnapshot& sample) {
#if ENABLE_LINE_SENSORS
    sample.rawLine[0] = readLineSensorFront(LINE_FRONT_LEFT);
    sample.rawLine[1] = readLineSensorFront(LINE_FRONT_RIGHT);
    sample.line[0] = checkLineSensora(sample.rawLine[0]);
    sample.line[1] = checkLineSensorb(sample.rawLine[1]);
#endif
}

void readIrSensors(SensorSnapshot& sample) {
#if ENABLE_IR_SENSORS
#ifdef MBARETECH_2
    sample.ir[SIDE_LEFT] = !digitalRead(IR1);
    sample.ir[SIDE_RIGHT] = !digitalRead(IR7);
    sample.ir[SHORT_LEFT] = !digitalRead(IR2);
    sample.ir[TOP_LEFT] = !digitalRead(IR3);
    sample.ir[TOP_RIGHT] = !digitalRead(IR5);
    sample.ir[SHORT_RIGHT] = !digitalRead(IR6);
#else
    sample.ir[SHORT_LEFT] = digitalRead(IR2);
    sample.ir[TOP_LEFT] = digitalRead(IR3);
    sample.ir[TOP_RIGHT] = digitalRead(IR5);
    sample.ir[SHORT_RIGHT] = digitalRead(IR6);
#endif
    sample.ir[TOP_MID] = !digitalRead(IR4);
#endif
}

void readDipSwitches(SensorSnapshot& sample) {
#if ENABLE_DIP_SWITCHES
    sample.dip[DIP_A] = digitalRead(DIPA);
    sample.dip[DIP_B] = digitalRead(DIPB);
    sample.dip[DIP_C] = digitalRead(DIPC);
    sample.dip[DIP_D] = digitalRead(DIPD);
    sample.dip[DIP_E] = digitalRead(DIPE);
#endif
}

SensorSnapshot acquireSensors() {
    SensorSnapshot sample;
    readLineSensors(sample);
    readIrSensors(sample);
    readDipSwitches(sample);
    sample.sampledAtMs = millis();
    sample.valid = true;
#if ENABLE_LINE_SENSORS
    sample.valid = sample.rawLine[0] >= 0 && sample.rawLine[1] >= 0;
#endif
    return sample;
}
}

SensorSnapshot readSensorSnapshot() {
    portENTER_CRITICAL(&snapshotMux);
    SensorSnapshot result = latest;
    portEXIT_CRITICAL(&snapshotMux);
    // START proviene del latch ISR o del comando remoto; no leemos GPIO aquí.
#if ENABLE_RECIPE_FSM
    result.startActive = fsm::parameterServiceEffectiveStart(startSignal, result.startRemoteControlled);
#else
    result.startActive = startSignal;
#endif
    result.startObservedAtMs = millis();
    return result;
}

void sensorReadTask(void*) {
    TickType_t wake = xTaskGetTickCount();
#if ENABLE_TASK_TIMING
    uint32_t previousStartUs = 0;
    bool hasPreviousStart = false;
#endif
    for (;;) {
#if ENABLE_TASK_TIMING
        const uint32_t startUs = micros();
#endif
        SensorSnapshot sample = acquireSensors();
#if ENABLE_RECIPE_FSM
        sample.startActive = fsm::parameterServiceEffectiveStart(startSignal, sample.startRemoteControlled);
#else
        sample.startActive = startSignal;
#endif
        sample.startObservedAtMs = millis();
        portENTER_CRITICAL(&snapshotMux);
        latest = sample;
        portEXIT_CRITICAL(&snapshotMux);
        telemetryPublishSnapshot(sample, THRESHOLD);
#if ENABLE_TASK_TIMING
        recordTaskTiming(TimedTask::Sensor, uint32_t(micros() - startUs),
                         hasPreviousStart ? uint32_t(startUs - previousStartUs) : 0);
        previousStartUs = startUs;
        hasPreviousStart = true;
#endif
        // If acquisition overruns, yield a full period rather than catch up in a burst.
        if (xTaskGetTickCount() - wake >= SENSOR_PERIOD_TICKS)
            wake = xTaskGetTickCount();
        vTaskDelayUntil(&wake, SENSOR_PERIOD_TICKS);
    }
}

bool startSensorTask() {
#if !(ENABLE_LINE_SENSORS || ENABLE_IR_SENSORS || ENABLE_DIP_SWITCHES || ENABLE_RECIPE_FSM)
    return true; // Gyro-only logging needs no fast acquisition task.
#else
    if (sensorTaskHandle) return true;
    return xTaskCreate(sensorReadTask, "sensorRead", 3072, nullptr, SENSOR_TASK_PRIORITY,
                       &sensorTaskHandle) == pdPASS;
#endif
}
#if ENABLE_TASK_TIMING
void recordTaskTiming(TimedTask task, uint32_t executionUs, uint32_t gapUs) {
    const unsigned index = task == TimedTask::Sensor ? 0 : 1;
    const uint32_t periodTicks = index == 0 ? SENSOR_PERIOD_TICKS : FSM_PERIOD_TICKS;
    const uint32_t periodUs = periodTicks * (1000000u / configTICK_RATE_HZ);
    portENTER_CRITICAL(&snapshotMux);
    TaskTiming& timing = timings[index];
    if (executionUs > timing.maxExecutionUs) timing.maxExecutionUs = executionUs;
    if (gapUs > timing.maxStartGapUs) timing.maxStartGapUs = gapUs;
    if (executionUs >= periodUs) ++timing.overrunCount;
    portEXIT_CRITICAL(&snapshotMux);
    static uint32_t lastPublished[2] = {};
    const uint32_t now = millis();
    if (uint32_t(now - lastPublished[index]) >= 100) {
        lastPublished[index] = now;
        TelemetryEvent event{};
        event.type = TelemetryType::TaskTiming;
        const char* name = task == TimedTask::Sensor ? "SENSOR" : "FSM";
        for (unsigned i = 0; name[i] && i + 1 < sizeof(event.state); ++i) event.state[i] = name[i];
        event.value = executionUs;
        event.extra = gapUs;
        telemetryPublish(event);
    }
}

TaskTiming readTaskTiming(TimedTask task) {
    portENTER_CRITICAL(&snapshotMux);
    const TaskTiming result = timings[task == TimedTask::Sensor ? 0 : 1];
    portEXIT_CRITICAL(&snapshotMux);
    return result;
}
#endif
#endif
