#pragma once
#include "globals.h"
#include "firmwareConfig.h"
#include "sensorSnapshot.h"

// Periods are ticks; maneuver durations and snapshot age are milliseconds.
constexpr TickType_t SENSOR_PERIOD_TICKS = SENSOR_READ_PERIOD_TICKS;
constexpr TickType_t FSM_PERIOD_TICKS = FSM_STEP_PERIOD_TICKS;
constexpr uint32_t SENSOR_MAX_AGE_MS = 50;

// The FSM outranks acquisition so slow ADC calls cannot starve stop handling.
constexpr unsigned FSM_TASK_PRIORITY = 3;
constexpr unsigned SENSOR_TASK_PRIORITY = 2;

bool startSensorTask();
void sensorReadTask(void* parameter);
SensorSnapshot readSensorSnapshot();

#if ENABLE_TASK_TIMING
// Maxima since boot. Timing measurements are enabled only for diagnostics.
struct TaskTiming {
    uint32_t maxExecutionUs = 0;
    uint32_t maxStartGapUs = 0;
    uint32_t overrunCount = 0;
};
enum class TimedTask { Sensor, Fsm };
void recordTaskTiming(TimedTask task, uint32_t executionUs, uint32_t gapUs);
TaskTiming readTaskTiming(TimedTask task);
#endif
