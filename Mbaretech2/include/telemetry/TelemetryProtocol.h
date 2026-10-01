#pragma once
#include <stddef.h>
#include <stdint.h>
#include "sensorSnapshot.h"
#include "fsm/RuntimeParameters.h"

// Transport-independent capture record. Producers fill fields and publish it;
// only the low-priority service serializes the record for transport workers.
enum class TelemetryType : uint8_t {
    Hello, Heartbeat, StartChanged, SensorSnapshot, IrChanged, LineChanged,
    MotorCommand, FsmState, FsmTransition, FsmStepTransition, TaskTiming,
    TelemetryStats, ImuSample, Error, Log,
    ParamSchema, ParamValues, ParamAck, ParamChanged, StartAck
};

struct TelemetryEvent {
    TelemetryType type = TelemetryType::Log;
    uint32_t sequence = 0;
    uint64_t atUs = 0;
    SensorSnapshot sensors{};
    char machine[32] = {};
    char state[32] = {};
    char next[32] = {};
    char condition[96] = {};
    int32_t left = 0;
    int32_t right = 0;
    int32_t step = -1;
    int32_t nextStep = -1;
    uint32_t elapsedMs = 0;
    uint32_t timerMs = 0;
    uint32_t value = 0;
    uint32_t extra = 0;
    const fsm::ParameterMetadata* parameter = nullptr; // Registro estático; no se copia a la cola.
    uint32_t paramRevision = 0;
    uint32_t paramTransaction = 0;
    int32_t paramValue = 0;
    int32_t paramPrevious = 0;
    uint16_t paramIndex = 0;
    uint16_t paramCount = 0;
    bool paramAccepted = false;
};

const char* telemetryTypeName(TelemetryType type);
bool formatTelemetry(const TelemetryEvent& event, char* output, size_t capacity);
