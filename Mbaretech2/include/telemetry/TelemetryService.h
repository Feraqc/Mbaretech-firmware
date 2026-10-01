#pragma once
#include "firmwareConfig.h"
#include "telemetry/TelemetryProtocol.h"

// Publish only from tasks (not ISRs). This function stamps a microsecond
// timestamp and performs a zero-wait bounded queue write. The dispatcher
// assigns the shared sequence in queue order before fan-out.
#if ENABLE_TELEMETRY
bool telemetryStart(const char* machine);
void telemetryPublish(TelemetryEvent event);
void telemetryPublishSnapshot(const SensorSnapshot& snapshot, int lineThreshold);
void telemetryPublishMotor(int left, int right, const char* source, uint32_t parameterRevision = 0);
void telemetryPublishState(const char* machine, const char* state, int step,
                           uint32_t elapsedMs, uint32_t stepElapsedMs, bool running,
                           const char* reason, uint32_t parameterRevision = 0);
void telemetryPublishText(TelemetryType type, const char* message);
void telemetryPublishHello();
void telemetryAwaitWifiHello();
#else
inline bool telemetryStart(const char*) { return true; }
inline void telemetryPublish(TelemetryEvent) {}
inline void telemetryPublishSnapshot(const SensorSnapshot&, int) {}
inline void telemetryPublishMotor(int, int, const char*, uint32_t = 0) {}
inline void telemetryPublishState(const char*, const char*, int, uint32_t, uint32_t, bool, const char*, uint32_t = 0) {}
inline void telemetryPublishText(TelemetryType, const char*) {}
inline void telemetryPublishHello() {}
inline void telemetryAwaitWifiHello() {}
#endif
