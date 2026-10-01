#pragma once
#include "firmwareConfig.h"

namespace fsm {
#if ENABLE_WIFI_TELEMETRY
void startWifiTelemetry();
// Called only by the isolated WiFi transport worker, never from control.
bool sendWifiTelemetry(const char* line);
bool wifiTelemetryConnected();
#else
inline void startWifiTelemetry() {}
inline bool sendWifiTelemetry(const char*) { return false; }
inline bool wifiTelemetryConnected() { return false; }
#endif
}
