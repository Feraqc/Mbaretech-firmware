#pragma once

enum class TelemetrySink { Serial, Bluetooth, Wifi };

struct TelemetryFanoutDrops {
    bool serial = false;
    bool bluetooth = false;
    bool wifi = false;
};

// The writer must use zero-wait bounded queues. A rejected sink does not stop
// delivery to later sinks; all three receive the same serialized bytes.
template <typename Writer>
TelemetryFanoutDrops telemetryFanout(const char* line, bool serial, bool bluetooth,
                                    bool wifi, Writer writer) {
    TelemetryFanoutDrops drops{};
    if (serial) drops.serial = !writer(TelemetrySink::Serial, line);
    if (bluetooth) drops.bluetooth = !writer(TelemetrySink::Bluetooth, line);
    if (wifi) drops.wifi = !writer(TelemetrySink::Wifi, line);
    return drops;
}
