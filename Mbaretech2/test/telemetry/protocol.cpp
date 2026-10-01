#include "telemetry/TelemetryProtocol.h"
#include "telemetry/TelemetryFanout.h"
#include <cassert>
#include <cstring>

template <size_t N> void put(char (&target)[N], const char* text) {
    const size_t length = std::strlen(text) + 1;
    assert(length <= N);
    std::memcpy(target, text, length);
}

int main() {
    char line[512];
    TelemetryEvent hello{};
    hello.type = TelemetryType::Hello;
    hello.sequence = 17;
    hello.atUs = 1234567;
    put(hello.machine, "STATE_TEST");
    hello.extra = 1234;
    assert(formatTelemetry(hello, line, sizeof(line)));
    assert(std::strstr(line, "\"seq\":17"));
    assert(std::strstr(line, "\"t\":1234,\"us\":1234567"));
    assert(std::strstr(line, "\"protocol\":1,\"schema\":2"));
    assert(std::strstr(line, "\"bootId\":1234"));

    TelemetryEvent sensors{};
    sensors.type = TelemetryType::SensorSnapshot;
    sensors.sequence = 18;
    sensors.atUs = 1235000;
    sensors.sensors.valid = true;
    sensors.sensors.startActive = true;
    sensors.sensors.ir[3] = true;
    sensors.sensors.rawLine[0] = 123;
    sensors.value = 145;
    assert(formatTelemetry(sensors, line, sizeof(line)));
    assert(std::strstr(line, "\"ir\":[0,0,0,1,0,0,0]"));
    assert(std::strstr(line, "\"lineThreshold\":145"));

    TelemetryEvent transition{};
    transition.type = TelemetryType::FsmTransition;
    transition.sequence = 19;
    transition.atUs = 1240000;
    put(transition.machine, "STATE_TEST");
    put(transition.state, "SEARCH");
    put(transition.next, "ATTACK");
    put(transition.condition, "IR3_DETECTED OR IR4_DETECTED");
    assert(formatTelemetry(transition, line, sizeof(line)));
    assert(std::strstr(line, "\"from\":\"SEARCH\",\"to\":\"ATTACK\""));
    assert(std::strstr(line, "IR3_DETECTED OR IR4_DETECTED"));

    // Las respuestas de parámetros deben caber en una línea canónica.
    fsm::ParameterMetadata metadata{};
    put(metadata.id, "state.SEARCH.timer.0");
    put(metadata.name, "State timer");
    put(metadata.group, "SEARCH");
    metadata.unit = "ms";
    metadata.defaultValue = 400;
    metadata.minimum = 0;
    metadata.maximum = 60000;
    metadata.step = 1;
    metadata.policy = fsm::ApplyPolicy::NextStateEntry;
    metadata.writable = true;
    TelemetryEvent schema{};
    schema.type = TelemetryType::ParamSchema;
    schema.parameter = &metadata;
    schema.paramRevision = 3;
    schema.paramIndex = 0;
    schema.paramCount = 1;
    assert(formatTelemetry(schema, line, sizeof(line)));
    assert(std::strstr(line, "\"id\":\"state.SEARCH.timer.0\""));
    assert(std::strstr(line, "\"applyPolicy\":\"next_state_entry\""));
    TelemetryEvent ack{};
    ack.type = TelemetryType::ParamAck;
    ack.paramTransaction = 19;
    ack.paramRevision = 4;
    ack.paramAccepted = true;
    put(ack.next, "next_state_entry");
    assert(formatTelemetry(ack, line, sizeof(line)));
    assert(std::strstr(line, "\"transaction\":19,\"revision\":4,\"status\":\"accepted\""));
    TelemetryEvent startAck{};
    startAck.type = TelemetryType::StartAck;
    startAck.paramTransaction = 71;
    startAck.value = 0;
    assert(formatTelemetry(startAck, line, sizeof(line)));
    assert(std::strstr(line, "\"transaction\":71,\"status\":\"accepted\",\"active\":false"));

    TelemetryEvent error{};
    error.type = TelemetryType::Error;
    put(error.condition, "quote \" and newline\n");
    assert(formatTelemetry(error, line, sizeof(line)));
    assert(std::strstr(line, "quote \\\" and newline\\n"));
    assert(!formatTelemetry(error, line, 16));

    // A congested BLE queue cannot prevent the same canonical message from
    // reaching Serial or WiFi; only BLE reports a transport drop.
    unsigned serialWrites = 0, bleWrites = 0, wifiWrites = 0;
    const TelemetryFanoutDrops lost = telemetryFanout(line, true, true, true,
        [&](TelemetrySink sink, const char* frame) {
            assert(frame == line);
            if (sink == TelemetrySink::Serial) { ++serialWrites; return true; }
            if (sink == TelemetrySink::Bluetooth) { ++bleWrites; return false; }
            ++wifiWrites; return true;
        });
    assert(serialWrites == 1 && bleWrites == 1 && wifiWrites == 1);
    assert(!lost.serial && lost.bluetooth && !lost.wifi);
    return 0;
}
