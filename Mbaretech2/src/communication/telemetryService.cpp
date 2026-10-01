#include "firmwareConfig.h"
#if ENABLE_TELEMETRY
#include "telemetry/TelemetryService.h"
#include "telemetry/TelemetryFanout.h"
#include "fsm/WifiTelemetry.h"
#if ENABLE_RECIPE_FSM
#include "fsm/ParameterService.h"
#endif
#include "bluetoothComm.h"
#include <Arduino.h>
#if ENABLE_WIFI_TELEMETRY
#include <WiFi.h>
#endif
#include <esp_timer.h>
#include <esp_system.h>
#include <atomic>
#include <cstring>
#if ENABLE_GYRO
#include "IMU.h"
#endif

namespace {
struct Line { char text[512]; };
QueueHandle_t captureQueue = nullptr;
QueueHandle_t serialQueue = nullptr;
QueueHandle_t bleQueue = nullptr;
QueueHandle_t wifiQueue = nullptr;
uint32_t sequence = 0; // Only the dispatcher writes this counter.
std::atomic<uint32_t> captureDropped{0}, serialDropped{0}, bleDropped{0}, wifiDropped{0};
char machineName[32] = {};
uint32_t bootId = 0; // Constante por arranque; permite invalidar cachés del editor.
std::atomic<bool> wifiAwaitHello{false};

void copyName(char* target, size_t capacity, const char* source) {
    if (!source) source = "";
    strncpy(target, source, capacity - 1);
    target[capacity - 1] = '\0';
}

void dispatchTask(void*) {
    TelemetryEvent event{};
    Line line{};
    uint32_t lastStats = 0, lastHeartbeat = 0, lastHello = 0;
    for (;;) {
        if (xQueueReceive(captureQueue, &event, pdMS_TO_TICKS(25)) == pdTRUE) {
            event.sequence = ++sequence;
            if (!formatTelemetry(event, line.text, sizeof(line.text))) {
                ++captureDropped;
                continue;
            }
#if ENABLE_WIFI_TELEMETRY
            if (event.type == TelemetryType::Hello) wifiAwaitHello = false;
#endif
            const bool wifiActive = ENABLE_WIFI_TELEMETRY && fsm::wifiTelemetryConnected() && !wifiAwaitHello;
            const TelemetryFanoutDrops lost = telemetryFanout(
                line.text, ENABLE_SERIAL, ENABLE_BLE, wifiActive,
                [&](TelemetrySink sink, const char*) {
                    QueueHandle_t target = sink == TelemetrySink::Serial ? serialQueue :
                        sink == TelemetrySink::Bluetooth ? bleQueue : wifiQueue;
                    return target && xQueueSend(target, &line, 0) == pdTRUE;
                });
            if (lost.serial) ++serialDropped;
            if (lost.bluetooth) ++bleDropped;
            if (lost.wifi) ++wifiDropped;
        }
        const uint32_t now = millis();
        if (uint32_t(now - lastHeartbeat) >= 1000) {
            lastHeartbeat = now;
            TelemetryEvent heartbeat{};
            heartbeat.type = TelemetryType::Heartbeat;
            heartbeat.value = now;
#if ENABLE_WIFI_TELEMETRY
            heartbeat.extra = WiFi.getMode() == WIFI_AP || WiFi.status() == WL_CONNECTED;
            heartbeat.left = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
#endif
            telemetryPublish(heartbeat);
        }
        // Serial has no reliable attach callback; a periodic HELLO lets a
        // terminal opened after boot identify the stream without a command.
        if (uint32_t(now - lastHello) >= 5000) {
            lastHello = now;
            telemetryPublishHello();
        }
        if (uint32_t(now - lastStats) >= 1000) {
            lastStats = now;
            TelemetryEvent stats{};
            stats.type = TelemetryType::TelemetryStats;
            stats.value = captureDropped.load();
            stats.extra = serialDropped.load();
            stats.left = bleDropped.load();
            stats.right = wifiDropped.load();
            telemetryPublish(stats);
        }
    }
}

void serialTask(void*) {
    Line line{};
    for (;;) {
#if ENABLE_SERIAL && ENABLE_RECIPE_FSM && !ENABLE_LOGGING
        // En builds sin menú BLE, este worker es el único lector Serial.
        for (unsigned i = 0; i < 64 && Serial.available(); ++i)
            fsm::parameterReceiveByte(fsm::ParameterSource::Serial, char(Serial.read()));
#endif
        if (xQueueReceive(serialQueue, &line, pdMS_TO_TICKS(5)) != pdTRUE) continue;
#if ENABLE_SERIAL
        const size_t length = strlen(line.text);
        // This worker may wait for its own transport. Its bounded input queue
        // drops later copies, while BLE/WiFi dispatch continues independently.
        size_t offset = 0;
        while (offset < length + 1) {
            const int room = Serial.availableForWrite();
            if (room <= 0) { vTaskDelay(1); continue; }
            const size_t count = size_t(room) < length - offset ? size_t(room) : length - offset;
            if (count) offset += Serial.write(reinterpret_cast<const uint8_t*>(line.text + offset), count);
            else { Serial.write('\n'); ++offset; }
        }
#endif
    }
}

void bleTask(void*) {
    Line line{};
    for (;;) {
        if (xQueueReceive(bleQueue, &line, portMAX_DELAY) == pdTRUE) {
#if ENABLE_BLE
            if (!sendTelemetryBle(line.text)) ++bleDropped;
#endif
        }
    }
}

void wifiTask(void*) {
    Line line{};
    for (;;) {
        if (xQueueReceive(wifiQueue, &line, portMAX_DELAY) == pdTRUE) {
#if ENABLE_WIFI_TELEMETRY
            if (!fsm::sendWifiTelemetry(line.text)) ++wifiDropped;
#endif
        }
    }
}

#if ENABLE_GYRO
void imuTask(void*) {
    IMU imu;
    imu.begin();
    if (!imu.isReady()) {
        telemetryPublishText(TelemetryType::Error, "IMU_INIT");
        vTaskDelete(nullptr);
        return;
    }
    for (;;) {
        imu.getData();
        TelemetryEvent event{};
        event.type = TelemetryType::ImuSample;
        event.extra = imu.hasYaw();
        event.left = imu.currentAngle;
        telemetryPublish(event);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
#endif
}

bool telemetryStart(const char* machine) {
    copyName(machineName, sizeof(machineName), machine);
    bootId = esp_random();
    captureQueue = xQueueCreate(128, sizeof(TelemetryEvent));
#if ENABLE_SERIAL
    serialQueue = xQueueCreate(32, sizeof(Line));
#endif
#if ENABLE_BLE
    bleQueue = xQueueCreate(16, sizeof(Line));
#endif
#if ENABLE_WIFI_TELEMETRY
    wifiQueue = xQueueCreate(32, sizeof(Line));
#endif
    if (!captureQueue || (ENABLE_SERIAL && !serialQueue) ||
        (ENABLE_BLE && !bleQueue) || (ENABLE_WIFI_TELEMETRY && !wifiQueue)) return false;
    if (xTaskCreate(dispatchTask, "telemetryDispatch", 6144, nullptr, 1, nullptr) != pdPASS) return false;
#if ENABLE_SERIAL
    if (xTaskCreate(serialTask, "telemetrySerial", 3072, nullptr, 1, nullptr) != pdPASS) return false;
#endif
#if ENABLE_BLE
    if (xTaskCreate(bleTask, "telemetryBle", 4096, nullptr, 1, nullptr) != pdPASS) return false;
#endif
#if ENABLE_WIFI_TELEMETRY
    if (xTaskCreate(wifiTask, "telemetryWifi", 4096, nullptr, 1, nullptr) != pdPASS) return false;
#endif
#if ENABLE_GYRO
    if (xTaskCreate(imuTask, "telemetryIMU", 4096, nullptr, 1, nullptr) != pdPASS) return false;
#endif
    telemetryPublishHello();
    return true;
}

void telemetryPublish(TelemetryEvent event) {
    if (!event.atUs) event.atUs = uint64_t(esp_timer_get_time());
    if (!captureQueue || xQueueSend(captureQueue, &event, 0) != pdTRUE) ++captureDropped;
}

void telemetryPublishHello() {
    TelemetryEvent event{};
    event.type = TelemetryType::Hello;
    copyName(event.machine, sizeof(event.machine), machineName);
    event.extra = bootId;
#if ENABLE_RECIPE_FSM
    fsm::ParameterSnapshot snapshot{};
    fsm::parameterServiceStore().snapshot(snapshot);
    event.paramRevision = snapshot.revision;
    event.paramCount = 1; // Versión del esquema de parámetros disponible.
#endif
    telemetryPublish(event);
}

void telemetryAwaitWifiHello() {
#if ENABLE_WIFI_TELEMETRY
    wifiAwaitHello = true;
    if (wifiQueue) xQueueReset(wifiQueue);
#endif
}

void telemetryPublishSnapshot(const SensorSnapshot& snapshot, int lineThreshold) {
    static SensorSnapshot previous{};
    static bool seen = false;
    if (seen) {
        if (snapshot.startActive != previous.startActive) {
            TelemetryEvent event{};
            event.type = TelemetryType::StartChanged;
            event.value = snapshot.startActive;
            telemetryPublish(event);
        }
        for (unsigned i = 0; i < 7; ++i) if (snapshot.ir[i] != previous.ir[i]) {
            TelemetryEvent event{};
            event.type = TelemetryType::IrChanged;
            event.value = i + 1; event.extra = snapshot.ir[i];
            telemetryPublish(event);
        }
        for (unsigned i = 0; i < 2; ++i) if (snapshot.line[i] != previous.line[i]) {
            TelemetryEvent event{};
            event.type = TelemetryType::LineChanged;
            event.value = i; event.extra = snapshot.line[i]; event.left = snapshot.rawLine[i];
            telemetryPublish(event);
        }
    }
    static uint32_t lastSample = 0;
    if (!seen || uint32_t(snapshot.sampledAtMs - lastSample) >= 100) {
        TelemetryEvent event{};
        event.type = TelemetryType::SensorSnapshot;
        event.sensors = snapshot;
        event.value = lineThreshold;
        telemetryPublish(event);
        lastSample = snapshot.sampledAtMs;
    }
    previous = snapshot;
    seen = true;
}

void telemetryPublishMotor(int left, int right, const char* source, uint32_t parameterRevision) {
    static int previousLeft = 999, previousRight = 999;
    static uint32_t lastSample = 0;
    const uint32_t now = millis();
    if (left == previousLeft && right == previousRight && uint32_t(now - lastSample) < 1000) return;
    previousLeft = left; previousRight = right;
    lastSample = now;
    TelemetryEvent event{};
    event.type = TelemetryType::MotorCommand;
    event.left = left; event.right = right;
    event.paramRevision = parameterRevision;
    copyName(event.state, sizeof(event.state), source);
    telemetryPublish(event);
}

void telemetryPublishState(const char* machine, const char* state, int step,
                           uint32_t elapsedMs, uint32_t stepElapsedMs, bool running,
                           const char* reason, uint32_t parameterRevision) {
    static uint32_t lastSample = 0;
    const uint32_t now = millis();
    if (uint32_t(now - lastSample) < 100) return;
    lastSample = now;
    TelemetryEvent event{};
    event.type = TelemetryType::FsmState;
    copyName(event.machine, sizeof(event.machine), machine);
    copyName(event.state, sizeof(event.state), state);
    copyName(event.condition, sizeof(event.condition), reason);
    event.step = step; event.elapsedMs = elapsedMs; event.extra = stepElapsedMs; event.value = running;
    event.paramRevision = parameterRevision;
    telemetryPublish(event);
}

void telemetryPublishText(TelemetryType type, const char* message) {
    TelemetryEvent event{};
    event.type = type;
    copyName(event.condition, sizeof(event.condition), message);
    telemetryPublish(event);
}
#endif
