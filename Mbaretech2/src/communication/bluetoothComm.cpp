#include "firmwareConfig.h"
#if ENABLE_LOGGING
#include "bluetoothComm.h"
#include "dataLogging.h"
#include "telemetry/TelemetryService.h"
#if ENABLE_RECIPE_FSM && ENABLE_TELEMETRY
#include "fsm/ParameterService.h"
#endif
#if ENABLE_RECIPE_FSM
#include "fsm/TransitionLog.h"
#endif
#if ENABLE_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <atomic>
#endif

#if ENABLE_BLE
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static BLECharacteristic* tx;
static std::atomic<bool> connected{false};
static std::atomic<bool> disconnected{false};
static QueueHandle_t commands;
struct Command { char text[64]; };
#endif

void sendData(const String& data) {
#if ENABLE_SERIAL
    Serial.print(data);
#endif
#if ENABLE_BLE
    // Clients reassemble the newline stream, even with the default ATT MTU.
    for (size_t offset = 0; connected && offset < data.length(); offset += 20) {
        size_t count = data.length() - offset;
        if (count > 20) count = 20;
        tx->setValue(reinterpret_cast<uint8_t*>(const_cast<char*>(data.c_str() + offset)), count);
        tx->notify();
    }
#endif
}

bool sendTelemetryBle(const char* line) {
#if ENABLE_BLE
    if (!connected || !tx || !line) return false;
    const size_t length = strlen(line);
    // BLE UART is a newline-delimited byte stream. The receiver reassembles
    // ATT fragments before running the common telemetry decoder.
    for (size_t offset = 0; connected && offset <= length; offset += 20) {
        const size_t remaining = length + 1 - offset;
        const size_t count = remaining > 20 ? 20 : remaining;
        if (!count) break;
        uint8_t fragment[20];
        for (size_t i = 0; i < count; ++i)
            fragment[i] = offset + i == length ? '\n' : uint8_t(line[offset + i]);
        tx->setValue(fragment, count);
        tx->notify();
    }
    return connected;
#else
    (void)line;
    return false;
#endif
}

#if ENABLE_BLE
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer*) override {
        connected = true;
        telemetryPublishHello();
        Command command = {"ayuda"};
        xQueueSend(commands, &command, 0);
    }
    void onDisconnect(BLEServer* server) override {
        connected = false;
        disconnected = true;
        server->startAdvertising();
    }
};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* characteristic) override {
        const std::string value = characteristic->getValue();
#if ENABLE_RECIPE_FSM && ENABLE_TELEMETRY
        if (!value.empty() && (value[0] == '{' ||
            fsm::parameterIngressActive(fsm::ParameterSource::Bluetooth))) {
            for (char byte : value) fsm::parameterReceiveByte(fsm::ParameterSource::Bluetooth, byte);
            return;
        }
#endif
        if (value.empty() || value.size() >= sizeof(Command::text)) return;
        Command command{};
        memcpy(command.text, value.data(), value.size());
        xQueueSend(commands, &command, 0);
    }
};

#endif

#if ENABLE_SERIAL
static void readSerialCommands() {
    static char buffer[64];
    static size_t length = 0;
    static bool overflow = false, previousCR = false;
    // Bound work per pass; never wait for an incomplete terminal line.
    for (int i = 0; i < 64 && Serial.available() > 0; ++i) {
        const char ch = static_cast<char>(Serial.read());
#if ENABLE_RECIPE_FSM && ENABLE_TELEMETRY
        if (ch == '{' || fsm::parameterIngressActive(fsm::ParameterSource::Serial)) {
            fsm::parameterReceiveByte(fsm::ParameterSource::Serial, ch);
            continue;
        }
#endif
        if (ch == '\n' && previousCR) { previousCR = false; continue; }
        previousCR = ch == '\r';
        if (ch == '\r' || ch == '\n') {
            if (overflow) sendData("Comando demasiado largo (maximo 63 bytes).\n");
            else {
                buffer[length] = '\0';
                bluetoothCommand(String(buffer));
            }
            length = 0;
            overflow = false;
        } else if (!overflow) {
            if (length < sizeof(buffer) - 1) buffer[length++] = ch;
            else overflow = true;
        }
    }
}

#endif

static void communicationsTask(void*) {
    for (;;) {
#if ENABLE_BLE
        if (disconnected.exchange(false)) {
            loggingDisconnected();
            xQueueReset(commands);
        }
        Command command;
        for (int i = 0; i < 4 && xQueueReceive(commands, &command, 0) == pdTRUE; ++i)
            bluetoothCommand(String(command.text));
#endif
#if ENABLE_SERIAL
        readSerialCommands();
#endif
#if !ENABLE_TELEMETRY
        loggingPoll();
#endif
#if ENABLE_RECIPE_FSM
        fsm::pollRecipeTransitions();
#endif
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void communicationsInit(const char* deviceName) {
#if ENABLE_BLE
    commands = xQueueCreate(16, sizeof(Command));
    if (!commands) return;
    BLEDevice::init(deviceName);
    BLEServer* server = BLEDevice::createServer();
    server->setCallbacks(new MyServerCallbacks());
    BLEService* service = server->createService(SERVICE_UUID);
    tx = service->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    tx->addDescriptor(new BLE2902());
    BLECharacteristic* rx = service->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
    rx->setCallbacks(new MyCallbacks());
    service->start();
    server->getAdvertising()->start();
#endif
    loggingInit();
    xTaskCreate(communicationsTask, "communications", 6144, nullptr, 1, nullptr);
}

void bluetoothCommand(const String& input) {
    String command = input;
    command.trim();
    // Preserve INDEX VALUE in every menu.
    long index, value;
    char extra;
    if (sscanf(command.c_str(), "%ld %ld %c", &index, &value, &extra) == 2) {
        if (index >= 0 && index < ARRAY_PARAMETROS_SIZE) parametros[index] = value;
        else sendData("Indice fuera de rango (0-18).\n");
        return;
    }
    loggingCommand(command);
}
#endif // ENABLE_LOGGING
