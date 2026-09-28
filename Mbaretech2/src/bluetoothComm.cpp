#ifndef RUN_GYRO_TEST
#include "bluetoothComm.h"
#include "dataLogging.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <atomic>

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static BLECharacteristic* tx;
static std::atomic<bool> connected{false};
static std::atomic<bool> disconnected{false};
static QueueHandle_t commands;
struct Command { char text[64]; };

void sendData(const String& data) {
    Serial.print(data);
    // Clients reassemble the newline stream, even with the default ATT MTU.
    for (size_t offset = 0; connected && offset < data.length(); offset += 20) {
        size_t count = data.length() - offset;
        if (count > 20) count = 20;
        tx->setValue(reinterpret_cast<uint8_t*>(const_cast<char*>(data.c_str() + offset)), count);
        tx->notify();
    }
}

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer*) override {
        connected = true;
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
        if (value.empty() || value.size() >= sizeof(Command::text)) return;
        Command command{};
        memcpy(command.text, value.data(), value.size());
        xQueueSend(commands, &command, 0);
    }
};

static void readSerialCommands() {
    static char buffer[64];
    static size_t length = 0;
    static bool overflow = false, previousCR = false;
    // Bound work per pass; never wait for an incomplete terminal line.
    for (int i = 0; i < 64 && Serial.available() > 0; ++i) {
        const char ch = static_cast<char>(Serial.read());
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

static void bluetoothTask(void*) {
    for (;;) {
        if (disconnected.exchange(false)) {
            loggingDisconnected();
            xQueueReset(commands);
        }
        Command command;
        for (int i = 0; i < 4 && xQueueReceive(commands, &command, 0) == pdTRUE; ++i)
            bluetoothCommand(String(command.text));
        readSerialCommands();
        loggingPoll();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void BLE_UART_Init(const char* deviceName) {
    commands = xQueueCreate(16, sizeof(Command));
    if (!commands) return;
    loggingInit();
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
    xTaskCreate(bluetoothTask, "bluetoothLog", 6144, nullptr, 1, nullptr);
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
#endif // RUN_GYRO_TEST
