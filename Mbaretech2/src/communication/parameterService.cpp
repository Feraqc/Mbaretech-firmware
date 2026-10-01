#include "firmwareConfig.h"
#if ENABLE_RECIPE_FSM
#include "fsm/ParameterService.h"
#include "fsm/ParameterCommand.h"
#include "telemetry/TelemetryService.h"
#include "sensorTasks.h"
#include <Arduino.h>
#include <atomic>
#include <string.h>

namespace fsm {
namespace {
struct CommandLine { char text[1024]; };
struct Ingress { char text[1024]; size_t length = 0; bool active = false; };
RuntimeParameters parameters;
const char* activeMachine = nullptr;
portMUX_TYPE parameterMux = portMUX_INITIALIZER_UNLOCKED;
QueueHandle_t commandQueue = nullptr;
Ingress serialIngress, bleIngress;
std::atomic<bool> runningNow{false};
std::atomic<int8_t> remoteStart{-1}; // -1: pin físico; 0/1: comando remoto hasta reset.
std::atomic<uint32_t> startGeneration{0};

void enter(void*) { portENTER_CRITICAL(&parameterMux); }
void leave(void*) { portEXIT_CRITICAL(&parameterMux); }

void publishSnapshot(bool schema) {
    ParameterSnapshot snapshot{};
    parameters.snapshot(snapshot);
    for (unsigned i = 0; i < parameters.count(); ++i) {
        TelemetryEvent event{};
        event.type = schema ? TelemetryType::ParamSchema : TelemetryType::ParamValues;
        event.parameter = &parameters.metadata(i);
        event.paramRevision = snapshot.revision;
        event.paramIndex = i;
        event.paramCount = parameters.count();
        event.paramValue = snapshot.values[i];
        telemetryPublish(event);
        // Evitar saturar la cola de captura con catálogos largos.
        if ((i & 7u) == 7u) vTaskDelay(1);
    }
    if (!parameters.count()) {
        TelemetryEvent event{};
        event.type = schema ? TelemetryType::ParamSchema : TelemetryType::ParamValues;
        event.paramRevision = snapshot.revision;
        telemetryPublish(event);
    }
}
void publishAck(const ParameterCommand& command, const ParameterResult& result) {
    TelemetryEvent ack{};
    ack.type = TelemetryType::ParamAck;
    ack.paramTransaction = command.transaction;
    ack.paramRevision = result.revision;
    ack.paramAccepted = result.accepted;
    strncpy(ack.next, applyPolicyName(result.effective), sizeof(ack.next) - 1);
    strncpy(ack.condition, result.error ? result.error : "", sizeof(ack.condition) - 1);
    telemetryPublish(ack);
}
void process(const char* text) {
    ParameterCommand command{};
    const char* error = nullptr;
    if (!decodeParameterCommand(text, command, error)) {
        ParameterResult rejected{}; rejected.error = error;
        ParameterSnapshot current{}; parameters.snapshot(current);
        rejected.revision = current.revision;
        publishAck(command, rejected); return;
    }
    if (command.operation == ParameterOperation::SchemaRequest) { publishSnapshot(true); return; }
    if (command.operation == ParameterOperation::ValuesRequest) { publishSnapshot(false); return; }
    if (command.operation == ParameterOperation::StartSet) {
        remoteStart.store(command.startActive ? 1 : 0);
        startGeneration.fetch_add(1);
        TelemetryEvent ack{};
        ack.type = TelemetryType::StartAck;
        ack.paramTransaction = command.transaction;
        ack.value = command.startActive;
        telemetryPublish(ack);
        return;
    }
    const bool reset = command.operation == ParameterOperation::Reset;
    if (!activeMachine || strcmp(command.machine, activeMachine) != 0) {
        ParameterResult rejected{};
        rejected.error = "machine mismatch";
        ParameterSnapshot current{}; parameters.snapshot(current);
        rejected.revision = current.revision;
        publishAck(command, rejected); return;
    }
    // Live Test edita RAM sólo con START apagado y el control detenido.
    // Se comprueba el START efectivo (pin o override remoto), no el enlace WiFi.
    if (readSensorSnapshot().startActive || runningNow.load()) {
        ParameterResult rejected{};
        rejected.error = "start must be off";
        ParameterSnapshot current{}; parameters.snapshot(current);
        rejected.revision = current.revision;
        publishAck(command, rejected); return;
    }
    ParameterSnapshot before{};
    if (reset) parameters.snapshot(before);
    const ParameterResult result = reset ? parameters.reset(command.revision, runningNow.load()) :
        parameters.set(command.revision, command.changes, command.count, runningNow.load());
    publishAck(command, result);
    if (!result.accepted) return;
    ParameterSnapshot current{}; parameters.snapshot(current);
    if (reset) {
        // Cada valor restaurado deja una marca cronológica para el analizador.
        for (unsigned i = 0; i < parameters.count(); ++i) {
            if (before.values[i] == current.values[i]) continue;
            TelemetryEvent change{};
            change.type = TelemetryType::ParamChanged;
            change.parameter = &parameters.metadata(i);
            change.paramTransaction = command.transaction;
            change.paramRevision = result.revision;
            change.paramPrevious = before.values[i];
            change.paramValue = current.values[i];
            telemetryPublish(change);
            if ((i & 7u) == 7u) vTaskDelay(1);
        }
        publishSnapshot(false); return;
    }
    for (unsigned i = 0; i < command.count; ++i) {
        const int index = parameters.find(command.changes[i].id);
        TelemetryEvent change{};
        change.type = TelemetryType::ParamChanged;
        change.parameter = &parameters.metadata(index);
        change.paramTransaction = command.transaction;
        change.paramRevision = result.revision;
        change.paramPrevious = result.previous[i];
        change.paramValue = command.changes[i].value;
        telemetryPublish(change);
    }
    publishSnapshot(false);
}
void commandTask(void*) {
    CommandLine command{};
    for (;;) if (xQueueReceive(commandQueue, &command, portMAX_DELAY) == pdTRUE)
        process(command.text);
}
} // namespace

bool parameterServiceStart(const MachineRecipe& recipe) {
    activeMachine = recipe.name;
    parameters.setLock(enter, leave, nullptr);
    if (!parameters.initialize(recipe)) return false;
#if ENABLE_TELEMETRY
    commandQueue = xQueueCreate(8, sizeof(CommandLine));
    if (!commandQueue || xTaskCreate(commandTask, "parameterCommands", 6144,
                                     nullptr, 1, nullptr) != pdPASS) return false;
#endif
    return true;
}
RuntimeParameters& parameterServiceStore() { return parameters; }
void parameterServiceRunning(bool running) { runningNow = running; }
bool parameterServiceEffectiveStart(bool physicalStart, bool& remoteControlled) {
    const int8_t command = remoteStart.load();
    remoteControlled = command >= 0;
    return remoteControlled ? command == 1 : physicalStart;
}
uint32_t parameterServiceStartGeneration() { return startGeneration.load(); }
bool parameterSubmitFrame(const char* data, size_t length) {
    if (!commandQueue || !data || !length || length >= sizeof(CommandLine::text)) return false;
    CommandLine line{};
    memcpy(line.text, data, length);
    return xQueueSend(commandQueue, &line, 0) == pdTRUE;
}
bool parameterIngressActive(ParameterSource source) {
    return (source == ParameterSource::Serial ? serialIngress : bleIngress).active;
}
bool parameterReceiveByte(ParameterSource source, char byte) {
    Ingress& ingress = source == ParameterSource::Serial ? serialIngress : bleIngress;
    if (!ingress.active) {
        if (byte != '{') return false;
        ingress.active = true; ingress.length = 0;
    }
    if (byte == '\r' || byte == '\n') {
        if (ingress.length) parameterSubmitFrame(ingress.text, ingress.length);
        ingress.active = false; ingress.length = 0;
        return true;
    }
    if (ingress.length + 1 >= sizeof(ingress.text)) {
        ingress.active = false; ingress.length = 0;
        return true;
    }
    ingress.text[ingress.length++] = byte;
    return true;
}
} // namespace fsm
#endif
