#ifndef RUN_GYRO_TEST
#include "dataLogging.h"
#include "bluetoothComm.h"
#include "IMU.h"
#include <atomic>

static LoggingConfig config;
static bool menu = false, intervalPrompt = false;
static uint32_t lastSample;
static std::atomic<int> leftSample{-1}, rightSample{-1};
static std::atomic<bool> yawRequested{false}, yawValid{false};
static std::atomic<int> yawValue{0};
static std::atomic<uint32_t> yawTime{0};
static std::atomic<int> imuStatus{0}; // 0: idle, 1: starting, 2: ready; negative: error
static std::atomic<uint32_t> imuRequest{0};
static int reportedImuStatus = 99;
static std::atomic<uint32_t> eventSession{0}, dropped{0};
static uint32_t sessionCounter = 0;
struct Transition { uint32_t time, session; State from, to; };
static QueueHandle_t transitions;

static const char* stateName(State state) {
    switch (state) {
#define STATE_NAME(name) case name: return #name;
        STATE_NAME(IDLE) STATE_NAME(FORWARD) STATE_NAME(BACKWARD)
        STATE_NAME(TURN_RIGHT) STATE_NAME(TURN_LEFT_45) STATE_NAME(TURN_RIGHT_45)
        STATE_NAME(TURN_RIGHT_90) STATE_NAME(TURN_LEFT_90) STATE_NAME(TURN_LEFT_45_IF)
        STATE_NAME(TURN_RIGHT_45_IF) STATE_NAME(TURN_RIGHT_90_IF) STATE_NAME(TURN_LEFT_90_IF)
        STATE_NAME(FORWARD_LEFT) STATE_NAME(FORWARD_RIGHT) STATE_NAME(MOVEMENT_45)
        STATE_NAME(L_MOVEMENT_45) STATE_NAME(R_MOVEMENT_45) STATE_NAME(TURN_180)
        STATE_NAME(BRAKE) STATE_NAME(SHORT_LEFT_MOVE) STATE_NAME(SHORT_RIGHT_MOVE)
        STATE_NAME(LINE_RETREAT) STATE_NAME(INITIAL_MOVEMENT) STATE_NAME(SNAKE)
        STATE_NAME(TURKISH) STATE_NAME(GIRO_U_L) STATE_NAME(GIRO_U_R)
        STATE_NAME(GIRO_U_L_LONG) STATE_NAME(GIRO_U_R_LONG)
#undef STATE_NAME
    }
    return "DESCONOCIDO";
}

void changeState(State next) {
    const State previous = currentState;
    currentState = next;
    const uint32_t session = eventSession.load();
    if (previous == next || !session || !transitions) return;
    Transition event{millis(), session, previous, next};
    if (xQueueSend(transitions, &event, 0) != pdTRUE) ++dropped;
}

void loggingLineSample(adc1_channel_t channel, int value) {
    if (channel == LINE_FRONT_LEFT) leftSample = value;
    if (channel == LINE_FRONT_RIGHT) rightSample = value;
}

static void imuLoggingTask(void*) {
    IMU imu;

    imu.begin();

    for (;;) {
        if (yawRequested) {
            if (imu.getData()) {
                yawValue = imu.currentAngle;
                yawTime = millis();
                yawValid = true;
            } else {
                yawValid = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void loggingInit() {
    transitions = xQueueCreate(64, sizeof(Transition));
    if (xTaskCreate(imuLoggingTask, "loggingIMU", 4096, nullptr, 1, nullptr) != pdPASS)
        imuStatus = -1;
}

static const char* onOff(bool value) { return value ? "ON" : "OFF"; }
static void refreshEventSession() {
    eventSession = 0;
    dropped = 0;
    if (transitions) xQueueReset(transitions);
    if (config.activo && config.estado && transitions) {
        if (++sessionCounter == 0) ++sessionCounter;
        eventSession = sessionCounter;
    }
}
static void showMenu() {
    String text = "=== REGISTRO DE DATOS ===\nEstado: ";
    text += config.activo ? "ACTIVO" : "DETENIDO";
    text += "\nIntervalo: " + String(config.intervaloMs) + " ms\n";
    text += "1. Sensores de línea [" + String(onOff(config.linea)) + "]\n";
    text += "2. Sensores IR [" + String(onOff(config.ir)) + "]\n";
    text += "3. Máquina de estados [" + String(onOff(config.estado)) + "]\n";
    text += "4. IMU [" + String(onOff(config.yaw)) + "]\n";
    text += "5. Iniciar registro\n6. Detener registro\n7. Cambiar intervalo\n0. Volver\n";
    sendData(text);
}
void loggingDisconnected() {
    config.activo = false;
    yawRequested = false;
    menu = intervalPrompt = false;
    refreshEventSession();
}
void loggingCommand(const String& command) {
    if (command == "menu" || command == "registro") {
        menu = true;
        intervalPrompt = false;
        showMenu();
        return;
    }
    if (command == "ayuda") {
        sendData("conectado\nPulse cualquier tecla para REGISTRO DE DATOS. Parametros: INDICE VALOR.\n");
        return;
    }
    if (intervalPrompt) {
        if (command == "0") { intervalPrompt = false; showMenu(); return; }
        bool digits = command.length() > 0 && command.length() <= 5;
        for (size_t i = 0; i < command.length(); ++i)
            digits = digits && command[i] >= '0' && command[i] <= '9';
        const long value = digits ? command.toInt() : 0;
        if (value < 20 || value > 60000) {
            sendData("Intervalo invalido: 20-60000 ms; 0 cancela.\n");
            return;
        }
        config.intervaloMs = value;
        lastSample = millis();
        intervalPrompt = false;
        showMenu();
        return;
    }
    if (!menu) { menu = true; showMenu(); return; }
    if (command == "1") config.linea = !config.linea;
    else if (command == "2") config.ir = !config.ir;
    else if (command == "3") {
        config.estado = !config.estado;
        refreshEventSession();
        if (!transitions) sendData("ERROR: cola de estados no disponible.\n");
    }
    else if (command == "4") {
        config.yaw = !config.yaw;
        if (config.yaw)
            sendData("IMU: mantener inmovil durante la inicializacion/calibracion al iniciar.\n");
    }
    else if (command == "5") {
        config.activo = true;
        lastSample = millis();
        refreshEventSession();
        sendData("=== REGISTRO INICIADO ===\nLinea : " + String(onOff(config.linea)) +
                 "\nIR    : " + onOff(config.ir) + "\nEstado: " + onOff(config.estado) +
                 "\nYaw   : " + onOff(config.yaw) + "\nIntervalo: " +
                 String(config.intervaloMs) + " ms\n=========================\n");
    }
    else if (command == "6") { config.activo = false; refreshEventSession(); }
    else if (command == "7") {
        intervalPrompt = true;
        sendData("Intervalo en ms (20-60000); 0 cancela:\n");
        return;
    }
    else if (command == "0") {
        menu = false;
        sendData("Menu principal. Pulse cualquier tecla para volver.\n");
        return;
    }
    else { showMenu(); return; }
    const bool requestYaw = config.activo && config.yaw;
    if (requestYaw && (!yawRequested.load() || command == "5")) {
        ++imuRequest;
        reportedImuStatus = 99;
    }
    yawRequested = requestYaw;
    showMenu();
}

void loggingPoll() {
    if (!config.activo) return;
    Transition event;
    for (int i = 0; transitions && i < 8 && xQueueReceive(transitions, &event, 0) == pdTRUE; ++i)
        if (event.session == eventSession.load())
            sendData("ESTADO," + String(event.time) + "," + stateName(event.from) + "->" + stateName(event.to) + "\n");
    const uint32_t lost = dropped.exchange(0);
    if (lost) sendData("PERDIDOS,ESTADO," + String(lost) + "\n");
    const uint32_t now = millis();
    if (uint32_t(now - lastSample) < config.intervaloMs) return;
    lastSample = now;
    if (config.linea)
        sendData("LINEA," + String(now) + "," + String(leftSample.load()) + "," + String(rightSample.load()) + "\n");
    if (config.ir) {
        String line = "IR," + String(now);
#ifdef MBARETECH_1
        const int first = SHORT_LEFT, last = SHORT_RIGHT;
#else
        const int first = SIDE_LEFT, last = SIDE_RIGHT;
#endif
        for (int i = first; i <= last; ++i) line += "," + String(irSensor[i] ? 1 : 0);
        sendData(line + "\n");
    }
    if (config.yaw) {
        const int status = imuStatus.load();
        const bool fresh = status == 2 && yawValid && uint32_t(millis() - yawTime.load()) <= 500;
        const int diagnostic = status == 2 && !fresh ? 3 : status;
        if (diagnostic != reportedImuStatus) {
            reportedImuStatus = diagnostic;
            switch (diagnostic) {
                case 0: sendData("IMU,ESPERANDO_INICIO\n"); break;
                case 1: sendData("IMU,CALIBRANDO: mantener inmovil\n"); break;
                case 2: sendData("IMU,LISTA\n"); break;
                case 3: sendData("IMU,SIN_PAQUETES_RECIENTES_DMP\n"); break;
                case -1: sendData("IMU,ERROR_TAREA\n"); break;
                case -2: sendData("IMU,ERROR_BUS_I2C: SDA=15 SCL=16\n"); break;
                case -3: sendData("IMU,NO_DETECTADA: MPU6050 direccion 0x68 SDA=15 SCL=16\n"); break;
                default: sendData("IMU,ERROR_DMP," + String(-status - 3) + "\n"); break;
            }
        }
        if (fresh)
            sendData("YAW," + String(now) + "," + String(yawValue.load()) + "\n");
        else
            sendData("YAW," + String(now) + ",NA\n");
    }
}
#endif // RUN_GYRO_TEST
