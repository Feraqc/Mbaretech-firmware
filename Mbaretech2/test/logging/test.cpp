#include "dataLogging.cpp"
#include "states.cpp"


volatile bool irSensor[7] = {true, false, true, false, true, false, true};
static std::string output;
void sendData(const String& text) { output += text.c_str(); }
static void clear() { output.clear(); }
static bool contains(const char* text) { return output.find(text) != std::string::npos; }

int main() {
    loggingInit();
    loggingCommand("menu");
    assert(contains("REGISTRO DE DATOS") && contains("DETENIDO") && contains("100 ms"));
    clear(); loggingCommand("");
    assert(contains("REGISTRO DE DATOS") && !config.linea && !config.activo);
    loggingCommand("0"); clear(); loggingCommand("1");
    assert(menu && contains("REGISTRO DE DATOS") && !config.linea);
    loggingCommand("1"); loggingCommand("2"); loggingCommand("3"); loggingCommand("4");
    loggingLineSample(LINE_FRONT_LEFT, 2180);
    loggingLineSample(LINE_FRONT_RIGHT, 2255);
    clear(); loggingPoll(); assert(output.empty());
    loggingCommand("5"); assert(contains("REGISTRO INICIADO"));
    clear();
    changeState(FORWARD); changeState(BRAKE); changeState(BRAKE);
    loggingPoll();
    assert(contains("ESTADO,0,IDLE->FORWARD\n"));
    assert(contains("ESTADO,0,FORWARD->BRAKE\n"));
    clear(); fakeTime = 99; loggingPoll(); assert(output.empty());
    fakeTime = 100; loggingPoll();
    assert(contains("LINEA,100,2180,2255\n"));
    assert(contains("IR,100,1,0,1,0,1,0,1\n"));
    assert(contains("YAW,100,NA\n"));
    assert(!contains("ESTADO,"));
    imuStatus = 2; yawValid = true; yawValue = 87; yawTime = 100;
    clear(); fakeTime = 200; loggingPoll(); assert(contains("YAW,200,87\n"));
    clear(); fakeTime = 800; loggingPoll(); assert(contains("YAW,800,NA\n"));

    loggingCommand("7"); loggingCommand("-1"); assert(config.intervaloMs == 100);
    loggingCommand("100x"); assert(config.intervaloMs == 100);
    loggingCommand("999999999999"); assert(config.intervaloMs == 100);
    loggingCommand("20"); assert(config.intervaloMs == 20);
    loggingCommand("1"); clear(); fakeTime += 20; loggingPoll();
    assert(!contains("LINEA,") && contains("IR,"));
    loggingCommand("6"); changeState(IDLE); clear(); fakeTime += 100; loggingPoll();
    assert(output.empty());
    loggingCommand("5"); clear(); loggingPoll(); assert(!contains("ESTADO,"));
    loggingCommand("3"); changeState(FORWARD); clear(); loggingPoll(); assert(output.empty());
    loggingCommand("3");
    for (int i = 0; i < 70; ++i) changeState(i % 2 ? FORWARD : BRAKE);
    clear(); loggingPoll(); assert(contains("PERDIDOS,ESTADO,6\n"));
    loggingCommand("0"); assert(!menu && config.activo);
    loggingDisconnected(); clear(); fakeTime += 100; loggingPoll(); assert(output.empty());

    loggingCommand("menu"); loggingCommand("5");
    lastSample = UINT32_MAX - 9;
    fakeTime = 10;
    clear(); loggingPoll(); assert(contains("IR,10,"));
    const uint32_t previousRequest = imuRequest.load();
    loggingCommand("5");
    assert(imuRequest.load() == previousRequest + 1);
    imuStatus = -3;
    clear(); fakeTime += 20; loggingPoll();
    assert(contains("IMU,NO_DETECTADA") && contains(",NA\n"));
    clear(); fakeTime += 20; loggingPoll();
    assert(!contains("IMU,NO_DETECTADA"));
    imuStatus = -4;
    clear(); fakeTime += 20; loggingPoll();
    assert(contains("IMU,ERROR_DMP,1\n"));
    std::cout << "PASS: menu, channels, timing/wraparound, transitions, overflow, stop/restart, disconnect, yaw validity\n";
}


