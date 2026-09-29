#include "dataLogging.cpp"
#include "states.cpp"

volatile bool irSensor[7] = {};
static std::string output;
void sendData(const String& text) { output += text.c_str(); }
int main() {
    loggingInit();
    assert(taskCreateCount == 0); // No gyro task when ENABLE_GYRO is absent.
    loggingCommand("menu");
    loggingCommand("1"); loggingCommand("2"); loggingCommand("4");
    assert(!config.linea && !config.ir && !config.yaw);
    assert(output.find("ENABLE_GYRO=0") != std::string::npos);
    assert(output.find("ENABLE_LINE_SENSORS=0") != std::string::npos);
    assert(output.find("ENABLE_IR_SENSORS=0") != std::string::npos);
    loggingCommand("5");
    assert(!yawRequested && imuRequest == 0);
    output.clear(); fakeTime = 100; loggingPoll();
    assert(output.empty());
    std::cout << "PASS: disabled sensor channels rejected; no gyro task or request\n";
}
