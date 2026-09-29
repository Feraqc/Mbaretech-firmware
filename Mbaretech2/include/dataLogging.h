#ifndef DATA_LOGGING_H
#define DATA_LOGGING_H
#include "globals.h"
struct LoggingConfig {
    bool linea = false, ir = false, estado = false, yaw = false, activo = false;
    uint32_t intervaloMs = 100;
};
void loggingInit();
void loggingStateChanged(State previous, State next);
void loggingCommand(const String& command);
void loggingPoll();
void loggingDisconnected();
void loggingLineSample(adc1_channel_t channel, int value);
#endif
