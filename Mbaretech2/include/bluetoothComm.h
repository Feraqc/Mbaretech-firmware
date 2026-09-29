#ifndef BLUETOOTHCOMM_H
#define BLUETOOTHCOMM_H
#include "globals.h"
void communicationsInit(const char* deviceName = "ESP32S3_UART");
void sendData(const String& data);
void bluetoothCommand(const String& input);
#endif
