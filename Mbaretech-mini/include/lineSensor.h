//#include "globals.h"
#include <driver/adc.h>


void lineSensorsInit();
int readLineSensorFront(adc1_channel_t channel);
int readLineSensor(adc2_channel_t channel);
bool checkLineSensor(bool lineSensor,int measurement);