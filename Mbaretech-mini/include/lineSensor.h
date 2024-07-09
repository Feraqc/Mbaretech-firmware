//#include "globals.h"
#include <driver/adc.h>


void lineSensorsInit();
int readLineSensor(adc2_channel_t channel);
bool checkLineSensor(int measurement);