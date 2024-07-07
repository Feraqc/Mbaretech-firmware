//#include "globals.h"
#include <driver/adc.h>


void lineSensorsInit();
int readLineSensorFront(adc1_channel_t channel);
int readLineSensorBack(adc2_channel_t channel);
bool checkLineSensor(int measurement,int threshold);