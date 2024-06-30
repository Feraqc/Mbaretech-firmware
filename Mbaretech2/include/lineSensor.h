#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H
#include <driver/adc.h>
#include "globals.h"

#define ADC_WIDTH ADC_WIDTH_BIT_12

void lineSensorsInit();
int readLineSensorFront(adc1_channel_t channel);
int readLineSensorBack(adc2_channel_t channel);












#endif