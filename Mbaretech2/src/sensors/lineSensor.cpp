#include "globals.h"
#include "dataLogging.h"
#include <driver/adc.h>

void lineSensorsInit(){
#if ENABLE_LINE_SENSORS

    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);
    
    adc2_config_channel_atten(ADC2_CHANNEL_8, ADC_ATTEN_DB_12);
    adc2_config_channel_atten(ADC2_CHANNEL_9, ADC_ATTEN_DB_12);
#endif
}

int readLineSensorFront(adc1_channel_t channel){
#if ENABLE_LINE_SENSORS
  const int value = adc1_get_raw(channel);
#if ENABLE_LOGGING
  loggingLineSample(channel, value);
#endif
  return value;
#else
  return -1;
#endif
}

int readLineSensorBack(adc2_channel_t channel) {
#if ENABLE_LINE_SENSORS
    int adc_reading;
    if (adc2_get_raw(channel, ADC_WIDTH, &adc_reading) == ESP_OK) {
        return adc_reading;
    } else {
        return -1; 
    }
#else
    return -1;
#endif
}

bool checkLineSensora(int measurement){
  static uint8_t countera = 0;
  if(measurement<=THRESHOLD){if(countera < 7) ++countera;}
  else{countera = 0;}
  return (countera >=7); // 20 es mucho con 169 //con  10 es un 50/50
}

bool checkLineSensorb(int measurement){
  static uint8_t counterb = 0;
  if(measurement<=THRESHOLD){if(counterb < 7) ++counterb;}
  else{counterb = 0;}
  return (counterb >=7); // 20 es mucho con 169 //con  10 es un 50/50
}
