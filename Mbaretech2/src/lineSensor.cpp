#if defined(RUN_SENSORS_TEST) || defined(RUN_LINE_SENSOR)
#include "globals.h"
#include <driver/adc.h>

void lineSensorsInit(){

    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);
    
    //adc2_config_channel_atten(ADC2_CHANNEL_8, ADC_ATTEN_DB_12);
    //adc2_config_channel_atten(ADC2_CHANNEL_9, ADC_ATTEN_DB_12);
}

int readLineSensorFront(adc1_channel_t channel){
  return adc1_get_raw(channel);
}

int readLineSensorBack(adc2_channel_t channel) {
    int adc_reading;
    if (adc2_get_raw(channel, ADC_WIDTH, &adc_reading) == ESP_OK) {
        return adc_reading;
    } else {
        return -1; 
    }
}

bool checkLineSensora(int measurement){
  static uint8_t countera = 0;
  if(measurement<=THRESHOLD){countera++;}
  else{countera = 0;}
  return (countera >=7); // 20 es mucho con 169 //con  10 es un 50/50
}

bool checkLineSensorb(int measurement){
  static uint8_t counterb = 0;
  if(measurement<=THRESHOLD){counterb++;}
  else{counterb = 0;}
  return (counterb >=7); // 20 es mucho con 169 //con  10 es un 50/50
}
#endif