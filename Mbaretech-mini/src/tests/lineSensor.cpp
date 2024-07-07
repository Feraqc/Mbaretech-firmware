#ifdef RUN_LINE_SENSOR
#include "globals.h"
#include <driver/adc.h>
#include "lineSensor.h"

void lineSensorsInit(){

    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);
    
    adc2_config_channel_atten(ADC2_CHANNEL_8, ADC_ATTEN_DB_12);
    adc2_config_channel_atten(ADC2_CHANNEL_9, ADC_ATTEN_DB_12);
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

bool checkLineSensor(int measurement,int threshold){
  static uint8_t counter = 0;
  if(measurement<=threshold){counter++;}
  else{counter = 0;}
  return (counter >=10); 
}
#endif