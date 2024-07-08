#ifdef RUN_LINE_SENSOR
#include "globals.h"
#include <driver/adc.h>
#include "lineSensor.h"

void lineSensorsInit(){

    // adc1_config_width(ADC_WIDTH);
    // adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_12);
    // adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);
    
    adc2_config_channel_atten(ADC2_CHANNEL_4, ADC_ATTEN_DB_12);
    adc2_config_channel_atten(ADC2_CHANNEL_6, ADC_ATTEN_DB_12);
}

int readLineSensorFront(adc1_channel_t channel){
  return adc1_get_raw(channel);
}

int readLineSensor(adc2_channel_t channel) {
    int adc_reading;
    if (adc2_get_raw(channel, ADC_WIDTH, &adc_reading) == ESP_OK) {
        return adc_reading;
    } else {
        return 500; 
    }
}

bool checkLineSensor(bool lineSensor,int measurement){
  static uint8_t counterLeft = 0;
  static uint8_t counterRight = 0;
  if(lineSensor){
    if(measurement <= THRESHOLD){counterLeft++;}
    else{counterLeft = 0;}
    return (counterLeft >=3);
  }
  else{
    if(measurement <= THRESHOLD){counterRight++;}
    else{counterRight = 0;}
    return (counterRight >=3);
  }
}

#endif