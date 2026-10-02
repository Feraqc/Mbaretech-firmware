#if defined(RUN_SENSORS_TEST) || defined(RUN_LINE_SENSOR)
#include "globals.h"
#include <driver/adc.h>

void lineSensorsInit(){

    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);

    #if LINE_BACK_INSTALLED
    // No configurar mientras LINE_BACK_LEFT/RIGHT sigan apuntando a pines en
    // conflicto (GPIO19=DIPE, GPIO20=PIN_B1) -- ver nota en globals.h.
    adc2_config_channel_atten(LINE_BACK_LEFT, ADC_ATTEN_DB_12);
    adc2_config_channel_atten(LINE_BACK_RIGHT, ADC_ATTEN_DB_12);
    #endif
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

// Simetrico desde 2026-09-29 -- antes pedian 7 lecturas seguidas para
// confirmar blanco pero una sola lectura para volver a negro (contador que
// solo sumaba, se reseteaba de golpe con cualquier lectura por encima del
// umbral). A pedido del usuario se saco el debounce: una sola lectura
// alcanza para blanco o negro, en ambos sentidos por igual.
bool checkLineSensora(int measurement){
  return (measurement <= THRESHOLD);
}

bool checkLineSensorb(int measurement){
  return (measurement <= THRESHOLD);
}

bool checkLineSensorc(int measurement){
  return (measurement <= THRESHOLD);
}

bool checkLineSensord(int measurement){
  return (measurement <= THRESHOLD);
}
#endif