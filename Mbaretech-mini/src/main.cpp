#include "globals.h"  // Include the header file

// Define the shared variables
volatile bool irSensor[3];
volatile bool startSignal = false; // Creo que debe ser volatile si le trato con interrupt
bool dipSwitch[4];  // de A a D
bool lineSensor[2];

Motor leftMotor(PWM_B,FORWARD_MAX_LEFT,FORWARD_MIN_LEFT,BACKWARD_MAX_LEFT,BACKWARD_MIN_LEFT);
Motor rightMotor(PWM_A,FORWARD_MAX_RIGHT,FORWARD_MIN_RIGHT,BACKWARD_MAX_RIGHT,BACKWARD_MIN_RIGHT);

//TASK HANDLERS
TaskHandle_t motorTaskHandle;
TaskHandle_t lineSensorTaskHandle;

//TASKS
#ifdef RUN_TASK_TEST
void motorTask(void *param);
#endif
void lineSensorTask(void *param);

void IRAM_ATTR KS_ISR(){startSignal = digitalRead(START_PIN);};

void setup() {
    esp_efuse_write_field_cnt(ESP_EFUSE_VDD_SPI_FORCE, 1); 
    // Initialize Serial communication
    Serial.begin(115200);
    rightMotor.begin();
    leftMotor.begin();
    lineSensorsInit();

    //imu.begin();
    // IR sensors
    pinMode(IR1, INPUT);
    pinMode(IR2, INPUT);
    pinMode(IR3, INPUT);

    pinMode(START_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(START_PIN), KS_ISR, CHANGE);

    // Start pins
    pinMode(START_PIN, INPUT);
    pinMode(DIPA, INPUT);
    pinMode(DIPB, INPUT);
    pinMode(DIPC, INPUT);
    pinMode(DIPD, INPUT);

    #ifdef RUN_TASK_TEST
    xTaskCreate(motorTask, "motorTask", 4096, NULL, 1, &motorTaskHandle);
    #endif
    //xTaskCreate(lineSensorTask, "lineSensorTask", 4096, NULL, 1, &lineSensorTaskHandle);

}


void lineSensorsInit(){

    // adc1_config_width(ADC_WIDTH);
    // adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_12);
    // adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12);
    
    adc2_config_channel_atten(ADC2_CHANNEL_8, ADC_ATTEN_DB_12);
    adc2_config_channel_atten(ADC2_CHANNEL_9, ADC_ATTEN_DB_12);
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

bool elapsedTime(TickType_t duration) {
    static TickType_t startTime = 0;
    static bool firstCall = true;
    TickType_t currentTime = xTaskGetTickCount();

    if (firstCall) {
        startTime = currentTime;
        firstCall = false;
    }

    if ((currentTime - startTime) >= duration) {
        startTime = currentTime;
        firstCall = true;
        return true;
    }
    else {
        return false;
    }
}

bool readIrSensor(int irSensor){
    static int samplesIzq[NUM_SAMPLES] = {0};
    static int samplesMid[NUM_SAMPLES] = {0};
    static int samplesDer[NUM_SAMPLES] = {0};
    int sum = 0;
    float avg = 0;

    if(irSensor == IR1){
        for(int i=0;i<NUM_SAMPLES-1;i++){
        samplesIzq[i] =samplesIzq[i+1];
        sum += samplesIzq[i];
        }
        samplesIzq[NUM_SAMPLES-1] = !digitalRead(irSensor);
        sum += samplesIzq[NUM_SAMPLES-1];
        avg = sum/NUM_SAMPLES;
        return (avg > 0.5);
    }
    else if(irSensor == IR3){
        for(int i=0;i<NUM_SAMPLES-1;i++){
        samplesDer[i] =samplesDer[i+1];
        sum += samplesDer[i];
        }
        samplesDer[NUM_SAMPLES-1] = !digitalRead(irSensor);
        sum += samplesDer[NUM_SAMPLES-1];
        avg = sum/NUM_SAMPLES;
        return (avg > 0.5);
    }
    else if(irSensor == IR2){
        for(int i=0;i<NUM_SAMPLES-1;i++){
        samplesMid[i] =samplesMid[i+1];
        sum += samplesMid[i];
        }
        samplesMid[NUM_SAMPLES-1] = !digitalRead(irSensor);
        sum += samplesMid[NUM_SAMPLES-1];
        avg = sum/NUM_SAMPLES;
        return (avg > 0.5);
    }
    return 0;
}







