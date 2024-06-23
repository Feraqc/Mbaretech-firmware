#include "Arduino.h"

#define IR1 40
#define IR2 39
#define IR3 38
#define IR4 18
#define IR5 17
#define IR6 4
#define IR7 5

volatile bool sensorReadings[7];

// usar IRAM_ATTR para usar la ram interna
void IR1_ISR() {sensorReadings[0] = digitalRead(IR1);}  
void IR2_ISR() { sensorReadings[1] = digitalRead(IR2); }
void IR3_ISR() { sensorReadings[2] = digitalRead(IR3); }
void IR4_ISR() { sensorReadings[3] = digitalRead(IR4); }
void IR5_ISR() { sensorReadings[4] = digitalRead(IR5); }
void IR6_ISR() { sensorReadings[5] = digitalRead(IR6); }
void IR7_ISR() { sensorReadings[6] = digitalRead(IR7); }


void setup(){
    Serial.begin(115200);

    pinMode(IR1, INPUT);
    pinMode(IR2, INPUT);
    pinMode(IR3, INPUT);
    pinMode(IR4, INPUT);
    pinMode(IR5, INPUT);
    pinMode(IR6, INPUT);
    pinMode(IR7, INPUT);

    attachInterrupt(digitalPinToInterrupt(IR1), IR1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR2), IR2_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR3), IR3_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR4), IR4_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR5), IR5_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR6), IR6_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(IR7), IR7_ISR, CHANGE);

}

void loop() {
    for (int i = 0; i < 6; i++) {
        Serial.print(sensorReadings[i]);
        Serial.print("\t");
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);

}