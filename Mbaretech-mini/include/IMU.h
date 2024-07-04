#include <cmath>
#include <stdint.h>
#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

#define SCL_PIN 9
#define SDA_PIN 10
#define INT_PIN 11
class IMU{
  public:
    MPU6050 mpu;
    uint8_t fifoBuffer[64];
    Quaternion q;
    VectorFloat gravity;
    VectorInt16 aa;
    VectorInt16 aaReal;
    float euler[3];
    float ypr[3];
    char data[6][20];

    float currentAngle;


     void getData(){
      if(mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){
        mpu.dmpGetQuaternion(&q, fifoBuffer);
       // mpu.dmpGetEuler(euler, &q);
       // mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        currentAngle = ypr[0]*(180/M_PI);
   
      }
    }

    void begin(){
      Wire.setPins(SDA_PIN,SCL_PIN);
      Wire.begin();
      Wire.setClock(400000);
      bool dmpReady = false;
      uint8_t mpuIntStatus;
      uint8_t devStatus;
      uint16_t packetSize;
      uint16_t fifoCount;
      
      mpu.initialize();
      Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
      
      devStatus = mpu.dmpInitialize();
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
      mpu.setXGyroOffset(-96);
      mpu.setYGyroOffset(-107);
      mpu.setZGyroOffset(-12);
      mpu.setXAccelOffset(-2654);
      mpu.setYAccelOffset(-10);
      mpu.setZAccelOffset(31);
      
      if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
      } 
      else {
          Serial.print(F("DMP Initialization failed (code "));
          Serial.print(devStatus);
          Serial.println(F(")"));
      }
    }

    void transmitData(){
      dtostrf(ypr[0]*(180/M_PI),6,2,data[0]);
      dtostrf(ypr[1]*(180/M_PI),6,2,data[1]);
      dtostrf(ypr[2]*(180/M_PI),6,2,data[2]);
      // dtostrf(aaReal.x,8,0,data[3]);
      // dtostrf(aaReal.y,8,0,data[4]);
      // dtostrf(aaReal.z,8,0,data[5]);
      for(int i=0;i<3;i++){
        Serial.print(data[i]);
        Serial.print("\t");
      }
        Serial.print("\n");

     // Serial.println(currentAngle);
    }

    bool checkRotation(float desiredAngle){
      static float initialAngle = currentAngle;
      if(abs(initialAngle-currentAngle) >= desiredAngle){
        initialAngle = currentAngle;
        return true;
      }
      return false;
    }

};


#endif