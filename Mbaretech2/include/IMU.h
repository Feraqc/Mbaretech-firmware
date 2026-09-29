#include <cmath>
#include <stdint.h>
#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "firmwareConfig.h"
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
      
#define SDA 15
#define SCL 16

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
    float currentAngle = 0;
    bool dmpReady = false;
    bool yawAvailable = false;
    int initError = 0;
    bool isReady() const { return dmpReady; }
    bool hasYaw() const { return yawAvailable; }
    int getInitError() const { return initError; }

    bool getData() {
#if !ENABLE_GYRO
      return false;
#else
      yawAvailable = false;
      if (!dmpReady || !mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
          return false;
      }

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      getYaw(&currentAngle, &q);
      yawAvailable = true;

      return true;
#endif
    }

    void begin(){
#if !ENABLE_GYRO
      dmpReady = yawAvailable = false;
      initError = -256; // Acquisition disabled at build time.
#else
      //Wire.setPins(SDA,SCL);
      dmpReady = yawAvailable = false;
      initError = 0;
      if (!Wire.begin(SDA,SCL)) { initError = -2; return; }
      Wire.setClock(400000);


      uint8_t devStatus;


      
      mpu.initialize();
      if (!mpu.testConnection()) { initError = -3; return; }
      
      devStatus = mpu.dmpInitialize();
      mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
      mpu.setXGyroOffset(48);
      mpu.setYGyroOffset(-59);
      mpu.setZGyroOffset(-13);
      mpu.setXAccelOffset(5940);
      mpu.setYAccelOffset(5786);
      mpu.setZAccelOffset(13490);
      
      if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
#if ENABLE_DEBUG
        mpu.PrintActiveOffsets();
#endif
        mpu.setDMPEnabled(true);
        dmpReady = true;
        mpu.resetFIFO();
      } 
      else {
          initError = -3 - devStatus;
#if ENABLE_DEBUG
          Serial.print(F("DMP Initialization failed (code "));
          Serial.print(devStatus);
          Serial.println(F(")"));
#endif
      }
#endif
    }

    void transmitData(){
#if ENABLE_SERIAL
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
#endif
    }

    bool checkRotation(int desiredAngle){
      static float initialAngle = currentAngle;
      if(abs(initialAngle-currentAngle) >= desiredAngle){
        initialAngle = currentAngle;
        return true;
      }
      return false;
    }

void getYaw(float *yawDeg, Quaternion *q)
{
    float yawRad = atan2(
        2.0f * q->x * q->y - 2.0f * q->w * q->z,
        q->w * q->w + q->x * q->x - q->y * q->y - q->z * q->z
    );

    *yawDeg = yawRad * 180.0f / M_PI;
}
    

};


#endif