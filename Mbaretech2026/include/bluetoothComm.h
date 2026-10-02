#ifndef BLUETOOTHCOMM_H
#define BLUETOOTHCOMM_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WString.h>
#include "globals.h" 


#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
 

extern BLEServer* pServer;
extern BLECharacteristic* pTxCharacteristic;
extern bool deviceConnected;

void sendData(const String& data);

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    sendData("conectado");
    
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string rxValueStd = pCharacteristic->getValue();
    String rxValue = String(rxValueStd.c_str());  // convert std::string → Arduino String

    #ifdef DEBUG
    Serial.print("BLE RX: \"");
    Serial.print(rxValue);
    Serial.println("\"");
    #endif

    if (rxValue.length() > 0) {

      // 🔹 Parse format "INDEX DATA"
      int spaceIndex = rxValue.indexOf(' ');
      if (spaceIndex > 0) {
        int index = rxValue.substring(0, spaceIndex).toInt();
        int value = rxValue.substring(spaceIndex + 1).toInt();

        if (index >= 0 && index < ARRAY_PARAMETROS_SIZE) {
          parametros[index] = value;
          sendData(String("OK parametros[") + index + "]=" + value);
        } else {
          sendData(String("ERROR indice fuera de rango (0-") + (ARRAY_PARAMETROS_SIZE - 1) + "): " + index);
        }
      } else {
        sendData("ERROR formato esperado \"INDICE VALOR\"");
      }
    }
  }
};


void BLE_UART_Init(const char* deviceName = "ESP32S3_UART");

#endif