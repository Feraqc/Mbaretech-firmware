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
 

BLEServer* pServer;
BLECharacteristic* pTxCharacteristic;
bool deviceConnected = false;


void sendData(const String& data) {
  if (deviceConnected) {
    pTxCharacteristic->setValue(data.c_str());
    pTxCharacteristic->notify();
  }
}

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

    if (rxValue.length() > 0) {

      // 🔹 Parse format "INDEX DATA"
      int spaceIndex = rxValue.indexOf(' ');
      if (spaceIndex > 0) {
        int index = rxValue.substring(0, spaceIndex).toInt();
        int value = rxValue.substring(spaceIndex + 1).toInt();

        if (index >= 0) {
          parametros[index] = value;  
        }
      }
    }
  }
};


void BLE_UART_Init(const char* deviceName = "ESP32S3_UART") {
  BLEDevice::init(deviceName);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  // TX (Notify)
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // RX (Write)
  BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();

}

#endif