#include <Arduino.h>
#include "DroneBLE.h"
#include "DroneConfig.h"

using namespace std;

void DroneBLE::setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);

    Serial.println("Starting BLE server...");

    BLEDevice::init("Learnque Drone");
    bleServer = BLEDevice::createServer();
    bleService = bleServer->createService(SERVICE_UUID);
    bleCharacteristic = bleService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE
    );
    bleService->start();
    bleAdvertising = BLEDevice::getAdvertising();
    bleAdvertising->addServiceUUID(SERVICE_UUID);
    bleAdvertising->setScanResponse(true);
    bleAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
    bleAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("BLE server started.");
}

string DroneBLE::getValue() {
    string data = bleCharacteristic->getValue();
    if (data.length() > 0) {
        Serial.printf("BLE content recieved : %s\n", data.c_str());
        Serial.printf("Content length : %d\n", data.length());
        bleCharacteristic->setValue("");
    }
    return data;
}
