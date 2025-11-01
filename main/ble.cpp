
#include "nvs_flash.h"
#include <iostream>
#include <vector>
#include "NimBLEDevice.h"

#include "ble.h"
#include "constants.h"

using std::cout;
using std::endl;
using std::vector;

/*
    Start BLE
*/
void BLE::startBLE() {
    NimBLEDevice::init("NimBLE");


    pServer = NimBLEDevice::createServer();
    pService = pServer->createService("703dd90c-8afc-4b2a-93a9-5f7f5944488c");
    pCharacteristic = pService->createCharacteristic(
        "1c53263d-0e1e-4d60-a533-0157cc8a1682",
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY      
    );

    pDictChar = pService->createCharacteristic(
        "32bfb8fd-3b67-4098-8c3b-93e4281f3f7c",
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY       
    );

    pService->start();
    vector<uint8_t> con_val(43);
    pCharacteristic->setValue(con_val);
    initDict();
    
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID("703dd90c-8afc-4b2a-93a9-5f7f5944488c"); 
    pAdvertising->start(); 
}

/*
    Update the Values of prediction and notify subscribers
*/
void BLE::updateVal(int32_t val[7], float pred[14]) {
    // 0 indicating prediction values characteristic
    // 1-28, 4 bytes each, for sensor readings
    // 29 - 42, each byte indicating confidence (0 - 100)
    vector<uint8_t> con_val(43);
    int32_t temp = 0;
    con_val[0] = 0;

    for (int i = 0; i < 7; i += 1) {
        temp = val[i];

        con_val[i * 4 + 1] = (temp >> 24) & 0xFF;
        con_val[i * 4 + 2] = (temp >> 16) & 0xFF;
        con_val[i * 4 + 3] = (temp >> 8) & 0xFF;
        con_val[i * 4 + 4] = temp & 0xFF;
    }

    for (int i = 0; i < 14; i++) {
        con_val[29 + i] = (pred[i] * 100);
    }
    
    pCharacteristic->setValue(con_val);
    vTaskDelay(pdMS_TO_TICKS(100));
    pCharacteristic->notify();
    vTaskDelay(pdMS_TO_TICKS(100));
    pDictChar->notify();
}

/*
    Initializer the dictionary
    Doing so to allow for more plastic types to be added and let the ble 
    be dynamic
*/
void BLE::initDict() {
    std::vector<uint8_t> output;

    // Index 0 is 1 to indicate dictionary
    output.push_back(1);
    // Iterate through each string in plastic_type_mapping
    for (const auto& plastic_type : plastic_type_mapping) {
        // Add each character's ASCII value to the output vector
        for (char c : plastic_type) {
            output.push_back(static_cast<uint8_t>(c));
        }
        // Append 0 to mark the end of the current string
        output.push_back(0);
    }

    pDictChar->setValue(output);
}
