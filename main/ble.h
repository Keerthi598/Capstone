#ifndef BLE_H_
#define BLE_H_

/*
    The BLE File
    Uses a C++ implementation of NimBLE
*/

#ifdef __cplusplus
extern "C" {
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdlib.h>
#include <cstdint>

// Some forward declarations to reduce potential errors
class NimBLEServer;
class NimBLEService;
class NimBLECharacteristic;


class BLE {
public:
    NimBLEServer* pServer = nullptr;
    NimBLEService* pService = nullptr;

    NimBLECharacteristic* pCharacteristic = nullptr;    // The characteristic containing the scan values and prediction values
    NimBLECharacteristic* pDictChar = nullptr;          // The characteristic containing the plastic types used on this scanner

    void startBLE();

    void updateVal(int32_t val[7], float pred[14]);
    void initDict();

    BLE() {}
};



#ifdef __cplusplus
}
#endif

#endif