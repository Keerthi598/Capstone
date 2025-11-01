#ifndef P_SCANNER_H_
#define P_SCANNER_H_

/*
    The main Logic of this code
    Contains all the different components of this Scanner
        BLE (Bluetooth Low Energy)
        NAU (Analog to Digital Convertor)
        TLC (To control each led)
        TFL (TensorFlow Lite Micro)
        LCD (Screen)

    Also contains the trigger button
*/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdlib.h>
#include <cstdint>

#include "ble.h"
#include "tfl_c.h"
#include "src/Screen.h"
#include "src/NAU7802.h"
#include "src/TLC59208.h"

class BLE;

class P_Scanner {
public:
    // Value From the NAU
    int32_t curr_val[7] = {0, 0, 0, 0, 0, 0, 0};

    // Converted to be given to the TF
    float c_val[7];

    // Prediction from TF
    float c_pred[14];

    int32_t x[7] = {138578, 214514, 460214, 476942, 219187, 249415,  77141};
    float y[14] = {0.82, 0.917, 0.473, 0.8, 0.12, 0.13, 0.01, 0.01, 0.001, 0.8, 0.6, 0.2, 0.1, 0.1};


    BLE ble;        // BLE    
    NAU7802 nau;    // NAU    Analog-To-Digital Convertor
    TLC59208 tlc;   // TLC    Control LEDs
    TflC tfl;       // TFL    Run Neural Net
    LCD_I2C lcd;    // Screen 
    int64_t norm_helper = 0;

    static QueueHandle_t gpio_evt_queue;
    static P_Scanner* curr_scanner;

    void init_Buttons();
    static void IRAM_ATTR isrHandler(void *args);
    static void gpio_task(void *args);

    P_Scanner();
    
    // Initialize All Peripherals
    void init();

    // void loop();
    
    void trigger_seq();

    // Normalize the results
    void l2Norm();

};



#endif
