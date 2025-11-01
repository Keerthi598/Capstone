
#include "p_scanner.h"
#include <iostream>
#include <string>
#include <cmath>
#include <freertos/queue.h>
#include <driver/gpio.h>

using std::cout;
using std::endl;

#define GPIO_TRIGGER GPIO_NUM_39    // The Trigger Pin used for this specific microcontroller
QueueHandle_t P_Scanner::gpio_evt_queue = nullptr;
P_Scanner* P_Scanner::curr_scanner = nullptr;       // Used because Trigger events are static and need to hold teh pointer to the object

/*
    Constructor
*/
P_Scanner::P_Scanner() {
    // cout << "P_Scanner Created ^.^" << endl;
    curr_scanner = this;    // Initialize to this specific scanner object. Since only 1 instance is ever created, this is fine
}

/*
    Initialize all requirements
*/
void P_Scanner::init() {
    // Init BLE
    ble.startBLE();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Initialize the I2C Requirements
    // Init Screen
    vTaskDelay(pdMS_TO_TICKS(100));
    lcd.init(0x27, 19, 20, 20, 4);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Initializing...");

    // Init NAU
    nau.begin();
    //Flush Out initial readings and re calibrate
    for (int i = 0; i < 10; i++) {
        while (!nau.available())
            vTaskDelay(pdMS_TO_TICKS(5));
        nau.getReading();
    }
    cout << "Flush Out Readings" << endl;
    nau.setLDO(NAU7802_LDO_EXTERNAL);   // Set the Low-Dropout Regulator voltage to 3.3V
    nau.setGain(NAU7802_GAIN_1);        // Set the sensor gain to 128
    nau.setSampleRate(NAU7802_SPS_320); // Increase the sample rate to the maximum
    nau.calibrateAFE();                 // Recalibrate the analog front end when settings are changed

    // Init TLC
    tlc.init();
    vTaskDelay(pdMS_TO_TICKS(10));
}

/*
    Once a scan has been triggered
*/
void P_Scanner::trigger_seq() {
    lcd.clear();
    lcd.print("Scanning...");

    // Flush out some readings
    for (int i = 0; i < 10; i++) {
        while (!nau.available())
            vTaskDelay(pdMS_TO_TICKS(5));
        nau.getReading();
    }

    for (int i = 0; i < 7; i++) {
        tlc.toggleLED(i, true);         // Turn led on
        vTaskDelay(pdMS_TO_TICKS(500)); // Wait for .5s
        curr_val[i] = nau.getReading();    // Get reading
        tlc.toggleLED(i, false);        // Turn led off
        vTaskDelay(pdMS_TO_TICKS(500)); // Wait for 0.5s
    }
    
    vTaskDelay(pdMS_TO_TICKS(3));

    for (int i = 0; i < 7; i++)
        cout << curr_val[i] << endl;


    // Normalize
    l2Norm();

    // Run Neural Network
    int val = tfl.run_inf(c_val, c_pred);

    // Display
    lcd.print_pred(val, c_pred[val]);

    vTaskDelay(pdMS_TO_TICKS(10));
    // BLE
    ble.updateVal(curr_val, c_pred);

    vTaskDelay(pdMS_TO_TICKS(3));
    // Return
    return;
}

/*
    Normalize the values so that the neural network can use them
*/
void P_Scanner::l2Norm() {
    norm_helper = 0;
    norm_helper = std::pow(curr_val[0], 2);
    norm_helper += std::pow(curr_val[1], 2);
    norm_helper += std::pow(curr_val[2], 2);
    norm_helper += std::pow(curr_val[3], 2);
    norm_helper += std::pow(curr_val[4], 2);
    norm_helper += std::pow(curr_val[5], 2);
    norm_helper += std::pow(curr_val[6], 2);

    float norm = std::sqrt(norm_helper);

    c_val[0] = (curr_val[0]) / norm;
    c_val[1] = (curr_val[1]) / norm;
    c_val[2] = (curr_val[2]) / norm;
    c_val[3] = (curr_val[3]) / norm;
    c_val[4] = (curr_val[4]) / norm;
    c_val[5] = (curr_val[5]) / norm;
    c_val[6] = (curr_val[6]) / norm;
}

/*
    Initialize the trigger event handler

    Logic for the Trigger
        The Event handler asynchronously check for a trigger
        If Triggered, Adds teh event to a static queue

    Logic for Queue Handler (gpio_task())
        Check queue all the time
        If something in there, remove and trigger sequence
*/
void P_Scanner::init_Buttons() {
    gpio_evt_queue = xQueueCreate(5, sizeof(uint32_t));
    xTaskCreate(gpio_task, "gpio_task", 4098, NULL, 10, NULL);

    gpio_config_t scanButton;
    scanButton.intr_type = GPIO_INTR_NEGEDGE;
    scanButton.mode = GPIO_MODE_INPUT;
    scanButton.pin_bit_mask = (1ULL << GPIO_TRIGGER);
    scanButton.pull_up_en = GPIO_PULLUP_ENABLE;
    scanButton.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&scanButton);

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(GPIO_TRIGGER, isrHandler, (void*) GPIO_TRIGGER);

    lcd.clear();
    lcd.print("Ready To Scan");
}

/*
    The event handler function
*/
void P_Scanner::isrHandler(void* args) {
    uint32_t gpio_num = (uint32_t) args;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/*
    The Queue Handler
*/
void P_Scanner::gpio_task(void *args) {
    uint32_t io_num;
    bool isScanning = false;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {    // Wait for something to show up in teh queue
            if (!curr_scanner)
                continue;
            if (isScanning) 
                continue;
            // Start Trigger Sequence
            isScanning = true;
            cout << "Start Trigger Sequence in Task" << endl;
            // Trigger sequence
            curr_scanner->trigger_seq();
            isScanning = false;
        }
    }
}