#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <iostream>
#include <esp_timer.h>

#include "TLC59208.h"

#define SDA_PIN 19
#define SCL_PIN 20

#define TLC_ADDR 0x20

void TLC59208::init() {
    vTaskDelay(pdMS_TO_TICKS(500));

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));

    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (TLC_ADDR << 1) | I2C_MASTER_WRITE, true));

    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x80, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x81, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0xAA, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0xAA, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, true));

    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));

    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        //std::cout << "This works ^.^ " << std::endl;
        return;
    }
    else    
        std::cout << "This don't work ;_; " << std::endl;
}

void TLC59208::toggleLED(int x, bool on) {
    vTaskDelay(pdMS_TO_TICKS(10));

    i2c_cmd_handle_t cmd_led = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd_led));

    ESP_ERROR_CHECK(i2c_master_write_byte(cmd_led, (TLC_ADDR << 1) | I2C_MASTER_WRITE, true));

    ESP_ERROR_CHECK(i2c_master_write_byte(cmd_led, 0x02 + x, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd_led, 0xff, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd_led, (on) ? 0xff : 0x00, true));


    ESP_ERROR_CHECK(i2c_master_stop(cmd_led));

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd_led, pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(10));

    i2c_cmd_link_delete(cmd_led);

    if (ret == ESP_OK)
        vTaskDelay(pdMS_TO_TICKS(10));
    else    
        std::cout << "LED don't work ;_; " << std::endl;

    // vTaskDelay(pdMS_TO_TICKS(10));
}
