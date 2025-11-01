#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <iostream>
#include <esp_timer.h>

#include "NAU7802.h"

#define SDA_PIN 19
#define SCL_PIN 20

#define NAU_ADDR 0x2a

bool NAU7802::is_connected() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (NAU_ADDR << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? true : false;
}

bool NAU7802::available() {
    return (getBit(NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL));
}

bool NAU7802::reset() {
    setBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);
    vTaskDelay(pdMS_TO_TICKS(1000));
    return (clearBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL)); //Clear RR to leave reset state
}

bool NAU7802::powerUp() {
    setBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
    setBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);

    uint8_t counter = 0;
    while (1) {
        if (getBit(NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL) == true)
            break;
        vTaskDelay(pdMS_TO_TICKS(10));
        if (counter++ > 100)
            return false;
    }
    return true;
}

bool NAU7802::powerDown() {
    clearBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
    return (clearBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL));  
}

bool NAU7802::begin()
{
    if (this->is_connected() == false) {
        if (this->is_connected() == false)
            return false;
    }

    bool result = true;

    if (1) {
        std::cout << "Initializing ..." << std::endl;
        result &= reset();                          // Reset all registers
        result &= powerUp();                        // Power on analog and digital sections of the scale
        result &= setLDO(NAU7802_LDO_2V4);          // Set LDO to 3.3V
        result &= setGain(NAU7802_GAIN_128);        // Set gain to 128
        result &= setSampleRate(NAU7802_SPS_80);    // Set samples per second to 10
        result &= setRegister(NAU7802_ADC, 0x30);   // Turn off CLK_CHP. From 9.1 power on sequencing.
        result &= setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR); // Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.
        result &= calibrateAFE();
    }

    return result;
}

int32_t NAU7802::getReading() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (NAU_ADDR << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, NAU7802_ADCO_B2, true));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return -1;
    }

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (NAU_ADDR << 1) | I2C_MASTER_READ, true));

    uint8_t data[3];
    ESP_ERROR_CHECK(i2c_master_read(cmd, data, 2, I2C_MASTER_ACK));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &data[2], I2C_MASTER_NACK));

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
        return 0;
    
    uint32_t valueRaw = (uint32_t)data[0] << 16;  // MSB
    valueRaw |= (uint32_t)data[1] << 8;           // MidSB
    valueRaw |= (uint32_t)data[2];                // LSB

    int32_t valueShifted = (int32_t)(valueRaw << 8);
    int32_t value = (valueShifted >> 8);

    return value;
}

int32_t NAU7802::getAverage(uint8_t samplesToTake) {
    long total = 0;
    uint8_t samplesAquired = 0;

    unsigned long startTime = esp_timer_get_time() / 1000;
    while (1) {
        if (available()) {
            total += getReading();
            if (++samplesAquired == samplesToTake)
                break;
        }
        if ((esp_timer_get_time() / 1000) - startTime > 1000)
            return 0;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    total /= samplesToTake;
    return static_cast<int32_t>(total);
}

bool NAU7802::setGain(uint8_t gainValue) {
    if (gainValue > 0b111)
        gainValue = 0b111; //Error check

    uint8_t value = getRegister(NAU7802_CTRL1);
    value &= 0b11111000; //Clear gain bits
    value |= gainValue;  //Mask in new bits

    return (setRegister(NAU7802_CTRL1, value));
}

bool NAU7802::setLDO(uint8_t ldoValue) {
    if (ldoValue > 0b111)
        ldoValue = 0b111; //Error check

    //Set the value of the LDO
    uint8_t value = getRegister(NAU7802_CTRL1);
    value &= 0b11000111;    //Clear LDO bits
    value |= ldoValue << 3; //Mask in new LDO bits    
    setRegister(NAU7802_CTRL1, value);

    if (ldoValue == NAU7802_LDO_EXTERNAL)
        return (clearBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL)); //Disable the internal LDO
    
    else
        return (setBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL)); //Enable the internal LDO
}

bool NAU7802::setSampleRate(uint8_t rate) {
    if (rate > 0b111)
        rate = 0b111; //Error check

    uint8_t value = getRegister(NAU7802_CTRL2);
    value &= 0b10001111; //Clear CRS bits
    value |= rate << 4;  //Mask in new CRS bits

    return (setRegister(NAU7802_CTRL2, value));
}

bool NAU7802::calibrateAFE() {
    beginCalibrateAFE();
    return waitForCalibrateAFE(1000);
}

void NAU7802::beginCalibrateAFE() {
    setBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2);
}

NAU7802_Cal_Status NAU7802::calAFEStatus() {
    if (getBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2)) {
        return NAU7802_CAL_IN_PROGRESS;
    }

    if (getBit(NAU7802_CTRL2_CAL_ERROR, NAU7802_CTRL2)) {
        return NAU7802_CAL_FAILURE;
    }

    // Calibration passed
    return NAU7802_CAL_SUCCESS;
}

bool NAU7802::waitForCalibrateAFE(uint32_t timeout_ms) {
    uint32_t begin = esp_timer_get_time() / 1000;
    NAU7802_Cal_Status cal_ready;

    while ((cal_ready = calAFEStatus()) == NAU7802_CAL_IN_PROGRESS) {
        if ((timeout_ms > 0) && (((esp_timer_get_time() / 1000) - begin) > timeout_ms))
            break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (cal_ready == NAU7802_CAL_SUCCESS) {
        return true;
    }
    return false;
}

bool NAU7802::setBit(uint8_t bitNumber, uint8_t regAddr) {
    uint8_t value = this->getRegister(regAddr);
    value |= (1 << bitNumber); // Set Bit
    return (this->setRegister(regAddr, value));
}

bool NAU7802::clearBit(uint8_t bitNumber, uint8_t regAddr) {
    uint8_t value = this->getRegister(regAddr);
    value &= ~(1 << bitNumber); // Set Bit
    return (this->setRegister(regAddr, value));
}

bool NAU7802::getBit(uint8_t bitNumber, uint8_t regAddr) {
    uint8_t value = this->getRegister(regAddr);
    value &= (1 << bitNumber); // Clear Bit
    return (value);
}

uint8_t NAU7802::getRegister(uint8_t regAddr) {
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (NAU_ADDR << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, true));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return -1;
    }

    uint8_t data;

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (NAU_ADDR << 1) | I2C_MASTER_READ, true));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? data : -1;
}

bool NAU7802::setRegister(uint8_t regAddr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (NAU_ADDR << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, true));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? true : false;
}
