/*
HDC Universal Library - Supports both HDC1080 and HDC2080 sensors
Auto-detects device type and provides unified interface

This library combines and builds upon the excellent work of:

HDC1080 Support based on:
  - ClosedCube_HDC1080 library by AA for ClosedCube Limited
  - Copyright (c) 2016-2017 ClosedCube Limited
  - Licensed under MIT License
  - Original repository: https://github.com/closedcube/ClosedCube_HDC1080_Arduino

HDC2080 Support based on:
  - HDC2080-Arduino library by Brandon Fisher
  - Originally created August 1st, 2017
  - Released into the public domain
  - Original repository: https://github.com/lime-labs/HDC2080-Arduino

This unified library is released under MIT License.
Copyright (c) 2024 - Combined implementation

Many functions and algorithms are directly adapted from the original libraries
with modifications for unified interface and auto-detection capability.
*/

#include "HDC_Universal.h"

HDC_Universal::HDC_Universal() {
    _address = 0x40;
    _deviceType = HDC_UNKNOWN;
}

bool HDC_Universal::begin(uint8_t address) {
    _address = address;
    Wire.begin();
    
    // Validate address - both devices support 0x40, HDC2080 also supports 0x41
    if (address != 0x40 && address != 0x41) {
        return false;
    }
    
    // Detect which device is connected
    _deviceType = detectDevice();
    
    return (_deviceType != HDC_UNKNOWN);
}

HDC_DeviceType HDC_Universal::detectDevice() {
    // Try to read device ID registers
    Wire.beginTransmission(_address);
    if (Wire.endTransmission() != 0) {
        return HDC_UNKNOWN; // Device not responding
    }
    
    // Check for HDC1080 first - read manufacturer and device ID
    uint16_t manufacturerId = readData16(HDC1080_MANUFACTURER_ID);
    uint16_t deviceId = readData16(HDC1080_DEVICE_ID);
    
    if (manufacturerId == HDC1080_MANUFACTURER_ID_VAL && deviceId == HDC1080_DEVICE_ID_VAL) {
        return HDC_1080;
    }
    
    // Check for HDC2080 - different register layout
    uint16_t hdc2080_device_id = (readData8(HDC2080_DEVICE_ID_H) << 8) | readData8(HDC2080_DEVICE_ID_L);
    uint16_t hdc2080_mid = (readData8(HDC2080_MID_H) << 8) | readData8(HDC2080_MID_L);
    
    // HDC2080 has device ID 0xD0xx and manufacturer ID 0x5449
    if ((hdc2080_device_id & 0xFF00) == 0xD000 && hdc2080_mid == 0x5449) {
        return HDC_2080;
    }
    
    return HDC_UNKNOWN;
}

bool HDC_Universal::isConnected() {
    Wire.beginTransmission(_address);
    return (Wire.endTransmission() == 0);
}

HDC_DeviceType HDC_Universal::getDeviceType() {
    return _deviceType;
}

float HDC_Universal::readTemperature() {
    uint16_t rawTemp;
    
    if (_deviceType == HDC_1080) {
        rawTemp = readData16(HDC1080_TEMPERATURE);
        return (rawTemp / 65536.0) * 165.0 - 40.0;
    }
    else if (_deviceType == HDC_2080) {
        uint8_t tempLow = readData8(HDC2080_TEMP_LOW);
        uint8_t tempHigh = readData8(HDC2080_TEMP_HIGH);
        rawTemp = (tempHigh << 8) | tempLow;
        return ((rawTemp * 165.0) / 65536.0) - 40.0;
    }
    
    return NAN; // Unknown device
}

float HDC_Universal::readHumidity() {
    uint16_t rawHumid;
    
    if (_deviceType == HDC_1080) {
        rawHumid = readData16(HDC1080_HUMIDITY);
        return (rawHumid / 65536.0) * 100.0;
    }
    else if (_deviceType == HDC_2080) {
        uint8_t humidLow = readData8(HDC2080_HUMID_LOW);
        uint8_t humidHigh = readData8(HDC2080_HUMID_HIGH);
        rawHumid = (humidHigh << 8) | humidLow;
        return (rawHumid / 65536.0) * 100.0;
    }
    
    return NAN; // Unknown device
}

uint16_t HDC_Universal::readManufacturerId() {
    if (_deviceType == HDC_1080) {
        return readData16(HDC1080_MANUFACTURER_ID);
    }
    else if (_deviceType == HDC_2080) {
        return (readData8(HDC2080_MID_H) << 8) | readData8(HDC2080_MID_L);
    }
    
    return 0;
}

uint16_t HDC_Universal::readDeviceId() {
    if (_deviceType == HDC_1080) {
        return readData16(HDC1080_DEVICE_ID);
    }
    else if (_deviceType == HDC_2080) {
        return (readData8(HDC2080_DEVICE_ID_H) << 8) | readData8(HDC2080_DEVICE_ID_L);
    }
    
    return 0;
}

void HDC_Universal::setTemperatureResolution(HDC_Resolution resolution) {
    if (_deviceType == HDC_1080) {
        // HDC1080 resolution control through configuration register
        // Per datasheet: bit 10 controls temperature resolution (0=14-bit, 1=11-bit)
        uint16_t config = readData16(HDC1080_CONFIGURATION);
        config &= ~(1 << 10); // Clear temp resolution bit
        if (resolution == HDC_RES_11BIT) {
            config |= (1 << 10); // Set 11-bit resolution
        }
        // Note: HDC1080 only supports 14-bit and 11-bit for temperature
        writeData16(HDC1080_CONFIGURATION, config);
    }
    else if (_deviceType == HDC_2080) {
        // HDC2080 resolution control
        uint8_t config = readData8(HDC2080_MEASUREMENT_CONFIG);
        config &= 0x3F; // Clear temp resolution bits [7:6]
        
        switch (resolution) {
            case HDC_RES_9BIT:
                config |= 0x80; // 10
                break;
            case HDC_RES_11BIT:
                config |= 0x40; // 01
                break;
            case HDC_RES_14BIT:
            default:
                // 00 - already cleared
                break;
        }
        
        writeData8(HDC2080_MEASUREMENT_CONFIG, config);
    }
}

void HDC_Universal::setHumidityResolution(HDC_Resolution resolution) {
    if (_deviceType == HDC_1080) {
        // HDC1080 humidity resolution control
        // Per datasheet: bits [9:8] control humidity resolution
        // 00 = 14-bit, 01 = 11-bit, 10 = 8-bit
        uint16_t config = readData16(HDC1080_CONFIGURATION);
        config &= ~(3 << 8); // Clear humidity resolution bits [9:8]
        
        switch (resolution) {
            case HDC_RES_8BIT:
                config |= (2 << 8); // 10
                break;
            case HDC_RES_11BIT:
                config |= (1 << 8); // 01
                break;
            case HDC_RES_14BIT:
            default:
                // 00 - already cleared
                break;
        }
        
        writeData16(HDC1080_CONFIGURATION, config);
    }
    else if (_deviceType == HDC_2080) {
        // HDC2080 humidity resolution control
        uint8_t config = readData8(HDC2080_MEASUREMENT_CONFIG);
        config &= 0xCF; // Clear humidity resolution bits [5:4]
        
        switch (resolution) {
            case HDC_RES_9BIT:
                config |= 0x20; // 10
                break;
            case HDC_RES_11BIT:
                config |= 0x10; // 01
                break;
            case HDC_RES_14BIT:
            default:
                // 00 - already cleared
                break;
        }
        
        writeData8(HDC2080_MEASUREMENT_CONFIG, config);
    }
}

void HDC_Universal::enableHeater() {
    if (_deviceType == HDC_1080) {
        // Per datasheet: bit 13 enables heater (1=enabled, 0=disabled)
        uint16_t config = readData16(HDC1080_CONFIGURATION);
        config |= (1 << 13); // Set heater bit
        writeData16(HDC1080_CONFIGURATION, config);
    }
    else if (_deviceType == HDC_2080) {
        // Per datasheet: bit 3 of CONFIG register enables heater
        uint8_t config = readData8(HDC2080_CONFIG);
        config |= 0x08; // Set bit 3 to enable heater
        writeData8(HDC2080_CONFIG, config);
    }
}

void HDC_Universal::disableHeater() {
    if (_deviceType == HDC_1080) {
        uint16_t config = readData16(HDC1080_CONFIGURATION);
        config &= ~(1 << 13); // Clear heater bit
        writeData16(HDC1080_CONFIGURATION, config);
    }
    else if (_deviceType == HDC_2080) {
        uint8_t config = readData8(HDC2080_CONFIG);
        config &= 0xF7; // Clear bit 3 to disable heater
        writeData8(HDC2080_CONFIG, config);
    }
}

void HDC_Universal::heatUp(uint8_t seconds) {
    if (_deviceType == HDC_1080) {
        // HDC1080 specific heating routine from original library
        enableHeater();
        
        uint16_t config = readData16(HDC1080_CONFIGURATION);
        config |= (1 << 12); // Set acquisition mode bit
        writeData16(HDC1080_CONFIGURATION, config);
        
        uint8_t buf[4];
        for (int i = 1; i < (seconds * 66); i++) {
            Wire.beginTransmission(_address);
            Wire.write(0x00);
            Wire.endTransmission();
            delay(20);
            Wire.requestFrom(_address, (uint8_t)4);
            Wire.readBytes(buf, (size_t)4);
        }
        
        config &= ~(1 << 12); // Clear acquisition mode bit
        writeData16(HDC1080_CONFIGURATION, config);
        disableHeater();
    }
    else if (_deviceType == HDC_2080) {
        // Simple heating for HDC2080
        enableHeater();
        delay(seconds * 1000);
        disableHeater();
    }
}

// HDC2080 specific functions (no-op for HDC1080)
void HDC_Universal::triggerMeasurement() {
    if (_deviceType == HDC_2080) {
        uint8_t config = readData8(HDC2080_MEASUREMENT_CONFIG);
        config |= 0x01; // Set measurement trigger bit
        writeData8(HDC2080_MEASUREMENT_CONFIG, config);
    }
}

void HDC_Universal::reset() {
    if (_deviceType == HDC_2080) {
        uint8_t config = readData8(HDC2080_CONFIG);
        config |= 0x80; // Set soft reset bit
        writeData8(HDC2080_CONFIG, config);
        delay(50);
    }
}

void HDC_Universal::setTempOffsetAdjust(uint8_t offset) {
    if (_deviceType == HDC_2080) {
        writeData8(0x08, offset); // TEMP_OFFSET_ADJUST register
    }
}

void HDC_Universal::setHumidityOffsetAdjust(uint8_t offset) {
    if (_deviceType == HDC_2080) {
        writeData8(0x09, offset); // HUM_OFFSET_ADJUST register
    }
}

void HDC_Universal::setLowTempThreshold(float temp) {
    if (_deviceType == HDC_2080) {
        if (temp < -40.0f) temp = -40.0f;
        if (temp > 125.0f) temp = 125.0f;
        uint8_t threshold = (uint8_t)(256.0f * (temp + 40.0f) / 165.0f);
        writeData8(0x0A, threshold); // TEMP_THR_L register
    }
}

void HDC_Universal::setHighTempThreshold(float temp) {
    if (_deviceType == HDC_2080) {
        if (temp < -40.0f) temp = -40.0f;
        if (temp > 125.0f) temp = 125.0f;
        uint8_t threshold = (uint8_t)(256.0f * (temp + 40.0f) / 165.0f);
        writeData8(0x0B, threshold); // TEMP_THR_H register
    }
}

void HDC_Universal::setLowHumidityThreshold(float humidity) {
    if (_deviceType == HDC_2080) {
        if (humidity < 0.0f) humidity = 0.0f;
        if (humidity > 100.0f) humidity = 100.0f;
        uint8_t threshold = (uint8_t)(256.0f * humidity / 100.0f);
        writeData8(0x0C, threshold); // HUMID_THR_L register
    }
}

void HDC_Universal::setHighHumidityThreshold(float humidity) {
    if (_deviceType == HDC_2080) {
        if (humidity < 0.0f) humidity = 0.0f;
        if (humidity > 100.0f) humidity = 100.0f;
        uint8_t threshold = (uint8_t)(256.0f * humidity / 100.0f);
        writeData8(0x0D, threshold); // HUMID_THR_H register
    }
}

void HDC_Universal::enableInterrupts() {
    if (_deviceType == HDC_2080) {
        uint8_t config = readData8(HDC2080_CONFIG);
        config |= 0x04; // Set interrupt enable bit
        writeData8(HDC2080_CONFIG, config);
    }
}

void HDC_Universal::disableInterrupts() {
    if (_deviceType == HDC_2080) {
        uint8_t config = readData8(HDC2080_CONFIG);
        config &= 0xFB; // Clear interrupt enable bit
        writeData8(HDC2080_CONFIG, config);
    }
}

uint8_t HDC_Universal::readInterruptStatus() {
    if (_deviceType == HDC_2080) {
        return readData8(0x04); // INTERRUPT_DRDY register
    }
    return 0;
}

// Low-level I2C operations
uint16_t HDC_Universal::readData16(uint8_t reg) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.endTransmission();
    
    // Per HDC1080 datasheet: typical conversion time is 6.35ms for temp, 6.5ms for humidity
    // Use 15ms to be safe for both measurements
    delay(15); // Wait for measurement
    Wire.requestFrom(_address, (uint8_t)2);
    
    byte msb = Wire.read();
    byte lsb = Wire.read();
    
    return (msb << 8) | lsb;
}

uint8_t HDC_Universal::readData8(uint8_t reg) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(_address, (uint8_t)1);
    
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

void HDC_Universal::writeData8(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void HDC_Universal::writeData16(uint8_t reg, uint16_t data) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write((uint8_t)(data >> 8));   // MSB first
    Wire.write((uint8_t)(data & 0xFF)); // LSB
    Wire.endTransmission();
    delay(10);
}