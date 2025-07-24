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

The MIT License (MIT)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef HDC_UNIVERSAL_H
#define HDC_UNIVERSAL_H

#include <Arduino.h>
#include <Wire.h>

// Device types
enum HDC_DeviceType {
    HDC_UNKNOWN = 0,
    HDC_1080 = 1,
    HDC_2080 = 2
};

// Resolution settings (compatible with both devices)
enum HDC_Resolution {
    HDC_RES_8BIT = 0,
    HDC_RES_9BIT = 1,
    HDC_RES_11BIT = 2,
    HDC_RES_14BIT = 3
};

class HDC_Universal {
public:
    HDC_Universal();
    
    // Initialization
    bool begin(uint8_t address = 0x40);
    bool isConnected();
    HDC_DeviceType getDeviceType();
    
    // Core temperature and humidity reading
    float readTemperature();
    float readHumidity();
    
    // Short aliases
    float readTemp() { return readTemperature(); }
    float readHumid() { return readHumidity(); }
    
    // Device information
    uint16_t readManufacturerId();
    uint16_t readDeviceId();
    
    // Resolution control (works on both devices)
    void setTemperatureResolution(HDC_Resolution resolution);
    void setHumidityResolution(HDC_Resolution resolution);
    
    // Heater control (works on both devices)
    void enableHeater();
    void disableHeater();
    void heatUp(uint8_t seconds); // HDC1080 style heating
    
    // HDC2080 advanced features (no-op on HDC1080)
    void triggerMeasurement();
    void reset();
    void setTempOffsetAdjust(uint8_t offset);
    void setHumidityOffsetAdjust(uint8_t offset);
    
    // HDC2080 threshold features (no-op on HDC1080)
    void setLowTempThreshold(float temp);
    void setHighTempThreshold(float temp);
    void setLowHumidityThreshold(float humidity);
    void setHighHumidityThreshold(float humidity);
    
    // HDC2080 interrupt features (no-op on HDC1080)
    void enableInterrupts();
    void disableInterrupts();
    uint8_t readInterruptStatus();

private:
    uint8_t _address;
    HDC_DeviceType _deviceType;
    
    // Device detection
    HDC_DeviceType detectDevice();
    
    // Low-level I2C operations
    uint16_t readData16(uint8_t reg);
    uint8_t readData8(uint8_t reg);
    void writeData8(uint8_t reg, uint8_t data);
    void writeData16(uint8_t reg, uint16_t data);
    
    // HDC1080 specific registers
    static const uint8_t HDC1080_TEMPERATURE = 0x00;
    static const uint8_t HDC1080_HUMIDITY = 0x01;
    static const uint8_t HDC1080_CONFIGURATION = 0x02;
    static const uint8_t HDC1080_MANUFACTURER_ID = 0xFE;
    static const uint8_t HDC1080_DEVICE_ID = 0xFF;
    
    // HDC2080 specific registers
    static const uint8_t HDC2080_TEMP_LOW = 0x00;
    static const uint8_t HDC2080_TEMP_HIGH = 0x01;
    static const uint8_t HDC2080_HUMID_LOW = 0x02;
    static const uint8_t HDC2080_HUMID_HIGH = 0x03;
    static const uint8_t HDC2080_CONFIG = 0x0E;
    static const uint8_t HDC2080_MEASUREMENT_CONFIG = 0x0F;
    static const uint8_t HDC2080_DEVICE_ID_L = 0xFE;
    static const uint8_t HDC2080_DEVICE_ID_H = 0xFF;
    static const uint8_t HDC2080_MID_L = 0xFC;
    static const uint8_t HDC2080_MID_H = 0xFD;
    
    // Device identification values
    static const uint16_t HDC1080_DEVICE_ID_VAL = 0x1050;
    static const uint16_t HDC1080_MANUFACTURER_ID_VAL = 0x5449;
};

#endif