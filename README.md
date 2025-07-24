# HDC Universal Library

A unified Arduino library that automatically detects and supports both HDC1080 and HDC2080 temperature and humidity sensors with a single, consistent interface.

## Features

- **Auto-Detection**: Automatically detects whether HDC1080 or HDC2080 is connected
- **Unified API**: Single interface works seamlessly with both sensors
- **Full Feature Support**: Supports basic features on both sensors, plus advanced HDC2080 features when available
- **Drop-in Replacement**: Can replace existing HDC1080 or HDC2080 library usage

## Supported Sensors

- **HDC1080**: Texas Instruments temperature and humidity sensor
- **HDC2080**: Texas Instruments next-generation temperature and humidity sensor with advanced features

## Installation

1. Download or clone this library
2. Place it in your Arduino libraries folder
3. Restart Arduino IDE
4. Include with: `#include <HDC_Universal.h>`

## Basic Usage

```cpp
#include <HDC_Universal.h>

HDC_Universal sensor;

void setup() {
  Serial.begin(9600);
  
  if (!sensor.begin()) {
    Serial.println("Sensor not found!");
    while(1);
  }
  
  // Check which sensor was detected
  if (sensor.getDeviceType() == HDC_1080) {
    Serial.println("HDC1080 detected");
  } else if (sensor.getDeviceType() == HDC_2080) {
    Serial.println("HDC2080 detected");  
  }
}

void loop() {
  float temperature = sensor.readTemperature();
  float humidity = sensor.readHumidity();
  
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("°C, Humidity: ");
  Serial.print(humidity);
  Serial.println("%");
  
  delay(2000);
}
```

## API Reference

### Core Functions (Both Sensors)
- `bool begin(uint8_t address = 0x40)` - Initialize sensor with auto-detection
- `float readTemperature()` - Read temperature in Celsius
- `float readHumidity()` - Read relative humidity percentage
- `HDC_DeviceType getDeviceType()` - Get detected sensor type
- `bool isConnected()` - Check if sensor is responding

### Device Information
- `uint16_t readManufacturerId()` - Read manufacturer ID
- `uint16_t readDeviceId()` - Read device ID

### Configuration (Both Sensors)
- `setTemperatureResolution(HDC_Resolution)` - Set temperature measurement resolution
- `setHumidityResolution(HDC_Resolution)` - Set humidity measurement resolution
- `enableHeater()` / `disableHeater()` - Control internal heater
- `heatUp(uint8_t seconds)` - Heat sensor for specified duration

### Advanced Features (HDC2080 Only)
- `triggerMeasurement()` - Manually trigger measurement
- `reset()` - Software reset
- `setTempOffsetAdjust(uint8_t)` - Set temperature offset
- `setHumidityOffsetAdjust(uint8_t)` - Set humidity offset
- `setLowTempThreshold(float)` / `setHighTempThreshold(float)` - Set temperature thresholds
- `setLowHumidityThreshold(float)` / `setHighHumidityThreshold(float)` - Set humidity thresholds
- `enableInterrupts()` / `disableInterrupts()` - Control interrupt functionality
- `readInterruptStatus()` - Read interrupt status

## Resolution Options

- `HDC_RES_8BIT` - 8-bit resolution (HDC1080 humidity only)
- `HDC_RES_9BIT` - 9-bit resolution (HDC2080 only)
- `HDC_RES_11BIT` - 11-bit resolution
- `HDC_RES_14BIT` - 14-bit resolution (default, highest accuracy)

## Examples

The library includes several example sketches:

- **BasicReading**: Simple temperature and humidity reading
- **AdvancedFeatures**: Demonstrates HDC2080 advanced features
- **HeaterTest**: Shows heater functionality

## Credits and Acknowledgments

This library combines and builds upon the excellent work of:

### HDC1080 Support
- **ClosedCube_HDC1080** library by AA for ClosedCube Limited
- Copyright (c) 2016-2017 ClosedCube Limited
- Licensed under MIT License
- Original repository: https://github.com/closedcube/ClosedCube_HDC1080_Arduino

### HDC2080 Support  
- **HDC2080-Arduino** library by Brandon Fisher
- Originally created August 1st, 2017
- Released into the public domain
- Original repository: https://github.com/lime-labs/HDC2080-Arduino

## License

This unified library is released under the MIT License. See LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit issues, feature requests, or pull requests.