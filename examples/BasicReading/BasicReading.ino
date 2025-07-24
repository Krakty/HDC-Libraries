/*
HDC Universal Library - Basic Reading Example

This example demonstrates how to use the HDC Universal library
to read temperature and humidity from either HDC1080 or HDC2080 sensors.
The library automatically detects which device is connected.
*/

#include <HDC_Universal.h>

HDC_Universal sensor;

void setup() {
    Serial.begin(9600);
    while (!Serial) delay(10);
    
    Serial.println("HDC Universal Library - Basic Reading");
    Serial.println("=====================================");
    
    // Initialize sensor (will auto-detect HDC1080 or HDC2080)
    if (!sensor.begin()) {
        Serial.println("Failed to find HDC sensor!");
        while (1) delay(10);
    }
    
    // Print detected device information
    HDC_DeviceType deviceType = sensor.getDeviceType();
    switch (deviceType) {
        case HDC_1080:
            Serial.println("Detected: HDC1080");
            break;
        case HDC_2080:
            Serial.println("Detected: HDC2080");
            break;
        default:
            Serial.println("Unknown device");
            break;
    }
    
    Serial.print("Manufacturer ID: 0x");
    Serial.println(sensor.readManufacturerId(), HEX);
    Serial.print("Device ID: 0x");
    Serial.println(sensor.readDeviceId(), HEX);
    Serial.println();
}

void loop() {
    // Read temperature and humidity
    float temperature = sensor.readTemperature();
    float humidity = sensor.readHumidity();
    
    // Print results
    Serial.print("Temperature: ");
    Serial.print(temperature, 2);
    Serial.print("°C, Humidity: ");
    Serial.print(humidity, 2);
    Serial.println("%");
    
    delay(2000);
}