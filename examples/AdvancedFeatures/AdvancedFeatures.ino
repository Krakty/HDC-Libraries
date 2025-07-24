/*
HDC Universal Library - Advanced Features Example

This example demonstrates advanced features available in the unified library.
Some features only work with HDC2080 sensors and are safely ignored on HDC1080.
*/

#include <HDC_Universal.h>

HDC_Universal sensor;

void setup() {
    Serial.begin(9600);
    while (!Serial) delay(10);
    
    Serial.println("HDC Universal Library - Advanced Features");
    Serial.println("========================================");
    
    // Initialize sensor
    if (!sensor.begin()) {
        Serial.println("Failed to find HDC sensor!");
        while (1) delay(10);
    }
    
    // Print detected device information
    HDC_DeviceType deviceType = sensor.getDeviceType();
    Serial.print("Detected device: ");
    switch (deviceType) {
        case HDC_1080:
            Serial.println("HDC1080 (basic features only)");
            break;
        case HDC_2080:
            Serial.println("HDC2080 (all features available)");
            break;
        default:
            Serial.println("Unknown device");
            break;
    }
    
    // Set resolution (works on both devices)
    Serial.println("Setting 14-bit resolution for both temperature and humidity");
    sensor.setTemperatureResolution(HDC_RES_14BIT);
    sensor.setHumidityResolution(HDC_RES_14BIT);
    
    // HDC2080 specific features (ignored on HDC1080)
    if (deviceType == HDC_2080) {
        Serial.println("Configuring HDC2080 advanced features:");
        
        // Set temperature thresholds
        sensor.setLowTempThreshold(20.0);
        sensor.setHighTempThreshold(30.0);
        Serial.println("- Temperature thresholds: 20°C - 30°C");
        
        // Set humidity thresholds  
        sensor.setLowHumidityThreshold(40.0);
        sensor.setHighHumidityThreshold(60.0);
        Serial.println("- Humidity thresholds: 40% - 60%");
        
        // Enable interrupts
        sensor.enableInterrupts();
        Serial.println("- Interrupts enabled");
        
        // Set offset adjustments (example values)
        sensor.setTempOffsetAdjust(0);
        sensor.setHumidityOffsetAdjust(0);
        Serial.println("- Offset adjustments set to 0");
    }
    
    Serial.println();
}

void loop() {
    // Trigger measurement (HDC2080 only, ignored on HDC1080)
    sensor.triggerMeasurement();
    delay(100); // Wait for measurement
    
    // Read temperature and humidity
    float temperature = sensor.readTemperature();
    float humidity = sensor.readHumidity();
    
    // Print results
    Serial.print("Temperature: ");
    Serial.print(temperature, 2);
    Serial.print("°C, Humidity: ");
    Serial.print(humidity, 2);
    Serial.println("%");
    
    // Check interrupt status (HDC2080 only)
    HDC_DeviceType deviceType = sensor.getDeviceType();
    if (deviceType == HDC_2080) {
        uint8_t interruptStatus = sensor.readInterruptStatus();
        if (interruptStatus != 0) {
            Serial.print("Interrupt status: 0x");
            Serial.println(interruptStatus, HEX);
        }
    }
    
    delay(2000);
}