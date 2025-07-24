/*
HDC Universal Library - Heater Test Example

This example demonstrates the heater functionality available on both sensors.
The heater can be used to drive off condensation or for calibration purposes.
*/

#include <HDC_Universal.h>

HDC_Universal sensor;

void setup() {
    Serial.begin(9600);
    while (!Serial) delay(10);
    
    Serial.println("HDC Universal Library - Heater Test");
    Serial.println("===================================");
    
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
            Serial.println("HDC1080");
            break;
        case HDC_2080:
            Serial.println("HDC2080");
            break;
        default:
            Serial.println("Unknown device");
            break;
    }
    
    Serial.println("\nTesting heater functionality...");
    Serial.println("Reading before heating:");
    printReading();
    
    Serial.println("\nActivating heater for 5 seconds...");
    sensor.heatUp(5);  // Heat for 5 seconds
    
    Serial.println("Reading after heating:");
    printReading();
    
    Serial.println("\nWaiting 30 seconds for cool down...");
    for (int i = 30; i > 0; i--) {
        Serial.print(i);
        Serial.print(" ");
        delay(1000);
    }
    Serial.println();
    
    Serial.println("Reading after cool down:");
    printReading();
    
    Serial.println("\nHeater test complete!");
}

void loop() {
    // Normal operation - read every 10 seconds
    printReading();
    delay(10000);
}

void printReading() {
    float temperature = sensor.readTemperature();
    float humidity = sensor.readHumidity();
    
    Serial.print("  Temperature: ");
    Serial.print(temperature, 2);
    Serial.print("°C, Humidity: ");
    Serial.print(humidity, 2);
    Serial.println("%");
}