/**
 * @file main.cpp
 * @brief Empty Bootstrap for Air Quality Module
 * 
 * Supports MSP32-DevKitC V4.
 * Connects to WiFi/MQTT and enables Over-The-Air (OTA) updates.
 * Ready for sensor integration.
 */

#include <Arduino.h>
#include <IotMesurable.h>
#include "pins.h"

// Module ID
// Change this to a unique name for your device
#define MODULE_ID "module-air-bootstrap"

IotMesurable brain(MODULE_ID);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== IoT Mesurable - Air Module Bootstrap ===\n");
    
    // Configure MQTT Broker from build flags if available
    #ifdef MQTT_HUB_IP
        brain.setBroker(MQTT_HUB_IP, 1883);
    #endif

    // Initialize Connect
    // This handles WiFiManager (Portal) AND MQTT connection
    if (!brain.begin()) {
        Serial.println("Failed to initialize connection!");
        // Blink fast to indicate error
    } else {
        Serial.println("Connected and ready!");
    }
    
    // Set Module Type for dashboard identification
    brain.setModuleType("air-quality");
    
    // Example: Registering a status LED as a basic "sensor" or just logging
    brain.log("info", "Module booted successfully");
}

void loop() {
    // REQUIRED: Handle network traffic and system messages
    brain.loop();
    
    // ... Add your sensor logic here later ...
}
