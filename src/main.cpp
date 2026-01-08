#include <Arduino.h>
#include <IotMesurable.h>
#include "pins.h"
#include <Wire.h>
#include <SensirionI2CScd4x.h>
#include <Adafruit_SGP40.h>
#include <cmath>

// Module ID
#define MODULE_ID "air-quality"

IotMesurable brain(MODULE_ID);
SensirionI2CScd4x scd4x;
Adafruit_SGP40 sgp40;
HardwareSerial tpm200aSerial(2);  // UART2 for TPM200A-CO

// Sensor Data State
float temperature = 25.0; // Default for compensation
float humidity = 50.0;    // Default for compensation
uint16_t co2 = 0;
int32_t vocIndex = 0;
float coLevel = 0.0;      // CO level in ppm

// TPM200A-CO Protocol (Proprietary 6-byte format, NOT standard Winsen ZE07-CO)
// Frame format: [0x2C] [PPM-High] [PPM-Low] [0x03] [0xE8] [Checksum]
// Bytes 3-4 (0x03E8 = 1000) represent the sensor range (0-1000 ppm)
// Checksum = sum of bytes 0-4 (simple additive, no complement)
const uint8_t TPM200A_HEADER = 0x2C;      // Start byte
const uint8_t TPM200A_FRAME_SIZE = 6;     // 6-byte frame
uint8_t tpm200aBuffer[8];                 // Buffer with margin
int tpm200aBufferIndex = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== IoT Mesurable - Air Quality Module ===\n");

    // Initialize I2C
    Wire.begin(PIN_I2C_SDA_MAIN, PIN_I2C_SCL_MAIN);

    // Initialize Connection (WiFi + MQTT)
    #ifdef MQTT_HUB_IP
        brain.setBroker(MQTT_HUB_IP, 1883);
    #endif

    if (!brain.begin()) {
        Serial.println("Failed to initialize connection!");
    } else {
        Serial.println("Connected and ready!");
    }
    
    // Set Module Type
    brain.setModuleType("air-quality");
    
    // --- Sensor Initialization ---
    
    // SCD41
    scd4x.begin(Wire);
    uint16_t error;
    char errorMessage[256];
    error = scd4x.stopPeriodicMeasurement(); // Stop potentially running measurement
    if (error) {
        Serial.print("Error stopping SCD4x: "); errorToString(error, errorMessage, 256); Serial.println(errorMessage);
    }
    
    error = scd4x.startPeriodicMeasurement();
    if (error) {
        Serial.print("Error starting SCD4x: "); errorToString(error, errorMessage, 256); Serial.println(errorMessage);
    } else {
        brain.registerHardware("scd41", "SCD41");
        brain.addSensor("scd41", "co2");
        brain.addSensor("scd41", "temperature");
        brain.addSensor("scd41", "humidity");
        Serial.println("SCD41 initialized.");
    }

    // SGP40
    if (!sgp40.begin()) {
        Serial.println("SGP40 not found!");
    } else {
        brain.registerHardware("sgp40", "SGP40");
        brain.addSensor("sgp40", "voc");
        Serial.println("SGP40 initialized.");
    }

    // TPM200A-CO
    tpm200aSerial.begin(9600, SERIAL_8N1, PIN_UART_RX_TPM200A, PIN_UART_TX_TPM200A);
    brain.registerHardware("tpm200a", "TPM200A-CO");
    brain.addSensor("tpm200a", "co");
    Serial.println("TPM200A-CO initialized (waiting for data).");

    brain.log("info", "Module booted with sensors.");
}

unsigned long lastRead = 0;
const unsigned long READ_INTERVAL = 1000; // Read sensors every 1s, throttling controls publish rate

void loop() {
    brain.loop();
    
    unsigned long now = millis();
    if (now - lastRead >= READ_INTERVAL) {
        lastRead = now;

        // --- Read SCD41 ---
        bool readyStatus = false;
        uint16_t error = scd4x.getDataReadyFlag(readyStatus);
        if (!error && readyStatus) {
            uint16_t scd_co2;
            float scd_temp, scd_hum;
            uint16_t error = scd4x.readMeasurement(scd_co2, scd_temp, scd_hum);
            if (!error) {
                if (scd_co2 != 0) {
                    co2 = scd_co2;
                    temperature = scd_temp;
                    humidity = scd_hum;
                    
                    brain.publish("scd41", "co2", (int)co2);
                    brain.publish("scd41", "temperature", temperature);
                    brain.publish("scd41", "humidity", humidity);
                    Serial.printf("SCD41: CO2=%d ppm, T=%.1f C, RH=%.1f %%\n", co2, temperature, humidity);
                }
            }
        }

        // --- Read SGP40 ---
        // Use SCD41 temperature and humidity for compensation
        vocIndex = sgp40.measureVocIndex(temperature, humidity);
        brain.publish("sgp40", "voc", (int)vocIndex);
        Serial.printf("SGP40: VOC Index=%d (T=%.1f, H=%.1f)\n", vocIndex, temperature, humidity);
    }
    
    // --- Read TPM200A-CO (UART parsing - 6-byte proprietary protocol) ---
    // Frame format: [0x2C] [PPM-High] [PPM-Low] [0x03] [0xE8] [Checksum]
    // Sensor auto-uploads data every ~1 second
    while (tpm200aSerial.available()) {
        uint8_t byte = tpm200aSerial.read();
        
        // Look for header byte (0x2C)
        if (tpm200aBufferIndex == 0 && byte != TPM200A_HEADER) {
            // Skip bytes until we find the header
            continue;
        }
        
        // If we see header mid-buffer, it's a new frame - reset and restart
        if (byte == TPM200A_HEADER && tpm200aBufferIndex > 0) {
            tpm200aBufferIndex = 0;
        }
        
        tpm200aBuffer[tpm200aBufferIndex++] = byte;
        
        // Once we have 6 bytes, parse the complete frame
        if (tpm200aBufferIndex >= TPM200A_FRAME_SIZE) {
            // Validate header
            if (tpm200aBuffer[0] == TPM200A_HEADER) {
                // Calculate checksum (sum of bytes 0-4)
                uint8_t calculatedChecksum = 0;
                for (int i = 0; i < 5; i++) {
                    calculatedChecksum += tpm200aBuffer[i];
                }
                
                // Verify checksum
                if (calculatedChecksum == tpm200aBuffer[5]) {
                    // Extract CO concentration (PPM) from bytes 1-2
                    uint16_t ppm = (tpm200aBuffer[1] << 8) | tpm200aBuffer[2];
                    
                    // Validate range (0-1000 ppm)
                    if (ppm <= 1000) {
                        coLevel = (float)ppm;
                        brain.publish("tpm200a", "co", coLevel);
                        Serial.printf("TPM200A: CO=%d ppm\n", ppm);
                    } else {
                        Serial.printf("[TPM200A ERROR] Invalid PPM: %d (> 1000)\n", ppm);
                    }
                } else {
                    Serial.printf("[TPM200A ERROR] Checksum mismatch (got 0x%02X, expected 0x%02X)\n", 
                                tpm200aBuffer[5], calculatedChecksum);
                }
            }
            
            // Reset buffer for next frame
            tpm200aBufferIndex = 0;
        }
    }
}
