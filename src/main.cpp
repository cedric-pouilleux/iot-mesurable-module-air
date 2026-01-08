#include <Arduino.h>
#include <IotMesurable.h>
#include "pins.h"
#include <Wire.h>
#include <SensirionI2CScd4x.h>
#include <Adafruit_SGP40.h>

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

// TPM200A-CO Protocol
const uint8_t TPM200A_HEADER = 0x2C;
const uint8_t TPM200A_BYTE4 = 0x03;
const uint8_t TPM200A_BYTE5 = 0xE8;
uint8_t tpm200aBuffer[6];
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
    
    // --- Read TPM200A-CO (UART parsing) ---
    // Parse UART data continuously (sensor sends data automatically)
    // but only publish when interval allows
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 5000) {
        if (tpm200aSerial.available()) {
            Serial.printf("[TPM200A DEBUG] UART bytes available: %d\n", tpm200aSerial.available());
        }
        lastDebug = millis();
    }
    
    while (tpm200aSerial.available()) {
        uint8_t byte = tpm200aSerial.read();
        
        // Debug: Print received byte
        Serial.printf("[TPM200A] RX byte: 0x%02X (buffer idx: %d)\n", byte, tpm200aBufferIndex);
        
        // Look for header
        if (tpm200aBufferIndex == 0 && byte != TPM200A_HEADER) {
            Serial.printf("[TPM200A] Waiting for header (got 0x%02X, expected 0x%02X)\n", byte, TPM200A_HEADER);
            continue;  // Skip until we find header
        }
        
        tpm200aBuffer[tpm200aBufferIndex++] = byte;
        
        // Once we have 6 bytes, parse the packet
        if (tpm200aBufferIndex >= 6) {
            Serial.printf("[TPM200A] Full packet: %02X %02X %02X %02X %02X %02X\n",
                         tpm200aBuffer[0], tpm200aBuffer[1], tpm200aBuffer[2],
                         tpm200aBuffer[3], tpm200aBuffer[4], tpm200aBuffer[5]);
            
            // Validate fixed bytes
            if (tpm200aBuffer[0] == TPM200A_HEADER && 
                tpm200aBuffer[3] == TPM200A_BYTE4 && 
                tpm200aBuffer[4] == TPM200A_BYTE5) {
                
                // Calculate checksum (sum of bytes 0-4)
                uint8_t checksum = 0;
                for (int i = 0; i < 5; i++) {
                    checksum += tpm200aBuffer[i];
                }
                
                // Verify checksum
                if (checksum == tpm200aBuffer[5]) {
                    // Extract CO level (PPM)
                    uint16_t ppm = (tpm200aBuffer[1] << 8) | tpm200aBuffer[2];
                    
                    // Validate range (0-1000 ppm)
                    if (ppm <= 1000) {
                        // Update stored value (always keep latest)
                        coLevel = (float)ppm;
                        
                        // Publish via brain (handles throttling automatically)
                        // Only publishes if interval has elapsed
                        brain.publish("tpm200a", "co", coLevel);
                        Serial.printf("TPM200A: CO=%.0f ppm\n", coLevel);
                    } else {
                        Serial.printf("[TPM200A ERROR] Invalid PPM: %d (> 1000)\n", ppm);
                    }
                } else {
                    Serial.printf("[TPM200A ERROR] Checksum mismatch (got 0x%02X, expected 0x%02X)\n", 
                                tpm200aBuffer[5], checksum);
                }
            } else {
                Serial.printf("[TPM200A ERROR] Invalid packet structure\n");
            }
            
            // Reset buffer
            tpm200aBufferIndex = 0;
        }
    }
}
