#ifndef PINS_H
#define PINS_H

// ============================================================================
// Pin Definitions for ESP32-DevKitC V4 (ESP32-WROOM-32)
// ============================================================================

#if defined(BOARD_ESP32_DEVKIT_V4) || defined(ARDUINO_ESP32_DEV)

    // I2C Buses
    #define PIN_I2C_SDA_MAIN    21
    #define PIN_I2C_SCL_MAIN    22
    #define PIN_I2C_SDA_SGP     32
    #define PIN_I2C_SCL_SGP     33

    // UART - MH-Z14A (CO2)
    #define PIN_UART_RX_CO2     25
    #define PIN_UART_TX_CO2     26

    // UART - SPS30 (Particulate Matter)
    #define PIN_UART_RX_SPS30   13
    #define PIN_UART_TX_SPS30   27

    // UART - SC16-CO (Carbon Monoxide)
    // Note: SoftwareSerial might be used depending on implementation
    #define PIN_UART_RX_CO_SC16 14
    #define PIN_UART_TX_CO_SC16 12

    // GPIO - DHT22 (Temp/Humidity)
    #define PIN_DHT             4

#else
    #error "Board pinout not defined! Please select a valid environment."
#endif

#endif // PINS_H
