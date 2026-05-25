#pragma once
#include <Arduino.h>

namespace pins {
// Core controls (preserve established hardware defaults)
constexpr int MAP_SENSOR_ADC = A0;      // A0
constexpr int FLOAT_SENSOR_DIGITAL = D3; // D3
constexpr int PUMP_PWM = D2;            // D2
constexpr int WARNING_LED = 17;         // GPIO17
constexpr int KNOCK_SENSOR_ADC = A1;    // A1

// MCP2515 SPI CAN module control pins (when using MCP2515 profile)
constexpr int CAN_SPI_CS = D10;  // D10
constexpr int CAN_SPI_INT = D7;  // D7

// Thermistors (default disabled unless wired)
constexpr int IAT_THERM_PIN = D4;       // D4
constexpr int DHT11_PIN = D6;           // D6
constexpr int ENGINE_BAY_THERM_PIN = 8; // GPIO8
constexpr int CABIN_THERM_PIN = 9;      // GPIO9
constexpr int AMBIENT_THERM_PIN = 10;   // GPIO10

// Pressure transducers (divider-scaled to ADC)
constexpr int OIL_PRESSURE_ADC = A4;       // A4
constexpr int FUEL_PRESSURE_ADC = A5;      // A5
constexpr int METH_PRESSURE_ADC = 13;      // GPIO13
constexpr int BOOST_REF_PRESSURE_ADC = 14; // GPIO14
constexpr int SPARE_PRESSURE_1_ADC = 15;   // GPIO15
constexpr int SPARE_PRESSURE_2_ADC = 16;   // GPIO16
} // namespace pins
