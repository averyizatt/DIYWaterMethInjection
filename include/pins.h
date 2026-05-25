#pragma once
#include <Arduino.h>

<<<<<<< HEAD
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

// TWAI CAN transceiver pins
constexpr int CAN_TX = D5; // D5
constexpr int CAN_RX = D6; // D6 (shares D6 with DHT11_PIN if both features are enabled)

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
=======
// Arduino Nano ESP32 pin assignments
// Board silk label -> ESP32-S3 GPIO
//
// Pinout reference (top view, USB-C at top):
//
//  [USB-C]
//  D13/GPIO48 (SCK)  |  D12/GPIO47 (MISO)
//  D11/GPIO38 (MOSI) |  D10/GPIO21 (CAN_CS)  <- SPI to MCP2515
//  D9/GPIO18  (PUMP) |  D8/GPIO17  (WARN_LED)
//  D7/GPIO10         |  D7  -- spare
//  D6/GPIO9          |  D5/GPIO8   (FLOAT_SW)
//  D4/GPIO7  (CAN_INT)|  D3/GPIO6  (TEMP_AMB)
//  D2/GPIO5  (TEMP_ENG)| D1/GPIO43 (RX)
//  D0/GPIO44 (TX)    |  GND
//  RESET             |  3V3
//  A0/GPIO1  (MAP)   |  A1/GPIO2
//  A2/GPIO3          |  A3/GPIO4
//  A4/GPIO11 (SDA)   |  A5/GPIO12 (SCL)
//  3V3               |  GND
//  VIN               |  5V

#pragma once

// Arduino Nano ESP32 pin assignments
// Board silk label -> ESP32-S3 GPIO
//
// Pinout reference (top view, USB-C at top):
//
//  [USB-C]
//  D13/GPIO48 (SCK)    |  D12/GPIO47 (MISO)
//  D11/GPIO38 (MOSI)   |  D10/GPIO21 (CAN_CS)  <- SPI to MCP2515
//  D9/GPIO18  (spare)  |  D8/GPIO17  (WARN_LED)
//  D7/GPIO10  (spare)  |  D6/GPIO9   (spare)
//  D5/GPIO8   (spare)  |  D4/GPIO7   (CAN_INT)
//  D3/GPIO6   (FLOAT)  |  D2/GPIO5   (PUMP PWM)
//  D1/GPIO43  (TX)     |  D0/GPIO44  (RX)
//  RESET               |  3V3
//  A0/GPIO1   (MAP)    |  A1/GPIO2
//  A2/GPIO3            |  A3/GPIO4
//  A4/GPIO11  (SDA)    |  A5/GPIO12  (SCL)
//  3V3                 |  GND
//  VIN                 |  5V

namespace pins {

// MAP sensor — GM-style 0.5–4.5 V linear pressure output
// A0 on board -> GPIO1 (ADC1_CH0)
constexpr int MAP_SENSOR_ADC = 1;

// Pump MOSFET gate — PWM output to logic-level MOSFET (e.g. IRLB8721)
// D2 on board -> GPIO5 (LEDC-capable)
constexpr int PUMP_PWM = 5;

// Tank float switch — active-low, internal pull-up enabled
// D3 on board -> GPIO6
constexpr int FLOAT_SENSOR_DIGITAL = 6;

// DS18B20 1-Wire temperature sensors — set to real GPIO when wired up
// Set to -1 to disable (safe for bench testing)
constexpr int TEMP_ENGINE_BAY = -1;
constexpr int TEMP_AMBIENT    = -1;

// Warning LED or active-high relay driver
// D8 on board -> GPIO17
constexpr int WARNING_LED = 17;

// MCP2515 CAN bus module (SPI)
// Shared SPI bus: SCK=D13/GPIO48, MOSI=D11/GPIO38, MISO=D12/GPIO47
// D10 on board -> GPIO21  (SPI chip-select)
constexpr int CAN_CS = 21;
// D7  on board -> GPIO10  (MCP2515 /INT — active-low interrupt)
constexpr int CAN_INT = 10;

>>>>>>> 9f85b5a265fae4c4b6d3167df006c2b6cc9d5f89
} // namespace pins
