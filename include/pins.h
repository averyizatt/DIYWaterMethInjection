#pragma once
#include <Arduino.h>

namespace pins {
// Classic Nano wiring.
// Wiring: MAP=A4, KNOCK=A5.
constexpr int MAP_SENSOR_ADC = A4;
constexpr int KNOCK_SENSOR_ADC = A5;

constexpr int FLOAT_SENSOR_DIGITAL = 3;
constexpr int PUMP_OUT = 2;
constexpr int WARNING_LED = 7;

// MCP2515 SPI CAN
constexpr int CAN_SPI_CS = 10;
constexpr int CAN_SPI_INT = 8;

// Bench-test button active LOW with internal pull-up.
constexpr int BENCH_TEST_BUTTON = 9;

// Thermistors disabled until wired.
constexpr int IAT_THERM_PIN = -1;
constexpr int DHT11_PIN = -1;
constexpr int ENGINE_BAY_THERM_PIN = -1;
constexpr int CABIN_THERM_PIN = -1;
constexpr int AMBIENT_THERM_PIN = -1;

// 0.5-4.5 V pressure transducers through 10k/20k dividers.
// Nano A6/A7 are analog-input only pins.
constexpr int OIL_PRESSURE_ADC = A6;
constexpr int FUEL_PRESSURE_ADC = A7;
constexpr int METH_PRESSURE_ADC = -1;
constexpr int BOOST_REF_PRESSURE_ADC = -1;
constexpr int SPARE_PRESSURE_1_ADC = -1;
constexpr int SPARE_PRESSURE_2_ADC = -1;
} // namespace pins
