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

// Pressure transducers disabled until dedicated hardware is wired.
constexpr int OIL_PRESSURE_ADC = -1;
constexpr int FUEL_PRESSURE_ADC = -1;
constexpr int METH_PRESSURE_ADC = -1;
constexpr int BOOST_REF_PRESSURE_ADC = -1;
constexpr int SPARE_PRESSURE_1_ADC = -1;
constexpr int SPARE_PRESSURE_2_ADC = -1;
} // namespace pins
