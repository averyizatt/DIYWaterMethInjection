#pragma once

namespace pins {
// Defaults are intentionally disabled to force board-specific assignment.
// Set these to validated GPIOs for your exact ESP32-S3 board and wiring.
// Leaving these at -1 disables related I/O and injection output will not function.
constexpr int MAP_SENSOR_ADC = -1;
constexpr int FLOAT_SENSOR_DIGITAL = -1;
constexpr int PUMP_PWM = -1;
constexpr int WARNING_LED = -1;
} // namespace pins
