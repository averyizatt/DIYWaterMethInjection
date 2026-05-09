#pragma once

namespace pins {
// Defaults are intentionally disabled to force board-specific assignment.
// Set these to validated GPIOs for your exact ESP32-S3 board and wiring.
constexpr int MAP_SENSOR_ADC = -1;
constexpr int FLOAT_SENSOR_DIGITAL = -1;
constexpr int PUMP_PWM = -1;
} // namespace pins
