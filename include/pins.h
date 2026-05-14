#pragma once

namespace pins {
// ESP32-S3 GPIO assignments for the water/meth injection controller.
// Adjust these to match your actual board wiring before flashing.

// Analog MAP sensor input (ADC1 channel, 0–3.3 V output)
constexpr int MAP_SENSOR_ADC = 1;       // GPIO1 / ADC1_CH0

// Tank level switch — INPUT_PULLUP.
// Switch is 0 Ω (closed to GND) when tank is FULL  → pin reads LOW  → NOT empty.
// Switch is open-circuit when tank is EMPTY          → pin reads HIGH → empty.
// floatActiveLow is therefore false in app_config.h.
constexpr int FLOAT_SENSOR_DIGITAL = 2; // GPIO2

// Pump PWM output (LEDC channel 0)
constexpr int PUMP_PWM = 3;             // GPIO3

// Warning LED / relay output — active-HIGH, driven HIGH on failsafe
constexpr int WARNING_LED = 4;          // GPIO4

// CAN bus (ESP32-S3 TWAI peripheral)
constexpr int CAN_TX = 5;              // GPIO5 → CAN transceiver TXD
constexpr int CAN_RX = 6;              // GPIO6 → CAN transceiver RXD
} // namespace pins
