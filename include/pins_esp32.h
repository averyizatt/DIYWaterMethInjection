#pragma once

// ESP32-S3 DevKitC-1 wiring for the water/meth and knock controller.
// Full wiring, parts and the reasons are in docs/ESP32_HARDWARE.md.
//
// Not used on purpose: GPIO0, 3, 45, 46 (boot straps), 19, 20 (USB), 26-37 (flash/PSRAM).
namespace pins {
// MCP2515 CAN module on the default SPI pins. INT is not connected: the bus is polled.
constexpr int CAN_SPI_CS = 10;
constexpr int CAN_SPI_MOSI = 11;
constexpr int CAN_SPI_SCK = 12;
constexpr int CAN_SPI_MISO = 13;

// Analog inputs, all on ADC1 (0-3.1 V at the pin). Sampled together by adc_sampler.
constexpr int KNOCK_SENSOR_ADC = 1;        // ADC1 channel 0: knock amplifier, resting near 1.65 V
constexpr int OIL_PRESSURE_ADC = 2;        // ADC1 channel 1: 0.5-4.5 V sensor through 10k/20k
constexpr int FUEL_PRESSURE_ADC = 4;       // ADC1 channel 3: 0.5-4.5 V sensor through 10k/20k
constexpr int PRE_INJECTION_TEMP_ADC = 5;  // ADC1 channel 4: intake air before the nozzle
constexpr int POST_INJECTION_TEMP_ADC = 6; // ADC1 channel 5: spare; intake air after the nozzle if fitted
constexpr float INTAKE_TEMP_PULLUP_OHMS = 2490.0f;  // Each thermistor: 3.3 V -> 2.49k -> pin -> sensor -> GND
constexpr float INTAKE_TEMP_SUPPLY_MV = 3300.0f;

constexpr int FLOAT_SENSOR_DIGITAL = 7;    // Tank float switch to GND, internal pull-up
// Pump MOSFET gate (logic-level, low side). A 10k resistor from gate to ground on the
// board keeps the pump off while the ESP32 is booting, being flashed or reset.
constexpr int PUMP_OUT = 15;
constexpr int WARNING_LED = 16;            // High = warning
// Optional master relay in the pump's 12 V feed: high from the start of a spray until a
// few seconds after it, so a MOSFET that has failed "on" cannot run the pump. Also
// needs a 10k pull-down at the relay driver's input.
constexpr int PUMP_RELAY_OUT = 17;
} // namespace pins

// Pump PWM. Low enough for any MOSFET module to switch cleanly from a 3.3 V gate
// signal, high enough that the pump runs smoothly.
constexpr unsigned PUMP_PWM_HZ = 200;
constexpr unsigned PUMP_PWM_BITS = 10;
