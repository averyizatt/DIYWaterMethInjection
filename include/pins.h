#pragma once

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

namespace pins {
<<<<<<< HEAD

// MAP sensor — GM-style 0.5–4.5 V linear pressure output
// A0 on board -> GPIO1 (ADC1_CH0, safe with WiFi off)
constexpr int MAP_SENSOR_ADC = 1;

// Pump MOSFET gate — PWM output to logic-level MOSFET (e.g. IRLB8721)
// D9 on board -> GPIO18 (LEDC-capable)
constexpr int PUMP_PWM = 18;

// Tank float switch — active-low, internal pull-up enabled
// D5 on board -> GPIO8
constexpr int FLOAT_SENSOR_DIGITAL = 8;

// DS18B20 1-Wire temperature sensors (one sensor per pin)
// D2 on board -> GPIO5  — inside engine bay
constexpr int TEMP_ENGINE_BAY = 5;
// D3 on board -> GPIO6  — outside / intake air
constexpr int TEMP_AMBIENT = 6;

// Warning LED or active-high relay driver
// D8 on board -> GPIO17
constexpr int WARNING_LED = 17;

// MCP2515 CAN bus module (SPI)
// Shared SPI bus: SCK=D13/GPIO48, MOSI=D11/GPIO38, MISO=D12/GPIO47
// D10 on board -> GPIO21  (SPI chip-select)
constexpr int CAN_CS = 21;
// D4  on board -> GPIO7   (MCP2515 /INT — active-low interrupt)
constexpr int CAN_INT = 7;

=======
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
>>>>>>> 54c27c114127d33f6a78ce395b3c255993478aad
} // namespace pins
