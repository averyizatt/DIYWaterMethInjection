#pragma once

// The water/meth controller PCB, rev D (averyizatt.com; KiCad project WaterMethController):
// a Seeed XIAO ESP32-S3 with a gate-driven pump MOSFET and current sense, a solenoid valve
// output, two intake air inputs, a 0.5-4.5 V pressure input, a tank level switch, 12 V and
// sensor-supply monitors and a CAN transceiver (TCAN1042V) on the ESP32's own CAN
// controller. Everything is on one 14-way AMPSEAL connector (docs/PCB.md).
//
// Both PCB builds use this: WaterMethPCBFrogDash (the Frogdash dash's CAN protocol) and
// WaterMethPCBStandalone (any car, set up from a phone). The DevKitC-1 board in pins_esp32.h is a
// different board; its firmware (main_esp32.cpp) is not for this one, and the reverse.
#include <stdint.h>

namespace pcb {

// XIAO pin, GPIO and use (the board's silkscreen and README list the same).
constexpr int IAT1_ADC = 1;     // D0: intake air 1 (thermistor), ADC1 channel 0
constexpr int IAT2_ADC = 2;     // D1: intake air 2 (thermistor), ADC1 channel 1
constexpr int VBAT_ADC = 3;     // D2: 12 V monitor, 100k / 15k (a boot strap pin: input only)
constexpr int PRESS_ADC = 4;    // D3: pressure input, 0.5-4.5 V sensors
constexpr int VSENS_ADC = 5;    // D4: the sensors' own 5 V supply, measured
constexpr int SOLENOID = 6;     // D5: HIGH opens the valve; 4.7k pull-down keeps it shut otherwise
constexpr int LEVEL = 43;       // D6: tank level switch, HIGH = open. The ROM's boot log pin: read after start-up only
constexpr int PUMP = 44;        // D7: pump gate driver, HIGH = on; 4.7k pull-down keeps it off while booting
constexpr int ISENSE_ADC = 7;   // D8: pump current, 0.1 V per amp (5 mohm x INA240 gain 20)
constexpr int CAN_TX = 8;       // D9: to the transceiver
constexpr int CAN_RX = 9;       // D10
constexpr int LED = 21;         // The XIAO's own orange LED, LOW = lit

// Pump PWM, used only for each run's soft start (the pump otherwise runs full on).
// 100 Hz to 1 kHz on this board: above that its bulk capacitors run past their ripple
// rating, and 2-10 kHz sits on the harness resonance (README, Simulation).
constexpr unsigned PUMP_PWM_HZ = 500;
constexpr unsigned PUMP_PWM_BITS = 10;

// Every image built for this board carries this text, and firmware arriving for it is
// refused without it (a DevKitC-1 image would drive the wrong pins). A different text
// for each build, so one is never put on a board set up for the other by mistake: the
// WaterMethPCBFrogDash build talks to the dash, WaterMethPCBStandalone has its own settings.
constexpr char kImageMarkFrogdash[] = "WMPCB-REVD-FROGDASH-IMAGE-1";
constexpr char kImageMarkStandalone[] = "WMPCB-REVD-STANDALONE-IMAGE-1";

} // namespace pcb
