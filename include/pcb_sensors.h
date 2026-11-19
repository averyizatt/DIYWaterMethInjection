#pragma once

// What the PCB's inputs mean, from millivolts at the ESP32's pins (pcb_board.h). Pure
// logic with no Arduino calls, tested on a PC (test/test_pcb).
//
// Every 0-5 V input reaches its pin through 10k in series and 18k to ground: the pin sees
// 18/28 = 0.643 of the input. The sensors' supply is the 5 V rail less a protection diode,
// about 4.7 V and drifting a little with temperature; it is measured (VSENS) and the
// sensors are read as a share of it, so its drift cancels.
#include <math.h>
#include <stdint.h>

namespace pcb_sensors {

constexpr float kInputShare = 18.0f / 28.0f;            // Pin voltage / input voltage
constexpr float kBatteryShare = 15.0f / 115.0f;         // 100k / 15k: pin / battery
constexpr float kAmpsPerPinMv = 1.0f / 100.0f;          // 0.1 V per amp
constexpr float kPullupOhms = 2490.0f;                  // Intake air and level inputs, to the sensor supply
constexpr float kDividerOhms = 28000.0f;                // 10k + 18k, in parallel with the thermistor
constexpr float kRailLowMv = 4000.0f;                   // Below this the sensor supply is failing (shorted wire)
constexpr float kRailHighMv = 5400.0f;

inline float inputMv(float pinMv) { return pinMv / kInputShare; }
inline float batteryV(float pinMv) { return pinMv / kBatteryShare / 1000.0f; }
inline float pumpAmps(float pinMv) { return pinMv * kAmpsPerPinMv; }
inline bool railOk(float railMv) { return railMv >= kRailLowMv && railMv <= kRailHighMv; }

// GM-style intake air sensor (the curve meth_tune::thermistorC uses): Steinhart-Hart
// through -40 C 100.7k, 30 C 2238 ohm, 99 C 177 ohm.
inline float gmThermistorC(float ohms) {
  const float lnR = logf(ohms);
  return 1.0f / (1.502886e-3f + 2.254721e-4f * lnR + 1.234848e-7f * lnR * lnR * lnR) - 273.15f;
}

// Intake air: sensor supply - 2.49k - input - (thermistor || 28k divider) - ground. False
// for no sensor (the input then sits at the divider's own 92 % of the supply) or a short.
// The divider in parallel limits the cold end: about -30 C is the coldest told apart from
// an open input.
inline bool intakeC(float pinMv, float railMv, float &tempC) {
  if (!railOk(railMv)) return false;
  const float v = inputMv(pinMv);
  if (!(v > 15.0f) || v >= railMv) return false;
  const float bottom = kPullupOhms * v / (railMv - v);              // Thermistor || divider
  if (bottom >= kDividerOhms * 0.80f) return false;                 // Open: no sensor (> about 112k)
  const float ohms = 1.0f / (1.0f / bottom - 1.0f / kDividerOhms);
  if (!(ohms > 40.0f) || ohms > 110000.0f) return false;            // Shorted, or colder than the curve
  tempC = gmThermistorC(ohms);
  return true;
}

// A 0.5-4.5 V ratiometric sensor's share of its supply (0.1 at 0.5 V, 0.9 at 4.5 V of 5 V).
inline float shareOf(float pinMv, float railMv) { return railMv > 0.0f ? inputMv(pinMv) / railMv : 0.0f; }

// Line pressure from a 0.5-4.5 V sensor of fullScalePsi. False outside 2-98 % of the
// supply: unplugged (the divider pulls the input to 0), shorted, or no supply.
inline bool linePsi(float pinMv, float railMv, float fullScalePsi, float &psi) {
  if (!railOk(railMv)) return false;
  const float share = shareOf(pinMv, railMv);
  if (!(share >= 0.02f && share <= 0.98f)) return false;
  psi = (share - 0.1f) / 0.8f * fullScalePsi;
  if (psi < 0.0f) psi = 0.0f;
  return true;
}

// Tank level switch on the level input (pull-up to the sensor supply, switch to ground),
// read as a digital pin. With the usual float that closes while there is fluid, a HIGH pin
// (switch open, or a broken wire) is low fluid. closedWhenLow reverses that.
inline bool tankLow(bool pinHigh, bool closedWhenLow) { return closedWhenLow ? !pinHigh : pinHigh; }

// A reading smoothed over a few passes; the first value is taken as it is.
struct Smooth {
  float value{0.0f};
  bool seen{false};
  float take(float v, float share) {
    value = seen ? value + (v - value) * share : v;
    seen = true;
    return value;
  }
  void forget() { seen = false; }
};

} // namespace pcb_sensors
