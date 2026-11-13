#pragma once

// What the ESP32 board's analog and switch inputs mean, from millivolts at the pin.
// Pure logic with no Arduino calls, tested on a PC (test/test_esp32_inputs).
#include <math.h>
#include <stdint.h>

#include "meth_tune.h"

namespace analog_inputs {

// 0.5-4.5 V, 0-100 psi pressure transducer (5 V supply) read through a 6.8k / 10k divider
// (sensor - 6.8k - pin - 10k - ground, 100 nF from the pin to ground): the pin sees 0.595
// of the sensor voltage, 2.68 V at full scale and 2.98 V for a sensor stuck at 5 V, so a
// fault still reads inside the ADC's range.
struct Pressure {
  float psi{0.0f};
  float sensorV{0.0f};
  bool valid{false};
  bool seen{false};

  // Returns the filtered pressure state after one reading of the pin voltage.
  void update(float pinMv) {
    constexpr float kTopOhms = 6800.0f, kBottomOhms = 10000.0f;
    sensorV = pinMv / 1000.0f * (kTopOhms + kBottomOhms) / kBottomOhms;
    // Below 0.1 V: unplugged or shorted to ground. Above 4.85 V: open ground or shorted to 5 V.
    if (sensorV < 0.10f || sensorV > 4.85f) { valid = false; seen = false; return; }
    const float raw = (sensorV - 0.5f) * 100.0f / 4.0f;
    if (raw < -5.0f || raw > 120.0f) { valid = false; seen = false; return; }
    const float clamped = raw < 0.0f ? 0.0f : raw;
    psi = seen ? psi + (clamped - psi) * 0.20f : clamped;
    seen = true;
    valid = true;
  }
};

// GM-style intake air thermistor on a 2.49k pull-up to the 3.3 V rail (meth_tune's curve).
//
// adcTopMv is the highest the ADC can read on that pin (adc_sampler::topMv(), about
// 3.17 V on the ESP32-S3). It is below the 3.3 V that an unplugged sensor pulls the pin
// to, so an open sensor reads at the ADC's top, not at the supply, and would otherwise
// pass for air at -32 C. A reading that close to the top is no sensor; a real one there
// would be colder than about -25 C, which this circuit cannot measure on this ADC either.
constexpr float kAdcTopMarginMv = 40.0f;

inline bool thermistorC(float pinMv, float supplyMv, float pullupOhms, float adcTopMv, float &tempC) {
  if (!(supplyMv > 0.0f) || pinMv < 0.0f) return false;
  if (pinMv >= adcTopMv - kAdcTopMarginMv) return false;
  const float count = pinMv * 1023.0f / supplyMv;
  if (count >= 1023.0f) return false;
  return meth_tune::thermistorC(static_cast<uint16_t>(count + 0.5f), pullupOhms, tempC);
}

// Two intake air inputs (GPIO5 and GPIO6) and one sensor in the car, before the nozzle,
// plugged into either (owner, November 2026: read both and throw out the one that is
// obviously not connected). An input with nothing on it reads at the top of the ADC (its
// pull-up holds it there, and the pin's own pull-up too in case the 2.49 k is not fitted)
// and a shorted one at the bottom: thermistorC() rejects both. The input reading a sensor
// is the air before the nozzle. With a sensor on each, GPIO5 is before the nozzle and
// GPIO6 after it, as docs/ESP32_HARDWARE.md wires them.
//
// The first read decides. After that an input has to read the other way kIntakeSettleReads
// times in a row before the choice changes, so a loose plug does not swap them back and
// forth; a single bad read only makes that read invalid. Each temperature is smoothed lightly.
constexpr uint8_t kIntakeSettleReads = 40;   // 2 s at one read every 50 ms

class IntakeInputs {
public:
  // One read of both inputs: whether each reads a sensor, and its temperature if it does.
  void update(bool okA, float cA, bool okB, float cB) {
    take(0, okA, cA);
    take(1, okB, cB);
    started_ = true;
  }
  // The input that is the air before the nozzle: 0 (GPIO5), 1 (GPIO6), or -1 for none.
  int8_t preInput() const { return fitted_[0] ? 0 : fitted_[1] ? 1 : -1; }
  // The air after the nozzle: only with a sensor on both inputs, and then GPIO6.
  int8_t postInput() const { return fitted_[0] && fitted_[1] ? 1 : -1; }
  bool preValid() const { return preInput() >= 0 && valid_[preInput()]; }
  float preC() const { return preInput() >= 0 ? c_[preInput()] : 0.0f; }
  bool postValid() const { return postInput() >= 0 && valid_[postInput()]; }
  float postC() const { return postInput() >= 0 ? c_[postInput()] : 0.0f; }

private:
  void take(uint8_t i, bool ok, float c) {
    if (ok) c_[i] = valid_[i] ? c_[i] + (c - c_[i]) * 0.3f : c;
    valid_[i] = ok;
    if (!started_) {
      fitted_[i] = ok;
    } else if (ok == fitted_[i]) {
      streak_[i] = 0;
    } else if (++streak_[i] >= kIntakeSettleReads) {
      fitted_[i] = ok;
      streak_[i] = 0;
    }
  }

  float c_[2]{0.0f, 0.0f};
  uint8_t streak_[2]{0, 0};
  bool valid_[2]{false, false};
  bool fitted_[2]{false, false};
  bool started_{false};
};

// Tank float switch: debounced, and "low" only after it has stayed low for a while so
// slosh does not stop injection. Starts as low until proven otherwise (fail-safe).
class FloatSwitch {
public:
  void begin(bool rawLow, uint32_t now, uint32_t debounceMs, uint32_t holdMs) {
    debounceMs_ = debounceMs; holdMs_ = holdMs;
    lastRaw_ = debounced_ = sustained_ = rawLow;
    lastChangeMs_ = lowSinceMs_ = now;
  }
  // Returns true while the tank counts as low.
  bool update(bool rawLow, uint32_t now) {
    if (rawLow != lastRaw_) { lastRaw_ = rawLow; lastChangeMs_ = now; }
    if (rawLow != debounced_ && static_cast<uint32_t>(now - lastChangeMs_) >= debounceMs_) {
      debounced_ = rawLow;
      if (debounced_) lowSinceMs_ = now;
    }
    if (!debounced_) sustained_ = false;
    else if (static_cast<uint32_t>(now - lowSinceMs_) >= holdMs_) sustained_ = true;
    return sustained_;
  }
  bool isLow() const { return sustained_; }

private:
  uint32_t debounceMs_{100}, holdMs_{2000}, lastChangeMs_{0}, lowSinceMs_{0};
  bool lastRaw_{true}, debounced_{true}, sustained_{true};
};

} // namespace analog_inputs
