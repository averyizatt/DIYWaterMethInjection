#pragma once

// What the ESP32 board's analog and switch inputs mean, from millivolts at the pin.
// Pure logic with no Arduino calls, tested on a PC (test/test_esp32_inputs).
#include <math.h>
#include <stdint.h>

#include "meth_tune.h"

namespace analog_inputs {

// 0.5-4.5 V, 0-100 psi pressure transducer (5 V supply) read through a 10k/20k divider:
// the pin sees two thirds of the sensor voltage, 3.0 V at full scale.
struct Pressure {
  float psi{0.0f};
  float sensorV{0.0f};
  bool valid{false};
  bool seen{false};

  // Returns the filtered pressure state after one reading of the pin voltage.
  void update(float pinMv) {
    constexpr float kTopOhms = 10000.0f, kBottomOhms = 20000.0f;
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

// GM-style intake air thermistor on a 2.49k pull-up to the 3.3 V rail. The curve is the
// one the relay controller used; only the reference voltage differs.
inline bool thermistorC(float pinMv, float supplyMv, float pullupOhms, float &tempC) {
  if (!(supplyMv > 0.0f) || pinMv < 0.0f) return false;
  const float count = pinMv * 1023.0f / supplyMv;
  if (count >= 1023.0f) return false;
  return meth_tune::thermistorC(static_cast<uint16_t>(count + 0.5f), pullupOhms, tempC);
}

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
