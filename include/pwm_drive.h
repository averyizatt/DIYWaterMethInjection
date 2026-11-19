#pragma once

// PWM pump drive, a choice in the stand-alone PCB build (setting "pump_drive"): the pump runs
// continuously while injecting, at a duty that rises with boost from the lowest duty
// (meth_tune kMinDutyPct, at start boost) to the highest (kMaxDutyPct, at full boost and
// above), for pumps and nozzles that work that way. Below start boost (an early start, hot
// air) it runs at the lowest duty or the hot-air share, whichever is more. The default stays the pulse drive
// (meth_tune::PulseGovernor), full pressure in timed pulses.
//
// The decision to inject at all is the pulse timing's, so every gate is the same: armed,
// boost, engine speed, air temperature, the longest spray and its rest. The governor runs
// beside this, and its hold says whether to spray; only the output differs.
//
// Dose limit: at part duty a pump still holds part pressure, so flow falls more slowly than
// duty. Flow is taken as the nozzle's figure times the square root of the duty (a nozzle's
// flow goes with the square root of its pressure, pressure with the motor's voltage), which
// errs towards less fluid: the duty is capped at the square of the share of time the pulse
// drive would be allowed. Pure logic, tested on a PC (test/test_pcb).
#include <stdint.h>

#include "meth_tune.h"

namespace pwm_drive {

// 0..1 along a ramp from start to full; 0 below start.
inline float along(float value, float start, float full) {
  if (!(full > start)) return value >= start ? 1.0f : 0.0f;
  const float f = (value - start) / (full - start);
  return f < 0.0f ? 0.0f : f > 1.0f ? 1.0f : f;
}

// The duty wanted for these inputs, before the soft start: 0 when the pulse timing holds.
inline float targetDuty(const meth_tune::Settings &s, const meth_tune::Inputs &in, uint8_t hold) {
  if (hold != meth_tune::kHoldNone) return 0.0f;
  const float lo = s.value[meth_tune::kMinDutyPct], hi = s.value[meth_tune::kMaxDutyPct];
  float f = along(in.boostPsi, s.value[meth_tune::kStartPsiX10] / 10.0f, s.value[meth_tune::kFullPsiX10] / 10.0f);
  if (s.value[meth_tune::kHotStartC] > 0 && in.preValid && in.preC > s.value[meth_tune::kHotStartC]) {
    const float hot = along(in.preC, s.value[meth_tune::kHotStartC], s.value[meth_tune::kHotFullC]);
    if (hot > f) f = hot;
  }
  float duty = lo + (hi - lo) * f;
  // The dose limit, through the square-root flow above.
  if (in.rpmValid || in.rpmRequired) {
    const float share = static_cast<float>(meth_tune::doseLimitOnMs(s, in.rpm, in.mapKpa, in.fuelScale)) / s.value[meth_tune::kPeriodMs];
    const float cap = 100.0f * share * share;
    if (duty > cap) duty = cap;
  }
  if (in.capDutyPercent < 100.0f && duty > in.capDutyPercent) duty = in.capDutyPercent;
  return duty >= lo ? duty : 0.0f;   // Below the lowest duty the pump does not spray: off
}

class Drive {
public:
  void setSoftStartMs(uint32_t ms) { softStartMs_ = ms; }
  // The duty for this instant: the target, reached over the soft start after each stop.
  float update(uint32_t now, const meth_tune::Settings &s, const meth_tune::Inputs &in, uint8_t hold) {
    const float target = targetDuty(s, in, hold);
    if (target > 0.0f && !on_) startMs_ = now;
    on_ = target > 0.0f;
    if (!on_) return duty_ = 0.0f;
    if (softStartMs_ > 0 && now - startMs_ < softStartMs_) {
      const float ramp = meth_tune::kSoftStartFloorPct + (100.0f - meth_tune::kSoftStartFloorPct) * (now - startMs_) / softStartMs_;
      return duty_ = ramp < target ? ramp : target;
    }
    return duty_ = target;
  }
  float duty() const { return duty_; }

private:
  uint32_t startMs_{0};
  uint32_t softStartMs_{meth_tune::kSoftStartMs};
  float duty_{0.0f};
  bool on_{false};
};

} // namespace pwm_drive
