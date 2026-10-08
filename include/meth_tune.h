#pragma once

// Water/meth pulse tuning: the adjustable settings with their hard limits, and the
// governor that decides on every loop whether the pump relay is on.
//
// The pump is switched directly by a mechanical relay, so flow is set by how long the
// pump runs in each cycle, not by PWM, and the cycles are slow: the pump runs for at
// least half a second, rests for at least half a second (or not at all: continuous),
// and starts at most once per cycle. Short blips would wear the contacts and never
// build nozzle pressure anyway. Everything here is pure logic with no Arduino calls,
// so it is tested on a PC (test/test_meth_tune).
#include <math.h>
#include <stdint.h>

namespace meth_tune {

// Same numbers as can_protocol::meth_tune_setting (checked in main_nano.cpp).
enum Key : uint8_t {
  kPeriodMs = 1,     // Relay cycle length
  kMinOnMs = 2,      // Pump on-time per cycle at start boost
  kMaxOnMs = 3,      // Pump on-time per cycle at full boost
  kStartPsiX10 = 4,  // Boost where injection starts
  kFullPsiX10 = 5,   // Boost where on-time reaches kMaxOnMs
  kMinRpm = 6,       // Engine RPM must be known and at least this
  kMaxSprayS = 7,    // Longest continuous injection
  kRestS = 8,        // Pause after kMaxSprayS; 0 = no time limit
  kRampMs = 9,       // On-time growth allowed per cycle; 0 = no ramp
  kMinPreTempC = 10, // No injection while the air before the nozzle is colder; 0 = off
  kOverboostAssist = 11,
  kMethPercent = 12, // Methanol in the tank by volume; 0 = water only
  kNozzleMlMin = 13, // Nozzle flow with the pump running
  kMaxDosePct = 14,  // Most fluid allowed, as a percentage of estimated fuel flow; 0 = off
};
constexpr uint8_t kCount = 14;

// The relay's hard limits, whatever the settings say.
constexpr uint16_t kShortestPulseMs = 500; // Pump runs at least this long, or not at all.
constexpr uint16_t kShortestGapMs = 500;   // Off at least this long, or stays on.

// Fuel flow estimate for the dose limit: grams of fuel per minute per (RPM x kPa of
// manifold absolute pressure). 2.3 L four-stroke, 85% volumetric efficiency, 50 C
// charge air, 12:1 air/fuel at full load. Change it for a different engine.
constexpr float kFuelGPerMinPerRpmKpa = 8.79e-4f;

// Same numbers as can_protocol::meth_hold: why the pump is not pulsing.
enum Hold : uint8_t {
  kHoldNone = 0,
  kHoldDisarmed = 1,
  kHoldBelowBoost = 2,
  kHoldRpmLow = 3,
  kHoldRpmMissing = 4,
  kHoldAirCold = 5,
  kHoldTempSensor = 6,
  kHoldResting = 7,
  kHoldTankLow = 8,
  kHoldFault = 9,
  kHoldDoseLimit = 10, // The dose limit allows less than the shortest pulse here.
};

// Same numbers as can_protocol::config_ack_status.
constexpr uint8_t kSetOk = 0;
constexpr uint8_t kSetUnsupported = 1;
constexpr uint8_t kSetClamped = 3;

struct Range {
  uint16_t min;
  uint16_t max;
  uint16_t def;
};

// Hard limits and the conservative defaults: the pump runs 1 s in every 4 s at start
// boost and 2 s in every 4 s at full boost, and never more than 10% of fuel flow.
inline Range rangeOf(uint8_t key) {
  switch (key) {
  case kPeriodMs: return {1000, 10000, 4000};
  case kMinOnMs: return {kShortestPulseMs, 10000, 1000};
  case kMaxOnMs: return {kShortestPulseMs, 10000, 2000};
  case kStartPsiX10: return {10, 300, 50};
  case kFullPsiX10: return {20, 350, 100};
  case kMinRpm: return {0, 8000, 2500};
  case kMaxSprayS: return {1, 120, 12};
  case kRestS: return {0, 60, 6};
  case kRampMs: return {0, 5000, 500};
  case kMinPreTempC: return {0, 120, 0};
  case kOverboostAssist: return {0, 1, 0};
  case kMethPercent: return {0, 100, 0};
  case kNozzleMlMin: return {20, 1000, 60};
  case kMaxDosePct: return {0, 40, 10};
  default: return {0, 0, 0};
  }
}

struct Settings {
  uint16_t value[kCount + 1];  // Indexed by Key; [0] unused.
};

inline void loadDefaults(Settings &s) {
  s.value[0] = 0;
  for (uint8_t key = 1; key <= kCount; ++key) s.value[key] = rangeOf(key).def;
}

// True when every value is in range and the pairs are consistent (used for saved data).
inline bool valid(const Settings &s) {
  for (uint8_t key = 1; key <= kCount; ++key) {
    const Range r = rangeOf(key);
    if (s.value[key] < r.min || s.value[key] > r.max) return false;
  }
  return s.value[kMinOnMs] <= s.value[kMaxOnMs] && s.value[kMaxOnMs] <= s.value[kPeriodMs] &&
         s.value[kFullPsiX10] >= s.value[kStartPsiX10] + 10;
}

// Change one setting. The edited value is clamped to its range and against its
// partner; other settings are never raised as a side effect. Shortening the period
// only ever shortens the on-times. Returns an acknowledgement status.
inline uint8_t set(Settings &s, uint8_t key, uint16_t requested, uint16_t &applied) {
  if (key < 1 || key > kCount) {
    applied = 0;
    return kSetUnsupported;
  }
  const Range r = rangeOf(key);
  uint16_t v = requested < r.min ? r.min : requested > r.max ? r.max : requested;
  if (key == kMinOnMs && v > s.value[kMaxOnMs]) v = s.value[kMaxOnMs];
  if (key == kMaxOnMs) {
    if (v > s.value[kPeriodMs]) v = s.value[kPeriodMs];
    if (v < s.value[kMinOnMs]) v = s.value[kMinOnMs];
  }
  if (key == kStartPsiX10 && v + 10 > s.value[kFullPsiX10]) v = s.value[kFullPsiX10] - 10;
  if (key == kFullPsiX10 && v < s.value[kStartPsiX10] + 10) v = s.value[kStartPsiX10] + 10;
  s.value[key] = v;
  if (key == kPeriodMs) {
    if (s.value[kMaxOnMs] > v) s.value[kMaxOnMs] = v;
    if (s.value[kMinOnMs] > s.value[kMaxOnMs]) s.value[kMinOnMs] = s.value[kMaxOnMs];
  }
  applied = v;
  return v == requested ? kSetOk : kSetClamped;
}

// On-time for this boost: kMinOnMs at start boost, rising in a straight line to
// kMaxOnMs at full boost and flat above it.
inline uint16_t targetOnMs(const Settings &s, float boostPsi) {
  const float start = s.value[kStartPsiX10] / 10.0f;
  const float full = s.value[kFullPsiX10] / 10.0f;
  float fraction = full > start ? (boostPsi - start) / (full - start) : 1.0f;
  if (!(fraction > 0.0f)) fraction = 0.0f;  // Also catches NaN.
  if (fraction > 1.0f) fraction = 1.0f;
  const uint16_t minOn = s.value[kMinOnMs];
  const uint16_t maxOn = s.value[kMaxOnMs];
  return static_cast<uint16_t>(minOn + static_cast<uint16_t>((maxOn - minOn) * fraction + 0.5f));
}

// Estimated fuel flow (g/min) at this engine speed and manifold absolute pressure.
inline float fuelGPerMin(uint16_t rpm, float mapKpa) {
  return mapKpa > 0.0f ? rpm * mapKpa * kFuelGPerMinPerRpmKpa : 0.0f;
}

// Longest on-time the dose limit allows in one cycle: fluid may be at most
// kMaxDosePct of fuel flow, and the nozzle delivers kNozzleMlMin while the pump runs
// (1 ml taken as 1 g). Returns the period when the limit is off or not reached.
inline uint16_t doseLimitOnMs(const Settings &s, uint16_t rpm, float mapKpa) {
  const uint16_t period = s.value[kPeriodMs];
  if (s.value[kMaxDosePct] == 0) return period;
  const float allowedMlMin = fuelGPerMin(rpm, mapKpa) * s.value[kMaxDosePct] / 100.0f;
  const float duty = allowedMlMin / s.value[kNozzleMlMin];
  return duty >= 1.0f ? period : static_cast<uint16_t>(period * duty);
}

// The relay's rules applied to a wanted on-time: too short is nothing, and an off gap
// too short to be worth a switch is widened by shortening the pulse (never lengthened).
inline uint16_t relaySafeOnMs(uint16_t onMs, uint16_t period) {
  if (onMs >= period) return period;
  if (period - onMs < kShortestGapMs) onMs = period > kShortestGapMs ? period - kShortestGapMs : 0;
  return onMs < kShortestPulseMs ? 0 : onMs;
}

// GM-style intake air thermistor on a pull-up to the 10-bit ADC reference.
// Returns false for an open or shorted sensor.
inline bool thermistorC(uint16_t adcCount, float pullupOhms, float &tempC) {
  if (adcCount <= 8 || adcCount >= 1010) return false;
  const float ohms = pullupOhms * adcCount / (1023.0f - adcCount);
  const float lnR = static_cast<float>(log(ohms));
  // Steinhart-Hart through the GM calibration points -40 C 100.7k, 30 C 2238, 99 C 177 ohm.
  tempC = 1.0f / (1.502886e-3f + 2.254721e-4f * lnR + 1.234848e-7f * lnR * lnR * lnR) - 273.15f;
  return true;
}

struct Inputs {
  bool wantSpray{false};        // Armed, above start boost and no failsafe.
  uint8_t idleHold{kHoldDisarmed};  // The reason, while wantSpray is false.
  float boostPsi{0.0f};
  float mapKpa{0.0f};           // Manifold absolute pressure, for the dose limit.
  bool rpmValid{false};         // A fresh engine RPM message is arriving.
  uint16_t rpm{0};
  bool preValid{false};
  float preC{0.0f};
  bool assistActive{false};     // Overboost assist requested by the controller.
  float assistDutyPercent{0.0f};
  float capDutyPercent{100.0f}; // Master duty cap; 100 = none.
};

class Governor {
public:
  // Normal injection. Returns the pump relay state for this instant.
  bool update(uint32_t now, const Settings &s, const Inputs &in) {
    const uint32_t dt = started_ ? (now - lastMs_ > 1000UL ? 1000UL : now - lastMs_) : 0;
    lastMs_ = now;
    started_ = true;

    uint8_t hold = kHoldNone;
    if (!in.wantSpray) hold = in.idleHold == kHoldNone ? static_cast<uint8_t>(kHoldBelowBoost) : in.idleHold;
    else if (!in.rpmValid) hold = kHoldRpmMissing;
    else if (in.rpm < s.value[kMinRpm]) hold = kHoldRpmLow;
    else if (s.value[kMinPreTempC] > 0 && !in.preValid) hold = kHoldTempSensor;
    else if (s.value[kMinPreTempC] > 0 && in.preC < static_cast<float>(s.value[kMinPreTempC])) hold = kHoldAirCold;

    // Spray time budget: fills while injecting, drains while not. A full rest refills it.
    const uint32_t maxMs = static_cast<uint32_t>(s.value[kMaxSprayS]) * 1000UL;
    const uint16_t restS = s.value[kRestS];
    const bool injecting = hold == kHoldNone && !resting_;
    if (restS == 0) {
      usedMs_ = 0;
      resting_ = false;
    } else if (injecting) {
      usedMs_ += dt;
      if (usedMs_ >= maxMs) {
        usedMs_ = maxMs;
        resting_ = true;
      }
    } else {
      const uint32_t drain = dt * s.value[kMaxSprayS] / restS;
      usedMs_ = usedMs_ > drain ? usedMs_ - drain : 0;
      if (usedMs_ == 0) resting_ = false;
    }
    if (hold == kHoldNone && resting_) hold = kHoldResting;

    const uint16_t period = s.value[kPeriodMs];
    uint16_t target = targetOnMs(s, in.boostPsi);
    uint16_t ramp = s.value[kRampMs];
    if (in.assistActive && s.value[kOverboostAssist]) {
      const uint16_t assist = onFromDuty(in.assistDutyPercent, period);
      if (assist > target) target = assist;
      ramp = 0;
    }
    // Limits come last, so nothing above can exceed them.
    const uint16_t dose = doseLimitOnMs(s, in.rpm, in.mapKpa);
    if (target > dose) target = dose;
    if (in.capDutyPercent < 100.0f) {
      const uint16_t cap = onFromDuty(in.capDutyPercent, period);
      if (target > cap) target = cap;
    }
    if (hold == kHoldNone && relaySafeOnMs(target, period) == 0) hold = kHoldDoseLimit;
    hold_ = hold;
    return pulse(now, hold == kHoldNone, target, period, ramp, s.value[kMinOnMs]);
  }

  // Pump test: no RPM, temperature, dose or time gates, but the same relay rules.
  // Any test above 0% runs the pump for at least the shortest pulse.
  bool manual(uint32_t now, const Settings &s, uint8_t dutyPercent) {
    lastMs_ = now;
    started_ = true;
    hold_ = kHoldNone;
    const uint16_t period = s.value[kPeriodMs];
    uint16_t on = onFromDuty(dutyPercent, period);
    if (dutyPercent > 0 && on < kShortestPulseMs) on = kShortestPulseMs;
    return pulse(now, dutyPercent > 0, on, period, 0, 0);
  }

  uint8_t hold() const { return hold_; }
  bool resting() const { return resting_; }
  // On-time of the cycle now running; 0 while not injecting.
  uint16_t onMs() const { return cycleActive_ && !cut_ ? cycleOn_ : 0; }

private:
  static uint16_t onFromDuty(float dutyPercent, uint16_t period) {
    if (!(dutyPercent > 0.0f)) return 0;
    if (dutyPercent >= 100.0f) return period;
    return static_cast<uint16_t>(period * dutyPercent / 100.0f + 0.5f);
  }

  // One pump run per cycle at most. A cycle that is blocked part-way stays off until
  // the next one, so the relay can never be asked to switch faster than the period.
  bool pulse(uint32_t now, bool want, uint16_t targetOn, uint16_t period, uint16_t rampMs, uint16_t minOn) {
    const bool out = decide(now, want, targetOn, period, rampMs, minOn);
    if (out_ && !out) offAt_ = now;
    out_ = out;
    return out;
  }

  bool decide(uint32_t now, bool want, uint16_t targetOn, uint16_t period, uint16_t rampMs, uint16_t minOn) {
    if (cycleActive_ && now - cycleStart_ >= cyclePeriod_) cycleActive_ = false;
    if (!cycleActive_) {
      if (!want || targetOn == 0) return false;
      // A pump that was just switched off (a cut pulse) rests before it may start again.
      // Rolling straight from one continuous cycle into the next is not a switch.
      if (!out_ && everCycled_ && now - offAt_ < kShortestGapMs) return false;
      // After a pause, start again from the minimum on-time and grow from there.
      const bool fresh = !everCycled_ || now - cycleStart_ >= 2UL * cyclePeriod_;
      uint16_t on = targetOn;
      if (rampMs > 0) {
        const uint32_t limit = fresh ? minOn : static_cast<uint32_t>(lastOn_) + rampMs;
        if (on > limit) on = static_cast<uint16_t>(limit);
      }
      on = relaySafeOnMs(on, period);
      if (on == 0) return false;
      cycleActive_ = true;
      everCycled_ = true;
      cut_ = false;
      cycleStart_ = now;
      cyclePeriod_ = period;
      cycleOn_ = on;
      lastOn_ = on;
    }
    if (!want) cut_ = true;
    return !cut_ && now - cycleStart_ < cycleOn_;
  }

  uint32_t lastMs_{0};
  uint32_t cycleStart_{0};
  uint32_t usedMs_{0};
  uint32_t offAt_{0};
  uint16_t cyclePeriod_{1000};
  uint16_t cycleOn_{0};
  uint16_t lastOn_{0};
  uint8_t hold_{kHoldDisarmed};
  bool started_{false};
  bool cycleActive_{false};
  bool everCycled_{false};
  bool cut_{false};
  bool out_{false};
  bool resting_{false};
};

} // namespace meth_tune
