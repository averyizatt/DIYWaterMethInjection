#pragma once

// The PCB's own checks on the pump, from what only this board can measure: the pump's
// current, the 12 V supply and (with a sensor fitted) the line pressure. Both PCB builds
// run it every control pass. Pure logic with no Arduino calls, tested on a PC (test/test_pcb).
//
//   Short circuit   one reading over kShortAmps with the pump on: off at once.
//   Over-current    over the limit (kOverAmps, or a lower setting) for kOverMs at full drive
//                   (a seizing pump, a wiring fault).
//   Not connected   under kOpenAmps after kOpenMs running steadily (a broken wire, a blown fuse).
//   Switch stuck on current with the pump off, kStuckMs long: the MOSFET has failed shorted.
//                   The solenoid valve, shut whenever the pump should be off, is then the
//                   only thing between the pump and the nozzle.
//   No pressure     line pressure under the set minimum after kPressureRiseMs at full drive:
//                   an empty tank, a dead pump, a line off (only with a pressure sensor).
//   Over-pressure   over the set maximum for kOverPressureMs: a blocked nozzle or filter.
// These latch: the pump stays off until faults are cleared (the dash's Clear faults, the
// web page's, or a power cycle). The supply checks do not latch:
//   Supply low      under kSupplyLowV (cranking, a failing battery): the pump would run
//                   weak and spray coarsely. Released above kSupplyLowV + 0.5 V.
//   Supply high     over kSupplyHighV for a second (a charging fault): released under it - 0.5 V.
#include <stdint.h>

namespace pump_guard {

enum Fault : uint8_t {
  kNone = 0,
  kShort = 1,
  kOverCurrent = 2,
  kNotConnected = 3,
  kStuckOn = 4,
  kNoPressure = 5,
  kOverPressure = 6,
};
enum Hold : uint8_t { kHoldNone = 0, kSupplyLow = 1, kSupplyHigh = 2 };

constexpr float kShortAmps = 25.0f;
constexpr float kOverAmps = 10.0f;          // The board and pump are rated 10 A (its fuse 15 A)
constexpr uint32_t kOverMs = 300;
constexpr float kOpenAmps = 0.3f;
constexpr uint32_t kOpenMs = 600;
constexpr float kStuckAmps = 1.0f;
constexpr uint32_t kStuckMs = 500;
constexpr uint32_t kSettleOffMs = 300;      // After the pump stops, its current dies away first
constexpr uint32_t kPressureRiseMs = 1500;
constexpr uint32_t kOverPressureMs = 500;
constexpr float kSupplyLowV = 10.5f;
constexpr float kSupplyHighV = 16.5f;
constexpr uint32_t kSupplyHighMs = 1000;

inline const char *name(uint8_t fault) {
  switch (fault) {
  case kShort: return "pump short circuit";
  case kOverCurrent: return "pump over-current";
  case kNotConnected: return "pump not connected";
  case kStuckOn: return "pump switch stuck on";
  case kNoPressure: return "no line pressure";
  case kOverPressure: return "line pressure too high";
  default: return "none";
  }
}

struct Pass {
  float dutyPct{0.0f};          // What the pump is driven at now
  bool ampsValid{false};
  float amps{0.0f};
  bool supplyValid{false};
  float supplyV{0.0f};
  bool pressureFitted{false};   // A line pressure sensor is set up
  bool pressureValid{false};
  float pressurePsi{0.0f};
  float minPsi{0.0f};           // 0: the no-pressure check is off
  float maxPsi{0.0f};           // 0: the over-pressure check is off
  // Duty from which the pump counts as running steadily, for the current checks: full drive
  // for the pulse drive; the PWM drive's lowest duty for it. The pressure check is at full drive only.
  float steadyFromPct{99.5f};
  // The over-current limit: a setting for a smaller pump, never above the board's kOverAmps.
  float overAmps{kOverAmps};
};

class Guard {
public:
  // One control pass. Returns the latched fault (kNone while the pump may run).
  uint8_t update(uint32_t now, const Pass &p) {
    const bool on = p.dutyPct > 0.0f;
    const bool full = p.dutyPct >= p.steadyFromPct;
    const bool fullPressure = p.dutyPct >= 99.5f;
    if (on && !wasOn_) onSinceMs_ = now;
    if (!on && wasOn_) offSinceMs_ = now;
    if (full && !wasFull_) fullSinceMs_ = now;
    if (fullPressure && !wasFullPressure_) fullPressureSinceMs_ = now;
    wasOn_ = on;
    wasFull_ = full;
    wasFullPressure_ = fullPressure;
    if (!started_) { offSinceMs_ = now; started_ = true; }

    if (p.ampsValid) {
      if (on && p.amps >= kShortAmps) trip(kShort);
      const float limit = p.overAmps > 0.0f && p.overAmps < kOverAmps ? p.overAmps : kOverAmps;
      overMs_ = full && p.amps >= limit ? overMs_ + step(now) : 0;
      if (overMs_ >= kOverMs) trip(kOverCurrent);
      if (full && now - fullSinceMs_ >= kOpenMs && p.amps < kOpenAmps) trip(kNotConnected);
      stuckMs_ = !on && now - offSinceMs_ >= kSettleOffMs && p.amps >= kStuckAmps ? stuckMs_ + step(now) : 0;
      if (stuckMs_ >= kStuckMs) trip(kStuckOn);
    }
    if (p.pressureFitted && p.pressureValid) {
      if (p.minPsi > 0.0f && fullPressure && now - fullPressureSinceMs_ >= kPressureRiseMs && p.pressurePsi < p.minPsi) trip(kNoPressure);
      overPressureMs_ = p.maxPsi > 0.0f && p.pressurePsi > p.maxPsi ? overPressureMs_ + step(now) : 0;
      if (overPressureMs_ >= kOverPressureMs) trip(kOverPressure);
    }
    if (p.supplyValid) {
      if (p.supplyV < kSupplyLowV) hold_ = kSupplyLow;
      else if (hold_ == kSupplyLow && p.supplyV >= kSupplyLowV + 0.5f) hold_ = kHoldNone;
      highMs_ = p.supplyV > kSupplyHighV ? highMs_ + step(now) : 0;
      if (highMs_ >= kSupplyHighMs) hold_ = kSupplyHigh;
      else if (hold_ == kSupplyHigh && p.supplyV < kSupplyHighV - 0.5f) hold_ = kHoldNone;
    }
    lastMs_ = now;
    return fault_;
  }

  uint8_t fault() const { return fault_; }
  uint8_t hold() const { return hold_; }
  // The pump may run now (no latched fault, supply in range).
  bool allows() const { return fault_ == kNone && hold_ == kHoldNone; }
  // The first latched fault since the last clear; count of trips since start-up.
  uint16_t trips() const { return trips_; }
  void clear() {
    fault_ = kNone;
    overMs_ = stuckMs_ = overPressureMs_ = 0;
  }

private:
  uint32_t step(uint32_t now) const { return lastMs_ == 0 || now - lastMs_ > 1000 ? 0 : now - lastMs_; }
  void trip(uint8_t f) {
    if (fault_ == kNone) {
      fault_ = f;
      if (trips_ < 0xFFFF) ++trips_;
    }
  }

  uint32_t lastMs_{0};
  uint32_t onSinceMs_{0};
  uint32_t offSinceMs_{0};
  uint32_t fullSinceMs_{0};
  uint32_t fullPressureSinceMs_{0};
  uint32_t overMs_{0};
  uint32_t stuckMs_{0};
  uint32_t overPressureMs_{0};
  uint32_t highMs_{0};
  uint16_t trips_{0};
  uint8_t fault_{kNone};
  uint8_t hold_{kHoldNone};
  bool wasOn_{false};
  bool wasFull_{false};
  bool wasFullPressure_{false};
  bool started_{false};
};

} // namespace pump_guard
