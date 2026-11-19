#pragma once

// Flow from line pressure, for a board with a line pressure transducer (100-250 psi,
// 0.5-4.5 V) on its pressure input, between the pump and the nozzle. No flow meter is
// needed: a nozzle's flow goes with the square root of the pressure across it,
//
//     flow = nozzle flow x sqrt(line pressure / the pressure that flow was measured at),
//
// so with the nozzle measured once (the web page's "measure it" test records both the flow
// and the pressure during it), the pressure gives the flow while spraying. Without a
// transducer, or with the pressure the nozzle was measured at unknown, the flow is the
// nozzle's figure while the valve is open (for the PWM drive, scaled by the square root of
// the duty, as pwm_drive.h does). Flow is counted only while the valve is open.
//
// The meter adds the flow up: fluid used since the tank was last filled, and so what is
// left in it. Pure logic with no Arduino calls, tested on a PC (test/test_pcb).
#include <math.h>
#include <stdint.h>

namespace flow {

constexpr float kMaxOverNozzle = 2.0f;   // The estimate never goes above twice the nozzle figure

// The flow (ml/min) through a nozzle of `nozzleMlMin` at `ratedPsi`, at `psi`.
inline float fromPressure(float nozzleMlMin, float ratedPsi, float psi) {
  if (!(nozzleMlMin > 0.0f) || !(ratedPsi > 0.0f) || !(psi > 0.0f)) return 0.0f;
  float q = nozzleMlMin * sqrtf(psi / ratedPsi);
  if (q > nozzleMlMin * kMaxOverNozzle) q = nozzleMlMin * kMaxOverNozzle;
  return q;
}

struct Pass {
  bool valveOpen{false};
  float dutyPct{0.0f};       // The pump's drive
  bool pwm{false};           // The PWM drive: part duty is part pressure
  float nozzleMlMin{0.0f};   // All nozzles together, at ratedPsi (or at full pump pressure)
  float ratedPsi{0.0f};      // 0: not known
  bool pressureValid{false};
  float linePsi{0.0f};
};

// The flow now, and whether it comes from the pressure (true) or the nozzle figure.
inline float estimate(const Pass &p, bool &fromLine) {
  fromLine = false;
  if (!p.valveOpen || !(p.dutyPct > 0.0f)) return 0.0f;
  if (p.pressureValid && p.ratedPsi > 0.0f) {
    fromLine = true;
    return fromPressure(p.nozzleMlMin, p.ratedPsi, p.linePsi);
  }
  if (p.pwm) return p.nozzleMlMin * sqrtf(p.dutyPct / 100.0f);
  return p.nozzleMlMin;
}

class Meter {
public:
  // Each pass: the flow now (ml/min). Steps over a second long are not counted (a stall).
  void update(uint32_t now, float mlPerMin) {
    if (started_ && now - lastMs_ <= 1000 && mlPerMin > 0.0f) {
      const float ml = mlPerMin * static_cast<float>(now - lastMs_) / 60000.0f;
      usedMl_ += ml;
      driveMl_ += ml;
    }
    started_ = true;
    lastMs_ = now;
    flow_ = mlPerMin;
  }
  float flowMlMin() const { return flow_; }
  float usedMl() const { return usedMl_; }     // Since the tank was filled
  float driveMl() const { return driveMl_; }   // Since power-up
  void setUsed(float ml) { usedMl_ = ml > 0.0f ? ml : 0.0f; }
  void filled() { usedMl_ = 0.0f; }
  // What is left of a tank of tankMl, never below 0; -1 for no tank size set.
  float leftMl(float tankMl) const {
    if (!(tankMl > 0.0f)) return -1.0f;
    const float left = tankMl - usedMl_;
    return left > 0.0f ? left : 0.0f;
  }

private:
  uint32_t lastMs_{0};
  float usedMl_{0.0f};
  float driveMl_{0.0f};
  float flow_{0.0f};
  bool started_{false};
};

// The solenoid valve's timing. By default it opens with the pump and shuts the moment it
// stops. With a lead time, after the pump has been off a while (the line's pressure bled
// away), the valve stays shut for the lead while the pump builds pressure, so the first of
// the spray is fully atomised instead of dribbling. Between pulses the shut valve keeps the
// line's pressure, so no lead is needed.
class Valve {
public:
  static constexpr uint32_t kBledAfterMs = 1000;
  // Each pass: whether the pump runs now; returns whether the valve should be open.
  bool update(uint32_t now, bool pumpOn, uint32_t leadMs) {
    if (pumpOn && !pumpWasOn_) {
      const bool bled = !everOn_ || now - offSinceMs_ >= kBledAfterMs;
      openFromMs_ = bled ? now + leadMs : now;
      everOn_ = true;
    }
    if (!pumpOn && pumpWasOn_) offSinceMs_ = now;
    pumpWasOn_ = pumpOn;
    open_ = pumpOn && static_cast<int32_t>(now - openFromMs_) >= 0;
    return open_;
  }
  bool open() const { return open_; }

private:
  uint32_t offSinceMs_{0};
  uint32_t openFromMs_{0};
  bool pumpWasOn_{false};
  bool everOn_{false};
  bool open_{false};
};

// The pump's current while running at full drive, averaged over runs: what this pump
// normally draws, to see it change (a clogging filter draws more, a worn pump less).
class PumpCurrent {
public:
  static constexpr uint32_t kSettleMs = 400;   // Past the start-up surge
  void update(uint32_t now, float dutyPct, float amps) {
    const bool full = dutyPct >= 99.5f;
    if (full && !wasFull_) fullSinceMs_ = now;
    wasFull_ = full;
    if (!full || now - fullSinceMs_ < kSettleMs) return;
    average_ = samples_ == 0 ? amps : average_ + (amps - average_) * 0.02f;
    if (samples_ < 0xFFFF) ++samples_;
    if (amps > peak_) peak_ = amps;
  }
  bool known() const { return samples_ >= 25; }   // Half a second of running
  float average() const { return average_; }
  float peak() const { return peak_; }

private:
  uint32_t fullSinceMs_{0};
  float average_{0.0f};
  float peak_{0.0f};
  uint16_t samples_{0};
  bool wasFull_{false};
};

} // namespace flow
