#pragma once

// Software watchdogs for the PCB builds, in the automotive way: each one watches one thing
// independently of the logic that is supposed to get it right, and each has one clear
// action. They sit on top of the hardware ones (the task watchdog on both cores, the pump
// keeper on the CAN task's core, the crash hook that switches the outputs off from
// registers, the brownout detector):
//
//   PassTimer     the control pass runs every 20 ms; one later than 100 ms is logged.
//   Heartbeat     the CAN task counts up; if the count stops for 300 ms the pump is held
//                 (the task watchdog resets the board after 2 s).
//   RunLimit      the pump's own on-time kept to the pulse timing's spray budget, counted
//                 from the output, not from the pulse timing that is meant to keep it: on-time
//                 fills it, off-time drains it at the pulse timing's rate (longest spray per
//                 rest). The pulse timing's budget fills over all of its injecting time and
//                 drains only while not injecting, so this one never runs ahead of it. Over
//                 the longest spray plus margin: pump off, latched. With no rest set (no time
//                 limit), 3 minutes of the pump on without a break.
//   OutputCheck   what the pump PWM and the valve pin are set to is read back from the
//                 hardware each pass; a difference for 3 passes running: off, latched.
//   Persist       a condition that has to hold for a while before it counts (a sensor
//                 open for a second, not one noisy reading).
//   plausible*    engine data out of any engine's range is ignored, not used.
//
// Pure logic with no Arduino calls, tested on a PC (test/test_pcb).
#include <stdint.h>

namespace watchdogs {

constexpr uint32_t kPassLateMs = 100;
constexpr uint32_t kHeartbeatStaleMs = 300;
constexpr uint32_t kRunMarginMs = 8000;   // Beyond the longest spray: covers a 5 s pump test just before
constexpr uint32_t kRunNoLimitMs = 180000;
constexpr uint8_t kOutputMismatchPasses = 3;
constexpr uint32_t kHeapLowBytes = 24 * 1024;
constexpr uint32_t kHeapCriticalBytes = 12 * 1024;

inline bool elapsed(uint32_t now, uint32_t start, uint32_t ms) { return static_cast<uint32_t>(now - start) >= ms; }

class PassTimer {
public:
  // Call at the start of each pass. True when this one came late.
  bool pass(uint32_t now) {
    const uint32_t gap = started_ ? now - last_ : 0;
    started_ = true;
    last_ = now;
    if (gap > worstMs_) worstMs_ = gap;
    if (gap > kPassLateMs) { ++late_; return true; }
    return false;
  }
  uint32_t late() const { return late_; }
  uint32_t worstMs() const { return worstMs_; }

private:
  uint32_t last_{0};
  uint32_t late_{0};
  uint32_t worstMs_{0};
  bool started_{false};
};

class Heartbeat {
public:
  // The other task's counter, as read now. True while it is alive.
  bool check(uint32_t now, uint32_t count) {
    if (!seen_ || count != count_) { seen_ = true; count_ = count; changedMs_ = now; }
    alive_ = !elapsed(now, changedMs_, kHeartbeatStaleMs);
    return alive_;
  }
  bool alive() const { return alive_; }

private:
  uint32_t count_{0};
  uint32_t changedMs_{0};
  bool seen_{false};
  bool alive_{true};
};

class RunLimit {
public:
  // The most on-time allowed, from the pulse timing's longest spray (seconds) and its rest
  // (0: no time limit in the pulse timing).
  static uint32_t limitMs(uint16_t maxSprayS, uint16_t restS) {
    return restS == 0 ? kRunNoLimitMs : static_cast<uint32_t>(maxSprayS) * 1000UL + kRunMarginMs;
  }
  // Each pass: whether the pump is on now, with the pulse timing's settings. True once over
  // the limit (it stays true until reset).
  bool update(uint32_t now, bool pumpOn, uint16_t maxSprayS, uint16_t restS) {
    const uint32_t dt = started_ && now - lastMs_ <= 1000 ? now - lastMs_ : 0;
    started_ = true;
    lastMs_ = now;
    limit_ = limitMs(maxSprayS, restS);
    if (pumpOn) {
      sprayMs_ += dt;
    } else if (restS == 0) {
      sprayMs_ = 0;   // No time limit: only an unbroken run counts
    } else {
      const uint32_t drain = dt * maxSprayS / restS;
      sprayMs_ = sprayMs_ > drain ? sprayMs_ - drain : 0;
    }
    if (sprayMs_ >= limit_) tripped_ = true;
    return tripped_;
  }
  uint32_t sprayMs() const { return sprayMs_; }
  uint32_t limit() const { return limit_; }
  bool tripped() const { return tripped_; }
  void reset() { tripped_ = false; sprayMs_ = 0; }

private:
  uint32_t lastMs_{0};
  uint32_t sprayMs_{0};
  uint32_t limit_{kRunNoLimitMs};
  bool started_{false};
  bool tripped_{false};
};

class OutputCheck {
public:
  // What was asked of the pump (PWM count) and the valve, and what the hardware reads back.
  bool update(uint32_t askedDuty, uint32_t readDuty, bool askedValve, bool readValve) {
    const bool differ = askedDuty != readDuty || askedValve != readValve;
    passes_ = differ ? (passes_ < 255 ? passes_ + 1 : passes_) : 0;
    if (passes_ >= kOutputMismatchPasses) tripped_ = true;
    return tripped_;
  }
  bool tripped() const { return tripped_; }
  void reset() { tripped_ = false; passes_ = 0; }

private:
  uint8_t passes_{0};
  bool tripped_{false};
};

class Persist {
public:
  // True once `condition` has held for `forMs` without a break.
  bool update(uint32_t now, bool condition, uint32_t forMs) {
    if (!condition) { holding_ = false; return on_ = false; }
    if (!holding_) { holding_ = true; sinceMs_ = now; }
    return on_ = elapsed(now, sinceMs_, forMs);
  }
  bool on() const { return on_; }

private:
  uint32_t sinceMs_{0};
  bool holding_{false};
  bool on_{false};
};

// Engine data any engine could give: manifold 10-450 kPa (to 50 psi boost), up to 12000 rpm.
inline bool plausibleMap(float kpa) { return kpa >= 10.0f && kpa <= 450.0f; }
inline bool plausibleRpm(uint16_t rpm) { return rpm <= 12000; }

} // namespace watchdogs
