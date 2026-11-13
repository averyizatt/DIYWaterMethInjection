#pragma once

// The ESP32-S3 DevKitC-1's own RGB LED: what the controller is doing, at a glance, for
// someone looking at the board instead of the dash (owner, November 2026: "red is bad,
// green is good"):
//   green            armed and ready: it sprays when boost comes
//   blue             spraying; brighter as the pump duty rises
//   purple           pump test
//   dim white        not armed
//   yellow           cannot spray just now, though nothing is broken: nothing heard on
//                    CAN yet, no fresh data from the MicroSquirt, resting after its
//                    longest spray, or waiting for an air temperature it was told to need
//   red, flashing    something needs fixing: tank low, CAN not working, held off after
//                    repeated crashes, the overboost latch, a setting it refuses, or the
//                    analog inputs dead
//   cyan, flashing   new firmware arriving from the dash
// At every start it shows red, green and blue in turn, under a second, so an LED that
// takes its colours in another order is caught on the bench and never read the wrong
// way round in the car.
//
// Pure logic, tested on a PC (test/test_status_light); main_esp32.cpp writes the colour.
#include <stdint.h>

namespace status_light {

enum class Show : uint8_t { LampTest, Updating, Spraying, PumpTest, Fault, Waiting, Disarmed, Ready };

// What the control pass found, in the LED's terms.
struct Facts {
  bool updating{false};   // Firmware arriving from the dash
  bool pumpTest{false};   // A pump test is running
  float dutyPct{0.0f};    // What the pump is driven at now; 0 = off
  bool fault{false};      // Something needs fixing (see above)
  bool waiting{false};    // Cannot spray just now, nothing broken
  bool armed{false};
};

struct Color {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};
inline bool same(Color a, Color b) { return a.r == b.r && a.g == b.g && a.b == b.b; }

constexpr uint32_t kLampStepMs = 300;   // Each colour of the start-up check
constexpr uint32_t kFlashMs = 250;      // Flashing: 250 ms on, 250 ms off
constexpr uint32_t kRefreshMs = 1000;   // Written again even unchanged: pump noise can upset the LED
constexpr uint8_t kBright = 40;         // Of 255: easily seen on the board, not dazzling
constexpr uint8_t kDim = 6;             // Not armed
constexpr uint8_t kSprayLow = 12;       // Spraying at the lowest duty...
constexpr uint8_t kSprayHigh = 62;      // ...and at 100%

// Most important first: a spray shows even with a fault waiting (the fault shows the
// moment it stops), and a fault shows over anything that only waits.
inline Show classify(uint32_t sinceStartMs, const Facts &f) {
  if (sinceStartMs < 3 * kLampStepMs) return Show::LampTest;
  if (f.updating) return Show::Updating;
  if (f.dutyPct > 0.0f) return f.pumpTest ? Show::PumpTest : Show::Spraying;
  if (f.fault) return Show::Fault;
  if (f.waiting) return Show::Waiting;
  return f.armed ? Show::Ready : Show::Disarmed;
}

// Blue for a spray, in eleven steps of brightness (every 10% of duty), so the LED is
// not rewritten on every small change of duty.
inline uint8_t sprayLevel(float dutyPct) {
  if (!(dutyPct > 0.0f)) dutyPct = 0.0f;
  if (dutyPct > 100.0f) dutyPct = 100.0f;
  const uint8_t step = static_cast<uint8_t>(dutyPct / 10.0f + 0.5f);
  return static_cast<uint8_t>(kSprayLow + (kSprayHigh - kSprayLow) * step / 10);
}

inline Color colorOf(Show show, uint32_t sinceStartMs, float dutyPct) {
  const bool flashOn = (sinceStartMs / kFlashMs) % 2 == 0;
  switch (show) {
  case Show::LampTest: {
    const uint32_t step = sinceStartMs / kLampStepMs;
    if (step == 0) return {kBright, 0, 0};
    if (step == 1) return {0, kBright, 0};
    return {0, 0, kBright};
  }
  case Show::Updating: return flashOn ? Color{0, kBright, kBright} : Color{0, 0, 0};
  case Show::Spraying: return {0, 0, sprayLevel(dutyPct)};
  case Show::PumpTest: return {kBright, 0, kBright};
  case Show::Fault: return flashOn ? Color{kBright, 0, 0} : Color{0, 0, 0};
  case Show::Waiting: return {kBright, static_cast<uint8_t>(kBright * 2 / 3), 0};
  case Show::Disarmed: return {kDim, kDim, kDim};
  case Show::Ready: return {0, kBright, 0};
  }
  return {0, 0, 0};
}

// Decides when to write: on a change, and once a second regardless (a write takes about
// 30 microseconds; a stray pulse from the pump's switching could have changed the LED).
class Light {
public:
  bool due(uint32_t now, Color c) {
    if (written_ && same(c, last_) && now - writtenMs_ < kRefreshMs) return false;
    written_ = true;
    last_ = c;
    writtenMs_ = now;
    return true;
  }

private:
  Color last_{0, 0, 0};
  uint32_t writtenMs_{0};
  bool written_{false};
};

} // namespace status_light
