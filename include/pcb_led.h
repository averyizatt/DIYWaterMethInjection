#pragma once

// The XIAO's one orange LED, showing the same states as the DevKitC-1 board's RGB LED
// (status_light.h decides which), as patterns instead of colours:
//   on, steady            armed and ready
//   fast flicker (10 Hz)  spraying, or a pump test
//   flashing (2 Hz)       something needs fixing (see the dash or the web page)
//   slow blink (1 s)      cannot spray just now, nothing broken (waiting for ECU data...)
//   short blip every 2 s  not armed
//   double blink          new firmware arriving
// Lit for the first 0.6 s after every start. Pure logic, tested on a PC (test/test_pcb).
#include <stdint.h>

#include "status_light.h"

namespace pcb_led {

inline bool lit(status_light::Show show, uint32_t sinceStartMs) {
  using status_light::Show;
  const uint32_t t = sinceStartMs;
  switch (show) {
  case Show::LampTest: return true;
  case Show::Updating: { const uint32_t p = t % 1000; return p < 100 || (p >= 200 && p < 300); }
  case Show::Spraying:
  case Show::PumpTest: return (t / 50) % 2 == 0;
  case Show::Fault: return (t / 250) % 2 == 0;
  case Show::Waiting: return (t / 1000) % 2 == 0;
  case Show::Disarmed: return t % 2000 < 100;
  case Show::Ready: return true;
  }
  return false;
}

} // namespace pcb_led
