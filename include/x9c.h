#pragma once

// X9C104 digital potentiometer (100 kohm, 100 wiper positions) as the gain control of
// the knock amplifier. Three control wires: CS, INC and U/D.
//
// How the chip works, and what this driver does about it:
//   - It is stepped, not written: each falling edge of INC while CS is low moves the
//     wiper one position, up or down as U/D says. The position cannot be read back.
//     So the driver first walks it down past the bottom (it stops there, it does not
//     wrap round) and counts from zero after that.
//   - Releasing CS while INC is high saves the position in the chip's memory, which
//     lasts 100,000 saves. Gain changes many times a minute, so CS is always released
//     with INC low: nothing is ever saved.
//   - The chip runs on 5 V. Its three inputs accept the ESP32's 3.3 V signals; it sends
//     nothing back, so no 5 V reaches the ESP32.
//
// The pin handling is a template argument so the same sequence runs against a model of
// the chip on a PC (test/test_knock_agc). An Io provides:
//   void select(bool on);   CS low when on
//   void inc(bool high);
//   void up(bool up);       U/D high when up
//   void wait();            a few microseconds
#include <stdint.h>

namespace x9c {

constexpr uint8_t kTopStep = 99;       // Positions 0..99
constexpr uint8_t kHomePulses = 110;   // More than the whole travel: always reaches the bottom

// `count` steps in one direction. Leaves the chip released with nothing saved.
template <class Io> void pulses(Io &io, bool up, uint8_t count) {
  if (count == 0) return;
  io.inc(true);              // High before selecting: the first falling edge is the first step.
  io.up(up);
  io.wait();
  io.select(true);
  io.wait();
  for (uint8_t n = 0; n < count; ++n) {
    io.inc(false);           // The step happens here.
    io.wait();
    if (n + 1 < count) {
      io.inc(true);
      io.wait();
    }
  }
  io.select(false);          // INC is low: released without saving.
  io.wait();
  io.inc(true);              // Idle level; ignored while the chip is not selected.
}

class Pot {
public:
  // Walks the wiper to the bottom, wherever it was. After this the position is known.
  template <class Io> void home(Io &io) {
    pulses(io, false, kHomePulses);
    step_ = 0;
  }

  template <class Io> void moveTo(Io &io, uint8_t target) {
    if (target > kTopStep) target = kTopStep;
    if (target > step_) pulses(io, true, static_cast<uint8_t>(target - step_));
    else if (target < step_) pulses(io, false, static_cast<uint8_t>(step_ - target));
    step_ = target;
  }

  // Home, then back to the given position: clears any step that was miscounted.
  template <class Io> void resync(Io &io, uint8_t target) {
    home(io);
    moveTo(io, target);
  }

  uint8_t step() const { return step_; }

private:
  uint8_t step_{0};
};

} // namespace x9c
