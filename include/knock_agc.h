#pragma once

// Automatic gain for the knock amplifier (ESP32 board, with a digital potentiometer
// setting the amplifier's gain).
//
// Engine noise at the knock sensor grows many times over between idle and high RPM. A
// fixed gain is a compromise: quiet at idle (few ADC counts, coarse measurement) or
// clipping at speed (no measurement at all). This keeps the normal engine noise in the
// lower part of the range the amplifier can reach, with room above it for knock:
//
//   - Slow: every half second it looks at the typical size of the signal. Too small:
//     gain up 1.5 dB. Too large: down 1.5 dB. In between (a wide band): nothing, so it
//     does not hunt.
//   - Fast: the moment the signal touches either limit, gain down 3 dB at once.
//   - Remembered per 1000 RPM: coming back to an engine speed it has seen, it goes
//     straight to the gain that suited it, instead of creeping there during a pull.
//   - Knock itself is not allowed to steer the gain: passes the detector calls loud are
//     left out of the average.
//
// Sizes are in millivolts at the ADC pin, and everything is measured against the
// circuit as it is: the lowest and highest voltage the amplifier can put on the pin
// (Circuit::railLow / railHigh) and where the signal actually rests. "Room" is the
// distance from the resting level to the nearer limit; the typical swing is kept
// between 16 % and 40 % of it, which leaves knock at least 2.5 times the normal noise
// before anything clips. An amplifier that rests off-centre, or reaches the pin through
// a divider, needs no other change than those two limits.
//
// Every gain change is reported as a ratio, and the detector's learned levels are
// rescaled by it (knock_dsp::Detector::rescale), so "louder than normal" keeps meaning
// the same thing across a change. The pass after a change is discarded.
//
// It also notices three things about the hardware:
//   overloaded   clipping at the lowest gain: the amplifier is too hot for this engine
//   silent       nothing at the highest gain with the engine turning: sensor not heard
//   ineffective  gain commanded up or down by 6 dB and the signal did not follow: the
//                potentiometer is not fitted or not working. Gain is then left alone.
//
// Pure logic, no Arduino calls; tested on a PC (test/test_knock_agc).
#include <stdint.h>

namespace knock_agc {

constexpr uint8_t kRpmBins = 8;            // Gain remembered per 1000 RPM
constexpr uint8_t kUnknown = 255;
constexpr uint16_t kRpmHysteresis = 150;   // RPM past a boundary before the range changes

constexpr float kClipMargin = 0.02f;       // Within this share of the range of a limit: clipping
constexpr float kClipMarginMinMv = 15.0f;
constexpr float kLowShare = 0.16f;         // Typical swing wanted between these shares of the room...
constexpr float kHighShare = 0.40f;        // ...which leaves at least 2.5x above it for knock
constexpr uint8_t kWindowPasses = 25;      // Passes averaged: half a second at 20 ms
constexpr float kNotch = 1.19f;            // 1.5 dB
constexpr float kAttack = 0.70f;           // 3 dB down on clipping

constexpr float kSilentShare = 0.009f;     // "Nothing" at the highest gain
constexpr uint16_t kSilentRpm = 2000;      // ...only meaningful with the engine at speed
constexpr uint8_t kSilentWindows = 3;
constexpr float kEffectMinShare = 0.035f;  // Enough signal to judge whether gain changes show
constexpr float kEffectCommanded = 2.0f;   // 6 dB asked for...
constexpr float kEffectSeen = 1.19f;       // ...and less than 1.5 dB seen

// The amplifier: gain at the potentiometer's bottom and top positions, in between in a
// straight line (a non-inverting stage with the potentiometer as its feedback
// resistor), and the lowest and highest voltage it can put on the ADC pin.
struct Circuit {
  float lowGain{1.0f};
  float highGain{11.0f};
  uint8_t topStep{99};
  float railLow{10.0f};
  float railHigh{1005.0f};
};

inline float gainAt(const Circuit &c, uint8_t step) {
  if (step > c.topStep) step = c.topStep;
  return c.lowGain + (c.highGain - c.lowGain) * step / c.topStep;
}

inline uint8_t stepFor(const Circuit &c, float gain) {
  if (!(gain > c.lowGain)) return 0;   // Also catches NaN.
  if (gain >= c.highGain) return c.topStep;
  return static_cast<uint8_t>((gain - c.lowGain) / (c.highGain - c.lowGain) * c.topStep + 0.5f);
}

// One control pass (20 ms): the lowest and highest the signal went since the last
// pass, and where it rests. rest 0 (not known): taken as midway between the limits.
struct Sample {
  float low{500.0f};
  float high{500.0f};
  float rest{0.0f};
  uint16_t rpm{0};
  bool rpmValid{false};
  bool knocking{false};   // The detector called this pass loud: not "normal" noise
};

class Controller {
public:
  void begin(const Circuit &circuit, uint8_t step = 0) {
    *this = Controller();
    circuit_ = circuit;
    step_ = step > circuit.topStep ? circuit.topStep : step;
    measure(0.0f);
  }

  // Returns true when the potentiometer has to move to step(); ratioX256() is then the
  // new gain over the old one.
  bool update(const Sample &in) {
    if (discard_) {            // The pass after a change spans both gains.
      discard_ = false;
      return false;
    }
    if (ineffective_) return false;

    measure(in.rest);
    const float span = circuit_.railHigh - circuit_.railLow;
    const float margin = span * kClipMargin > kClipMarginMinMv ? span * kClipMargin : kClipMarginMinMv;
    const bool clipped = in.low <= circuit_.railLow + margin || in.high >= circuit_.railHigh - margin;
    const float peak = in.high > in.low ? (in.high - in.low) * 0.5f : 0.0f;

    // Clipping with no gain left to take off. (A leaky count: about five clipped passes
    // close together raise it, and it is clear again within a second of the clipping
    // stopping.)
    if (clipped && step_ == 0) overload_ = overload_ > 36 ? 40 : static_cast<uint8_t>(overload_ + 4);
    else if (overload_ > 0) --overload_;
    if (overload_ >= 20) overloaded_ = true;
    else if (overload_ < 8) overloaded_ = false;

    uint8_t want = step_;
    const uint8_t bin = in.rpmValid ? binFor(in.rpm) : bin_;
    const bool moved = bin != bin_;
    if (moved) {
      memory_[bin_] = step_;
      bin_ = bin;
      if (memory_[bin] != kUnknown) want = memory_[bin];
      restartWindow();
    }
    if (clipped) {
      const uint8_t down = lower(kAttack);
      if (down < want) want = down;
      restartWindow();
    } else if (!moved) {
      if (!in.knocking) {
        sum_ += peak;
        ++count_;
      }
      if (count_ >= kWindowPasses) want = judgeWindow(in);
    }

    if (want == step_) return false;
    ratioX256_ = static_cast<uint16_t>(gainAt(circuit_, want) / gainAt(circuit_, step_) * 256.0f + 0.5f);
    step_ = want;
    memory_[bin_] = want;
    discard_ = true;
    return true;
  }

  uint8_t step() const { return step_; }
  float gain() const { return gainAt(circuit_, step_); }
  uint16_t ratioX256() const { return ratioX256_; }
  bool settling() const { return discard_; }      // This pass's burst should not be judged
  float typicalPeak() const { return typical_; }
  float room() const { return room_; }            // Resting level to the nearer limit
  float lowPeak() const { return room_ * kLowShare; }
  float highPeak() const { return room_ * kHighShare; }
  bool overloaded() const { return overloaded_; }
  bool silent() const { return silent_; }
  bool ineffective() const { return ineffective_; }

private:
  // How much room the signal has, from where it rests.
  void measure(float rest) {
    const float low = circuit_.railLow, high = circuit_.railHigh;
    if (!(rest > low && rest < high)) rest = 0.5f * (low + high);
    const float room = rest - low < high - rest ? rest - low : high - rest;
    room_ = room > 1.0f ? room : 1.0f;
  }

  void restartWindow() {
    sum_ = 0.0f;
    count_ = 0;
    runDirection_ = 0;
  }

  uint8_t binFor(uint16_t rpm) const {
    const uint8_t raw = rpm >= kRpmBins * 1000U ? kRpmBins - 1 : static_cast<uint8_t>(rpm / 1000U);
    if (raw > bin_) return rpm >= (bin_ + 1U) * 1000U + kRpmHysteresis ? raw : bin_;
    if (raw < bin_) return static_cast<uint32_t>(rpm) + kRpmHysteresis < bin_ * 1000U ? raw : bin_;
    return bin_;
  }

  // The step for a gain `factor` times the present one; always at least one step away.
  uint8_t lower(float factor) const {
    if (step_ == 0) return 0;
    const uint8_t target = stepFor(circuit_, gain() * factor);
    return target < step_ ? target : static_cast<uint8_t>(step_ - 1);
  }
  uint8_t raise(float factor) const {
    if (step_ >= circuit_.topStep) return circuit_.topStep;
    const uint8_t target = stepFor(circuit_, gain() * factor);
    return target > step_ ? target : static_cast<uint8_t>(step_ + 1);
  }

  uint8_t judgeWindow(const Sample &in) {
    typical_ = sum_ / count_;
    sum_ = 0.0f;
    count_ = 0;
    memory_[bin_] = step_;

    // Nothing to hear at full gain with the engine at speed.
    if (step_ == circuit_.topStep && typical_ < room_ * kSilentShare && in.rpmValid && in.rpm >= kSilentRpm) {
      if (silentWindows_ < 255) ++silentWindows_;
    } else {
      silentWindows_ = 0;
    }
    silent_ = silentWindows_ >= kSilentWindows;

    int8_t direction = 0;
    if (typical_ < lowPeak() && step_ < circuit_.topStep) direction = 1;
    else if (typical_ > highPeak() && step_ > 0) direction = -1;

    // Has the signal followed the gain over this run of changes in one direction?
    const float floor = room_ * 0.001f;
    const float seenNow = typical_ > floor ? typical_ : floor;
    if (direction != 0 && direction == runDirection_) {
      const float now = gain();
      const float commanded = direction > 0 ? now / runStartGain_ : runStartGain_ / now;
      const float seen = direction > 0 ? seenNow / runStartPeak_ : runStartPeak_ / seenNow;
      if (runStartPeak_ >= room_ * kEffectMinShare && commanded >= kEffectCommanded && seen < kEffectSeen) {
        ineffective_ = true;
        return step_;
      }
    } else {
      runDirection_ = direction;
      runStartGain_ = gain();
      runStartPeak_ = seenNow;
    }

    if (direction > 0) return raise(kNotch);
    if (direction < 0) return lower(1.0f / kNotch);
    return step_;
  }

  Circuit circuit_{};
  uint8_t step_{0};
  uint8_t bin_{0};
  uint8_t memory_[kRpmBins] = {kUnknown, kUnknown, kUnknown, kUnknown, kUnknown, kUnknown, kUnknown, kUnknown};
  float sum_{0.0f};
  uint8_t count_{0};
  float typical_{0.0f};
  float room_{1.0f};
  uint16_t ratioX256_{256};
  uint8_t overload_{0};
  uint8_t silentWindows_{0};
  int8_t runDirection_{0};
  float runStartGain_{1.0f};
  float runStartPeak_{1.0f};
  bool discard_{false};
  bool overloaded_{false};
  bool silent_{false};
  bool ineffective_{false};
};

} // namespace knock_agc
