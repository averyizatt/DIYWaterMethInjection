#pragma once

// Knock listening for the ESP32 board: every sample, all the time.
//
// The Nano can only look at one 3.3 ms burst in every 20 ms (knock_band.h) and so
// misses most single knock events. The ESP32 has the speed to do it properly:
//
//   FrontEnd      runs on every ADC sample (38 kHz). It removes the resting level and
//                 passes the signal through three filters:
//                   knock   a band-pass on the cylinder's knock frequency (about 6 kHz
//                           for a 96 mm bore)
//                   second  a band-pass on the cylinder's next resonance, 1.66 times
//                           higher (about 9.9 kHz). Real knock rings both; most other
//                           noises do not
//                   wide    everything from 1 to 17 kHz, to say how much of the sound
//                           is at the knock frequency
//                 and adds up the energy of each in half-millisecond frames.
//   PassAnalyser  every 20 ms slides a window the length of one knock (about 30
//                 degrees of crank rotation: 2.5 ms at 2000 RPM, 1 ms at 5000) over
//                 those frames, across the joins between passes, and takes the
//                 loudest position: the level there, its share of the wide band, the
//                 second resonance there, and the typical level of the rest of the
//                 pass (the median, which one knock does not move).
//   Detector      decides. The rules are the Nano detector's, in floating point:
//                 louder than the level learned for this RPM (times a multiplier,
//                 plus an offset), and narrow-band. Two things are new:
//                   - it must also stand out from the rest of its own 20 ms. Knock is
//                     a burst; a noise floor that has simply risen is not.
//                   - the second resonance, when it is also above its own learned
//                     level, counts as proof: a strong hit confirmed that way raises
//                     the warning at once instead of waiting for a second hit.
//
// The filters are designed for the sample rate that is actually measured, not the one
// asked for, so the knock frequency means what it says whatever the ADC really does.
//
// Pure logic, no Arduino calls; tested on a PC (test/test_knock_dsp). Signal sizes are
// in 10-bit ADC counts (fractions kept); the detector's level, learned level and
// threshold are in half counts, 255 = full scale, as the Nano's are and as the CAN
// messages carry them.
#include <math.h>
#include <stdint.h>

namespace knock_dsp {

constexpr float kTwoPi = 6.2831853f;
constexpr float kFrameSeconds = 0.0005f;     // Energy is added up in frames this long
constexpr uint8_t kWindowFrames = 6;         // Longest window: 3 ms
constexpr uint8_t kHistoryFrames = kWindowFrames - 1;
constexpr uint8_t kMaxFrames = 96;           // Frames kept per pass (40 arrive in 20 ms)
constexpr float kSecondModeRatio = 1.659f;   // Second cylinder resonance over the first
constexpr float kWideLowHz = 1000.0f;
constexpr float kWideHighHz = 17000.0f;
constexpr uint8_t kSections = 3;             // Band-pass sections in a row: 18 dB per octave each side
constexpr float kUsableNyquist = 0.45f;      // Filters stay below this share of the sample rate
constexpr float kBiasSeconds = 0.05f;        // How quickly the resting level is followed
constexpr float kMinCrest = 1.5f;            // A hit stands this far above the rest of its pass
constexpr uint8_t kRpmBins = 8;              // Normal level learned per 1000 RPM

// ---------------------------------------------------------------------------
// Filters
// ---------------------------------------------------------------------------
struct Biquad {
  float b0{0.0f}, b1{0.0f}, b2{0.0f}, a1{0.0f}, a2{0.0f};
  float z1{0.0f}, z2{0.0f};
  inline float step(float x) {
    const float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
};

// Band-pass with a gain of exactly 1 at its centre.
inline Biquad bandPass(float sampleRate, float centre, float q) {
  const float w = kTwoPi * centre / sampleRate;
  const float alpha = sinf(w) / (2.0f * q);
  const float a0 = 1.0f + alpha;
  Biquad f;
  f.b0 = alpha / a0;
  f.b1 = 0.0f;
  f.b2 = -alpha / a0;
  f.a1 = -2.0f * cosf(w) / a0;
  f.a2 = (1.0f - alpha) / a0;
  return f;
}

inline Biquad highPass(float sampleRate, float corner) {
  const float w = kTwoPi * corner / sampleRate;
  const float alpha = sinf(w) / 1.4142136f;   // Q = 0.707
  const float c = cosf(w), a0 = 1.0f + alpha;
  Biquad f;
  f.b0 = (1.0f + c) / 2.0f / a0;
  f.b1 = -(1.0f + c) / a0;
  f.b2 = f.b0;
  f.a1 = -2.0f * c / a0;
  f.a2 = (1.0f - alpha) / a0;
  return f;
}

inline Biquad lowPass(float sampleRate, float corner) {
  const float w = kTwoPi * corner / sampleRate;
  const float alpha = sinf(w) / 1.4142136f;
  const float c = cosf(w), a0 = 1.0f + alpha;
  Biquad f;
  f.b0 = (1.0f - c) / 2.0f / a0;
  f.b1 = (1.0f - c) / a0;
  f.b2 = f.b0;
  f.a1 = -2.0f * c / a0;
  f.a2 = (1.0f - alpha) / a0;
  return f;
}

// Mean square of each filter's output over one frame, in counts squared.
struct Frame {
  float knock{0.0f};
  float second{0.0f};
  float wide{0.0f};
};

// ---------------------------------------------------------------------------
// Every sample
// ---------------------------------------------------------------------------
class FrontEnd {
public:
  // bandwidth: width of the knock band at -3 dB. The second-resonance band has the
  // same width and is used only when it fits under the sample rate.
  void configure(float sampleRate, float centreHz, float bandwidthHz) {
    sampleRate_ = sampleRate;
    const float top = sampleRate * kUsableNyquist;
    if (centreHz > top * 0.9f) centreHz = top * 0.9f;
    if (centreHz < 1000.0f) centreHz = 1000.0f;
    if (bandwidthHz < 300.0f) bandwidthHz = 300.0f;
    if (bandwidthHz > centreHz) bandwidthHz = centreHz;
    centre_ = centreHz;
    bandwidth_ = bandwidthHz;
    // Three identical sections in a row: together they are 3 dB down at 0.509 * centre / Q.
    knock_[0] = knock_[1] = knock_[2] = bandPass(sampleRate, centreHz, 0.509f * centreHz / bandwidthHz);
    const float second = centreHz * kSecondModeRatio;
    secondActive_ = second + bandwidthHz * 0.5f < top;
    if (secondActive_) second_[0] = second_[1] = second_[2] = bandPass(sampleRate, second, 0.509f * second / bandwidthHz);
    wideHigh_ = highPass(sampleRate, kWideLowHz);
    wideLow_ = lowPass(sampleRate, kWideHighHz < top ? kWideHighHz : top);
    const float length = sampleRate * kFrameSeconds + 0.5f;
    frameLength_ = length < 4.0f ? 4 : static_cast<uint16_t>(length);
    biasAlpha_ = 1.0f / (sampleRate * kBiasSeconds);
    sumKnock_ = sumSecond_ = sumWide_ = 0.0f;
    count_ = 0;
  }

  // One sample, in 10-bit counts. Returns true when it completed a frame.
  inline bool push(float sample, Frame &out) {
    if (!biasKnown_) {
      bias_ = sample;
      biasKnown_ = true;
    }
    bias_ += (sample - bias_) * biasAlpha_;
    const float x = sample - bias_;
    const float knock = knock_[2].step(knock_[1].step(knock_[0].step(x)));
    const float wide = wideLow_.step(wideHigh_.step(x));
    sumKnock_ += knock * knock;
    sumWide_ += wide * wide;
    if (secondActive_) {
      const float second = second_[2].step(second_[1].step(second_[0].step(x)));
      sumSecond_ += second * second;
    }
    if (++count_ < frameLength_) return false;
    out.knock = sumKnock_ / count_;
    out.second = sumSecond_ / count_;
    out.wide = sumWide_ / count_;
    sumKnock_ = sumSecond_ = sumWide_ = 0.0f;
    count_ = 0;
    return true;
  }

  float bias() const { return bias_; }
  float sampleRate() const { return sampleRate_; }
  float centreHz() const { return centre_; }
  float bandwidthHz() const { return bandwidth_; }
  bool secondActive() const { return secondActive_; }
  uint16_t frameLength() const { return frameLength_; }

private:
  Biquad knock_[kSections], second_[kSections], wideHigh_, wideLow_;
  float sampleRate_{38462.0f}, centre_{6000.0f}, bandwidth_{1500.0f};
  float bias_{512.0f}, biasAlpha_{0.0005f};
  float sumKnock_{0.0f}, sumSecond_{0.0f}, sumWide_{0.0f};
  uint16_t count_{0}, frameLength_{19};
  bool biasKnown_{false}, secondActive_{false};
};

// ---------------------------------------------------------------------------
// Every 20 ms
// ---------------------------------------------------------------------------
// Knock rings for about 30 degrees of crank rotation, so the window that fits it gets
// shorter as the engine turns faster. Too long a window dilutes a short ring with the
// noise around it. In frames of half a millisecond; RPM unknown: 2 ms.
inline uint8_t windowFramesFor(uint16_t rpm) {
  if (rpm == 0) return 4;
  const uint16_t frames = static_cast<uint16_t>((10000U + rpm / 2U) / rpm);
  return frames < 2 ? 2 : frames > kWindowFrames ? kWindowFrames : static_cast<uint8_t>(frames);
}

struct Summary {
  float level{0.0f};        // Knock-band amplitude over the loudest window, counts
  float share{0.0f};        // Knock band's share of the wide band there, 0..1
  float second{0.0f};       // Second-resonance amplitude there, counts
  float background{0.0f};   // Typical knock-band amplitude over the pass (median frame), counts
  uint8_t frames{0};
  bool secondActive{false};
  bool valid{false};
};

class PassAnalyser {
public:
  void add(const Frame &frame) {
    if (count_ < kMaxFrames) {
      frames_[kHistoryFrames + count_++] = frame;
    } else if (frame.knock > frames_[kHistoryFrames + kMaxFrames - 1].knock) {
      frames_[kHistoryFrames + kMaxFrames - 1] = frame;   // A long pass: keep the louder end.
    }
  }

  Summary finish(bool secondActive, uint8_t windowFrames = kWindowFrames) {
    Summary s;
    s.secondActive = secondActive;
    if (count_ == 0) return s;
    if (windowFrames < 1) windowFrames = 1;
    if (windowFrames > kWindowFrames) windowFrames = kWindowFrames;
    // Every window that ends in this pass. Windows reach back into the last
    // frames of the pass before, so a knock across the join is whole in one of them.
    float best = -1.0f, bestSecond = 0.0f, bestWide = 0.0f;
    for (uint8_t end = kHistoryFrames; end < kHistoryFrames + count_; ++end) {
      float knock = 0.0f, second = 0.0f, wide = 0.0f;
      for (uint8_t back = 0; back < windowFrames; ++back) {
        const Frame &f = frames_[end - back];
        knock += f.knock;
        second += f.second;
        wide += f.wide;
      }
      if (knock > best) {
        best = knock;
        bestSecond = second;
        bestWide = wide;
      }
    }
    // The median frame: what the pass is like apart from any burst in it.
    float sorted[kMaxFrames];
    for (uint8_t i = 0; i < count_; ++i) {
      const float value = frames_[kHistoryFrames + i].knock;
      uint8_t at = i;
      while (at > 0 && sorted[at - 1] > value) {
        sorted[at] = sorted[at - 1];
        --at;
      }
      sorted[at] = value;
    }
    const float median = (count_ & 1) ? sorted[count_ / 2] : 0.5f * (sorted[count_ / 2 - 1] + sorted[count_ / 2]);

    s.level = sqrtf(2.0f * best / windowFrames);        // Amplitude of a sine with this power
    s.second = sqrtf(2.0f * bestSecond / windowFrames);
    s.share = bestWide > 0.0f ? best / bestWide : 0.0f;
    if (s.share > 1.0f) s.share = 1.0f;
    s.background = sqrtf(2.0f * median);
    s.frames = count_;
    s.valid = true;

    for (uint8_t h = 0; h < kHistoryFrames; ++h) frames_[h] = frames_[count_ + h];
    count_ = 0;
    return s;
  }

private:
  Frame frames_[kHistoryFrames + kMaxFrames];
  uint8_t count_{0};
};

// ---------------------------------------------------------------------------
// The decision
// ---------------------------------------------------------------------------
struct Settings {
  float offset{8.0f};          // Added to the learned level (half counts)
  float multiplier{2.5f};      // The learned level is multiplied by this
  float minShare{90.0f / 255.0f};
  float gain{1.0f};            // Software gain on the level
  uint16_t holdMs{300};        // A warning stays up this long after the last hit
};

class Detector {
public:
  // One pass. armed: the engine is in the speed and load range where knock matters.
  // rpm 0 means unknown; all engine speeds then share one learned level.
  void update(uint32_t now, const Summary &heard, const Settings &s, bool armed, uint16_t rpm) {
    const uint8_t bin = rpm >= kRpmBins * 1000U ? kRpmBins - 1 : static_cast<uint8_t>(rpm / 1000U);
    level_ = limit(heard.level * 0.5f * s.gain);
    share_ = heard.share;
    second_ = limit(heard.second * 0.5f * s.gain);

    const float base = baseline_[bin];
    float threshold = base * s.multiplier + s.offset;
    if (threshold > 255.0f) threshold = 255.0f;
    if (threshold < 1.0f) threshold = 1.0f;
    threshold_ = threshold;
    baselineNow_ = base > 255.0f ? 255.0f : base;
    learned_ = seen_[bin] >= kLearnPasses;

    const bool loud = level_ >= threshold_;
    const bool narrow = heard.share >= s.minShare;
    // A burst, not a noise floor that has simply come up.
    const bool standsOut = heard.level >= kMinCrest * heard.background;
    // The second resonance is ringing too.
    confirmed_ = heard.secondActive && learned_ && second_ >= secondBaseline_[bin] * s.multiplier + s.offset;
    const bool hit = armed && learned_ && loud && narrow && standsOut;
    const bool strong = hit && level_ >= threshold_ * 1.6f;

    if (hit) {
      // Two hits close together make a warning. A strong hit is critical at once only
      // when it is also unmistakable: clearly narrow-band, or confirmed by the second
      // resonance. A lone borderline hit proves nothing.
      const bool paired = haveHit_ && now - lastHitMs_ <= kPairMs;
      const bool unmistakable = heard.share >= s.minShare + kClearShareMargin || confirmed_;
      if (paired || (strong && unmistakable)) {
        warningUntil_ = now + s.holdMs;
        warning_ = true;
      }
      if (strong && (paired || unmistakable)) {
        criticalUntil_ = now + s.holdMs;
        critical_ = true;
      }
      lastHitMs_ = now;
      haveHit_ = true;
    } else if (!(loud && narrow) || !learned_) {
      // Learn the normal level only from passes that are not knock: a loud narrow-band
      // pass is never folded into "normal", armed or not.
      if (seen_[bin] == 0) {
        baseline_[bin] = level_;
        secondBaseline_[bin] = second_;
      } else {
        baseline_[bin] += (level_ - baseline_[bin]) / 32.0f;
        secondBaseline_[bin] += (second_ - secondBaseline_[bin]) / 32.0f;
      }
      if (seen_[bin] < 255) ++seen_[bin];
    }
    if (warning_ && static_cast<int32_t>(now - warningUntil_) >= 0) warning_ = false;
    if (critical_ && static_cast<int32_t>(now - criticalUntil_) >= 0) critical_ = false;
    if (haveHit_ && now - lastHitMs_ > 60000UL) haveHit_ = false;
  }

  // The amplifier's gain changed by this ratio. Everything learned is in ADC units at
  // the old gain, so it moves with the gain: "louder than normal" means the same
  // before and after (knock_agc.h).
  void rescale(float ratio) {
    for (uint8_t bin = 0; bin < kRpmBins; ++bin) {
      baseline_[bin] = limit(baseline_[bin] * ratio);
      secondBaseline_[bin] = limit(secondBaseline_[bin] * ratio);
    }
  }

  void reset() { *this = Detector(); }

  float level() const { return level_; }
  float share() const { return share_; }
  float second() const { return second_; }
  float baseline() const { return baselineNow_; }
  float threshold() const { return threshold_; }
  bool learned() const { return learned_; }
  bool confirmed() const { return confirmed_; }
  bool warning() const { return warning_; }
  bool critical() const { return critical_; }

private:
  static constexpr uint8_t kLearnPasses = 25;
  static constexpr uint16_t kPairMs = 600;
  static constexpr float kClearShareMargin = 50.0f / 255.0f;

  static float limit(float value) { return value > 255.0f ? 255.0f : value < 0.0f ? 0.0f : value; }

  float baseline_[kRpmBins] = {0.0f};        // Learned level per RPM bin
  float secondBaseline_[kRpmBins] = {0.0f};  // The same for the second resonance
  uint8_t seen_[kRpmBins] = {0};
  uint32_t lastHitMs_{0};
  uint32_t warningUntil_{0};
  uint32_t criticalUntil_{0};
  float level_{0.0f};
  float share_{0.0f};
  float second_{0.0f};
  float baselineNow_{0.0f};
  float threshold_{1.0f};
  bool learned_{false};
  bool confirmed_{false};
  bool haveHit_{false};
  bool warning_{false};
  bool critical_{false};
};

// A value for one byte of a CAN message.
inline uint8_t toByte(float value) {
  if (!(value > 0.0f)) return 0;
  return value >= 254.5f ? 255 : static_cast<uint8_t>(value + 0.5f);
}

} // namespace knock_dsp
