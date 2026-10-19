#pragma once

// The spectrum of the knock signal, and the engine's real knock frequency found from it.
//
// The knock frequency the firmware starts with comes from the cylinder bore. It is a
// good first guess and rarely exactly right: the real resonance depends on the chamber
// and the gas temperature, and can be several hundred hertz away. A filter sitting
// beside the resonance hears knock weaker than it is. So the firmware looks at the
// spectrum and moves the filter to where this engine actually rings.
//
//   Blocks   runs on every ADC sample, next to the knock filters. It cuts the signal
//            into overlapping blocks of 128 samples (3.3 ms: about one knock) and takes
//            the spectrum of each: 600 spectra a second, 300 Hz apart in frequency. It
//            keeps a noise floor for every frequency (what that frequency normally
//            carries) and, for each 20 ms pass, the one block that stood furthest above
//            the floor in the range where the knock frequency could be.
//   Finder   every 20 ms is given that block. When the detector says the pass was
//            "notable" - under load, clearly louder than normal in the knock band,
//            narrow-band and a burst (knock_dsp.h) - it notes at which frequency the
//            block peaks. When enough of those agree, that is the knock frequency.
//
// Why only notable passes: a spectrum has peaks for many reasons. Valve and piston
// noise ring the block's own resonances, an alternator or the pump's PWM puts steady
// lines in it. Those are either present every cycle (so they are the "normal" the
// detector has learned, not louder than it) or steady (so not bursts). What is left -
// bursts under boost that rise above normal near the expected frequency - is the
// cylinder ringing: incipient knock, and knock. The filter can therefore be pulled to
// the real resonance but not to an unrelated peak, and no further than a fifth either
// side of the bore's frequency.
//
// It also reports a steady tone inside the search range (a pump or alternator line),
// read off the noise floor, where bursts do not show and steady tones do.
//
// Pure logic, no Arduino calls; tested on a PC (test/test_knock_spectrum). On the board
// the transform is ESP-DSP's FFT (knock_dsp.h says when ESP-DSP is used).
#include <math.h>
#include <stdint.h>

#include "knock_dsp.h"

namespace knock_spectrum {

constexpr uint16_t kBlock = 128;          // Samples in one spectrum
constexpr uint16_t kHop = 64;             // A new block every this many samples
constexpr uint8_t kBins = 64;             // Frequencies in one spectrum
constexpr float kTwoPi = 6.2831853f;
constexpr float kSearchBelow = 0.80f;     // The knock frequency is looked for between
constexpr float kSearchAbove = 1.25f;     // these multiples of the bore's frequency
constexpr float kFloorRate = 1.0f / 256;  // The noise floor follows over about 0.4 s
constexpr float kFloorClip = 4.0f;        // ...and a burst counts as at most this times the floor
constexpr float kToneRatio = 8.0f;        // A steady line this far above its neighbours is a tone

// One block's spectrum and the noise floor at that moment (power per frequency).
struct Snapshot {
  float power[kBins];
  float floor[kBins];
  float binHz{300.0f};
  float standing{0.0f};   // How far the block stood above the floor in the search range
  bool valid{false};
};

// ---------------------------------------------------------------------------
// Every sample
// ---------------------------------------------------------------------------
class Blocks {
public:
  Blocks() {
    for (uint16_t n = 0; n < kBlock; ++n) {
      window_[n] = 0.5f - 0.5f * cosf(kTwoPi * n / kBlock);   // Hann
      uint16_t reversed = 0;
      for (uint8_t bit = 0; bit < 7; ++bit) {
        if (n & (1U << bit)) reversed |= 1U << (6 - bit);
      }
      reversed_[n] = static_cast<uint8_t>(reversed);
    }
    for (uint16_t n = 0; n < kBlock / 2; ++n) {
      cos_[n] = cosf(kTwoPi * n / kBlock);
      sin_[n] = sinf(kTwoPi * n / kBlock);
    }
    for (uint8_t b = 0; b < kBins; ++b) floor_[b] = 0.0f;
  }

  void configure(float sampleRate, float expectedHz) {
#if KNOCK_USE_ESP_DSP
    // ESP-DSP's twiddle table, once. Without it the portable transform is used.
    static bool ready = dsps_fft2r_init_fc32(nullptr, kBlock) == ESP_OK;
    if (!ready) knock_dsp::espDspOn() = false;
#endif
    binHz_ = sampleRate / kBlock;
    const float low = expectedHz * kSearchBelow / binHz_, high = expectedHz * kSearchAbove / binHz_;
    searchLow_ = low < 2.0f ? 2 : static_cast<uint8_t>(low);
    searchHigh_ = high > kBins - 2 ? kBins - 2 : static_cast<uint8_t>(high + 1.0f);
    if (searchHigh_ <= searchLow_) searchHigh_ = searchLow_ + 1;
    floorKnown_ = false;
    best_.valid = false;
    best_.standing = 0.0f;
    filled_ = 0;
  }

  // One sample with the resting level already taken off. Returns true when it
  // completed a block.
  bool push(float x) {
    samples_[filled_++] = x;
    if (filled_ < kBlock) return false;
    transform();
    // Keep the second half: the next block overlaps this one by half.
    for (uint16_t n = 0; n < kBlock - kHop; ++n) samples_[n] = samples_[n + kHop];
    filled_ = kBlock - kHop;
    return true;
  }

  // The block that stood furthest above the floor since the last call.
  bool takeLoudest(Snapshot &out) {
    if (!best_.valid) return false;
    out = best_;
    best_.valid = false;
    best_.standing = 0.0f;
    return true;
  }

  // The transform of one block (already windowed): re and im, kBlock values each, in
  // frequency order. Public so the start-up check can compare the two ways.
  void transformOf(const float *windowed, float *re, float *im, bool useEspDsp) const {
#if KNOCK_USE_ESP_DSP
    if (useEspDsp) {
      static float data[2 * kBlock] __attribute__((aligned(16)));
      for (uint16_t n = 0; n < kBlock; ++n) {
        data[2 * n] = windowed[n];
        data[2 * n + 1] = 0.0f;
      }
      dsps_fft2r_fc32(data, kBlock);
      dsps_bit_rev_fc32(data, kBlock);
      for (uint16_t n = 0; n < kBlock; ++n) {
        re[n] = data[2 * n];
        im[n] = data[2 * n + 1];
      }
      return;
    }
#else
    (void)useEspDsp;
#endif
    for (uint16_t n = 0; n < kBlock; ++n) {
      re[reversed_[n]] = windowed[n];
      im[n] = 0.0f;
    }
    for (uint16_t size = 2; size <= kBlock; size <<= 1) {
      const uint16_t half = size >> 1, stride = kBlock / size;
      for (uint16_t start = 0; start < kBlock; start += size) {
        for (uint16_t k = 0; k < half; ++k) {
          const float c = cos_[k * stride], s = sin_[k * stride];
          const uint16_t a = start + k, b = a + half;
          const float tr = re[b] * c + im[b] * s, ti = im[b] * c - re[b] * s;
          re[b] = re[a] - tr;
          im[b] = im[a] - ti;
          re[a] += tr;
          im[a] += ti;
        }
      }
    }
  }
  float windowAt(uint16_t n) const { return window_[n]; }

  const float *power() const { return power_; }
  const float *floor() const { return floor_; }
  float binHz() const { return binHz_; }
  uint8_t searchLow() const { return searchLow_; }
  uint8_t searchHigh() const { return searchHigh_; }

private:
  void transform() {
    float windowed[kBlock], re[kBlock], im[kBlock];
    for (uint16_t n = 0; n < kBlock; ++n) windowed[n] = samples_[n] * window_[n];
    transformOf(windowed, re, im, knock_dsp::espDspOn());
    // Power, scaled so a steady sine of amplitude A reads A*A/2 at its frequency.
    const float scale = 8.0f / (static_cast<float>(kBlock) * kBlock);
    float above = 0.0f, floorSum = 0.0f;
    for (uint8_t b = 0; b < kBins; ++b) {
      const float p = (re[b] * re[b] + im[b] * im[b]) * scale;
      power_[b] = p;
      if (!floorKnown_) {
        floor_[b] = p;
      } else {
        // A burst moves the floor as if it were only a few times the floor: the floor
        // is what the frequency normally carries, steady tones included.
        const float limit = floor_[b] * kFloorClip;
        floor_[b] += ((p < limit ? p : limit) - floor_[b]) * kFloorRate;
      }
      if (b >= searchLow_ && b <= searchHigh_) {
        above += p;
        floorSum += floor_[b];
      }
    }
    floorKnown_ = true;
    const float standing = floorSum > 0.0f ? above / floorSum : 0.0f;
    if (!best_.valid || standing > best_.standing) {
      for (uint8_t b = 0; b < kBins; ++b) {
        best_.power[b] = power_[b];
        best_.floor[b] = floor_[b];
      }
      best_.binHz = binHz_;
      best_.standing = standing;
      best_.valid = true;
    }
  }

  float samples_[kBlock];
  float window_[kBlock];
  float cos_[kBlock / 2], sin_[kBlock / 2];
  uint8_t reversed_[kBlock];
  float power_[kBins];
  float floor_[kBins];
  Snapshot best_;
  float binHz_{300.5f};
  uint16_t filled_{0};
  uint8_t searchLow_{14}, searchHigh_{26};
  bool floorKnown_{false};
};

// Start-up check on the board: ESP-DSP against the portable code, on a made-up knock
// signal (two resonances and noise) through a knock filter, the energy sum and the
// transform. When they differ by more than rounding, ESP-DSP is turned off and the
// portable code is used from then on. `blocks` must be configured (the FFT table).
// Returns whether ESP-DSP is in use afterwards.
inline bool checkEspDsp(const Blocks &blocks, float sampleRate) {
  if (!knock_dsp::espDspOn()) return false;
  alignas(16) float signal[kBlock];
  alignas(16) float fast[kBlock];
  alignas(16) float slow[kBlock];
  uint32_t seed = 12345U;
  for (uint16_t n = 0; n < kBlock; ++n) {
    seed = seed * 1664525U + 1013904223U;
    const float noise = static_cast<float>((seed >> 9) & 1023U) / 1023.0f - 0.5f;
    signal[n] = 40.0f * sinf(kTwoPi * 6000.0f * n / sampleRate) + 15.0f * sinf(kTwoPi * 9900.0f * n / sampleRate) + 5.0f * noise;
  }
  bool same = true;
  // The filter, a block at once against sample by sample.
  knock_dsp::Biquad quick = knock_dsp::bandPass(sampleRate, 6000.0f, 4.0f), reference = quick;
  quick.run(signal, fast, kBlock);
  float worst = 0.0f, peak = 1e-6f;
  for (uint16_t n = 0; n < kBlock; ++n) {
    slow[n] = reference.step(signal[n]);
    const float gap = fabsf(fast[n] - slow[n]), size = fabsf(slow[n]);
    if (gap > worst) worst = gap;
    if (size > peak) peak = size;
  }
  same = same && worst <= peak * 1e-3f;
  // The energy of a frame, from wherever frames start in a chunk (not only aligned places).
  const uint16_t starts[] = {0, 1, 3, 19, 57}, lengths[] = {kBlock, 19, 37, 19, 5};
  for (uint8_t i = 0; i < 5; ++i) {
    float sum = 0.0f;
    for (uint16_t n = 0; n < lengths[i] && starts[i] + n < kBlock; ++n) sum += signal[starts[i] + n] * signal[starts[i] + n];
    const uint16_t length = starts[i] + lengths[i] <= kBlock ? lengths[i] : kBlock - starts[i];
    same = same && fabsf(knock_dsp::energy(signal + starts[i], length) - sum) <= sum * 1e-4f + 1e-3f;
  }
  // The transform.
  float re1[kBlock], im1[kBlock], re2[kBlock], im2[kBlock];
  for (uint16_t n = 0; n < kBlock; ++n) slow[n] = signal[n] * blocks.windowAt(n);
  blocks.transformOf(slow, re1, im1, true);
  blocks.transformOf(slow, re2, im2, false);
  worst = 0.0f;
  peak = 1e-6f;
  for (uint16_t b = 0; b < kBins; ++b) {
    const float gap = fabsf(re1[b] - re2[b]) + fabsf(im1[b] - im2[b]);
    const float size = fabsf(re2[b]) + fabsf(im2[b]);
    if (gap > worst) worst = gap;
    if (size > peak) peak = size;
  }
  same = same && worst <= peak * 1e-3f;
  if (!same) knock_dsp::espDspOn() = false;
  return same;
}

// The frequency of the highest point between two bins, from the bin and its neighbours.
inline float peakHz(const float *values, uint8_t bin, float binHz) {
  const float left = values[bin - 1], mid = values[bin], right = values[bin + 1];
  const float denominator = left - 2.0f * mid + right;
  float shift = denominator < 0.0f ? 0.5f * (left - right) / denominator : 0.0f;
  if (shift > 0.5f) shift = 0.5f;
  if (shift < -0.5f) shift = -0.5f;
  return (bin + shift) * binHz;
}

// The value at a frequency, between the two nearest lines of a spectrum.
inline float valueAtHz(const float *values, float binHz, float hz) {
  if (!(binHz > 0.0f)) return 0.0f;
  const float position = hz / binHz;
  if (position <= 0.0f) return values[0];
  if (position >= kBins - 1) return values[kBins - 1];
  const uint8_t bin = static_cast<uint8_t>(position);
  const float part = position - bin;
  return values[bin] + (values[bin + 1] - values[bin]) * part;
}

// A steady tone inside the search range, read off the noise floor: 0 when there is none.
inline float steadyToneHz(const Snapshot &s, uint8_t searchLow, uint8_t searchHigh) {
  if (!s.valid || searchHigh < searchLow + 4) return 0.0f;
  // The typical floor of the range: its median.
  float sorted[kBins] = {0.0f};
  uint8_t count = 0;
  for (uint8_t b = searchLow; b <= searchHigh && count < kBins; ++b) {
    uint8_t at = count++;
    while (at > 0 && sorted[at - 1] > s.floor[b]) {
      sorted[at] = sorted[at - 1];
      --at;
    }
    sorted[at] = s.floor[b];
  }
  const float typical = sorted[count / 2];
  uint8_t top = searchLow;
  for (uint8_t b = searchLow; b <= searchHigh; ++b) {
    if (s.floor[b] > s.floor[top]) top = b;
  }
  if (!(typical > 0.0f) || s.floor[top] < kToneRatio * typical) return 0.0f;
  return peakHz(s.floor, top, s.binHz);
}

// The same, for the search range around an expected knock frequency.
inline float steadyToneHz(const Snapshot &s, float expectedHz) {
  if (!s.valid || !(s.binHz > 0.0f)) return 0.0f;
  int low = static_cast<int>(expectedHz * kSearchBelow / s.binHz), high = static_cast<int>(expectedHz * kSearchAbove / s.binHz + 1.0f);
  if (low < 2) low = 2;
  if (high > kBins - 2) high = kBins - 2;
  return steadyToneHz(s, static_cast<uint8_t>(low), static_cast<uint8_t>(high));
}

// ---------------------------------------------------------------------------
// Every 20 ms
// ---------------------------------------------------------------------------
constexpr float kGridHz = 100.0f;          // Votes are counted in steps this wide
constexpr uint8_t kCells = 80;             // ...from 2 kHz to 10 kHz
constexpr float kGridStartHz = 2000.0f;
constexpr float kMinPeak = 4.0f;           // A vote's peak stands this far above its floor
constexpr uint16_t kVotesToDecide = 40;
constexpr uint16_t kVotesKept = 400;       // Older votes fade beyond this
constexpr float kAgreement = 0.6f;         // Share of the votes within 350 Hz of the winner
constexpr float kRunnerUp = 0.5f;          // A second peak this strong beside the winner: no decision
constexpr float kDeadbandHz = 150.0f;      // The filter is not moved for less than this
constexpr float kMaxStepHz = 300.0f;       // ...nor by more than this at once

class Finder {
public:
  // expectedHz: the bore's frequency. The answer stays within the search range around it.
  void begin(float expectedHz) {
    expected_ = expectedHz;
    clearVotes();
    estimate_ = 0.0f;
    agreement_ = 0.0f;
  }

  float expectedHz() const { return expected_; }

  // One pass's loudest block. notable: the detector's verdict on the pass (see above).
  void add(const Snapshot &s, bool notable) {
    if (!s.valid || !notable) return;
    const float lowHz = expected_ * kSearchBelow, highHz = expected_ * kSearchAbove;
    uint8_t low = static_cast<uint8_t>(lowHz / s.binHz), high = static_cast<uint8_t>(highHz / s.binHz + 1.0f);
    if (low < 1) low = 1;
    if (high > kBins - 2) high = kBins - 2;
    if (high <= low) return;
    // Where this block stands furthest above what is normal.
    float excess[kBins];
    uint8_t top = low;
    for (uint8_t b = low - 1; b <= high + 1; ++b) {
      excess[b] = s.power[b] - s.floor[b];
      if (b >= low && b <= high && excess[b] > excess[top]) top = b;
    }
    if (!(s.floor[top] > 0.0f) || s.power[top] < kMinPeak * s.floor[top]) return;
    const float hz = peakHz(excess, top, s.binHz);
    if (hz < lowHz || hz > highHz) return;
    const int cell = static_cast<int>((hz - kGridStartHz) / kGridHz);
    if (cell < 0 || cell >= kCells) return;
    votes_[cell] += 1.0f;
    total_ += 1.0f;
    ++fresh_;
    if (total_ > kVotesKept) {
      for (uint8_t c = 0; c < kCells; ++c) votes_[c] *= 0.5f;
      total_ *= 0.5f;
    }
  }

  // Call after add(). Returns true when the knock filter should move, with the
  // frequency to move it to: toward where the votes agree, a limited step at a time.
  bool decide(float currentHz, float &newHz) {
    if (fresh_ < kVotesToDecide) return false;
    fresh_ = 0;
    // The winner: the 300 Hz stretch holding the most votes.
    int best = 1;
    float bestMass = -1.0f;
    for (int c = 1; c < kCells - 1; ++c) {
      const float mass = votes_[c - 1] + votes_[c] + votes_[c + 1];
      if (mass > bestMass) {
        bestMass = mass;
        best = c;
      }
    }
    float mass = 0.0f, weighted = 0.0f;
    for (int c = best - 3; c <= best + 3; ++c) {
      if (c < 0 || c >= kCells) continue;
      mass += votes_[c];
      weighted += votes_[c] * (kGridStartHz + (c + 0.5f) * kGridHz);
    }
    agreement_ = total_ > 0.0f ? mass / total_ : 0.0f;
    if (!(mass > 0.0f) || agreement_ < kAgreement) return false;   // The votes do not agree: stay.
    // Two places ringing about equally: not for this to choose between.
    for (int c = 1; c < kCells - 1; ++c) {
      if (c >= best - 5 && c <= best + 5) continue;
      if (votes_[c - 1] + votes_[c] + votes_[c + 1] > kRunnerUp * bestMass) return false;
    }
    estimate_ = weighted / mass;
    const float away = estimate_ - currentHz;
    if (away > -kDeadbandHz && away < kDeadbandHz) return false;
    newHz = currentHz + (away > kMaxStepHz ? kMaxStepHz : away < -kMaxStepHz ? -kMaxStepHz : away);
    // The filter moves, and with it which passes are notable: start counting afresh.
    clearVotes();
    return true;
  }

  float estimateHz() const { return estimate_; }      // 0 until the votes have agreed once
  float agreement() const { return agreement_; }
  uint16_t votes() const { return static_cast<uint16_t>(total_); }

private:
  void clearVotes() {
    for (uint8_t c = 0; c < kCells; ++c) votes_[c] = 0.0f;
    total_ = 0.0f;
    fresh_ = 0;
  }

  float votes_[kCells];
  float total_{0.0f};
  float expected_{6000.0f};
  float estimate_{0.0f};
  float agreement_{0.0f};
  uint16_t fresh_{0};
};

} // namespace knock_spectrum
