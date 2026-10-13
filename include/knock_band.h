#pragma once

// Knock listening for a small 8-bit controller: what one burst of fast samples says
// about the engine's knock frequency, and when that counts as knock.
//
// Knock is a short ringing of the cylinder at a frequency set by the bore: about
// 6 kHz for a 96 mm bore. One burst is 64 samples at 19.2 kHz (3.3 ms). It is measured
// three ways:
//   level  how strong the signal is in the knock band (five 300 Hz wide bins)
//   share  how much of the burst's total energy sits in that band; ringing is narrow,
//          valvetrain and piston noise are spread across all frequencies
//   peak   the largest swing, to see clipping and to set the amplifier gain
// The detector learns the normal level at each engine speed, because an engine is
// simply louder at high RPM, and calls a hit only when the level is well above that
// AND the energy is concentrated at the knock frequency.
//
// Without a crank or ignition signal the bursts are not timed to combustion, so a
// single knock event is caught only if a burst happens to overlap it. Sustained
// knock is caught within a fraction of a second. Everything here is integer maths
// with no Arduino calls, so it is tested on a PC (test/test_knock_band).
#include <stdint.h>

namespace knock_band {

constexpr uint8_t kSamples = 64;
constexpr uint16_t kSampleRateHz = 19231;  // 16 MHz / 64 / 13: ADC free-running, prescaler 64
constexpr uint8_t kMinCenterBin = 8;       // 2.4 kHz
constexpr uint8_t kMaxCenterBin = 26;      // 7.8 kHz
constexpr uint8_t kRpmBins = 8;            // Noise level learned per 1000 RPM
// Filter state limit: above any full-scale sine in the supported bins, and low enough
// that the power sum below stays inside 32 bits.
constexpr int16_t kStateLimit = 22000;

// 2*cos(2*pi*bin/64) in Q14 for bins 6..28. On the Nano the table is kept in flash:
// a constant table otherwise takes 46 bytes of its 2 KB of RAM.
#if defined(ARDUINO_ARCH_AVR)
#include <avr/pgmspace.h>
#define KNOCK_BAND_TABLE PROGMEM
#define KNOCK_BAND_READ(address) static_cast<int16_t>(pgm_read_word(address))
#else
#define KNOCK_BAND_TABLE
#define KNOCK_BAND_READ(address) (*(address))
#endif
inline int16_t coefficient(uint8_t bin) {
  static const int16_t table[] KNOCK_BAND_TABLE = {27246, 25330, 23170, 20788, 18205, 15447, 12540, 9512, 6393, 3212, 0, -3212,
                                                   -6393, -9512, -12540, -15447, -18205, -20788, -23170, -25330, -27246, -28899, -30274};
  return KNOCK_BAND_READ(&table[bin - 6]);
}

// Knock frequency for a cylinder bore: f = 1800 / (pi * bore_mm) kHz (first resonance).
inline uint16_t knockHzForBore(uint8_t boreMm) {
  return boreMm ? static_cast<uint16_t>(572958UL / boreMm) : 0;
}

inline uint8_t centerBinFor(uint16_t hz) {
  const uint32_t bin = (static_cast<uint32_t>(hz) * kSamples + kSampleRateHz / 2) / kSampleRateHz;
  return bin < kMinCenterBin ? kMinCenterBin : bin > kMaxCenterBin ? kMaxCenterBin : static_cast<uint8_t>(bin);
}

inline uint16_t isqrt32(uint32_t value) {
  uint32_t root = 0, bit = 1UL << 30;
  while (bit > value) bit >>= 2;
  while (bit) {
    if (value >= root + bit) {
      value -= root + bit;
      root = (root >> 1) + bit;
    } else {
      root >>= 1;
    }
    bit >>= 2;
  }
  return static_cast<uint16_t>(root);
}

struct Burst {
  uint16_t bias{0};     // Mean of the samples: the amplifier's resting level, in ADC counts
  uint16_t peak{0};     // Largest swing away from the bias, in ADC counts
  uint8_t level{0};     // Knock-band amplitude, in half ADC counts (255 = full scale)
  uint8_t share{0};     // Knock-band share of the burst's energy, 0..255 (255 = all of it)
  uint8_t railed{0};    // Samples at either end of the ADC range
};

// Measure one burst of 10-bit samples. halfWidth is the number of bins each side of
// the centre (2 = five bins, 1.5 kHz wide).
inline Burst analyze(const uint16_t *samples, uint8_t centerBin, uint8_t halfWidth = 2) {
  Burst out;
  uint32_t sum = 0;
  for (uint8_t i = 0; i < kSamples; ++i) {
    sum += samples[i];
    if (samples[i] <= 4 || samples[i] >= 1019) ++out.railed;
  }
  const int16_t bias = static_cast<int16_t>((sum + kSamples / 2) / kSamples);
  out.bias = static_cast<uint16_t>(bias);

  uint32_t total = 0;  // Energy of the whole burst
  for (uint8_t i = 0; i < kSamples; ++i) {
    const int16_t d = static_cast<int16_t>(samples[i]) - bias;
    const uint16_t swing = static_cast<uint16_t>(d < 0 ? -d : d);
    if (swing > out.peak) out.peak = swing;
    total += static_cast<uint32_t>(static_cast<int32_t>(d) * d);
  }

  // Goertzel filter per bin on half-scale samples, so the 16-bit state holds any sine
  // the ADC can deliver; anything wilder is clamped at kStateLimit.
  uint32_t band = 0;
  for (uint8_t bin = centerBin - halfWidth; bin <= centerBin + halfWidth; ++bin) {
    const int16_t coeff = coefficient(bin);
    int16_t s1 = 0, s2 = 0;
    for (uint8_t i = 0; i < kSamples; ++i) {
      const int16_t x = static_cast<int16_t>((static_cast<int16_t>(samples[i]) - bias) / 2);
      int32_t s0 = x + ((static_cast<int32_t>(coeff) * s1) >> 14) - s2;
      if (s0 > kStateLimit) s0 = kStateLimit;
      if (s0 < -kStateLimit) s0 = -kStateLimit;
      s2 = s1;
      s1 = static_cast<int16_t>(s0);
    }
    const int32_t cross = ((static_cast<int32_t>(coeff) * s1) >> 14) * s2;
    const int32_t power = static_cast<int32_t>(s1) * s1 + static_cast<int32_t>(s2) * s2 - cross;
    if (power > 0) {
      const uint32_t add = static_cast<uint32_t>(power);
      band = band > 0x3FFFFFFFUL - add ? 0x3FFFFFFFUL : band + add;
    }
  }
  band *= 4;  // Undo the half-scale: band is now |X|^2 summed over the bins.

  // A sine of amplitude A exactly on a bin gives |X|^2 = (A * N / 2)^2 and total = N * A^2 / 2.
  const uint16_t amplitude = static_cast<uint16_t>(isqrt32(band) / (kSamples / 2));
  out.level = amplitude / 2 > 255 ? 255 : static_cast<uint8_t>(amplitude / 2);
  if (total > 0) {
    const uint32_t scaled = (band / (kSamples / 2)) / ((total + 254) / 255);  // 255 * 2 * band / (N * total)
    out.share = scaled > 255 ? 255 : static_cast<uint8_t>(scaled);
  }
  return out;
}

struct Settings {
  uint8_t offset{8};          // Added to the learned level
  uint8_t multiplierX10{25};  // Learned level is multiplied by this / 10
  uint8_t minShare{90};       // Knock band must hold at least this much of the energy (0..255)
  uint8_t gainX16{16};        // Software gain on the level, 16 = 1.0
  uint16_t holdMs{300};       // A warning stays up this long after the last hit
};

class Detector {
public:
  // One burst. armed: the engine is in the speed and load range where knock matters.
  // rpm 0 means unknown; all engine speeds then share one learned level.
  void update(uint32_t now, const Burst &burst, const Settings &s, bool armed, uint16_t rpm) {
    const uint8_t bin = rpm >= kRpmBins * 1000U ? kRpmBins - 1 : static_cast<uint8_t>(rpm / 1000U);
    const uint16_t gained = static_cast<uint16_t>(burst.level) * s.gainX16 / 16;
    level_ = gained > 255 ? 255 : static_cast<uint8_t>(gained);
    share_ = burst.share;

    const uint16_t base = baseline_[bin] >> 4;
    uint16_t threshold = base * s.multiplierX10 / 10 + s.offset;
    if (threshold > 255) threshold = 255;
    if (threshold < 1) threshold = 1;
    threshold_ = static_cast<uint8_t>(threshold);
    baselineNow_ = base > 255 ? 255 : static_cast<uint8_t>(base);
    learned_ = seen_[bin] >= kLearnBursts;

    const bool loud = level_ >= threshold_;
    const bool narrow = burst.share >= s.minShare;
    const bool hit = armed && learned_ && loud && narrow;
    const bool strong = hit && level_ >= (static_cast<uint16_t>(threshold_) * 8) / 5;

    if (hit) {
      // Two hits close together make a warning. A strong hit is critical at once only
      // when it is also unmistakably narrow-band; broadband noise passes the ordinary
      // narrow test about once in a hundred bursts, so a lone borderline hit proves nothing.
      const bool paired = haveHit_ && now - lastHitMs_ <= kPairMs;
      const uint16_t clearShare = static_cast<uint16_t>(s.minShare) + kClearShareMargin;
      const bool unmistakable = burst.share >= (clearShare > 255 ? 255 : clearShare);
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
    } else {
      // Learn the normal level only from bursts that are not knock: a loud narrow-band
      // burst is never folded into "normal", armed or not.
      if (!(loud && narrow) || !learned_) {
        const int16_t target = static_cast<int16_t>(level_) << 4;
        if (seen_[bin] == 0) baseline_[bin] = static_cast<uint16_t>(target);
        else baseline_[bin] = static_cast<uint16_t>(baseline_[bin] + (target - static_cast<int16_t>(baseline_[bin])) / 32);
        if (seen_[bin] < 255) ++seen_[bin];
      }
    }
    if (warning_ && static_cast<int32_t>(now - warningUntil_) >= 0) warning_ = false;
    if (critical_ && static_cast<int32_t>(now - criticalUntil_) >= 0) critical_ = false;
    if (haveHit_ && now - lastHitMs_ > 60000UL) haveHit_ = false;
  }

  void reset() { *this = Detector(); }

  // The amplifier's gain changed by this ratio (x256). Everything learned is in ADC
  // units at the old gain, so it moves with the gain: "louder than normal" then means
  // the same before and after. Used by the ESP32 board's automatic gain (knock_agc.h).
  void rescale(uint16_t ratioX256) {
    for (uint8_t bin = 0; bin < kRpmBins; ++bin) {
      const uint32_t scaled = (static_cast<uint32_t>(baseline_[bin]) * ratioX256 + 128) / 256;
      baseline_[bin] = scaled > (255UL << 4) ? static_cast<uint16_t>(255U << 4) : static_cast<uint16_t>(scaled);
    }
  }

  uint8_t level() const { return level_; }
  uint8_t share() const { return share_; }
  uint8_t baseline() const { return baselineNow_; }
  uint8_t threshold() const { return threshold_; }
  bool learned() const { return learned_; }
  bool warning() const { return warning_; }
  bool critical() const { return critical_; }

private:
  static constexpr uint8_t kLearnBursts = 25;
  static constexpr uint16_t kPairMs = 600;
  static constexpr uint8_t kClearShareMargin = 50;

  uint16_t baseline_[kRpmBins] = {0};  // Learned level per RPM bin, x16
  uint8_t seen_[kRpmBins] = {0};
  uint32_t lastHitMs_{0};
  uint32_t warningUntil_{0};
  uint32_t criticalUntil_{0};
  uint8_t level_{0};
  uint8_t share_{0};
  uint8_t baselineNow_{0};
  uint8_t threshold_{1};
  bool learned_{false};
  bool haveHit_{false};
  bool warning_{false};
  bool critical_{false};
};

} // namespace knock_band
