// Host tests for the knock listening logic: g++ -std=c++11 -I include test/test_knock_band/test_main.cpp
#include <initializer_list>
#include <math.h>
#include <stdio.h>

#include "knock_band.h"

using namespace knock_band;

namespace {
int failures = 0;
uint32_t seed = 12345;

void check(bool ok, const char *what) {
  if (!ok) {
    ++failures;
    printf("FAIL: %s\n", what);
  }
}

float noise() {
  seed = seed * 1664525u + 1013904223u;
  return ((seed >> 8) & 0xFFFF) / 65535.0f * 2.0f - 1.0f;
}

// One burst as the ADC would deliver it: broadband noise around a 2.5 V bias, plus an
// optional decaying ring (what knock looks like) and an optional steady tone.
struct Signal {
  float noiseAmp = 0.0f;
  float ringAmp = 0.0f, ringHz = 5966.0f, ringStartMs = 0.3f, ringDecayMs = 1.5f;
  float toneAmp = 0.0f, toneHz = 0.0f;
  float bias = 512.0f;
};

Burst capture(const Signal &sig, uint8_t centerBin) {
  uint16_t samples[kSamples];
  const float phase = noise() * 3.14159f;
  for (int i = 0; i < kSamples; ++i) {
    const float t = i / static_cast<float>(kSampleRateHz);
    float v = sig.bias + sig.noiseAmp * noise();
    const float since = t * 1000.0f - sig.ringStartMs;
    if (sig.ringAmp > 0.0f && since >= 0.0f) v += sig.ringAmp * expf(-since / sig.ringDecayMs) * sinf(6.2831853f * sig.ringHz * t + phase);
    if (sig.toneAmp > 0.0f) v += sig.toneAmp * sinf(6.2831853f * sig.toneHz * t + phase);
    samples[i] = static_cast<uint16_t>(v < 0.0f ? 0.0f : v > 1023.0f ? 1023.0f : v);
  }
  return analyze(samples, centerBin);
}

const uint8_t kCenter = centerBinFor(knockHzForBore(96));

Signal engineNoise(float amplitude) {
  Signal s;
  s.noiseAmp = amplitude;
  return s;
}

Signal knock(float noiseAmplitude, float ringAmplitude, float hz = 5966.0f) {
  Signal s = engineNoise(noiseAmplitude);
  s.ringAmp = ringAmplitude;
  s.ringHz = hz;
  return s;
}

void frequencyComesFromTheBore() {
  check(knockHzForBore(96) > 5900 && knockHzForBore(96) < 6050, "a 96 mm bore rings at about 6 kHz");
  check(knockHzForBore(88) > 6400 && knockHzForBore(88) < 6600, "a smaller bore rings higher");
  check(kCenter == 20, "6 kHz is bin 20 at 19.2 kHz sampling");
  check(centerBinFor(500) == kMinCenterBin && centerBinFor(9500) == kMaxCenterBin, "the centre stays inside the supported range");
}

void aToneIsMeasuredCorrectly() {
  Signal s;
  s.toneAmp = 100.0f;
  s.toneHz = 5966.0f;
  const Burst b = capture(s, kCenter);
  check(b.level >= 47 && b.level <= 51, "amplitude 100 counts reads as level 50");
  check(b.share > 240, "a pure tone is all in the band");
  check(b.bias >= 510 && b.bias <= 514 && b.peak >= 95 && b.peak <= 101 && b.railed == 0, "bias and peak are reported");
  s.toneAmp = 500.0f;
  const Burst loud = capture(s, kCenter);
  check(loud.level > 240 && loud.share > 240, "full scale does not overflow the integer filter");
  // The hardest input for it: a full-scale square wave.
  uint16_t square[kSamples];
  for (int i = 0; i < kSamples; ++i) square[i] = (i * 5966 * 2 / kSampleRateHz) % 2 ? 1023 : 0;
  const Burst clipped = analyze(square, kCenter);
  check(clipped.level == 255 && clipped.railed == kSamples, "a clipped square wave saturates cleanly and is flagged as railed");
  uint16_t flat[kSamples];
  for (int i = 0; i < kSamples; ++i) flat[i] = 512;
  const Burst silent = analyze(flat, kCenter);
  check(silent.level == 0 && silent.share == 0 && silent.peak == 0, "silence is zero, not a divide by zero");
}

void onlyTheKnockBandCounts() {
  // A ring of the same size at other frequencies must look like nothing special.
  for (float hz : {5400.0f, 5700.0f, 5966.0f, 6300.0f, 6600.0f}) {
    int level = 0, share = 0;
    for (int i = 0; i < 200; ++i) {
      const Burst b = capture(knock(40.0f, 150.0f, hz), kCenter);
      level += b.level;
      share += b.share;
    }
    check(level / 200 >= 28 && share / 200 >= 160, "ringing between 5.4 and 6.6 kHz is in the band");
  }
  for (float hz : {3000.0f, 4500.0f, 7200.0f, 9000.0f}) {
    int level = 0, share = 0;
    for (int i = 0; i < 200; ++i) {
      const Burst b = capture(knock(40.0f, 150.0f, hz), kCenter);
      level += b.level;
      share += b.share;
    }
    check(level / 200 <= 12 && share / 200 <= 40, "ringing at other frequencies is not");
  }
  // A big low-frequency thump (valvetrain, piston slap) raises neither measure.
  Signal thump = engineNoise(40.0f);
  thump.toneAmp = 150.0f;
  thump.toneHz = 900.0f;
  const Burst b = capture(thump, kCenter);
  check(b.level <= 12 && b.share <= 10 && b.peak > 150, "a loud low-frequency thump is ignored");
}

void broadbandNoiseNeverLooksNarrow() {
  Settings settings;
  int narrow = 0, maxLevel = 0;
  for (int i = 0; i < 5000; ++i) {
    const Burst b = capture(engineNoise(120.0f), kCenter);
    if (b.share >= settings.minShare) ++narrow;
    if (b.level > maxLevel) maxLevel = b.level;
  }
  check(narrow < 50, "under 1% of pure-noise bursts pass the narrow-band test");
  check(maxLevel < 45, "loud broadband noise has a modest knock-band level");
}

// Feeds bursts every 20 ms, as the firmware does.
struct Bench {
  Detector detector;
  Settings settings;
  uint32_t now = 0;
  void run(const Signal &signal, uint16_t rpm, int bursts, bool armed = true) {
    for (int i = 0; i < bursts; ++i) {
      now += 20;
      detector.update(now, capture(signal, kCenter), settings, armed, rpm);
    }
  }
  bool one(const Signal &signal, uint16_t rpm, bool armed = true) {
    run(signal, rpm, 1, armed);
    return detector.warning();
  }
};

void learnsNormalNoiseAndFlagsKnock() {
  Bench bench;
  bench.run(engineNoise(40.0f), 3500, 10);
  check(!bench.detector.learned(), "not judged until the normal level at this engine speed is learned");
  bench.run(knock(40.0f, 200.0f), 3500, 5);
  check(!bench.detector.warning(), "nothing is flagged before learning");
  Bench b2;
  b2.run(engineNoise(40.0f), 3500, 200);
  check(b2.detector.learned() && b2.detector.baseline() >= 4 && b2.detector.baseline() <= 8, "learns the noise level");
  check(b2.detector.threshold() >= 18 && b2.detector.threshold() <= 30 && !b2.detector.warning(), "threshold sits well above normal, no warning in plain noise");
  // One knock-like burst alone is not a warning; a second within 0.6 s is.
  check(!b2.one(knock(40.0f, 110.0f), 3500), "a single moderate hit does not warn");
  b2.run(engineNoise(40.0f), 3500, 5);
  check(b2.one(knock(40.0f, 110.0f), 3500), "a second hit soon after does");
  check(!b2.detector.critical(), "moderate knock is a warning, not critical");
  // The warning holds long enough to be seen, then clears.
  b2.run(engineNoise(40.0f), 3500, 10);
  check(b2.detector.warning(), "the warning is held for 300 ms");
  b2.run(engineNoise(40.0f), 3500, 10);
  check(!b2.detector.warning(), "and clears afterwards");
  // Two hits far apart are not a pair.
  b2.one(knock(40.0f, 110.0f), 3500);
  b2.run(engineNoise(40.0f), 3500, 60);
  check(!b2.one(knock(40.0f, 110.0f), 3500), "hits more than 0.6 s apart do not pair up");
  // One strong hit is critical at once.
  b2.run(engineNoise(40.0f), 3500, 60);
  b2.one(knock(40.0f, 400.0f), 3500);
  check(b2.detector.warning() && b2.detector.critical(), "one strong hit is critical immediately");
  // Knock does not teach the detector that knock is normal.
  const uint8_t before = b2.detector.baseline();
  b2.run(knock(40.0f, 250.0f), 3500, 300);
  check(b2.detector.baseline() <= before + 1 && b2.detector.warning(), "sustained knock does not raise the learned level");
}

void louderAtHighRpmIsNotKnock() {
  Bench bench;
  bench.run(engineNoise(30.0f), 2500, 200);
  bench.run(engineNoise(150.0f), 5500, 200);  // Five times the noise at high RPM
  check(!bench.detector.warning(), "an engine that is simply louder at 5500 RPM does not warn");
  check(bench.detector.baseline() >= 18, "the high-RPM level is learned separately");
  bench.run(engineNoise(30.0f), 2500, 1);
  check(bench.detector.baseline() <= 8, "and the low-RPM level is kept");
  // The same knock that is obvious at 2500 RPM is judged against the louder floor at 5500.
  bench.one(knock(30.0f, 120.0f), 2500);
  bench.run(engineNoise(30.0f), 2500, 3);
  check(bench.one(knock(30.0f, 120.0f), 2500), "knock at 2500 RPM is caught");
  Bench noRpm;
  noRpm.run(engineNoise(40.0f), 0, 200);
  noRpm.one(knock(40.0f, 200.0f), 0);
  noRpm.run(engineNoise(40.0f), 0, 2);
  check(noRpm.one(knock(40.0f, 200.0f), 0), "with no RPM signal one shared level still works");
}

void armingSettingsAndGain() {
  Bench bench;
  bench.run(engineNoise(40.0f), 3500, 200);
  bench.run(knock(40.0f, 250.0f), 3500, 30, false);
  check(!bench.detector.warning(), "not armed (light load): nothing is flagged");
  const uint8_t level = bench.detector.level();
  check(level > 30, "but the level is still measured and reported");
  bench.run(engineNoise(40.0f), 3500, 100, false);
  check(bench.detector.baseline() <= 8, "and unarmed knock-like bursts are not learned as normal either");
  // A higher multiplier makes it less sensitive.
  bench.settings.multiplierX10 = 38;
  bench.settings.offset = 60;
  bench.run(knock(40.0f, 110.0f), 3500, 30);
  check(!bench.detector.warning(), "a high threshold ignores moderate knock");
  // Software gain scales the level.
  Bench gained;
  gained.settings.gainX16 = 64;
  gained.run(engineNoise(10.0f), 3000, 100);
  Bench plain;
  plain.run(engineNoise(10.0f), 3000, 100);
  check(gained.detector.baseline() >= plain.detector.baseline() * 3, "gain of 4 quadruples the level");
  // A narrow-band requirement of 0 falls back to level alone.
  Bench wide;
  wide.settings.minShare = 0;
  wide.run(engineNoise(20.0f), 3000, 200);
  wide.run(engineNoise(250.0f), 3000, 3);
  check(wide.detector.warning(), "with the narrow-band test off, loud broadband noise warns");
  Bench narrow;
  narrow.run(engineNoise(20.0f), 3000, 200);
  narrow.run(engineNoise(250.0f), 3000, 3);
  check(!narrow.detector.warning(), "with it on (the default), it does not");
}
void suddenBroadbandNoiseIsNotKnock() {
  // Twenty minutes at one engine speed with the noise level jumping fivefold either way
  // every two seconds (gear changes, bumps, a rattling heat shield), and no knock at all.
  Bench bench;
  int warnings = 0, criticals = 0;
  bool wasWarning = false, wasCritical = false;
  for (int step = 0; step < 600; ++step) {
    const float amplitude = step % 2 ? 200.0f : 40.0f;
    for (int i = 0; i < 100; ++i) {
      bench.run(engineNoise(amplitude), 4000, 1);
      if (bench.detector.warning() && !wasWarning) ++warnings;
      if (bench.detector.critical() && !wasCritical) ++criticals;
      wasWarning = bench.detector.warning();
      wasCritical = bench.detector.critical();
    }
  }
  printf("false alarms in 20 min of jumping broadband noise: %d warnings, %d critical\n", warnings, criticals);
  check(criticals == 0, "broadband noise never raises a critical knock");
  check(warnings <= 6, "and almost never a warning");
  // The same twenty minutes with real knock for one second in every twenty: every episode is caught.
  Bench real;
  real.run(engineNoise(40.0f), 4000, 200);
  int caught = 0;
  for (int episode = 0; episode < 60; ++episode) {
    bool seen = false;
    for (int i = 0; i < 50; ++i) {
      // A knocking cylinder rings about once per 30 ms at 4000 RPM; a burst overlaps about one in four.
      real.run(i % 4 == 0 ? knock(40.0f, 180.0f) : engineNoise(40.0f), 4000, 1);
      seen = seen || real.detector.warning();
    }
    if (seen) ++caught;
    real.run(engineNoise(40.0f), 4000, 950);
  }
  printf("knock episodes caught: %d of 60\n", caught);
  check(caught == 60, "one second of knock is always caught, even when only one burst in four overlaps it");
}
} // namespace

int main() {
  frequencyComesFromTheBore();
  aToneIsMeasuredCorrectly();
  onlyTheKnockBandCounts();
  broadbandNoiseNeverLooksNarrow();
  learnsNormalNoiseAndFlagsKnock();
  louderAtHighRpmIsNotKnock();
  armingSettingsAndGain();
  suddenBroadbandNoiseIsNotKnock();
  if (failures) {
    printf("%d knock band test(s) failed\n", failures);
    return 1;
  }
  printf("knock band tests passed\n");
  return 0;
}
