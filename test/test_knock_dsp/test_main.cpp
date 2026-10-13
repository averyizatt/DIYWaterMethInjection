// Host tests for the ESP32 board's knock listening (every sample, all the time):
// g++ -std=c++11 -O2 -I include test/test_knock_dsp/test_main.cpp
#include <initializer_list>
#include <math.h>
#include <stdio.h>

#include "knock_dsp.h"

using namespace knock_dsp;

namespace {
int failures = 0;
uint32_t seed = 2468;

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

const float kRate = 38462.0f;
const float kKnockHz = 5966.0f;    // 96 mm bore
const float kBias = 545.0f;

// The engine as the ADC hears it, one sample at a time.
struct Engine {
  float noiseAmp = 40.0f;       // Broadband noise
  float clickAmp = 0.0f;        // Valve and injector clicks: broadband, short, every firing
  float toneAmp = 0.0f;         // A steady tone (a whine)
  float toneHz = kKnockHz;
  float rpm = 3000.0f;
  float ringAmp = 0.0f;         // Knock: a decaying ring, started by knockAt()
  float ringSecond = 0.5f;      // The second resonance, as a share of the first
  float ringDecayMs = 1.0f;
  double time = 0.0, ringStart = -1.0, nextFire = 0.0;
  float rate = kRate;

  void knockAt(double when, float amplitude) {
    ringStart = when;
    ringAmp = amplitude;
  }
  float sample() {
    float v = noiseAmp * noise();
    const double firing = 30.0 / rpm;                 // Four cylinders
    if (time >= nextFire) nextFire += firing;
    const double sinceFire = time - (nextFire - firing);
    if (clickAmp > 0.0f && sinceFire < 0.0003) v += clickAmp * noise();
    if (toneAmp > 0.0f) v += toneAmp * sinf(kTwoPi * toneHz * static_cast<float>(time));
    const double since = time - ringStart;
    if (ringStart >= 0.0 && since >= 0.0 && since < 0.02) {
      const float envelope = ringAmp * expf(-static_cast<float>(since) * 1000.0f / ringDecayMs);
      v += envelope * (sinf(kTwoPi * kKnockHz * static_cast<float>(since)) +
                       ringSecond * sinf(kTwoPi * kKnockHz * kSecondModeRatio * static_cast<float>(since)));
    }
    time += 1.0 / rate;
    v += kBias;
    return v < 0.0f ? 0.0f : v > 1023.0f ? 1023.0f : v;
  }
};

// The whole chain as the firmware runs it.
struct Rig {
  FrontEnd front;
  PassAnalyser analyser;
  Detector detector;
  Settings settings;
  Engine engine;
  Summary heard;
  uint32_t now = 0;
  int warnings = 0, criticals = 0, passes = 0;
  bool warned = false, wasWarning = false, wasCritical = false;
  float rate = kRate;

  explicit Rig(float sampleRate = kRate, float bandwidth = 1500.0f) : rate(sampleRate) {
    front.configure(sampleRate, kKnockHz, bandwidth);
    engine.rate = sampleRate;
  }
  void pass(bool armed = true) {
    const int count = static_cast<int>(rate * 0.02f + 0.5f);
    Frame frame;
    for (int i = 0; i < count; ++i) {
      if (front.push(engine.sample(), frame)) analyser.add(frame);
    }
    heard = analyser.finish(front.secondActive(), windowFramesFor(static_cast<uint16_t>(engine.rpm)));
    now += 20;
    detector.update(now, heard, settings, armed, static_cast<uint16_t>(engine.rpm));
    if (detector.warning() && !wasWarning) ++warnings;
    if (detector.critical() && !wasCritical) ++criticals;
    wasWarning = detector.warning();
    wasCritical = detector.critical();
    warned = warned || detector.warning();
    ++passes;
  }
  void run(float seconds) {
    for (int n = static_cast<int>(seconds * 50.0f); n > 0; --n) pass();
  }
};

void aToneIsMeasuredAtItsTrueSize() {
  Rig rig;
  rig.engine.noiseAmp = 0.0f;
  rig.engine.toneAmp = 100.0f;
  rig.run(0.5f);
  check(fabsf(rig.heard.level - 100.0f) < 4.0f, "a 100-count tone at the knock frequency reads 100");
  check(rig.heard.share > 0.9f, "and nearly all of the sound is in the knock band");
  check(fabsf(rig.heard.background - 100.0f) < 6.0f, "a steady tone is the same all through the pass");
  rig.engine.toneHz = kKnockHz * kSecondModeRatio;
  rig.run(0.5f);
  check(rig.heard.secondActive && fabsf(rig.heard.second - 100.0f) < 5.0f, "a tone at the second resonance is measured by the second filter");
  check(rig.heard.level < 12.0f, "and hardly shows in the knock band");
  for (float hz : {2000.0f, 3500.0f, 8500.0f}) {
    rig.engine.toneHz = hz;
    rig.run(0.5f);
    check(rig.heard.level < 15.0f && rig.heard.share < 0.05f, "tones away from the knock frequency are rejected");
  }
  // The band is as wide as asked: 3 dB down at the edges.
  rig.engine.toneHz = kKnockHz + 750.0f;
  rig.run(0.5f);
  check(rig.heard.level > 62.0f && rig.heard.level < 80.0f, "half power at the edge of a 1500 Hz band");
  Rig narrow(kRate, 900.0f);
  narrow.engine.noiseAmp = 0.0f;
  narrow.engine.toneAmp = 100.0f;
  narrow.engine.toneHz = kKnockHz + 750.0f;
  narrow.run(0.5f);
  check(narrow.heard.level < 50.0f, "a 900 Hz band rejects what the 1500 Hz band half passes");
}

void theFiltersFollowTheRealSampleRate() {
  for (float rate : {19231.0f, 30000.0f, 38462.0f, 41000.0f}) {
    Rig rig(rate);
    rig.engine.noiseAmp = 0.0f;
    rig.engine.toneAmp = 100.0f;
    rig.run(0.5f);
    check(fabsf(rig.heard.level - 100.0f) < 5.0f, "the knock frequency is the knock frequency at any sample rate");
    rig.engine.toneHz = 3500.0f;
    rig.run(0.5f);
    check(rig.heard.level < 15.0f, "and other frequencies stay out");
    check(rig.heard.frames >= 38 && rig.heard.frames <= 42, "frames stay half a millisecond long");
    check(rig.front.secondActive() == (rate > 24000.0f), "the second resonance is used only when the sample rate can carry it");
  }
  // What goes wrong if the rate is assumed instead of measured: the filter sits on the wrong frequency.
  FrontEnd assumed;
  assumed.configure(38462.0f, kKnockHz, 1500.0f);
  PassAnalyser analyser;
  Engine engine;
  engine.rate = 30000.0f;       // The ADC really runs slower
  engine.noiseAmp = 0.0f;
  engine.toneAmp = 100.0f;
  Frame frame;
  Summary heard;
  for (int pass = 0; pass < 25; ++pass) {
    for (int i = 0; i < 600; ++i) {
      if (assumed.push(engine.sample(), frame)) analyser.add(frame);
    }
    heard = analyser.finish(true, 4);
  }
  check(heard.level < 30.0f, "a filter designed for the wrong sample rate misses the knock frequency");
}

void ordinaryNoiseIsNotNarrow() {
  Rig rig;
  rig.engine.noiseAmp = 100.0f;
  float share = 0.0f, level = 0.0f, background = 0.0f;
  rig.run(1.0f);
  for (int n = 0; n < 200; ++n) {
    rig.pass();
    share += rig.heard.share / 200.0f;
    level += rig.heard.level / 200.0f;
    background += rig.heard.background / 200.0f;
  }
  check(share > 0.08f && share < 0.30f, "broadband noise puts a small share in the knock band");
  check(share < rig.settings.minShare, "well under what counts as narrow-band");
  check(level / background < 2.2f, "and its loudest moment is not far above its typical level");
  check(rig.warnings == 0, "no alarm from noise");
}

// A single knock anywhere in time must be heard: this is what the Nano cannot do.
void aKnockIsHeardWhereverItFalls() {
  Rig rig;
  rig.engine.clickAmp = 120.0f;
  rig.run(2.0f);
  float typical = 0.0f;
  for (int n = 0; n < 100; ++n) {
    rig.pass();
    typical += rig.heard.level / 100.0f;
  }
  int heard = 0, narrow = 0, standing = 0;
  const int trials = 300;
  for (int trial = 0; trial < trials; ++trial) {
    // Anywhere in the next 20 ms, including right at the joins between passes.
    rig.engine.knockAt(rig.engine.time + 0.02 * (0.5 + 0.5 * noise()), 300.0f);
    float best = 0.0f, bestShare = 0.0f, bestCrest = 0.0f;
    for (int n = 0; n < 3; ++n) {
      rig.pass(false);   // Listening only: nothing is learned from these, nothing alarms
      if (rig.heard.level > best) {
        best = rig.heard.level;
        bestShare = rig.heard.share;
        bestCrest = rig.heard.level / rig.heard.background;
      }
    }
    if (best >= 2.5f * typical) ++heard;
    if (bestShare >= rig.settings.minShare) ++narrow;
    if (bestCrest >= kMinCrest) ++standing;
    rig.pass(false);
  }
  printf("single knocks heard: %d of %d (narrow-band %d, standing out %d)\n", heard, trials, narrow, standing);
  check(heard >= trials * 98 / 100, "a knock is heard wherever in time it falls");
  check(narrow >= trials * 98 / 100, "and is recognised as narrow-band");
  check(standing >= trials * 98 / 100, "and as a burst");
}

void windowsReachAcrossTheJoinBetweenPasses() {
  PassAnalyser analyser;
  Frame quiet, loud;
  quiet.knock = 1.0f;
  quiet.wide = 4.0f;
  loud.knock = 100.0f;
  loud.wide = 110.0f;
  // Pass A ends with two loud frames, pass B starts with two: one 2 ms burst across the join.
  for (int i = 0; i < 38; ++i) analyser.add(quiet);
  analyser.add(loud);
  analyser.add(loud);
  const Summary a = analyser.finish(false, 4);
  analyser.add(loud);
  analyser.add(loud);
  for (int i = 0; i < 38; ++i) analyser.add(quiet);
  const Summary b = analyser.finish(false, 4);
  check(fabsf(a.level - sqrtf(2.0f * (2 * 100.0f + 2 * 1.0f) / 4)) < 0.01f, "the first pass sees the half that is in it");
  check(fabsf(b.level - sqrtf(2.0f * 100.0f)) < 0.01f, "the second sees the whole burst, reaching back");
  check(fabsf(a.background - sqrtf(2.0f)) < 0.01f && fabsf(b.background - sqrtf(2.0f)) < 0.01f, "the typical level ignores the burst");
  check(b.share > 0.9f && b.frames == 40, "share is taken where the burst is");
  check(!analyser.finish(false, 4).valid, "a pass with no frames has nothing to say");
  // More frames than a pass can hold: nothing is overrun, and a loud late frame is kept.
  for (int i = 0; i < 200; ++i) analyser.add(quiet);
  analyser.add(loud);
  const Summary full = analyser.finish(false, 2);
  check(full.frames == kMaxFrames && full.level > 5.0f, "an over-long pass is handled");
}

void knockIsCaughtAndNormalRunningIsNot() {
  // Ten minutes of ordinary running at three engine speeds, with valve clicks.
  Rig rig;
  rig.engine.clickAmp = 150.0f;
  for (float rpm : {2500.0f, 4000.0f, 5500.0f}) {
    rig.engine.rpm = rpm;
    rig.engine.noiseAmp = 30.0f + rpm / 100.0f;
    rig.run(200.0f);
  }
  printf("false alarms in ten minutes of normal running: %d\n", rig.warnings);
  check(rig.warnings == 0 && rig.criticals == 0, "no alarm in ten minutes of ordinary noise and clicks");

  // Sustained knock: a ring on every other firing for 0.3 s, thirty times.
  rig.engine.rpm = 4000.0f;
  rig.engine.noiseAmp = 70.0f;
  rig.run(5.0f);
  int caught = 0, latencyMs = 0;
  for (int episode = 0; episode < 30; ++episode) {
    const int before = rig.warnings;
    const uint32_t start = rig.now;
    bool found = false;
    for (int n = 0; n < 15; ++n) {
      rig.engine.knockAt(rig.engine.time + 0.004, 260.0f);
      rig.pass();
      if (!found && rig.warnings > before) {
        found = true;
        latencyMs += static_cast<int>(rig.now - start);
      }
    }
    if (found) ++caught;
    rig.run(1.0f);
  }
  printf("knock episodes caught: %d of 30, on average after %d ms\n", caught, caught ? latencyMs / caught : 0);
  check(caught == 30, "every episode of sustained knock is caught");
  check(caught && latencyMs / caught <= 80, "within a few engine cycles");

  // One hard knock, on its own, ringing both resonances: caught at once.
  int single = 0;
  for (int trial = 0; trial < 30; ++trial) {
    const int before = rig.criticals;
    rig.engine.knockAt(rig.engine.time + 0.01 * (1.0 + noise()), 420.0f);
    rig.run(0.1f);
    if (rig.criticals > before) ++single;
    rig.run(1.0f);
  }
  printf("single hard knocks caught at once: %d of 30\n", single);
  check(single >= 28, "a single hard knock confirmed by the second resonance needs no second hit");

  // The same knock is ignored while not armed, and is not learned as normal.
  const float learnedBefore = rig.detector.baseline();
  for (int n = 0; n < 100; ++n) {
    rig.engine.knockAt(rig.engine.time + 0.004, 300.0f);
    rig.pass(false);
  }
  check(rig.detector.baseline() < learnedBefore * 1.2f, "knock while not armed is not folded into the normal level");
}

void aNoiseFloorThatRisesIsNotKnock() {
  Rig rig;
  rig.run(5.0f);
  const int before = rig.warnings;
  // A steady whine appears right on the knock frequency: loud and narrow, but not a burst.
  rig.engine.toneAmp = 150.0f;
  rig.run(5.0f);
  check(rig.heard.share > 0.5f && rig.detector.level() > rig.detector.threshold(), "the whine is loud and narrow-band");
  check(rig.warnings == before, "but it does not stand out from its own pass: no alarm");
  // Knock on top of the whine still stands out.
  bool caught = false;
  for (int n = 0; n < 15; ++n) {
    rig.engine.knockAt(rig.engine.time + 0.004, 600.0f);
    rig.pass();
    caught = caught || rig.detector.warning();
  }
  check(caught, "knock on top of it is still caught");
  // Everything three times louder at once (a load change): broadband, so no alarm either.
  Rig step;
  step.run(5.0f);
  step.engine.noiseAmp *= 3.0f;
  step.run(5.0f);
  check(step.warnings == 0, "the whole noise floor stepping up is not knock");
}

void theLearnedLevelFollowsAGainChange() {
  // Learn at one amplifier gain, change the gain, knock at once.
  struct Trial {
    static int passesToCatch(float from, float to, bool rescale) {
      seed = 97531;
      Rig rig;
      rig.engine.noiseAmp = 12.0f * from;
      rig.run(8.0f);
      if (rescale) rig.detector.rescale(to / from);
      rig.engine.noiseAmp = 12.0f * to;
      for (int n = 1; n <= 10; ++n) {
        rig.engine.knockAt(rig.engine.time + 0.004, 70.0f * to);
        rig.pass();
        if (rig.detector.warning()) return n;
      }
      return 99;
    }
  };
  const int steady = Trial::passesToCatch(4.0f, 4.0f, true);
  const int rescaled = Trial::passesToCatch(8.0f, 2.0f, true);
  const int stale = Trial::passesToCatch(8.0f, 2.0f, false);
  check(steady <= 3, "knock is caught within three passes at a steady gain");
  check(rescaled <= 3, "and as fast right after the gain is cut to a quarter, with the learned level rescaled");
  check(stale > rescaled, "without rescaling it is missed at first: the old level is four times too high");
  // A gain increase must not make ordinary noise look like knock.
  seed = 13579;
  Rig rig;
  rig.engine.noiseAmp = 24.0f;
  rig.run(8.0f);
  rig.detector.rescale(4.0f);
  rig.engine.noiseAmp = 96.0f;
  rig.run(30.0f);
  check(rig.warnings == 0, "thirty seconds of ordinary noise after a gain increase raise no alarm");
  rig.detector.rescale(1000.0f);
  rig.pass();
  check(rig.detector.baseline() == 255.0f, "an absurd ratio saturates");
}

void valuesFitTheirCanBytes() {
  check(toByte(-3.0f) == 0 && toByte(0.4f) == 0 && toByte(0.6f) == 1, "rounded, never negative");
  check(toByte(254.4f) == 254 && toByte(254.6f) == 255 && toByte(9000.0f) == 255, "and never past 255");
  check(toByte(NAN) == 0, "not-a-number is zero");
}
} // namespace

int main() {
  aToneIsMeasuredAtItsTrueSize();
  theFiltersFollowTheRealSampleRate();
  ordinaryNoiseIsNotNarrow();
  aKnockIsHeardWhereverItFalls();
  windowsReachAcrossTheJoinBetweenPasses();
  knockIsCaughtAndNormalRunningIsNot();
  aNoiseFloorThatRisesIsNotKnock();
  theLearnedLevelFollowsAGainChange();
  valuesFitTheirCanBytes();
  if (failures == 0) printf("knock listening tests passed\n");
  return failures == 0 ? 0 : 1;
}
