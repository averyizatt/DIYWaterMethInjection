// Host tests for the knock spectrum and the knock-frequency finder:
// g++ -std=c++11 -O2 -I include test/test_knock_spectrum/test_main.cpp
#include <initializer_list>
#include <math.h>
#include <stdio.h>

#include "knock_dsp.h"
#include "knock_spectrum.h"

using namespace knock_spectrum;

namespace {
int failures = 0;
uint32_t seed = 31337;

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
const float kBoreHz = 5966.0f;   // What a 96 mm bore suggests
const float kBias = 545.0f;

// The engine as the ADC hears it.
struct Engine {
  float noiseAmp = 40.0f;
  float clickAmp = 120.0f;       // Valve clicks: broadband, every firing
  float blockRingAmp = 0.0f;     // A resonance of the block itself, rung by every firing
  float blockRingHz = 5200.0f;
  float toneAmp = 0.0f;          // A steady line: an alternator, the pump's PWM
  float toneHz = 6000.0f;
  float knockHz = 6600.0f;       // Where the cylinder really rings
  float rpm = 4000.0f;
  float ringAmp = 0.0f;
  double time = 0.0, ringStart = -1.0, nextFire = 0.0, lastFire = -1.0;
  float rate = kRate;

  void ringAt(double when, float amplitude) {
    ringStart = when;
    ringAmp = amplitude;
  }
  float sample() {
    float v = noiseAmp * noise();
    const double firing = 30.0 / rpm;
    if (time >= nextFire) {
      lastFire = nextFire;
      nextFire += firing;
    }
    const float sinceFire = static_cast<float>(time - lastFire);
    if (clickAmp > 0.0f && sinceFire < 0.0003f) v += clickAmp * noise();
    if (blockRingAmp > 0.0f && lastFire >= 0.0) v += blockRingAmp * expf(-sinceFire * 1000.0f) * sinf(kTwoPi * blockRingHz * sinceFire);
    if (toneAmp > 0.0f) v += toneAmp * sinf(kTwoPi * toneHz * static_cast<float>(time));
    const float since = static_cast<float>(time - ringStart);
    if (ringStart >= 0.0 && since >= 0.0f && since < 0.02f) v += ringAmp * expf(-since * 1000.0f) * sinf(kTwoPi * knockHz * since);
    time += 1.0 / rate;
    v += kBias;
    return v < 0.0f ? 0.0f : v > 1023.0f ? 1023.0f : v;
  }
};

// The firmware's chain: filters, detector, spectrum, finder, and the filter following the finder.
struct Rig {
  knock_dsp::FrontEnd front;
  knock_dsp::PassAnalyser analyser;
  knock_dsp::Detector detector;
  knock_dsp::Settings settings;
  Blocks blocks;
  Finder finder;
  Engine engine;
  Snapshot snapshot;
  float centre = kBoreHz;
  uint32_t now = 0;
  int moves = 0, notable = 0;
  float largestMove = 0.0f;
  bool loaded = true;
  float ringChance = 0.0f, ringSize = 0.0f;

  Rig() {
    front.configure(kRate, centre, 1500.0f);
    blocks.configure(kRate, kBoreHz);
    finder.begin(kBoreHz);
  }
  void pass() {
    if (ringChance > 0.0f && noise() * 0.5f + 0.5f < ringChance) engine.ringAt(engine.time + 0.004 + 0.012 * (noise() * 0.5f + 0.5f), ringSize);
    const int count = static_cast<int>(kRate * 0.02f + 0.5f);
    knock_dsp::Frame frame;
    for (int i = 0; i < count; ++i) {
      const float sample = engine.sample();
      if (front.push(sample, frame)) analyser.add(frame);
      blocks.push(sample - front.bias());
    }
    const knock_dsp::Summary heard = analyser.finish(front.secondActive(), knock_dsp::windowFramesFor(static_cast<uint16_t>(engine.rpm)));
    now += 20;
    detector.update(now, heard, settings, loaded, static_cast<uint16_t>(engine.rpm));
    if (!blocks.takeLoudest(snapshot)) return;
    const bool learnFrom = loaded && detector.notable();
    if (learnFrom) ++notable;
    finder.add(snapshot, learnFrom);
    float moved = 0.0f;
    if (finder.decide(centre, moved)) {
      if (fabsf(moved - centre) > largestMove) largestMove = fabsf(moved - centre);
      centre = moved;
      ++moves;
      front.configure(kRate, centre, 1500.0f);
      detector.relearn();
    }
  }
  void run(float seconds) {
    for (int n = static_cast<int>(seconds * 50.0f); n > 0; --n) pass();
  }
};

void theSpectrumIsRight() {
  Blocks blocks;
  blocks.configure(kRate, kBoreHz);
  const float binHz = kRate / kBlock;
  check(fabsf(blocks.binHz() - 300.5f) < 0.1f, "one spectrum line every 300 Hz");
  // A tone exactly on line 20.
  bool done = false;
  for (int n = 0; n < 4 * kBlock; ++n) done = blocks.push(100.0f * sinf(kTwoPi * 20 * binHz * n / kRate)) || done;
  const float *p = blocks.power();
  check(done && fabsf(p[20] - 5000.0f) < 100.0f, "a 100-count tone reads its true power on its own line");
  check(fabsf(p[19] - 1250.0f) < 60.0f && fabsf(p[21] - 1250.0f) < 60.0f, "a quarter of it on each neighbour (the window's shape)");
  check(p[10] < 1.0f && p[30] < 1.0f && p[40] < 1.0f, "and nothing elsewhere");
  // Between two lines, the true frequency is found from the line and its neighbours.
  for (float hz : {5700.0f, 6150.0f, 6480.0f, 7010.0f}) {
    Blocks b;
    b.configure(kRate, kBoreHz);
    for (int n = 0; n < 4 * kBlock; ++n) b.push(100.0f * sinf(kTwoPi * hz * n / kRate));
    uint8_t top = 1;
    for (uint8_t i = 1; i < kBins - 1; ++i) {
      if (b.power()[i] > b.power()[top]) top = i;
    }
    check(fabsf(peakHz(b.power(), top, b.binHz()) - hz) < 45.0f, "a frequency between two lines is placed within 45 Hz");
  }
  check(blocks.searchLow() * binHz <= kBoreHz * kSearchBelow && blocks.searchHigh() * binHz >= kBoreHz * kSearchAbove,
        "the search range covers a fifth below to a quarter above the bore's frequency");
}

void theFloorShowsTonesNotBursts() {
  // Bursts and clicks, no steady tone.
  Rig bursts;
  bursts.ringChance = 0.3f;
  bursts.ringSize = 260.0f;
  bursts.run(10.0f);
  check(steadyToneHz(bursts.snapshot, bursts.blocks.searchLow(), bursts.blocks.searchHigh()) == 0.0f, "ringing bursts are not a steady tone");
  // A steady line at 6.3 kHz in the same noise.
  Rig tone;
  tone.engine.toneAmp = 60.0f;
  tone.engine.toneHz = 6300.0f;
  tone.run(10.0f);
  const float found = steadyToneHz(tone.snapshot, tone.blocks.searchLow(), tone.blocks.searchHigh());
  check(fabsf(found - 6300.0f) < 80.0f, "a steady line is reported at its frequency");
  check(tone.moves == 0 && tone.notable == 0, "and is never mistaken for the knock frequency: it is not a burst");
}

void theFilterIsPulledToWhereTheCylinderRings() {
  for (float real : {6600.0f, 6250.0f, 5400.0f}) {
    Rig rig;
    rig.engine.knockHz = real;
    rig.ringChance = 0.15f;      // Light ringing under boost on some cycles
    rig.ringSize = 260.0f;
    float secondsTaken = 0.0f;
    for (int second = 1; second <= 300; ++second) {
      rig.run(1.0f);
      if (secondsTaken == 0.0f && fabsf(rig.centre - real) < 150.0f) secondsTaken = static_cast<float>(second);
    }
    printf("real %.0f Hz: filter went from %.0f to %.0f Hz in %d moves, there after %.0f s (%d notable passes)\n", real, kBoreHz,
           rig.centre, rig.moves, secondsTaken, rig.notable);
    check(fabsf(rig.centre - real) < 150.0f, "the filter ends within 150 Hz of the real knock frequency");
    check(rig.largestMove <= kMaxStepHz + 0.5f, "moving no more than 300 Hz at a time");
    check(secondsTaken > 0.0f && secondsTaken <= 180.0f, "within three minutes of running with ringing");
    check(rig.moves <= 6, "and then stays there");
  }
}

void itStaysPutWithoutGoodReason() {
  // The bore's frequency is right already.
  Rig right;
  right.engine.knockHz = kBoreHz;
  right.ringChance = 0.15f;
  right.ringSize = 260.0f;
  right.run(180.0f);
  check(right.moves == 0 && right.notable > 100, "already on the resonance: not moved");
  check(fabsf(right.finder.estimateHz() - kBoreHz) < 120.0f && right.finder.agreement() > 0.6f, "and the estimate confirms the bore's figure");

  // No ringing at all: nothing notable, nothing learned.
  Rig quiet;
  quiet.run(180.0f);
  printf("noise only: %d notable passes in 3 minutes, %d moves\n", quiet.notable, quiet.moves);
  check(quiet.moves == 0 && quiet.notable < 9000 / 20, "ordinary noise and valve clicks are rarely notable and never move it");

  // The block's own resonance, rung by every firing: loud, narrow, a burst - and normal.
  Rig block;
  block.engine.blockRingAmp = 200.0f;
  block.engine.blockRingHz = 5300.0f;
  block.run(180.0f);
  check(block.moves == 0, "a resonance that rings on every firing is the engine's normal sound, not knock: not followed");
  // With the cylinder ringing as well, it is the cylinder that is followed.
  block.engine.knockHz = 6600.0f;
  block.ringChance = 0.15f;
  block.ringSize = 300.0f;
  block.run(300.0f);
  check(fabsf(block.centre - 6600.0f) < 200.0f, "with both present, the filter goes to the cylinder's ringing");

  // The same ringing off boost teaches nothing.
  Rig light;
  light.loaded = false;
  light.engine.knockHz = 6600.0f;
  light.ringChance = 0.15f;
  light.ringSize = 260.0f;
  light.run(180.0f);
  check(light.moves == 0 && light.finder.votes() == 0, "only passes under load are learned from");

  // Ringing far from where the cylinder could ring.
  Rig far;
  far.engine.knockHz = 8300.0f;
  far.ringChance = 0.3f;
  far.ringSize = 400.0f;
  far.run(180.0f);
  check(far.centre <= kBoreHz * kSearchAbove && fabsf(far.centre - kBoreHz) < 1.0f, "ringing outside the search range does not move it");

  // Two different ringings in equal measure: no agreement, no move.
  Rig split;
  split.ringSize = 280.0f;
  for (int n = 0; n < 9000; ++n) {
    split.engine.knockHz = (n / 7) % 2 ? 5450.0f : 6500.0f;
    split.ringChance = 0.15f;
    split.pass();
  }
  printf("two ringings: ended at %.0f Hz after %d moves\n", split.centre, split.moves);
  check(split.moves <= 4 && (fabsf(split.centre - 5450.0f) < 250.0f || fabsf(split.centre - 6500.0f) < 250.0f || split.moves == 0),
        "two ringings: it settles on one of them (or stays) and does not wander between them");
}

void itFollowsTheSampleRate() {
  // The same engine sampled at another rate: the spectrum's lines move, the answer does not.
  struct Slow : Rig {
    Slow() {
      engine.rate = 30000.0f;
      front.configure(30000.0f, centre, 1500.0f);
      blocks.configure(30000.0f, kBoreHz);
    }
  };
  Slow rig;
  rig.engine.knockHz = 6500.0f;
  rig.ringChance = 0.15f;
  rig.ringSize = 260.0f;
  for (int n = 0; n < 15000; ++n) {
    // (Rig::pass assumes kRate samples per pass; at 30 kHz a pass is a little longer, which does not matter here.)
    rig.pass();
    if (rig.moves && fabsf(rig.front.sampleRate() - kRate) < 1.0f) rig.front.configure(30000.0f, rig.centre, 1500.0f);
  }
  check(fabsf(rig.centre - 6500.0f) < 200.0f, "at 30 kHz sampling the knock frequency is still found");
}
} // namespace

int main() {
  theSpectrumIsRight();
  theFloorShowsTonesNotBursts();
  theFilterIsPulledToWhereTheCylinderRings();
  itStaysPutWithoutGoodReason();
  itFollowsTheSampleRate();
  if (failures == 0) printf("knock spectrum tests passed\n");
  return failures == 0 ? 0 : 1;
}
