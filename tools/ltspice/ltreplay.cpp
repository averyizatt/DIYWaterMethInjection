// Runs the ESP32 firmware's knock listening and gain control on the waveforms of the
// owner's LTspice simulation (Knock_Bosch_0261231183_Sweep), extracted by ltraw.py.
//
// The amplifier in that circuit is linear until it clips, so one clean run (lowest
// gain, AMP=1) is scaled to any source amplitude and any gain, and clamped to the
// voltages the amplifier can put on the ADC pin. That is checked first against the
// simulation's own other runs.
//
// usage: ltreplay <folder with step_NN.bin> [divider ratio] [threshold offset] [top gain]
//   divider ratio   amplifier output to ADC pin: 0.5 for 10k/10k (default), 0.82 for 3.3k/15k
//   threshold offset   the detector setting, in half ADC counts (default 2)
//   top gain        gain at the top of the potentiometer: 1 + 100k / RG (default 11)
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "knock_agc.h"
#include "knock_dsp.h"

namespace {
float gRatio = 0.5f;                          // Divider from the amplifier to the ADC pin
float gOffset = 2.0f;                         // Detector threshold offset (the ESP32 firmware default)
float gTopGain = 11.0f;
#define kRest (1.65f * gRatio)
#define kPinLow (0.05f * gRatio)
#define kPinHigh (3.6f * gRatio)
const float kBaseGain = 1.47f;               // The clean run: RF = 4.7k
const float kRate = 38462.0f;
const float kFullScaleMv = 3100.0f;
const float kKnockTimes[3] = {0.063f, 0.121f, 0.164f};
const float kJumperGain[5] = {1.47f, 2.0f, 3.2f, 5.7f, 11.0f};
const float kAmps[6] = {0.25f, 0.5f, 1.0f, 2.0f, 5.0f, 10.0f};
uint32_t seed = 99;

float noise() {
  seed = seed * 1664525u + 1013904223u;
  return ((seed >> 8) & 0xFFFF) / 65535.0f * 2.0f - 1.0f;
}

struct Wave {
  std::vector<float> t, adc, vout, sensor;
  size_t cursor = 0;
  bool load(const std::string &path) {
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) return false;
    float r[4];
    while (fread(r, sizeof(float), 4, f) == 4) {
      t.push_back(r[0]);
      adc.push_back(r[1]);
      vout.push_back(r[2]);
      sensor.push_back(r[3]);
    }
    fclose(f);
    return t.size() > 2;
  }
  float at(const std::vector<float> &v, float time) {
    if (time < t[cursor]) cursor = 0;
    while (cursor + 2 < t.size() && t[cursor + 1] < time) ++cursor;
    const float span = t[cursor + 1] - t[cursor];
    const float f = span > 0.0f ? (time - t[cursor]) / span : 0.0f;
    return v[cursor] + (v[cursor + 1] - v[cursor]) * (f < 0.0f ? 0.0f : f > 1.0f ? 1.0f : f);
  }
};

Wave base;

// Voltage at the ADC pin for a source `amp` times the simulated one and an amplifier gain.
float pin(float time, float amp, float gain) {
  const float v = kRest + (base.at(base.adc, time) - 0.825f) * (gRatio / 0.5f) * amp * gain / kBaseGain;
  return v < kPinLow ? kPinLow : v > kPinHigh ? kPinHigh : v;
}

// The firmware's chain for one amplitude.
struct Listener {
  knock_dsp::FrontEnd front;
  knock_dsp::PassAnalyser analyser;
  knock_dsp::Detector detector;
  knock_dsp::Settings settings;
  knock_dsp::Summary heard;
  knock_agc::Controller agc;
  knock_agc::Circuit circuit;
  bool automatic = false;
  float gain = 1.0f, amp = 1.0f;
  uint32_t now = 0;
  double sampleTime = 0.0;       // Position in the simulation
  float low = 1e9f, high = -1e9f;
  bool hit = false, clipped = false;
  int changes = 0;

  Listener() {
    front.configure(kRate, 5966.0f, 1500.0f);
    circuit.lowGain = 1.0f;
    circuit.highGain = gTopGain;
    settings.offset = gOffset;
    circuit.topStep = 30;
    circuit.railLow = kPinLow * 1000.0f;
    circuit.railHigh = kPinHigh * 1000.0f;
    agc.begin(circuit, 0);
  }

  // One 20 ms pass. loopFrom/loopTo: stay inside this stretch of the simulation (seconds).
  void pass(bool armed, float loopFrom, float loopTo) {
    const int count = static_cast<int>(kRate * 0.02f + 0.5f);
    knock_dsp::Frame frame;
    low = 1e9f;
    high = -1e9f;
    for (int i = 0; i < count; ++i) {
      if (loopTo > loopFrom && sampleTime >= loopTo) sampleTime -= (loopTo - loopFrom);
      const float mv = pin(static_cast<float>(sampleTime), amp, automatic ? agc.gain() : gain) * 1000.0f;
      sampleTime += 1.0 / kRate;
      if (mv < low) low = mv;
      if (mv > high) high = mv;
      const float counts = mv / kFullScaleMv * 1023.0f + noise();   // The ADC, with a count of noise
      if (front.push(counts, frame)) analyser.add(frame);
    }
    heard = analyser.finish(front.secondActive(), knock_dsp::windowFramesFor(3000));
    now += 20;
    const bool settling = automatic && agc.settling();
    if (!settling) detector.update(now, heard, settings, armed, 3000);
    hit = !settling && armed && detector.learned() && detector.level() >= detector.threshold() &&
          heard.share >= settings.minShare && heard.level >= knock_dsp::kMinCrest * heard.background;
    const float margin = (kPinHigh - kPinLow) * 1000.0f * knock_agc::kClipMargin;
    clipped = low <= kPinLow * 1000.0f + margin || high >= kPinHigh * 1000.0f - margin;
    if (automatic) {
      knock_agc::Sample s;
      s.low = low;
      s.high = high;
      s.rest = kRest * 1000.0f;
      s.rpm = 3000;
      s.rpmValid = true;
      s.knocking = detector.learned() && detector.level() >= detector.threshold();
      if (agc.update(s)) {
        detector.rescale(agc.ratioX256() / 256.0f);
        ++changes;
      }
    }
  }
};

struct Outcome {
  int hits = 0;
  bool warning = false, critical = false, clippedNormal = false, clippedKnock = false;
  float normalPeakMv = 0.0f, level = 0.0f, threshold = 0.0f, worstRatio = 1e9f, share = 0.0f, gain = 0.0f;
  int step = -1, changes = 0, changesDuringKnocks = 0;
  bool overloaded = false, silent = false;
};

Outcome play(float amp, bool automatic, float fixedGain) {
  seed = 99;
  Listener l;
  l.amp = amp;
  l.automatic = automatic;
  l.gain = fixedGain;
  Outcome out;
  // Normal running: the knock-free stretch from 20 to 50 ms, which repeats exactly
  // (all its tones and the firing rate fit a whole number of times in 10 ms).
  l.sampleTime = 0.020;
  const int warm = automatic ? 1500 : 300;   // 30 s for the gain to settle, 6 s to learn otherwise
  float peakSum = 0.0f;
  for (int n = 0; n < warm; ++n) {
    l.pass(true, 0.020f, 0.050f);
    if (n >= warm - 50) {
      peakSum += (l.high - l.low) * 0.5f;
      out.clippedNormal = out.clippedNormal || l.clipped;
    }
  }
  out.normalPeakMv = peakSum / 50.0f;
  out.step = automatic ? l.agc.step() : -1;
  out.gain = automatic ? l.agc.gain() : fixedGain;
  out.changes = l.changes;
  out.level = l.detector.baseline();
  out.threshold = l.detector.threshold();
  // Then the stretch with the three knocks: 50 to 200 ms.
  const int before = l.changes;
  l.sampleTime = 0.050;
  for (int n = 0; n < 8; ++n) {
    const float from = static_cast<float>(l.sampleTime), to = from + 0.02f;
    l.pass(true, 0.0f, 0.0f);
    bool knockHere = false;
    for (float k : kKnockTimes) knockHere = knockHere || (k >= from - 0.002f && k < to);
    if (knockHere) {
      if (l.hit) ++out.hits;
      out.clippedKnock = out.clippedKnock || l.clipped;
      const float ratio = l.detector.level() / l.detector.threshold();
      if (ratio < out.worstRatio) out.worstRatio = ratio;
      out.share = l.heard.share;
    }
    out.warning = out.warning || l.detector.warning();
    out.critical = out.critical || l.detector.critical();
  }
  out.changesDuringKnocks = l.changes - before;
  out.overloaded = automatic && l.agc.overloaded();
  out.silent = automatic && l.agc.silent();
  return out;
}
} // namespace

int main(int argc, char **argv) {
  const std::string folder = argc > 1 ? argv[1] : ".";
  if (argc > 2) gRatio = static_cast<float>(atof(argv[2]));
  if (argc > 3) gOffset = static_cast<float>(atof(argv[3]));
  if (argc > 4) gTopGain = static_cast<float>(atof(argv[4]));
  printf("== divider ratio %.2f (pin rests at %.0f mV, limits %.0f..%.0f mV), threshold offset %.0f, top gain %.0f\n", gRatio, kRest * 1000, kPinLow * 1000, kPinHigh * 1000, gOffset, gTopGain);
  if (!base.load(folder + "/step_11.bin")) {   // sel=1 (RF 4.7k), AMP=1: clean, far from clipping
    printf("could not read step_11.bin\n");
    return 1;
  }

  // ---- Is scaling one clean run a fair stand-in for the other runs? ----
  printf("Check: one clean run scaled, against the simulation's own runs (ADC pin, 60-170 ms)\n");
  printf("  amp  gain   simulated min..max V     scaled min..max V\n");
  const int checks[4][2] = {{2, 2}, {3, 3}, {4, 2}, {5, 3}};   // {jumper, amplitude index}
  bool fair = true;
  for (const auto &c : checks) {
    Wave real;
    char name[64];
    snprintf(name, sizeof(name), "/step_%02d.bin", c[1] * 5 + c[0]);
    if (!real.load(folder + name)) continue;
    float realLow = 9, realHigh = -9, low = 9, high = -9;
    for (size_t i = 0; i < real.t.size(); ++i) {
      if (real.t[i] < 0.06f || real.t[i] > 0.17f) continue;
      realLow = std::min(realLow, real.adc[i]);
      realHigh = std::max(realHigh, real.adc[i]);
    }
    for (float time = 0.06f; time < 0.17f; time += 2e-6f) {
      const float v = pin(time, kAmps[c[1]], kJumperGain[c[0] - 1]);
      low = std::min(low, v);
      high = std::max(high, v);
    }
    printf("  %4.2f  %4.1f   %6.3f .. %6.3f          %6.3f .. %6.3f\n", kAmps[c[1]], kJumperGain[c[0] - 1], realLow, realHigh, low, high);
    fair = fair && fabsf(realLow - low) < 0.03f && fabsf(realHigh - high) < 0.03f;
  }
  printf("  %s\n\n", fair ? "agrees within 30 mV" : "DOES NOT AGREE");

  // ---- The five jumper gains, as simulated ----
  printf("Fixed gains (the five jumpers). Each cell: knocks caught of 3, W = warning raised,\n");
  printf("c = clips in normal running, k = clips only at the knocks. Normal swing in mV at the pin.\n");
  printf("  source   gain 1.47        gain 2.0         gain 3.2         gain 5.7         gain 11\n");
  for (float amp : kAmps) {
    printf("  x%-5.2f", amp);
    for (float gain : kJumperGain) {
      const Outcome o = play(amp, false, gain);
      printf("  %d/3%s%s %4.0f mV   ", o.hits, o.warning ? " W" : "  ", o.clippedNormal ? "c" : o.clippedKnock ? "k" : " ", o.normalPeakMv);
    }
    printf("\n");
  }

  // ---- The potentiometer under automatic control ----
  printf("\nAutomatic gain with the 30-step potentiometer (gain 1 to top).\n");
  printf("Room at this circuit's pin: %.0f mV; the normal swing is steered to %.0f-%.0f mV.\n", (kRest - kPinLow) * 1000.0f,
         (kRest - kPinLow) * 1000.0f * knock_agc::kLowShare, (kRest - kPinLow) * 1000.0f * knock_agc::kHighShare);
  printf("  source   settles at        normal swing   knocks   warning   clipping            level/threshold at the weakest knock\n");
  for (float amp : kAmps) {
    const Outcome o = play(amp, true, 0.0f);
    printf("  x%-5.2f  step %2d gain %4.1f   %5.0f mV      %d of 3   %-7s   %-18s  %.1fx (share %.2f)%s\n", amp, o.step, o.gain, o.normalPeakMv, o.hits,
           o.critical ? "yes" : o.warning ? "yes" : "no", o.clippedNormal ? "in normal running" : o.clippedKnock ? "at a knock only" : "none", o.worstRatio, o.share,
           o.overloaded ? "  OVERLOADED: too hot even at gain 1" : o.silent ? "  SILENT" : "");
  }
  return 0;
}
