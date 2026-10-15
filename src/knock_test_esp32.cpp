// Knock test for the ESP32-S3 board, with the knock amplifier on fixed resistors (the gain
// jumpers J1-J5, no digital potentiometer). Environment esp32s3_knock_test. No CAN, no
// pump: just the knock listening of the real firmware and a report that is easy to read.
//
// The same code as esp32s3_water_meth: the ADC sampler and its filters on every sample
// (adc_sampler.cpp, knock_dsp.h), the detector that decides what is knock, and the
// spectrum and frequency finder (knock_spectrum.h).
//
// Wiring: the amplifier's output through 10k / 10k (2.2 nF to ground) to GPIO1.
//
// Every 2 s it prints a block like this:
//
//   ---- 24 s -------------------------------------------------------------
//    ADC       38461 samples/s                                     OK
//    Amp out   rests at 826 mV (expect about 825)                  OK
//              swing +/- 3 mV, range 821-830 mV (limits 25-1800)   quiet: tap the sensor
//    Sensor    +/- 1.5 mV at the sensor itself (gain 2.0x fitted)
//    Listening 5.97 kHz (from the 96 mm bore), second resonance 9.90 kHz
//    Level     now 4  normal 4  threshold 18  loudest 6
//    Bursts    last 2 s: 0 loud (0 knock-like)   since start: 3 (1)
//    Verdict   OK
//
// and a line at once for every burst louder than the threshold, saying whether it
// looked like knock (narrow-band at the knock frequency, a short burst) or not (a tap,
// a click: broadband). Every 10 s, the spectrum from 2 to 12 kHz.
//
// Typed into the serial monitor (then Enter):
//   g 3.2   the gain jumper fitted (1.47, 2, 3.2, 5.7 or 11), for the "at the sensor" figure
//   r 4000  the engine speed to pretend (default 3000): sets the listening window
//   f 6500  listen at this frequency; f 0 goes back to the bore's figure
//   m 2.5   threshold multiplier     o 8   threshold offset (as on the dash)
//   s       the spectrum now         c     start learning "normal" again
//
// Bench checks, in order:
//   1. Power on, nothing touching the sensor: "Amp out rests at" about 825 mV and the
//      swing a few mV. Far from 825: the 1.65 V reference or the 10k / 10k divider.
//   2. Tap the block or the sensor's bolt with a screwdriver: "LOUD BURST" lines. Taps are
//      usually broadband, so "not knock-like" is the right answer for a tap.
//   3. A 6 kHz tone (phone tone generator, speaker against the sensor): the level rises
//      and the spectrum shows a peak at 6 kHz; short beeps of it are "knock-like".
//   4. Too loud: "CLIPPING" means the amplifier hits its limits: fit a lower gain jumper.
#include <Arduino.h>
#include <math.h>

#include "adc_sampler.h"
#include "knock_band.h"
#include "knock_dsp.h"
#include "knock_spectrum.h"
#include "pins_esp32.h"

#ifndef KNOCK_TEST_GAIN
#define KNOCK_TEST_GAIN 2.0f      // The gain jumper fitted at start-up; "g" changes it
#endif
#ifndef KNOCK_TEST_BORE_MM
#define KNOCK_TEST_BORE_MM 96
#endif

namespace {
constexpr uint32_t kPassMs = 20;           // One pass, as in the real firmware
constexpr uint32_t kReportMs = 2000;
constexpr uint32_t kSpectrumMs = 10000;
constexpr float kDivider = 0.5f;           // 10k / 10k between the LM358 and the pin
constexpr float kExpectedRestMv = 825.0f;  // 1.65 V through that divider
constexpr float kBandwidthHz = 1500.0f;
constexpr uint8_t kMaxEventsPerReport = 12;

knock_dsp::Frame frames[knock_dsp::kMaxFrames];
knock_dsp::PassAnalyser analyser;
knock_dsp::Detector detector;
knock_spectrum::Finder finder;
knock_spectrum::Snapshot block;
float heldPower[knock_spectrum::kBins];    // The loudest each frequency has been since the last spectrum
bool held = false;
bool haveBlock = false;

float fittedGain = KNOCK_TEST_GAIN;
uint16_t rpm = 3000;
float handHz = 0.0f;                       // Set with "f"; 0: the bore's figure or what was found
float foundHz = 0.0f;
float multiplier = 2.5f;
float offset = 8.0f;
bool adcReady = false;
bool wasWarning = false, wasCritical = false;
float toneHz = 0.0f;

struct Period {
  float lowMv{1e9f}, highMv{-1e9f}, restMv{0.0f};
  uint32_t railed{0}, samples{0};
  float loudest{0.0f};
  uint16_t bursts{0}, knockLike{0}, printed{0};
  bool any{false};
};
Period period;
uint32_t totalBursts = 0, totalKnockLike = 0;
uint32_t lastPassMs = 0, lastReportMs = 0, lastSpectrumMs = 0;
char typed[32];
size_t typedLength = 0;

float boreHz() { return knock_band::knockHzForBore(KNOCK_TEST_BORE_MM); }
float centreHz() { return handHz > 0.0f ? handHz : foundHz > 0.0f ? foundHz : boreHz(); }
const char *centreWhy() { return handHz > 0.0f ? "set by hand" : foundHz > 0.0f ? "found from the bursts heard" : "from the bore"; }

knock_dsp::Settings settings() {
  knock_dsp::Settings s;
  s.offset = offset;
  s.multiplier = multiplier;
  return s;
}

void relearn() {
  detector.reset();
  finder.begin(boreHz());
  foundHz = 0.0f;
  totalBursts = totalKnockLike = 0;
  Serial.println(F(">> Learning \"normal\" again (half a second of quiet, please)."));
}

// ---------------------------------------------------------------------------- printing

void printSpectrum() {
  if (!haveBlock) {
    Serial.println(F(" Spectrum  not yet: no block analysed"));
    return;
  }
  const float bin = block.binHz;
  float top = -1e9f, bottom = 1e9f;
  for (uint8_t b = 0; b < knock_spectrum::kBins; ++b) {
    const float hz = (b + 0.5f) * bin;
    if (hz < 2000.0f || hz > 12500.0f) continue;
    const float loud = 10.0f * log10f(fmaxf(held ? heldPower[b] : block.power[b], 1e-6f));
    const float normal = 10.0f * log10f(fmaxf(block.floor[b], 1e-6f));
    top = fmaxf(top, fmaxf(loud, normal));
    bottom = fminf(bottom, normal);
  }
  bottom = floorf(bottom / 5.0f) * 5.0f - 5.0f;
  const float span = fmaxf(top - bottom, 20.0f);
  Serial.println(F(" Spectrum, dB: '#' normal, '+' loudest since the last one. <<< the band being listened in"));
  for (uint8_t b = 0; b + 1 < knock_spectrum::kBins; b += 2) {
    const float hz = (b + 1.0f) * bin;
    if (hz < 2000.0f || hz > 12500.0f) continue;
    const float normal = 10.0f * log10f(fmaxf(0.5f * (block.floor[b] + block.floor[b + 1]), 1e-6f));
    const float loudPower = held ? fmaxf(heldPower[b], heldPower[b + 1]) : fmaxf(block.power[b], block.power[b + 1]);
    const float loud = 10.0f * log10f(fmaxf(loudPower, 1e-6f));
    const int n = static_cast<int>((normal - bottom) / span * 40.0f + 0.5f);
    const int l = static_cast<int>((loud - bottom) / span * 40.0f + 0.5f);
    char bar[42];
    int i = 0;
    for (; i < 41 && i < n; ++i) bar[i] = '#';
    for (; i < 41 && i < l; ++i) bar[i] = '+';
    bar[i] = '\0';
    const bool listening = fabsf(hz - centreHz()) <= kBandwidthHz / 2.0f;
    const bool second = adc_sampler::knockSecondActive() && fabsf(hz - centreHz() * knock_dsp::kSecondModeRatio) <= kBandwidthHz / 2.0f;
    Serial.printf("  %5.1f kHz %4.0f %-41s%s\n", hz / 1000.0f, normal, bar, listening ? " <<< knock" : second ? " <<< 2nd" : "");
  }
  held = false;
}

void printReport(uint32_t now) {
  Serial.printf("---- %lu s -------------------------------------------------------------\n", static_cast<unsigned long>(now / 1000));
  if (!adcReady) {
    Serial.println(F(" ADC       did not start. Nothing can be measured."));
    return;
  }
  const uint32_t rate = adc_sampler::knockSamplesPerSecond();
  const bool rateOk = rate > 36000 && rate < 41000;
  Serial.printf(" ADC       %lu samples/s%*s%s\n", static_cast<unsigned long>(rate), 38, "", rateOk ? "OK" : "CHECK: expected about 38460");
  if (!period.any || adc_sampler::silentForMs(now) > 500) {
    Serial.println(F(" Amp out   no samples from GPIO1"));
    return;
  }
  const float rest = period.restMv;
  const bool restOk = fabsf(rest - kExpectedRestMv) < 150.0f;
  Serial.printf(" Amp out   rests at %.0f mV (expect about %.0f)%*s%s\n", rest, kExpectedRestMv, 18, "",
                restOk ? "OK" : "CHECK the 1.65 V reference and the 10k/10k divider");
  const float swing = (period.highMv - period.lowMv) / 2.0f;
  const bool clipping = period.samples > 0 && period.railed * 1000U >= period.samples;   // 0.1 % of samples at a limit
  const char *swingNote = clipping ? "CLIPPING: fit a lower gain jumper" : swing < 3.0f ? "quiet: tap the sensor" :
                          period.highMv > KNOCK_ADC_HIGH_MV * 0.85f || period.lowMv < 120.0f ? "loud: near the limits" : "OK";
  Serial.printf("           swing +/- %.0f mV, range %.0f-%.0f mV (limits %.0f-%.0f)   %s\n", swing, period.lowMv, period.highMv,
                KNOCK_ADC_LOW_MV, KNOCK_ADC_HIGH_MV, swingNote);
  Serial.printf(" Sensor    +/- %.1f mV at the sensor itself (gain %.2fx fitted)\n", swing / (kDivider * fittedGain), fittedGain);
  Serial.printf(" Listening %.2f kHz (%s), second resonance %.2f kHz %s\n", centreHz() / 1000.0f, centreWhy(),
                centreHz() * knock_dsp::kSecondModeRatio / 1000.0f, adc_sampler::knockSecondActive() ? "" : "(off: sample rate too low)");
  Serial.printf(" Level     now %.0f  normal %.0f  threshold %.0f  loudest %.0f   (the dash's 0-255 scale; at %u RPM)\n",
                detector.level(), detector.baseline(), detector.threshold(), period.loudest, rpm);
  Serial.printf(" Bursts    last 2 s: %u loud (%u knock-like)   since start: %lu (%lu)\n", period.bursts, period.knockLike,
                static_cast<unsigned long>(totalBursts), static_cast<unsigned long>(totalKnockLike));
  const uint16_t votes = finder.votes();
  if (votes) {
    Serial.printf(" Found     %u votes from knock-like bursts", votes);
    if (finder.estimateHz() > 0.0f) Serial.printf(", they agree on %.2f kHz (%.0f %%)", finder.estimateHz() / 1000.0f, finder.agreement() * 100.0f);
    Serial.println();
  }
  if (toneHz > 0.0f) Serial.printf(" Tone      a steady tone at %.2f kHz (pump PWM, alternator, a power supply?)\n", toneHz / 1000.0f);
  const char *verdict = !detector.learned() ? "LEARNING what normal sounds like" : detector.critical() ? "KNOCK (critical)" :
                        detector.warning() ? "KNOCK (warning)" : "OK";
  Serial.printf(" Verdict   %s\n", verdict);
}

// ---------------------------------------------------------------------------- typing

void command(char *text) {
  while (*text == ' ') ++text;
  if (!*text) return;
  const char what = text[0];
  const float value = atof(text + 1);
  switch (what) {
  case 'g': case 'G':
    if (value >= 1.0f && value <= 20.0f) fittedGain = value;
    Serial.printf(">> Gain fitted: %.2fx\n", fittedGain);
    break;
  case 'r': case 'R':
    if (value >= 0.0f && value <= 9000.0f) rpm = static_cast<uint16_t>(value);
    Serial.printf(">> Pretending %u RPM: listening window %.1f ms\n", rpm, knock_dsp::windowFramesFor(rpm) * 0.5f);
    break;
  case 'f': case 'F':
    handHz = value >= 2000.0f && value <= 9000.0f ? value : 0.0f;
    detector.relearn();
    Serial.printf(">> Listening at %.2f kHz (%s)\n", centreHz() / 1000.0f, centreWhy());
    break;
  case 'm': case 'M':
    if (value >= 1.0f && value <= 10.0f) multiplier = value;
    Serial.printf(">> Threshold = normal x %.1f + %.0f\n", multiplier, offset);
    break;
  case 'o': case 'O':
    if (value >= 0.0f && value <= 200.0f) offset = value;
    Serial.printf(">> Threshold = normal x %.1f + %.0f\n", multiplier, offset);
    break;
  case 's': case 'S':
    printSpectrum();
    break;
  case 'c': case 'C':
    relearn();
    break;
  default:
    Serial.println(F(">> g <gain>  r <rpm>  f <Hz>  m <multiplier>  o <offset>  s (spectrum)  c (learn again)"));
  }
}

void readTyping() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      typed[typedLength] = '\0';
      command(typed);
      typedLength = 0;
    } else if (typedLength + 1 < sizeof(typed)) {
      typed[typedLength++] = c;
    }
  }
}

// ---------------------------------------------------------------------------- one pass

void pass(uint32_t now) {
  adc_sampler::setKnockBand(centreHz(), kBandwidthHz);
  const size_t count = adc_sampler::takeKnockFrames(frames, knock_dsp::kMaxFrames);
  for (size_t n = 0; n < count; ++n) analyser.add(frames[n]);
  const knock_dsp::Summary heard = analyser.finish(adc_sampler::knockSecondActive(), knock_dsp::windowFramesFor(rpm));

  adc_sampler::KnockStats stats;
  if (adc_sampler::takeKnockStats(stats)) {
    period.lowMv = fminf(period.lowMv, stats.lowMv);
    period.highMv = fmaxf(period.highMv, stats.highMv);
    period.restMv = stats.restMv;
    period.railed += stats.railed;
    period.samples += stats.samples;
    period.any = true;
  }
  if (!heard.valid) return;

  // Always armed: on the bench everything is judged.
  detector.update(now, heard, settings(), true, rpm);
  period.loudest = fmaxf(period.loudest, detector.level());
  if (detector.learned() && detector.level() >= detector.threshold()) {
    const bool narrow = heard.share >= settings().minShare;
    const bool burst = heard.level >= knock_dsp::kMinCrest * heard.background;
    const bool knockLike = narrow && burst;
    ++period.bursts;
    ++totalBursts;
    if (knockLike) {
      ++period.knockLike;
      ++totalKnockLike;
    }
    if (period.printed < kMaxEventsPerReport) {
      ++period.printed;
      Serial.printf(">> LOUD BURST %7.2f s  level %3.0f (normal %.0f, threshold %.0f)  at the knock frequency %2.0f %%  stands out %.1fx  "
                    "2nd resonance %.0f (normal %.0f)  -> %s\n",
                    now / 1000.0f, detector.level(), detector.baseline(), detector.threshold(), heard.share * 100.0f,
                    heard.background > 0.0f ? heard.level / heard.background : 0.0f, detector.second(), detector.secondBaseline(),
                    knockLike ? (detector.confirmed() ? "KNOCK-LIKE, both resonances ring" : "KNOCK-LIKE") :
                    !narrow ? "not knock-like: broadband (a tap or a click)" : "not knock-like: not a short burst");
    }
  }
  if (detector.warning() && !wasWarning) Serial.printf(">> KNOCK WARNING at %.2f s\n", now / 1000.0f);
  if (detector.critical() && !wasCritical) Serial.printf(">> KNOCK CRITICAL at %.2f s\n", now / 1000.0f);
  wasWarning = detector.warning();
  wasCritical = detector.critical();

  // The spectrum, and the knock frequency learned from knock-like bursts.
  knock_spectrum::Snapshot fresh;
  if (adc_sampler::takeKnockSpectrum(fresh)) {
    block = fresh;
    haveBlock = true;
    for (uint8_t b = 0; b < knock_spectrum::kBins; ++b) heldPower[b] = held ? fmaxf(heldPower[b], block.power[b]) : block.power[b];
    held = true;
    toneHz = knock_spectrum::steadyToneHz(block, boreHz());
    finder.add(block, detector.notable());
    float moved = 0.0f;
    if (handHz == 0.0f && finder.decide(centreHz(), moved)) {
      Serial.printf(">> The knock-like bursts ring at %.2f kHz: listening there now (was %.2f)\n", moved / 1000.0f, centreHz() / 1000.0f);
      foundHz = moved;
      detector.relearn();
    }
  }
}
} // namespace

void setup() {
  Serial.begin(115200);
  delay(2000);   // Time to open the serial monitor.
  Serial.println();
  Serial.println(F("=== KNOCK TEST: fixed-gain amplifier, no CAN, no pump ==="));
  Serial.printf("Knock input GPIO%d (ADC1 channel 0). Amplifier limits at the pin %.0f-%.0f mV, rest expected %.0f mV.\n",
                pins::KNOCK_SENSOR_ADC, KNOCK_ADC_LOW_MV, KNOCK_ADC_HIGH_MV, kExpectedRestMv);
  Serial.printf("Bore %d mm: knock expected near %.2f kHz. Gain jumper assumed %.2fx (type g <gain> to change).\n",
                KNOCK_TEST_BORE_MM, boreHz() / 1000.0f, fittedGain);
  Serial.println(F("Type ? and Enter for the commands."));
  adc_sampler::setKnockLimitsMv(KNOCK_ADC_LOW_MV, KNOCK_ADC_HIGH_MV);
  adcReady = adc_sampler::begin(pins::KNOCK_SENSOR_ADC, pins::OIL_PRESSURE_ADC, pins::FUEL_PRESSURE_ADC,
                                pins::PRE_INJECTION_TEMP_ADC, pins::POST_INJECTION_TEMP_ADC);
  if (!adcReady) Serial.println(F("The ADC did not start."));
  adc_sampler::setKnockExpected(boreHz());
  adc_sampler::setKnockBand(centreHz(), kBandwidthHz);
  finder.begin(boreHz());
  lastPassMs = lastReportMs = lastSpectrumMs = millis();
}

void loop() {
  const uint32_t now = millis();
  readTyping();
  if (now - lastPassMs >= kPassMs) {
    lastPassMs = now;
    pass(now);
  }
  if (now - lastReportMs >= kReportMs) {
    lastReportMs = now;
    printReport(now);
    period = Period();
  }
  if (now - lastSpectrumMs >= kSpectrumMs) {
    lastSpectrumMs = now;
    printSpectrum();
  }
  delay(1);
}
