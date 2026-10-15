// Knock test for the ESP32-S3 board, with the knock amplifier on fixed resistors (the gain
// jumpers J1-J5, no digital potentiometer). Environment esp32s3_knock_test. No CAN, no
// pump: just the knock listening of the real firmware and a report that is easy to read.
//
// The same code as esp32s3_water_meth: the ADC sampler and its filters on every sample
// (adc_sampler.cpp, knock_dsp.h), the detector that decides what is knock, and the
// spectrum and frequency finder (knock_spectrum.h).
//
// Wiring: the amplifier's output through 10k / 10k (2.2 nF to ground) to GPIO2.
//
// Knock maker (optional): a speaker or tweeter clamped against the block next to the
// sensor, switched by a transistor on GPIO9: GPIO9 - 1k - base of an NPN (2N2222,
// PN2222, BC337), emitter to GND, collector to the speaker's -, the speaker's + through
// 33 ohm to 5 V, a diode (1N4148) across the speaker with its band to the 5 V side.
// "k 4" then rings it at the knock frequency for 1.5 ms on one firing in four, timed
// from the pretended RPM.
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
//   k 4     knock maker: a burst on one firing in 4 (k 1 every firing); k 0 stops it
//
// Bench checks, in order:
//   1. Power on, nothing touching the sensor: "Amp out rests at" about 825 mV and the
//      swing a few mV. Far from 825: the 1.65 V reference or the 10k / 10k divider.
//   2. Tap the block or the sensor's bolt with a screwdriver: "LOUD BURST" lines. Taps are
//      usually broadband, so "not knock-like" is the right answer for a tap.
//   3. A 6 kHz tone (phone tone generator, speaker against the sensor): the level rises
//      and the spectrum shows a peak at 6 kHz; short beeps of it are "knock-like".
//   4. Too loud: "CLIPPING" means the amplifier hits its limits: fit a lower gain jumper.
//   5. Knock maker: "k 4". Every burst should be a "LOUD BURST ... KNOCK-LIKE" line. Then
//      unclamp the speaker and hold it just off the block: if it is still heard, that is
//      the 5 V supply carrying the bursts into the amplifier, not vibration.
#include <Arduino.h>
#include <esp_timer.h>
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
// The knock input for this test. The real firmware uses GPIO1 (pins::KNOCK_SENSOR_ADC);
// the bench rig is wired to GPIO2.
#ifndef KNOCK_TEST_PIN
#define KNOCK_TEST_PIN 2
#endif
// The knock maker's transistor.
#ifndef KNOCK_TEST_DRIVE_PIN
#define KNOCK_TEST_DRIVE_PIN 9
#endif

namespace {
constexpr uint32_t kPassMs = 20;           // One pass, as in the real firmware
constexpr uint32_t kReportMs = 2000;
constexpr uint32_t kSpectrumMs = 10000;
constexpr float kDivider = 0.5f;           // 10k / 10k between the LM358 and the pin
constexpr float kExpectedRestMv = 825.0f;  // 1.65 V through that divider
constexpr float kBandwidthHz = 1500.0f;
constexpr uint8_t kMaxEventsPerReport = 12;
constexpr int kKnockPin = KNOCK_TEST_PIN;
constexpr int kDrivePin = KNOCK_TEST_DRIVE_PIN;
constexpr uint8_t kCylinders = 4;          // For the knock maker's timing
constexpr uint32_t kBurstUs = 1500;        // Knock rings for a millisecond or two
constexpr uint8_t kDriveBits = 8;

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

bool makerReady = false;
uint8_t knockEvery = 0;                    // Set with "k": a burst on one firing in this many; 0: off
uint32_t makerFirings = 0;
uint32_t driveHz = 0;
esp_timer_handle_t firingTimer = nullptr, burstTimer = nullptr;

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

// ---------------------------------------------------------------------------- knock maker

// It rings at the frequency set with "f", or the bore's; not at what was found, so the
// finder has something real to find.
float makerHz() { return handHz > 0.0f ? handHz : boreHz(); }
uint32_t firingUs() { return 120000000UL / (static_cast<uint32_t>(rpm) * kCylinders); }   // Four-stroke

void burstEnd(void *) { ledcWrite(kDrivePin, 0); }

void firing(void *) {
  if (knockEvery == 0 || ++makerFirings % knockEvery != 0) return;
  ledcWrite(kDrivePin, 1U << (kDriveBits - 1));   // Half on: a square wave at makerHz()
  esp_timer_start_once(burstTimer, kBurstUs);
}

// Starts it again for the RPM and frequency now set, or stops it.
void restartMaker() {
  if (!makerReady) return;
  esp_timer_stop(firingTimer);   // An error when it was not running: fine.
  ledcWrite(kDrivePin, 0);
  const uint32_t hz = static_cast<uint32_t>(makerHz() + 0.5f);
  if (hz != driveHz && ledcChangeFrequency(kDrivePin, hz, kDriveBits)) driveHz = hz;
  if (knockEvery > 0 && rpm > 0) esp_timer_start_periodic(firingTimer, firingUs());
}

bool beginMaker() {
  driveHz = static_cast<uint32_t>(makerHz() + 0.5f);
  if (!ledcAttach(kDrivePin, driveHz, kDriveBits)) return false;
  ledcWrite(kDrivePin, 0);
  esp_timer_create_args_t args = {};
  args.callback = firing;
  args.name = "firing";
  if (esp_timer_create(&args, &firingTimer) != ESP_OK) return false;
  args.callback = burstEnd;
  args.name = "burst";
  return esp_timer_create(&args, &burstTimer) == ESP_OK;
}

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
  // The rate is measured over 2 s, so the first report after start-up has none yet.
  Serial.printf(" ADC       %lu samples/s%*s%s\n", static_cast<unsigned long>(rate), 38, "",
                rateOk ? "OK" : rate == 0 && now < 5000 ? "measuring" : "CHECK: expected about 38460");
  if (!period.any || adc_sampler::silentForMs(now) > 500) {
    Serial.printf(" Amp out   no samples from GPIO%d\n", kKnockPin);
    return;
  }
  const float rest = period.restMv;
  const bool restOk = fabsf(rest - kExpectedRestMv) < 150.0f;
  // Resting at or beyond a limit (within 5 % of the range), the amplifier is not what
  // sets the pin, or it is pinned against its own supply: a wiring fault, and nothing it
  // hears gets through. Above the top one is more than the LM358 can reach through the
  // divider at all.
  const float edge = (KNOCK_ADC_HIGH_MV - KNOCK_ADC_LOW_MV) * 0.05f;
  const bool stuckHigh = rest > KNOCK_ADC_HIGH_MV - edge, stuckLow = rest < KNOCK_ADC_LOW_MV + edge;
  Serial.printf(" Amp out   rests at %.0f mV (expect about %.0f)%*s%s\n", rest, kExpectedRestMv, 18, "",
                restOk     ? "OK" :
                stuckHigh  ? "CHECK: more than the amp can give: the divider's 10k to ground, the grounds, the pin's wire" :
                stuckLow   ? "CHECK: about 0 V: the amp's 5 V and output, the 10k in series, the pin's wire" :
                             "CHECK the 1.65 V reference and the 10k/10k divider");
  const float swing = (period.highMv - period.lowMv) / 2.0f;
  const bool clipping = period.samples > 0 && period.railed * 1000U >= period.samples;   // 0.1 % of samples at a limit
  const char *swingNote = stuckHigh || stuckLow ? "STUCK at a limit: the wiring, not the gain" :
                          clipping ? "CLIPPING: fit a lower gain jumper" : swing < 3.0f ? "quiet: tap the sensor" :
                          period.highMv > KNOCK_ADC_HIGH_MV * 0.85f || period.lowMv < 120.0f ? "loud: near the limits" : "OK";
  Serial.printf("           swing +/- %.0f mV, range %.0f-%.0f mV (limits %.0f-%.0f)   %s\n", swing, period.lowMv, period.highMv,
                KNOCK_ADC_LOW_MV, KNOCK_ADC_HIGH_MV, swingNote);
  Serial.printf(" Sensor    +/- %.1f mV at the sensor itself (gain %.2fx fitted)\n", swing / (kDivider * fittedGain), fittedGain);
  Serial.printf(" Listening %.2f kHz (%s), second resonance %.2f kHz %s\n", centreHz() / 1000.0f, centreWhy(),
                centreHz() * knock_dsp::kSecondModeRatio / 1000.0f, adc_sampler::knockSecondActive() ? "" : "(off: sample rate too low)");
  if (knockEvery > 0) {
    Serial.printf(" Maker     ringing %.2f kHz for %.1f ms, one firing in %u (every %.0f ms)\n", makerHz() / 1000.0f,
                  kBurstUs / 1000.0f, knockEvery, firingUs() * knockEvery / 1000.0f);
  }
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
  const char *verdict = stuckHigh || stuckLow ? "CANNOT HEAR: the input is stuck (see Amp out)" :
                        !detector.learned() ? "LEARNING what normal sounds like" : detector.critical() ? "KNOCK (critical)" :
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
    restartMaker();
    Serial.printf(">> Pretending %u RPM: listening window %.1f ms\n", rpm, knock_dsp::windowFramesFor(rpm) * 0.5f);
    break;
  case 'f': case 'F':
    handHz = value >= 2000.0f && value <= 9000.0f ? value : 0.0f;
    detector.relearn();
    restartMaker();
    Serial.printf(">> Listening at %.2f kHz (%s)\n", centreHz() / 1000.0f, centreWhy());
    break;
  case 'k': case 'K':
    if (!makerReady) {
      Serial.printf(">> Knock maker: GPIO%d could not be set up\n", kDrivePin);
      break;
    }
    knockEvery = value >= 1.0f && value <= 50.0f ? static_cast<uint8_t>(value) : 0;
    restartMaker();
    if (knockEvery == 0) {
      Serial.println(F(">> Knock maker off"));
    } else {
      Serial.printf(">> Knock maker on GPIO%d: %.2f kHz for %.1f ms on one firing in %u (every %.0f ms at %u RPM)\n", kDrivePin,
                    makerHz() / 1000.0f, kBurstUs / 1000.0f, knockEvery, firingUs() * knockEvery / 1000.0f, rpm);
    }
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
    Serial.println(F(">> g <gain>  r <rpm>  f <Hz>  m <multiplier>  o <offset>  s (spectrum)  c (learn again)  k <1 in n> (knock maker)"));
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
  Serial.printf("Knock input GPIO%d. Amplifier limits at the pin %.0f-%.0f mV, rest expected %.0f mV.\n",
                kKnockPin, KNOCK_ADC_LOW_MV, KNOCK_ADC_HIGH_MV, kExpectedRestMv);
  Serial.printf("Bore %d mm: knock expected near %.2f kHz. Gain jumper assumed %.2fx (type g <gain> to change).\n",
                KNOCK_TEST_BORE_MM, boreHz() / 1000.0f, fittedGain);
  Serial.println(F("Type ? and Enter for the commands."));
  adc_sampler::setKnockLimitsMv(KNOCK_ADC_LOW_MV, KNOCK_ADC_HIGH_MV);
  // The second pin is an unused slot (oil pressure is not read here): GPIO1, so GPIO2 is
  // not sampled twice per round, which would space the knock samples unevenly.
  adcReady = adc_sampler::begin(kKnockPin, 1, pins::FUEL_PRESSURE_ADC,
                                pins::PRE_INJECTION_TEMP_ADC, pins::POST_INJECTION_TEMP_ADC);
  if (!adcReady) Serial.println(F("The ADC did not start."));
  adc_sampler::setKnockExpected(boreHz());
  adc_sampler::setKnockBand(centreHz(), kBandwidthHz);
  finder.begin(boreHz());
  makerReady = beginMaker();
  Serial.printf("Knock maker on GPIO%d: %s\n", kDrivePin, makerReady ? "ready, k 4 to start it" : "could not be set up");
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
