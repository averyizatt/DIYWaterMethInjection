// Bench test for the knock amplifier and its X9C104 gain potentiometer on the ESP32-S3
// board (environment esp32s3_knock_gain_test). No CAN, no pump, no injection rules.
//
// It walks the potentiometer to the bottom, then steps it through its range, three
// seconds at each stop, and goes round again. At each stop it prints what the firmware
// believes and what the ADC actually hears:
//
//   step 12 of 30  gain 5.0x  about 40 kohm VW-VL | rest 548  swing 131  knock 37.2  second 9.1  rate 38460
//
//   - With a meter across VW and VL (module out of the circuit): the resistance should
//     follow the printed value, and read the full 100 k at the last stop. If the last
//     stop reads about a third of that, the part has 99 steps, not 30: change
//     KNOCK_GAIN_TOP_STEP in include/pins_esp32.h. That also proves the control wires.
//   - In the circuit with a steady signal (tap the sensor, or feed a tone): "swing"
//     should grow in step with the gain. That proves the amplifier.
//   - "rest" should stay near 545 (1.65 V) at every gain. If it moves with the gain,
//     the amplifier is amplifying its bias: see docs/ESP32_HARDWARE.md.
//   - "knock" and "second" are the loudest levels heard in the two knock filters (about
//     6 and 9.9 kHz); a tone swept through those frequencies should peak there.
//   - "rate" is the knock samples a second the ADC really delivers.
#include <Arduino.h>

#include "adc_sampler.h"
#include "knock_agc.h"
#include "knock_band.h"
#include "knock_dsp.h"
#include "pins_esp32.h"
#include "x9c.h"

namespace {
constexpr uint32_t kStopMs = 3000;
constexpr uint8_t kStopCount = 11;   // Bottom, top and nine in between

struct Pins {
  void select(bool on) { digitalWrite(pins::KNOCK_GAIN_CS, on ? LOW : HIGH); }
  void inc(bool high) { digitalWrite(pins::KNOCK_GAIN_INC, high ? HIGH : LOW); }
  void up(bool up) { digitalWrite(pins::KNOCK_GAIN_UD, up ? HIGH : LOW); }
  void wait() { delayMicroseconds(5); }
};
Pins potPins;
x9c::Pot pot;
knock_agc::Circuit circuit;
knock_dsp::PassAnalyser analyser;
knock_dsp::Frame frames[knock_dsp::kMaxFrames];
bool adcReady = false;
uint8_t stop = 0;
uint32_t stopStartedMs = 0;
uint32_t printedMs = 0;
uint16_t low = 0xFFFF, high = 0;
float rest = 0.0f, loudest = 0.0f, loudestSecond = 0.0f;
bool heardAnything = false;

uint8_t stepAt(uint8_t index) {
  return static_cast<uint8_t>((static_cast<uint16_t>(index) * KNOCK_GAIN_TOP_STEP + (kStopCount - 1) / 2) / (kStopCount - 1));
}

void forget() {
  low = 0xFFFF;
  high = 0;
  loudest = loudestSecond = 0.0f;
  heardAnything = false;
}

void goTo(uint8_t index) {
  stop = index;
  pot.moveTo(potPins, stepAt(stop));
  stopStartedMs = printedMs = millis();
  // What was heard at the previous gain does not count.
  adc_sampler::KnockStats stats;
  adc_sampler::takeKnockStats(stats);
  adc_sampler::takeKnockFrames(frames, knock_dsp::kMaxFrames);
  forget();
}
} // namespace

void setup() {
  pinMode(pins::KNOCK_GAIN_CS, OUTPUT);
  pinMode(pins::KNOCK_GAIN_INC, OUTPUT);
  pinMode(pins::KNOCK_GAIN_UD, OUTPUT);
  digitalWrite(pins::KNOCK_GAIN_CS, HIGH);
  digitalWrite(pins::KNOCK_GAIN_INC, HIGH);
  digitalWrite(pins::KNOCK_GAIN_UD, LOW);
  circuit.lowGain = KNOCK_GAIN_AT_BOTTOM;
  circuit.highGain = KNOCK_GAIN_AT_TOP;
  circuit.topStep = KNOCK_GAIN_TOP_STEP;
  pot.setTopStep(KNOCK_GAIN_TOP_STEP);

  Serial.begin(115200);
  delay(2000);   // Time to open the serial monitor.
  Serial.println();
  Serial.printf("KNOCK GAIN TEST: X9C104 on CS=GPIO%d INC=GPIO%d U/D=GPIO%d, %u steps; knock input GPIO%d.\n",
                pins::KNOCK_GAIN_CS, pins::KNOCK_GAIN_INC, pins::KNOCK_GAIN_UD, static_cast<unsigned>(KNOCK_GAIN_TOP_STEP),
                pins::KNOCK_SENSOR_ADC);
  adcReady = adc_sampler::begin(pins::KNOCK_SENSOR_ADC, pins::OIL_PRESSURE_ADC, pins::FUEL_PRESSURE_ADC,
                                pins::PRE_INJECTION_TEMP_ADC, pins::POST_INJECTION_TEMP_ADC);
  if (!adcReady) Serial.println("The ADC did not start: the potentiometer is still stepped, but nothing is measured.");
  adc_sampler::setKnockBand(knock_band::knockHzForBore(96), 1500.0f);
  Serial.println("Walking the wiper to the bottom...");
  pot.home(potPins);
  goTo(0);
}

void loop() {
  const uint32_t now = millis();
  adc_sampler::KnockStats stats;
  if (adc_sampler::takeKnockStats(stats)) {
    if (stats.low < low) low = stats.low;
    if (stats.high > high) high = stats.high;
    rest = stats.bias;
    heardAnything = true;
  }
  const size_t count = adc_sampler::takeKnockFrames(frames, knock_dsp::kMaxFrames);
  for (size_t n = 0; n < count; ++n) analyser.add(frames[n]);
  const knock_dsp::Summary heard = analyser.finish(adc_sampler::knockSecondActive(), 4);
  if (heard.valid) {
    if (heard.level > loudest) loudest = heard.level;
    if (heard.second > loudestSecond) loudestSecond = heard.second;
  }

  if (now - printedMs >= 1000) {
    printedMs = now;
    const uint8_t step = stepAt(stop);
    Serial.printf("step %2u of %u  gain %4.1fx  about %3u kohm VW-VL", step, static_cast<unsigned>(KNOCK_GAIN_TOP_STEP),
                  knock_agc::gainAt(circuit, step), static_cast<unsigned>(100U * step / KNOCK_GAIN_TOP_STEP));
    if (adcReady && heardAnything) {
      const bool clipped = low <= knock_agc::kClipLow || high >= knock_agc::kClipHigh;
      Serial.printf(" | rest %4u  swing %3u  knock %5.1f  second %5.1f  rate %lu%s\n", static_cast<unsigned>(rest),
                    static_cast<unsigned>((high - low) / 2), loudest, loudestSecond,
                    static_cast<unsigned long>(adc_sampler::knockSamplesPerSecond()), clipped ? "  CLIPPED" : "");
    } else {
      Serial.println(" | no samples");
    }
    forget();
  }
  if (now - stopStartedMs >= kStopMs) {
    const uint8_t next = static_cast<uint8_t>((stop + 1) % kStopCount);
    if (next == 0) {
      Serial.println("Back to the bottom (walking the wiper down again)...");
      pot.home(potPins);
    }
    goTo(next);
  }
  delay(20);
}
