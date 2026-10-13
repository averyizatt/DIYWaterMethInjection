// Bench test for the knock amplifier and its X9C104 gain potentiometer on the ESP32-S3
// board (environment esp32s3_knock_gain_test). No CAN, no pump, no injection rules.
//
// It walks the potentiometer to the bottom, then steps it through its range, three
// seconds at each stop, and goes round again. At each stop it prints what the firmware
// believes and what the ADC actually hears:
//
//   step 44  gain 5.4x  about 44 kohm VW-VL | rest 548  swing 131  knock-band 37  clipped no
//
//   - With a meter across VW and VL (module out of the circuit): the resistance should
//     follow the printed value. That proves the three control wires.
//   - In the circuit with a steady signal (tap the sensor, or feed a tone): "swing"
//     should grow in step with the gain. That proves the amplifier.
//   - "rest" should stay near 545 (1.65 V) at every gain. If it moves with the gain,
//     the amplifier is amplifying its bias: see docs/ESP32_HARDWARE.md.
#include <Arduino.h>

#include "adc_sampler.h"
#include "knock_agc.h"
#include "knock_band.h"
#include "pins_esp32.h"
#include "x9c.h"

namespace {
constexpr uint32_t kStopMs = 3000;
constexpr uint8_t kStops[] = {0, 11, 22, 33, 44, 55, 66, 77, 88, 99};

struct Pins {
  void select(bool on) { digitalWrite(pins::KNOCK_GAIN_CS, on ? LOW : HIGH); }
  void inc(bool high) { digitalWrite(pins::KNOCK_GAIN_INC, high ? HIGH : LOW); }
  void up(bool up) { digitalWrite(pins::KNOCK_GAIN_UD, up ? HIGH : LOW); }
  void wait() { delayMicroseconds(5); }
};
Pins potPins;
x9c::Pot pot;
knock_agc::Circuit circuit;
bool adcReady = false;
uint8_t stop = 0;
uint32_t stopStartedMs = 0;
uint32_t printedMs = 0;
uint16_t low = 0xFFFF, high = 0;
bool clipped = false;
knock_band::Burst latest;
uint16_t samples[knock_band::kSamples];

void goTo(uint8_t index) {
  stop = index;
  pot.moveTo(potPins, kStops[stop]);
  stopStartedMs = printedMs = millis();
  low = 0xFFFF;
  high = 0;
  clipped = false;
  uint16_t a, b;
  adc_sampler::knockRange(a, b);   // Forget what was heard at the previous gain.
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
  circuit.topStep = x9c::kTopStep;

  Serial.begin(115200);
  delay(2000);   // Time to open the serial monitor.
  Serial.println();
  Serial.printf("KNOCK GAIN TEST: X9C104 on CS=GPIO%d INC=GPIO%d U/D=GPIO%d, knock input GPIO%d.\n",
                pins::KNOCK_GAIN_CS, pins::KNOCK_GAIN_INC, pins::KNOCK_GAIN_UD, pins::KNOCK_SENSOR_ADC);
  adcReady = adc_sampler::begin(pins::KNOCK_SENSOR_ADC, pins::OIL_PRESSURE_ADC, pins::FUEL_PRESSURE_ADC,
                                pins::PRE_INJECTION_TEMP_ADC, pins::POST_INJECTION_TEMP_ADC);
  if (!adcReady) Serial.println("The ADC did not start: the potentiometer is still stepped, but nothing is measured.");
  Serial.println("Walking the wiper to the bottom...");
  pot.home(potPins);
  goTo(0);
}

void loop() {
  const uint32_t now = millis();
  uint16_t a, b;
  if (adc_sampler::knockRange(a, b)) {
    if (a < low) low = a;
    if (b > high) high = b;
    if (a <= knock_agc::kClipLow || b >= knock_agc::kClipHigh) clipped = true;
  }
  if (adc_sampler::latestBurst(samples)) latest = knock_band::analyze(samples, knock_band::centerBinFor(knock_band::knockHzForBore(96)));

  if (now - printedMs >= 1000) {
    printedMs = now;
    const uint8_t step = kStops[stop];
    Serial.printf("step %2u  gain %4.1fx  about %3u kohm VW-VL", step, knock_agc::gainAt(circuit, step), step);
    if (adcReady && low <= high) {
      Serial.printf(" | rest %4u  swing %3u  knock-band %3u  clipped %s\n", latest.bias, (high - low) / 2, latest.level,
                    clipped ? "YES" : "no");
    } else {
      Serial.println(" | no samples");
    }
    low = 0xFFFF;
    high = 0;
    clipped = false;
  }
  if (now - stopStartedMs >= kStopMs) {
    const uint8_t next = static_cast<uint8_t>((stop + 1) % (sizeof(kStops) / sizeof(kStops[0])));
    if (next == 0) {
      Serial.println("Back to the bottom (walking the wiper down again)...");
      pot.home(potPins);
    }
    goTo(next);
  }
  delay(5);
}
