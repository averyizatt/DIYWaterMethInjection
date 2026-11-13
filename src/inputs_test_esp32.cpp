// Bench test for the tank float switch and the intake air temperature sensors on the
// ESP32-S3 board (environment esp32s3_inputs_test). No CAN, no pump, no knock.
//
// The same code the real firmware uses: the ADC sampler (adc_sampler.cpp), the
// thermistor curve and the float switch's debounce and hold (analog_inputs.h), with the
// default settings (switch closes to ground while there is fluid, 100 ms debounce, low
// only after 2 s). Every second it prints:
//
//   Float GPIO7: pin LOW  -> fluid         | IAT GPIO5: no sensor (3170 mV) | IAT GPIO6: 1830 mV  22.4 C  72.3 F
//
// and a line at once whenever the float switch's pin changes.
//
// Checks:
//   - Float: switch closed (or GPIO7 jumpered to ground) reads "pin LOW -> fluid". Open it:
//     "pin HIGH" at once, "TANK LOW" 2 s later (the real firmware's slosh delay).
//     If it never changes: the switch's wires, or the other side not on ground.
//   - IAT: the real firmware uses whichever input has the sensor on it (with one on each,
//     GPIO5 is before the nozzle). Room temperature should read about 20-25 C, around
//     1.7-1.9 V at the pin.
//     Warm the sensor in your hand: the temperature rises and the voltage falls.
//     "no sensor" at about 3170 mV (the top of the ADC's range): the sensor is not
//     connected (the 2.49k pulls the pin up) or its ground wire is open. "shorted" near 0 mV: the pin or sensor wire touches
//     ground, or the 2.49k is missing or not on 3V3.
#include <Arduino.h>
#include <driver/gpio.h>

#include "adc_sampler.h"
#include "analog_inputs.h"
#include "pins_esp32.h"

namespace {
constexpr uint32_t kFloatDebounceMs = 100;   // The firmware's defaults (app_config.h)
constexpr uint32_t kFloatHoldMs = 2000;
constexpr uint32_t kPrintMs = 1000;

analog_inputs::FloatSwitch floatSwitch;
bool adcReady = false;
bool lastPinHigh = false;
uint32_t printedMs = 0;

// The default wiring: the switch closes to ground while there is fluid, so HIGH means low.
bool pinHigh() { return digitalRead(pins::FLOAT_SENSOR_DIGITAL) == HIGH; }

void printFloat() {
  const bool high = pinHigh();
  const bool low = floatSwitch.isLow();
  Serial.printf("Float GPIO%d: pin %-4s -> %-13s", pins::FLOAT_SENSOR_DIGITAL, high ? "HIGH" : "LOW",
                low ? "TANK LOW" : high ? "going low" : "fluid");
}

void printTemp(int pin, adc_sampler::Slow input) {
  Serial.printf(" | IAT GPIO%d: ", pin);
  float mv = 0.0f, c = 0.0f;
  if (!adcReady || !adc_sampler::averageMv(input, mv)) {
    Serial.print("no samples");
    return;
  }
  if (analog_inputs::thermistorC(mv, pins::INTAKE_TEMP_SUPPLY_MV, pins::INTAKE_TEMP_EFFECTIVE_PULLUP_OHMS,
                                 adc_sampler::topMv(input), c)) {
    Serial.printf("%4u mV %5.1f C %5.1f F", static_cast<unsigned>(mv), c, c * 9.0f / 5.0f + 32.0f);
  } else if (mv > pins::INTAKE_TEMP_SUPPLY_MV / 2.0f) {
    Serial.printf("no sensor (%u mV)", static_cast<unsigned>(mv));
  } else {
    Serial.printf("shorted (%u mV)", static_cast<unsigned>(mv));
  }
}
} // namespace

void setup() {
  pinMode(pins::FLOAT_SENSOR_DIGITAL, INPUT_PULLUP);
  Serial.begin(115200);
  delay(2000);   // Time to open the serial monitor.
  Serial.println();
  Serial.printf("INPUTS TEST: float switch GPIO%d (closed to ground = fluid), intake air sensors GPIO%d and "
                "GPIO%d (2.49k to 3V3, and the pin's own pull-up as in the real firmware).\n",
                pins::FLOAT_SENSOR_DIGITAL, pins::PRE_INJECTION_TEMP_ADC, pins::POST_INJECTION_TEMP_ADC);
  adcReady = adc_sampler::begin(pins::KNOCK_SENSOR_ADC, pins::OIL_PRESSURE_ADC, pins::FUEL_PRESSURE_ADC,
                                pins::PRE_INJECTION_TEMP_ADC, pins::POST_INJECTION_TEMP_ADC);
  if (!adcReady) Serial.println("The ADC did not start: the temperatures cannot be read; the float switch still is.");
  gpio_pullup_en(static_cast<gpio_num_t>(pins::PRE_INJECTION_TEMP_ADC));
  gpio_pullup_en(static_cast<gpio_num_t>(pins::POST_INJECTION_TEMP_ADC));
  lastPinHigh = pinHigh();
  floatSwitch.begin(lastPinHigh, millis(), kFloatDebounceMs, kFloatHoldMs);
}

void loop() {
  const uint32_t now = millis();
  const bool high = pinHigh();
  const bool wasLow = floatSwitch.isLow();
  floatSwitch.update(high, now);
  if (high != lastPinHigh) {
    lastPinHigh = high;
    Serial.printf(">> Float pin went %s (switch %s)\n", high ? "HIGH" : "LOW", high ? "open" : "closed");
  }
  if (floatSwitch.isLow() != wasLow) {
    Serial.printf(">> Tank now counts as %s\n", floatSwitch.isLow() ? "LOW: the firmware would stop injecting" : "having fluid");
  }

  if (now - printedMs >= kPrintMs) {
    printedMs = now;
    printFloat();
    printTemp(pins::PRE_INJECTION_TEMP_ADC, adc_sampler::kPre);
    printTemp(pins::POST_INJECTION_TEMP_ADC, adc_sampler::kPost);
    Serial.println();
  }
  // The knock samples pile up unread here; the sampler drops the oldest by itself.
  delay(5);
}
