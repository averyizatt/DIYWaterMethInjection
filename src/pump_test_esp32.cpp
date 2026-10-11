// Bench test for the pump's MOSFET driver on the ESP32-S3 board (environment
// esp32s3_pump_test). Nothing else runs: no CAN, no sensors, no injection rules.
//
//   first 10 s   pump off
//   then         10 %, 20 %, ... 100 % duty, two seconds each
//   then         pump off, and it stays off. Press the board's reset button to run again.
//
// The pin and PWM frequency are the real firmware's (pins_esp32.h): the gate signal on
// GPIO15 at 200 Hz. What works here is what the real firmware will drive.
#include <Arduino.h>

#include "pins_esp32.h"

namespace {
constexpr uint32_t kWaitMs = 10000;      // Pump off after start-up
constexpr uint32_t kStepMs = 2000;       // Time at each speed
constexpr uint8_t kStepPercent = 10;     // Duty added at each step
constexpr uint8_t kTopPercent = 100;     // Last step

bool pwmReady = false;

void setPump(uint8_t percent) {
  const uint32_t top = (1UL << PUMP_PWM_BITS) - 1;
  // The optional master relay feeds the pump its 12 V: closed while the pump is driven.
  // With nothing wired to that pin this does nothing.
  digitalWrite(pins::PUMP_RELAY_OUT, percent > 0 ? HIGH : LOW);
  ledcWrite(pins::PUMP_OUT, (top * percent + 50) / 100);
}
} // namespace

void setup() {
  // Off before anything else.
  pinMode(pins::PUMP_RELAY_OUT, OUTPUT);
  digitalWrite(pins::PUMP_RELAY_OUT, LOW);
  pinMode(pins::PUMP_OUT, OUTPUT);
  digitalWrite(pins::PUMP_OUT, LOW);
  pwmReady = ledcAttach(pins::PUMP_OUT, PUMP_PWM_HZ, PUMP_PWM_BITS);
  ledcWrite(pins::PUMP_OUT, 0);

  Serial.begin(115200);
  Serial.println();
  Serial.printf("PUMP TEST: gate on GPIO%d, %u Hz PWM. Off for %lu s, then +%u %% every %lu s up to %u %%.\n",
                pins::PUMP_OUT, PUMP_PWM_HZ, static_cast<unsigned long>(kWaitMs / 1000), kStepPercent,
                static_cast<unsigned long>(kStepMs / 1000), kTopPercent);
  if (!pwmReady) Serial.println("PUMP TEST: the PWM output did not start; the pump stays off.");
}

void loop() {
  static int shown = -1;          // What was last printed, so each change is printed once
  const uint32_t now = millis();

  if (!pwmReady) {
    delay(1000);
    return;
  }
  if (now < kWaitMs) {
    const int left = static_cast<int>((kWaitMs - now + 999) / 1000);
    if (left != shown) {
      shown = left;
      Serial.printf("Pump off. Starting in %d s\n", left);
    }
    setPump(0);
    delay(10);
    return;
  }

  const uint32_t step = (now - kWaitMs) / kStepMs;             // 0, 1, 2, ...
  const uint32_t duty = (step + 1) * kStepPercent;
  const int percent = duty <= kTopPercent ? static_cast<int>(duty) : 0;
  const int marker = duty <= kTopPercent ? 1000 + percent : 2000;   // Never equal to a countdown number
  setPump(static_cast<uint8_t>(percent));
  if (marker != shown) {
    shown = marker;
    if (percent > 0) Serial.printf("Pump at %d %% duty\n", percent);
    else Serial.println("Test finished: pump off. Press reset to run it again.");
  }
  delay(10);
}
