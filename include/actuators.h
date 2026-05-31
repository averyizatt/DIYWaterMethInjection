#pragma once

#include <stdint.h>

struct PumpCommand {
  bool enabled{false};
  float dutyPercent{0.0f};
};

class PumpDriver {
public:
  // Legacy LEDC/PWM mode (not used with relay).
  void begin(int pwmPin, uint16_t frequencyHz, uint8_t resolutionBits);
  // Relay mode: time-sliced on/off at the given cycle period.
  // periodMs controls how fast the relay switches (e.g. 1000 = 1 Hz cycle).
  void beginRelay(int pin, uint16_t periodMs);
  void apply(const PumpCommand &command);

private:
  int pin_{-1};
  bool isRelay_{false};
  uint16_t relayPeriodMs_{1000};
  // LEDC fields
  int channel_{0};
  uint8_t resolutionBits_{10};
  uint32_t maxDutyCount_{1023};
};

class WarningOutput {
public:
  void begin(int pin, bool activeHigh);
  void set(bool active);

private:
  int pin_{-1};
  bool activeHigh_{true};
};
