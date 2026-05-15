#pragma once

#include <stdint.h>

struct PumpCommand {
  bool enabled{false};
  float dutyPercent{0.0f};
};

class PumpDriver {
public:
  void begin(int pwmPin, uint16_t frequencyHz, uint8_t resolutionBits);
  void apply(const PumpCommand &command);

private:
  int pin_{-1};
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
