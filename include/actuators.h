#pragma once

#include <stdint.h>

struct PumpCommand {
  bool enabled{false};
  uint8_t pwm{0};
};

class PumpDriver {
public:
  void begin(int pwmPin);
  void apply(const PumpCommand &command);

private:
  int pin_{-1};
};
