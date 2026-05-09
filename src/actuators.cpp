#include "actuators.h"

#include <Arduino.h>

void PumpDriver::begin(int pwmPin) {
  pin_ = pwmPin;
  pinMode(pin_, OUTPUT);
  analogWrite(pin_, 0);
}

void PumpDriver::apply(const PumpCommand &command) {
  if (pin_ < 0) {
    return;
  }

  if (!command.enabled) {
    analogWrite(pin_, 0);
    return;
  }

  analogWrite(pin_, command.pwm);
}
