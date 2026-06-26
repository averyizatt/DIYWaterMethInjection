#include "actuators.h"

#include <Arduino.h>

void PumpDriver::begin(int pwmPin, uint16_t frequencyHz, uint8_t resolutionBits) {
  pin_ = pwmPin;
  isRelay_ = false;
  if (pin_ < 0) {
    return;
  }

  resolutionBits_ = resolutionBits;
  maxDutyCount_ = (1UL << resolutionBits_) - 1UL;

  (void)frequencyHz;
  (void)resolutionBits_;
  maxDutyCount_ = 255;
  pinMode(pin_, OUTPUT);
  analogWrite(pin_, 0);
}

void PumpDriver::beginRelay(int pin, uint16_t periodMs) {
  pin_ = pin;
  isRelay_ = true;
  relayPeriodMs_ = (periodMs > 0) ? periodMs : 1000;
  if (pin_ < 0) return;
  pinMode(pin_, OUTPUT);
  digitalWrite(pin_, LOW);
}

void PumpDriver::apply(const PumpCommand &command) {
  if (pin_ < 0) {
    return;
  }

  if (isRelay_) {
    if (!command.enabled || command.dutyPercent <= 0.0f) {
      digitalWrite(pin_, LOW);
      return;
    }
    if (command.dutyPercent >= 100.0f) {
      digitalWrite(pin_, HIGH);
      return;
    }
    // Time-sliced relay PWM: ON for dutyPercent% of each period cycle.
    const uint32_t cyclePos = millis() % relayPeriodMs_;
    const uint32_t onTime = static_cast<uint32_t>((command.dutyPercent / 100.0f) * relayPeriodMs_);
    digitalWrite(pin_, cyclePos < onTime ? HIGH : LOW);
    return;
  }

  // LEDC path (not used with relay hardware)
  if (!command.enabled) {
    analogWrite(pin_, 0);
    return;
  }

  const float clampedDuty = constrain(command.dutyPercent, 0.0f, 100.0f);
  const uint32_t dutyCount = static_cast<uint32_t>((clampedDuty / 100.0f) * static_cast<float>(maxDutyCount_));
  analogWrite(pin_, static_cast<uint8_t>(dutyCount));
}

void WarningOutput::begin(int pin, bool activeHigh) {
  pin_ = pin;
  activeHigh_ = activeHigh;

  if (pin_ < 0) {
    return;
  }

  pinMode(pin_, OUTPUT);
  set(false);
}

void WarningOutput::set(bool active) {
  if (pin_ < 0) {
    return;
  }

  const bool pinHigh = activeHigh_ ? active : !active;
  digitalWrite(pin_, pinHigh ? HIGH : LOW);
}
