#include "actuators.h"

#include <Arduino.h>

void PumpDriver::begin(int pwmPin, uint16_t frequencyHz, uint8_t resolutionBits) {
  pin_ = pwmPin;
  if (pin_ < 0) {
    return;
  }

  resolutionBits_ = resolutionBits;
  maxDutyCount_ = (1UL << resolutionBits_) - 1UL;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  // Arduino ESP32 v3.x unified LEDC API.
  ledcAttachChannel(pin_, frequencyHz, resolutionBits_, channel_);
#else
  ledcSetup(channel_, frequencyHz, resolutionBits_);
  ledcAttachPin(pin_, channel_);
#endif
  ledcWrite(channel_, 0);
}

void PumpDriver::apply(const PumpCommand &command) {
  if (pin_ < 0) {
    return;
  }

  if (!command.enabled) {
    ledcWrite(channel_, 0);
    return;
  }

  const float clampedDuty = constrain(command.dutyPercent, 0.0f, 100.0f);
  const uint32_t dutyCount = static_cast<uint32_t>((clampedDuty / 100.0f) * static_cast<float>(maxDutyCount_));
  ledcWrite(channel_, dutyCount);
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
