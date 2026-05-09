#include "sensors.h"

#include <Arduino.h>

void MapSensor::begin(int analogPin) {
  pin_ = analogPin;
  valid_ = pin_ >= 0;
}

float MapSensor::readKpa() const {
  if (pin_ < 0) {
    return 100.0f;
  }

  // Placeholder conversion until GM MAP sensor calibration is implemented.
  int raw = analogRead(pin_);
  float voltage = static_cast<float>(raw) * (3.3f / 4095.0f);
  (void)voltage;
  return 100.0f;
}

bool MapSensor::valid() const { return valid_; }

void FloatSensor::begin(int digitalPin) {
  pin_ = digitalPin;
  pinMode(pin_, INPUT_PULLUP);
}

bool FloatSensor::isLow() const {
  if (pin_ < 0) {
    return true;
  }

  // Placeholder: assumes active-low float switch.
  return digitalRead(pin_) == LOW;
}
