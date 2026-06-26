#include "sensors.h"

#include <Arduino.h>

namespace {
constexpr float kAdcRefVoltage = 5.0f;
constexpr float kAdcMaxCount = 1023.0f;
constexpr float kPsiPerKpa = 0.1450377f;

inline bool elapsed(uint32_t now, uint32_t start, uint32_t intervalMs) {
  return static_cast<uint32_t>(now - start) >= intervalMs;
}
} // namespace

void MapSensor::begin(int analogPin, const MapCalibration &calibration) {
  pin_ = analogPin;
  calibration_ = calibration;
  valid_ = pin_ >= 0 && calibration_.vMax > calibration_.vMin && calibration_.kpaMax > calibration_.kpaMin;

  if (pin_ >= 0) {
    pinMode(pin_, INPUT);
  }
}

float MapSensor::kpaFromVoltage(float voltage) const {
  const float normalized = (voltage - calibration_.vMin) / (calibration_.vMax - calibration_.vMin);
  const float clamped = constrain(normalized, 0.0f, 1.0f);
  return calibration_.kpaMin + (calibration_.kpaMax - calibration_.kpaMin) * clamped;
}

SensorReadings MapSensor::read() const {
  SensorReadings readings{};

  if (pin_ < 0) {
    readings.mapValid = false;
    return readings;
  }

  const int raw = analogRead(pin_);
  const float voltage = static_cast<float>(raw) * (kAdcRefVoltage / kAdcMaxCount);
  const float mapKpa = kpaFromVoltage(voltage);
  const float boostPsi = (mapKpa - calibration_.baroKpa) * kPsiPerKpa;

  readings.mapRaw = raw;
  readings.mapVoltage = voltage;
  readings.mapKpa = mapKpa;
  readings.boostPsi = boostPsi;
  readings.mapValid = valid_;
  return readings;
}

bool MapSensor::valid() const { return valid_; }

void FloatSensor::begin(int digitalPin, bool activeLow, uint32_t debounceMs, uint32_t lowHoldMs) {
  pin_ = digitalPin;
  activeLow_ = activeLow;
  debounceMs_ = debounceMs;
  lowHoldMs_ = lowHoldMs;

  if (pin_ >= 0) {
    pinMode(pin_, INPUT_PULLUP);
  }

  lastRawLow_ = rawIsLow();
  debouncedLow_ = lastRawLow_;
  sustainedLow_ = lastRawLow_;
  lowSinceMs_ = sustainedLow_ ? millis() : 0;
  lastChangeMs_ = millis();
}

bool FloatSensor::rawIsLow() const {
  if (pin_ < 0) {
    return false;
  }

  const bool pinHigh = digitalRead(pin_) == HIGH;
  return activeLow_ ? !pinHigh : pinHigh;
}

bool FloatSensor::update() {
  const bool rawLow = rawIsLow();
  const uint32_t now = millis();

  if (rawLow != lastRawLow_) {
    lastRawLow_ = rawLow;
    lastChangeMs_ = now;
  }

  if (elapsed(now, lastChangeMs_, debounceMs_)) {
    debouncedLow_ = rawLow;
  }

  if (debouncedLow_) {
    if (lowSinceMs_ == 0) lowSinceMs_ = now;
    sustainedLow_ = elapsed(now, lowSinceMs_, lowHoldMs_);
  } else {
    lowSinceMs_ = 0;
    sustainedLow_ = false;
  }

  return sustainedLow_;
}

bool FloatSensor::isLow() const { return sustainedLow_; }
