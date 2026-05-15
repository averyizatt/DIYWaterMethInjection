#include "sensors.h"

#include <Arduino.h>

namespace {
constexpr float kAdcRefVoltage = 3.3f;
constexpr float kAdcMaxCount = 4095.0f;
constexpr float kPsiPerKpa = 0.1450377f;

// Unsigned subtraction is rollover-safe for millis() timestamps.
inline bool elapsed(uint32_t now, uint32_t start, uint32_t intervalMs) {
  return static_cast<uint32_t>(now - start) >= intervalMs;
}
} // namespace

void MapSensor::begin(int analogPin, const MapCalibration &calibration) {
  pin_ = analogPin;
  calibration_ = calibration;
  valid_ = pin_ >= 0 && calibration_.vMax > calibration_.vMin && calibration_.kpaMax > calibration_.kpaMin;

  if (pin_ >= 0) {
    analogReadResolution(12);
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

void FloatSensor::begin(int digitalPin, bool activeLow, uint32_t debounceMs) {
  pin_ = digitalPin;
  activeLow_ = activeLow;
  debounceMs_ = debounceMs;

  if (pin_ >= 0) {
    pinMode(pin_, INPUT_PULLUP);
  }

  lastRawLow_ = rawIsLow();
  debouncedLow_ = lastRawLow_;
  lastChangeMs_ = millis();
}

bool FloatSensor::rawIsLow() const {
  if (pin_ < 0) {
    return true;
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

  return debouncedLow_;
}

bool FloatSensor::isLow() const { return debouncedLow_; }

void TempSensor::begin(int dataPin) {
  pin_ = dataPin;
  if (pin_ < 0) {
    return;
  }
  wire_ = new OneWire(dataPin);
  sensors_ = new DallasTemperature(wire_);
  sensors_->begin();
  sensors_->setWaitForConversion(false); // non-blocking conversions
  valid_ = sensors_->getDeviceCount() > 0;
}

void TempSensor::requestConversion() {
  if (!valid_) {
    return;
  }
  sensors_->requestTemperatures();
}

void TempSensor::readResult() {
  if (!valid_) {
    return;
  }
  const float t = sensors_->getTempCByIndex(0);
  if (t != DEVICE_DISCONNECTED_C) {
    tempC_ = t;
  } else {
    valid_ = false;
  }
}

float TempSensor::celsius() const { return tempC_; }

float TempSensor::fahrenheit() const { return tempC_ * 9.0f / 5.0f + 32.0f; }

bool TempSensor::valid() const { return valid_; }
