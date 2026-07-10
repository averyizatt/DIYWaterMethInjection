#include "sensors.h"

#include <Arduino.h>
#include <math.h>

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

void PressureSensor::begin(const PressureConfig &config) {
  config_ = config;
  valid_ = false;
  fault_ = config_.enabled ? PressureFault::None : PressureFault::Disabled;
  filteredPsi_ = NAN;

  if (config_.enabled && config_.pin >= 0) {
    pinMode(config_.pin, INPUT);
  }
}

float PressureSensor::readAdcNodeVoltage() const {
  if (!config_.enabled || config_.pin < 0 || config_.adcMaxCount == 0) {
    return NAN;
  }

  const uint8_t samples = config_.oversampleCount == 0 ? 1 : config_.oversampleCount;
  uint32_t total = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    total += static_cast<uint16_t>(analogRead(config_.pin));
  }

  const float raw = static_cast<float>(total) / static_cast<float>(samples);
  return raw * (config_.adcVref / static_cast<float>(config_.adcMaxCount));
}

void PressureSensor::update(uint32_t nowMs) {
  (void)nowMs;

  if (!config_.enabled || config_.pin < 0) {
    valid_ = false;
    fault_ = PressureFault::Disabled;
    return;
  }

  if (config_.sensorMaxV <= config_.sensorMinV ||
      config_.pressureMaxPsi <= config_.pressureMinPsi ||
      config_.dividerBottomOhms <= 0.0f ||
      config_.adcMaxCount == 0) {
    valid_ = false;
    fault_ = PressureFault::InvalidConfig;
    return;
  }

  const float adcNodeVoltage = readAdcNodeVoltage();
  if (!isfinite(adcNodeVoltage)) {
    valid_ = false;
    fault_ = PressureFault::AdcError;
    return;
  }

  const float dividerScale =
      (config_.dividerTopOhms + config_.dividerBottomOhms) / config_.dividerBottomOhms;
  sensorVoltage_ = adcNodeVoltage * dividerScale;

  if (sensorVoltage_ >= config_.openCircuitThresholdV) {
    valid_ = false;
    fault_ = PressureFault::OpenCircuit;
    return;
  }
  if (sensorVoltage_ <= config_.shortThresholdV) {
    valid_ = false;
    fault_ = PressureFault::ShortToGround;
    return;
  }

  const float normalized =
      (sensorVoltage_ - config_.sensorMinV) / (config_.sensorMaxV - config_.sensorMinV);
  float psi = config_.pressureMinPsi +
              ((config_.pressureMaxPsi - config_.pressureMinPsi) * normalized);
  psi = (psi * config_.calibrationScale) + config_.calibrationOffsetPsi;

  if (psi < config_.minValidPsi || psi > config_.maxValidPsi) {
    valid_ = false;
    fault_ = PressureFault::OutOfRange;
    return;
  }

  if (!isfinite(filteredPsi_)) {
    filteredPsi_ = psi;
  } else {
    filteredPsi_ += (psi - filteredPsi_) * constrain(config_.filterAlpha, 0.0f, 1.0f);
  }

  valid_ = true;
  fault_ = PressureFault::None;
}
