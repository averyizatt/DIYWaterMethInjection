#include "sensors.h"

#include <Arduino.h>
#include <math.h>

namespace {
constexpr float kAdcRefVoltage = 3.3f;
constexpr float kAdcMaxCount = 4095.0f;
constexpr float kPsiPerKpa = 0.1450377f;
constexpr float kMinFilterAlpha = 0.01f;
constexpr float kMaxFilterAlpha = 1.0f;
constexpr float kMinResistanceOhms = 1.0f;
constexpr float kKelvinOffset = 273.15f;

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
    pinMode(pin_, INPUT);
    analogReadResolution(12);
#if defined(ADC_11db)
    analogSetPinAttenuation(pin_, ADC_11db);
#endif
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

void ThermistorSensor::begin(const ThermistorConfig &config) {
  config_ = config;
  filteredTempC_ = NAN;
  valid_ = false;
  fault_ = config_.enabled ? ThermistorFault::StaleReading : ThermistorFault::Disabled;
  lastUpdateMs_ = 0;

  if (!config_.enabled) return;
  if (config_.pin < 0 || config_.adcMaxCount == 0 || config_.pullupOhms <= 1.0f ||
      config_.adcVref <= 0.1f) {
    fault_ = ThermistorFault::InvalidConfig;
    return;
  }

  pinMode(config_.pin, INPUT);
  analogReadResolution(12);
#if defined(ADC_11db)
  analogSetPinAttenuation(config_.pin, ADC_11db);
#endif
}

float ThermistorSensor::readVoltage() const {
  const uint8_t samples = config_.oversampleCount == 0 ? 1 : config_.oversampleCount;
  uint32_t acc = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    acc += static_cast<uint32_t>(analogRead(config_.pin));
  }
  const float avg = static_cast<float>(acc) / static_cast<float>(samples);
  return (avg / static_cast<float>(config_.adcMaxCount)) * config_.adcVref;
}

void ThermistorSensor::update(uint32_t nowMs) {
  if (!config_.enabled) {
    valid_ = false;
    fault_ = ThermistorFault::Disabled;
    return;
  }
  if (fault_ == ThermistorFault::InvalidConfig) return;

  rawVoltage_ = readVoltage();
  if (!isfinite(rawVoltage_) || rawVoltage_ < 0.0f || rawVoltage_ > config_.adcVref + 0.01f) {
    valid_ = false;
    fault_ = ThermistorFault::AdcError;
    return;
  }
  if (rawVoltage_ >= config_.openCircuitThresholdV) {
    valid_ = false;
    fault_ = ThermistorFault::OpenCircuit;
    return;
  }
  if (rawVoltage_ <= config_.shortThresholdV) {
    valid_ = false;
    fault_ = ThermistorFault::ShortToGround;
    return;
  }

  const float denom = config_.adcVref - rawVoltage_;
  if (denom <= 0.0001f) {
    valid_ = false;
    fault_ = ThermistorFault::OpenCircuit;
    return;
  }
  const float resistance = (config_.pullupOhms * rawVoltage_) / denom;
  if (!isfinite(resistance) || resistance < kMinResistanceOhms) {
    valid_ = false;
    fault_ = ThermistorFault::AdcError;
    return;
  }

  const float lnR = logf(resistance);
  const float invT = config_.steinhartA + config_.steinhartB * lnR + config_.steinhartC * lnR * lnR * lnR;
  if (invT <= 0.0f) {
    valid_ = false;
    fault_ = ThermistorFault::OutOfRange;
    return;
  }
  const float tempC = (1.0f / invT) - kKelvinOffset;
  if (!isfinite(tempC) || tempC < config_.minValidTempC || tempC > config_.maxValidTempC) {
    valid_ = false;
    fault_ = ThermistorFault::OutOfRange;
    return;
  }

  if (!isfinite(filteredTempC_)) filteredTempC_ = tempC;
  else filteredTempC_ += (tempC - filteredTempC_) * constrain(config_.filterAlpha, kMinFilterAlpha, kMaxFilterAlpha);

  valid_ = true;
  fault_ = ThermistorFault::None;
  lastUpdateMs_ = nowMs;
}

void PressureSensor::begin(const PressureConfig &config) {
  config_ = config;
  filteredPsi_ = NAN;
  sensorVoltage_ = 0.0f;
  valid_ = false;
  fault_ = config_.enabled ? PressureFault::StaleReading : PressureFault::Disabled;
  lastUpdateMs_ = 0;

  if (!config_.enabled) return;
  if (config_.pin < 0 || config_.adcMaxCount == 0 || config_.adcVref <= 0.1f ||
      config_.dividerTopOhms <= 0.1f || config_.dividerBottomOhms <= 0.1f ||
      config_.sensorMaxV <= config_.sensorMinV || config_.pressureMaxPsi <= config_.pressureMinPsi) {
    fault_ = PressureFault::InvalidConfig;
    return;
  }

  pinMode(config_.pin, INPUT);
  analogReadResolution(12);
#if defined(ADC_11db)
  analogSetPinAttenuation(config_.pin, ADC_11db);
#endif
}

float PressureSensor::readAdcNodeVoltage() const {
  const uint8_t samples = config_.oversampleCount == 0 ? 1 : config_.oversampleCount;
  uint32_t acc = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    acc += static_cast<uint32_t>(analogRead(config_.pin));
  }
  const float avg = static_cast<float>(acc) / static_cast<float>(samples);
  return (avg / static_cast<float>(config_.adcMaxCount)) * config_.adcVref;
}

void PressureSensor::update(uint32_t nowMs) {
  if (!config_.enabled) {
    valid_ = false;
    fault_ = PressureFault::Disabled;
    return;
  }
  if (fault_ == PressureFault::InvalidConfig) return;

  const float adcNodeV = readAdcNodeVoltage();
  if (!isfinite(adcNodeV) || adcNodeV < 0.0f || adcNodeV > config_.adcVref + 0.01f) {
    valid_ = false;
    fault_ = PressureFault::AdcError;
    return;
  }

  const float dividerScale = (config_.dividerTopOhms + config_.dividerBottomOhms) / config_.dividerBottomOhms;
  sensorVoltage_ = adcNodeV * dividerScale;
  if (!isfinite(sensorVoltage_) || sensorVoltage_ < -0.01f || sensorVoltage_ > 5.5f) {
    valid_ = false;
    fault_ = PressureFault::AdcError;
    return;
  }
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

  const float norm = (sensorVoltage_ - config_.sensorMinV) / (config_.sensorMaxV - config_.sensorMinV);
  float psi = config_.pressureMinPsi + norm * (config_.pressureMaxPsi - config_.pressureMinPsi);
  psi = (psi * config_.calibrationScale) + config_.calibrationOffsetPsi;
  if (!isfinite(psi) || psi < config_.minValidPsi || psi > config_.maxValidPsi) {
    valid_ = false;
    fault_ = PressureFault::OutOfRange;
    return;
  }

  if (!isfinite(filteredPsi_)) filteredPsi_ = psi;
  else filteredPsi_ += (psi - filteredPsi_) * constrain(config_.filterAlpha, kMinFilterAlpha, kMaxFilterAlpha);

  valid_ = true;
  fault_ = PressureFault::None;
  lastUpdateMs_ = nowMs;
}
