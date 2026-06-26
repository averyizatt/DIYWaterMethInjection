#pragma once

#include "app_config.h"
#include "sensor_readings.h"
#include <DallasTemperature.h>
#include <OneWire.h>

class MapSensor {
public:
  void begin(int analogPin, const MapCalibration &calibration);
  SensorReadings read() const;
  bool valid() const;

private:
  float kpaFromVoltage(float voltage) const;

  int pin_{-1};
  MapCalibration calibration_{};
  bool valid_{false};
};

class FloatSensor {
public:
  void begin(int digitalPin, bool activeLow, uint32_t debounceMs, uint32_t lowHoldMs);
  bool update();
  bool isLow() const;

private:
  bool rawIsLow() const;

  int pin_{-1};
  bool activeLow_{true};
  uint32_t debounceMs_{100};
  uint32_t lowHoldMs_{0};
  uint32_t lastChangeMs_{0};
  uint32_t lowSinceMs_{0};
  bool lastRawLow_{true};
  bool debouncedLow_{true};
  bool sustainedLow_{true};
};

// DS18B20 1-Wire temperature sensor.
// Call requestConversion() to kick off a non-blocking measurement, then
// readResult() at least 800 ms later to latch the value into celsius().
class TempSensor {
public:
  void begin(int dataPin);
  void requestConversion();
  void readResult();
  float celsius() const;
  float fahrenheit() const;
  bool valid() const;

private:
  int pin_{-1};
  OneWire *wire_{nullptr};
  DallasTemperature *sensors_{nullptr};
  float tempC_{-127.0f};
  bool valid_{false};
};

enum class ThermistorFault : uint8_t {
  None = 0,
  Disabled = 1,
  OpenCircuit = 2,
  ShortToGround = 3,
  OutOfRange = 4,
  InvalidConfig = 5,
  StaleReading = 6,
  AdcError = 7,
};

struct ThermistorConfig {
  bool enabled{false};
  int pin{-1};
  float pullupOhms{10000.0f};
  float adcVref{3.3f};
  uint16_t adcMaxCount{4095};
  uint8_t oversampleCount{8};
  float filterAlpha{0.20f};
  uint16_t staleTimeoutMs{1000};
  float minValidTempC{-50.0f};
  float maxValidTempC{180.0f};
  float openCircuitThresholdV{3.15f};
  float shortThresholdV{0.08f};
  float steinhartA{1.129148e-3f};
  float steinhartB{2.34125e-4f};
  float steinhartC{8.76741e-8f};
};

class ThermistorSensor {
public:
  void begin(const ThermistorConfig &config);
  void update(uint32_t nowMs);
  float valueC() const { return filteredTempC_; }
  bool valid() const { return valid_; }
  ThermistorFault fault() const { return fault_; }
  const ThermistorConfig &config() const { return config_; }

private:
  float readVoltage() const;

  ThermistorConfig config_{};
  float filteredTempC_{NAN};
  float rawVoltage_{0.0f};
  bool valid_{false};
  ThermistorFault fault_{ThermistorFault::Disabled};
  uint32_t lastUpdateMs_{0};
};

enum class PressureFault : uint8_t {
  None = 0,
  Disabled = 1,
  OpenCircuit = 2,
  ShortToGround = 3,
  OutOfRange = 4,
  InvalidConfig = 5,
  StaleReading = 6,
  AdcError = 7,
};

struct PressureConfig {
  bool enabled{false};
  int pin{-1};
  float adcVref{3.3f};
  uint16_t adcMaxCount{4095};
  uint8_t oversampleCount{8};
  float filterAlpha{0.20f};
  uint16_t staleTimeoutMs{1000};
  float dividerTopOhms{10000.0f};
  float dividerBottomOhms{20000.0f};
  float sensorMinV{0.5f};
  float sensorMaxV{4.5f};
  float pressureMinPsi{0.0f};
  float pressureMaxPsi{100.0f};
  float calibrationScale{1.0f};
  float calibrationOffsetPsi{0.0f};
  float openCircuitThresholdV{4.9f};
  float shortThresholdV{0.1f};
  float minValidPsi{-5.0f};
  float maxValidPsi{300.0f};
};

class PressureSensor {
public:
  void begin(const PressureConfig &config);
  void update(uint32_t nowMs);
  float valuePsi() const { return filteredPsi_; }
  bool valid() const { return valid_; }
  PressureFault fault() const { return fault_; }
  const PressureConfig &config() const { return config_; }

private:
  float readAdcNodeVoltage() const;

  PressureConfig config_{};
  float filteredPsi_{NAN};
  float sensorVoltage_{0.0f};
  bool valid_{false};
  PressureFault fault_{PressureFault::Disabled};
  uint32_t lastUpdateMs_{0};
};
