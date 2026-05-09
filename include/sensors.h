#pragma once

#include "app_config.h"

struct SensorReadings {
  int mapRaw{0};
  float mapVoltage{0.0f};
  float mapKpa{100.0f};
  float boostPsi{0.0f};
  bool mapValid{false};
  bool tankLow{true};
};

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
  void begin(int digitalPin, bool activeLow, uint32_t debounceMs);
  bool update();
  bool isLow() const;

private:
  bool rawIsLow() const;

  int pin_{-1};
  bool activeLow_{true};
  uint32_t debounceMs_{100};
  uint32_t lastChangeMs_{0};
  bool lastRawLow_{true};
  bool debouncedLow_{true};
};
