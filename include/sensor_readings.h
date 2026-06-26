#pragma once

#include <stdint.h>

struct SensorReadings {
  int mapRaw{0};
  float mapVoltage{0.0f};
  float mapKpa{100.0f};
  float boostPsi{0.0f};
  bool mapValid{false};
  bool tankLow{true};
  float iatC{0.0f};
  float engineBayC{0.0f};
  float cabinC{0.0f};
  float ambientC{0.0f};
  float oilPressurePsi{0.0f};
  float fuelPressurePsi{0.0f};
  float methPressurePsi{0.0f};
  float boostRefPressurePsi{0.0f};
  float sparePressure1Psi{0.0f};
  float sparePressure2Psi{0.0f};
  bool iatValid{false};
  bool engineBayValid{false};
  bool cabinValid{false};
  bool ambientValid{false};
  bool oilPressureValid{false};
  bool fuelPressureValid{false};
  bool methPressureValid{false};
  bool boostRefPressureValid{false};
  bool sparePressure1Valid{false};
  bool sparePressure2Valid{false};
  uint16_t analogFaultFlags{0};
};
