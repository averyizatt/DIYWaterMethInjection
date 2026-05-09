#pragma once

#include "actuators.h"
#include "app_config.h"
#include "sensors.h"

enum class FailsafeReason : uint8_t {
  None = 0,
  LowFluid,
  MapInvalid,
  InvalidBlend,
  InvalidBoostConfig
};

struct ControlResult {
  PumpCommand pump{};
  FailsafeReason failsafe{FailsafeReason::None};
  float baseDutyPercent{0.0f};
  float finalDutyPercent{0.0f};
};

class InjectionController {
public:
  ControlResult update(const SensorReadings &readings, const AppConfig &config, const TankBlend &blend) const;
};
