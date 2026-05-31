#pragma once

#include "actuators.h"
#include "app_config.h"
#include "sensor_readings.h"

enum class FailsafeReason : uint8_t {
  None = 0,
  LowFluid,
  MapInvalid,
  BoostInvalid,
  InvalidBlend,
  InvalidBoostConfig
};

struct ControlResult {
  PumpCommand pump{};
  FailsafeReason failsafe{FailsafeReason::None};
  float baseDutyPercent{0.0f};
  float finalDutyPercent{0.0f};
  bool overboostAssistActive{false};
  bool overboostEmergencyActive{false};
  bool overboostAssistFaultLatched{false};
};

class InjectionController {
public:
  ControlResult update(const SensorReadings &readings, const AppConfig &config, const TankBlend &blend);
  void clearLatchedFaults() { overboostAssistFaultLatched_ = false; }

private:
  // Hysteresis prevents rapid on/off chatter when boost hovers at startPsi.
  bool sprayLatched_{false};
  bool overboostAssistFaultLatched_{false};
};
