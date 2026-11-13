#pragma once

#include "app_config.h"
#include "sensor_readings.h"

// Whether injection is wanted, and at what duty (the overboost assist reads the duty).
struct PumpCommand {
  bool enabled{false};
  float dutyPercent{0.0f};
};

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

inline bool manualPumpTestSafe(const SensorReadings& readings, const AppConfig& config,
                               const ControlResult& result) {
  return config.mode == InjectionMode::Off &&
         (!config.tankProtectionEnabled || !readings.tankLow) && readings.mapValid &&
         result.failsafe == FailsafeReason::None && !result.overboostAssistFaultLatched;
}

class InjectionController {
public:
  ControlResult update(const SensorReadings &readings, const AppConfig &config, const TankBlend &blend);
  void clearLatchedFaults() { overboostAssistFaultLatched_ = false; }

private:
  // Hysteresis prevents rapid on/off chatter when boost hovers at startPsi.
  bool sprayLatched_{false};
  bool overboostAssistFaultLatched_{false};
};
