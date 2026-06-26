#include "injection_controller.h"

#include <math.h>

namespace {
constexpr float kBoostHysteresisPsi = 0.6f;

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}
}

ControlResult InjectionController::update(const SensorReadings &readings, const AppConfig &config,
                                          const TankBlend &blend) {
  ControlResult result{};
  PumpCommand &command = result.pump;
  result.overboostAssistFaultLatched = overboostAssistFaultLatched_;

  if (config.mode == InjectionMode::Off) {
    sprayLatched_ = false;
    result.overboostAssistFaultLatched = overboostAssistFaultLatched_;
    return result;
  }

  if (config.mode == InjectionMode::Prime) {
    sprayLatched_ = true;
    command.enabled = true;
    command.dutyPercent = config.dutyMinPercent;
    result.baseDutyPercent = command.dutyPercent;
    result.finalDutyPercent = command.dutyPercent;
    result.overboostAssistFaultLatched = overboostAssistFaultLatched_;
    return result;
  }

  if (readings.tankLow) {
    sprayLatched_ = false;
    result.failsafe = FailsafeReason::LowFluid;
    result.overboostAssistFaultLatched = overboostAssistFaultLatched_;
    return result;
  }

  if (!readings.mapValid) {
    sprayLatched_ = false;
    result.failsafe = FailsafeReason::MapInvalid;
    result.overboostAssistFaultLatched = overboostAssistFaultLatched_;
    return result;
  }

  if (!isfinite(readings.boostPsi)) {
    sprayLatched_ = false;
    result.failsafe = FailsafeReason::BoostInvalid;
    result.overboostAssistFaultLatched = overboostAssistFaultLatched_;
    return result;
  }

  if (blend.totalLiters <= 0.0f || blend.methFraction < 0.0f || blend.methFraction > 1.0f) {
    sprayLatched_ = false;
    result.failsafe = FailsafeReason::InvalidBlend;
    result.overboostAssistFaultLatched = overboostAssistFaultLatched_;
    return result;
  }

  if (config.boost.fullPsi <= config.boost.startPsi ||
      config.boost.overboostWarnPsi < config.boost.startPsi ||
      config.boost.overboostEmergencyPsi < config.boost.overboostWarnPsi) {
    sprayLatched_ = false;
    result.failsafe = FailsafeReason::InvalidBoostConfig;
    result.overboostAssistFaultLatched = overboostAssistFaultLatched_;
    return result;
  }

  if (config.dutyMaxPercent <= 0.0f || config.dutyMaxPercent < config.dutyMinPercent ||
      config.overboostWarnDutyPercent < 0.0f || config.overboostWarnDutyPercent > 100.0f) {
    sprayLatched_ = false;
    result.failsafe = FailsafeReason::InvalidBoostConfig;
    result.overboostAssistFaultLatched = overboostAssistFaultLatched_;
    return result;
  }

  const float onThreshold = config.boost.startPsi + kBoostHysteresisPsi;
  const float offThreshold = config.boost.startPsi - kBoostHysteresisPsi;
  const bool emergencyNow = readings.boostPsi >= config.boost.overboostEmergencyPsi;
  const bool warnNow = readings.boostPsi >= config.boost.overboostWarnPsi;

  if (emergencyNow) {
    overboostAssistFaultLatched_ = true;
  }

  if (readings.boostPsi >= onThreshold) {
    sprayLatched_ = true;
  } else if (readings.boostPsi <= offThreshold) {
    sprayLatched_ = false;
  }

  if (!sprayLatched_) {
    result.overboostAssistFaultLatched = overboostAssistFaultLatched_;
    return result;
  }

  // Requested base model:
  // Duty = K(1 - M)(Pboost - Pstart), then constrained to configured duty bounds.
  const float methFraction = clampFloat(blend.methFraction, 0.0f, 1.0f);
  const float boostDelta = readings.boostPsi - config.boost.startPsi;
  result.baseDutyPercent = config.gainK * (1.0f - methFraction) * boostDelta;
  float dutyPercent = result.baseDutyPercent;

  // Progressive ramp to full by the configured fullPsi threshold.
  if (readings.boostPsi >= config.boost.fullPsi) {
    dutyPercent = config.dutyMaxPercent;
  }

  if (result.baseDutyPercent <= 0.0f) {
    dutyPercent = 0.0f;
  } else {
    dutyPercent = clampFloat(dutyPercent, config.dutyMinPercent, config.dutyMaxPercent);
  }

  if (warnNow) {
    result.overboostAssistActive = true;
    const float warnDuty = clampFloat(config.overboostWarnDutyPercent,
                                      config.dutyMinPercent,
                                      config.dutyMaxPercent);
    dutyPercent = dutyPercent > warnDuty ? dutyPercent : warnDuty;
  }

  if (emergencyNow) {
    result.overboostEmergencyActive = true;
    result.overboostAssistActive = true;
    dutyPercent = 100.0f;
  }

  result.finalDutyPercent = dutyPercent;
  result.overboostAssistFaultLatched = overboostAssistFaultLatched_;

  command.enabled = true;
  command.dutyPercent = dutyPercent;
  return result;
}
