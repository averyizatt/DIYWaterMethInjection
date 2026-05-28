#include "injection_controller.h"

#include <Arduino.h>

namespace {
constexpr float kBoostHysteresisPsi = 0.6f;
}

ControlResult InjectionController::update(const SensorReadings &readings, const AppConfig &config,
                                          const TankBlend &blend) {
  ControlResult result{};
  PumpCommand &command = result.pump;

  if (config.mode == InjectionMode::Off) {
    sprayLatched_ = false;
    return result;
  }

  if (config.mode == InjectionMode::Prime) {
    sprayLatched_ = true;
    command.enabled = true;
    command.dutyPercent = config.dutyMinPercent;
    result.baseDutyPercent = command.dutyPercent;
    result.finalDutyPercent = command.dutyPercent;
    return result;
  }

  if (readings.tankLow) {
    sprayLatched_ = false;
    result.failsafe = FailsafeReason::LowFluid;
    return result;
  }

  if (!readings.mapValid) {
    sprayLatched_ = false;
    result.failsafe = FailsafeReason::MapInvalid;
    return result;
  }

  if (blend.totalLiters <= 0.0f || blend.methFraction < 0.0f || blend.methFraction > 1.0f) {
    sprayLatched_ = false;
    result.failsafe = FailsafeReason::InvalidBlend;
    return result;
  }

  if (config.boost.fullPsi <= config.boost.startPsi) {
    sprayLatched_ = false;
    result.failsafe = FailsafeReason::InvalidBoostConfig;
    return result;
  }

  if (config.dutyMaxPercent <= 0.0f || config.dutyMaxPercent < config.dutyMinPercent) {
    sprayLatched_ = false;
    result.failsafe = FailsafeReason::InvalidBoostConfig;
    return result;
  }

  const float onThreshold = config.boost.startPsi + kBoostHysteresisPsi;
  const float offThreshold = config.boost.startPsi - kBoostHysteresisPsi;

  if (readings.boostPsi >= onThreshold) {
    sprayLatched_ = true;
  } else if (readings.boostPsi <= offThreshold) {
    sprayLatched_ = false;
  }

  if (!sprayLatched_) {
    return result;
  }

  // Requested base model:
  // Duty = K(1 - M)(Pboost - Pstart), then constrained to configured duty bounds.
  const float methFraction = constrain(blend.methFraction, 0.0f, 1.0f);
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
    dutyPercent = constrain(dutyPercent, config.dutyMinPercent, config.dutyMaxPercent);
  }

  result.finalDutyPercent = dutyPercent;

  command.enabled = true;
  command.dutyPercent = dutyPercent;
  return result;
}
