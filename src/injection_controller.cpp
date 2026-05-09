#include "injection_controller.h"

#include <Arduino.h>

ControlResult InjectionController::update(const SensorReadings &readings, const AppConfig &config,
                                          const TankBlend &blend) const {
  ControlResult result{};
  PumpCommand &command = result.pump;

  if (config.mode == InjectionMode::Off) {
    return result;
  }

  if (config.mode == InjectionMode::Prime) {
    command.enabled = true;
    command.dutyPercent = config.dutyMinPercent;
    result.baseDutyPercent = command.dutyPercent;
    result.finalDutyPercent = command.dutyPercent;
    return result;
  }

  if (readings.tankLow) {
    result.failsafe = FailsafeReason::LowFluid;
    return result;
  }

  if (!readings.mapValid) {
    result.failsafe = FailsafeReason::MapInvalid;
    return result;
  }

  if (blend.totalLiters <= 0.0f || blend.methFraction < 0.0f || blend.methFraction > 1.0f) {
    result.failsafe = FailsafeReason::InvalidBlend;
    return result;
  }

  if (config.boost.fullPsi <= config.boost.startPsi) {
    result.failsafe = FailsafeReason::InvalidBoostConfig;
    return result;
  }

  if (config.dutyMaxPercent <= 0.0f || config.dutyMaxPercent < config.dutyMinPercent) {
    result.failsafe = FailsafeReason::InvalidBoostConfig;
    return result;
  }

  if (readings.boostPsi < config.boost.startPsi) {
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
