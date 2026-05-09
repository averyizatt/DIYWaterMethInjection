#include "injection_controller.h"

#include <Arduino.h>

PumpCommand InjectionController::update(const SensorReadings &readings, const AppConfig &config) const {
  PumpCommand command{};

  if (config.mode == InjectionMode::Off) {
    return command;
  }

  if (config.mode == InjectionMode::Prime) {
    command.enabled = true;
    command.pwm = config.pwmMin;
    return command;
  }

  if (!readings.mapValid || readings.tankLow) {
    return command;
  }

  if (readings.mapKpa < config.boost.startKpa) {
    return command;
  }

  float span = config.boost.fullKpa - config.boost.startKpa;
  float normalized = span > 0.0f ? (readings.mapKpa - config.boost.startKpa) / span : 0.0f;
  normalized = constrain(normalized, 0.0f, 1.0f);
  float trimmed = constrain(normalized * ratioTrim(config.ratio), 0.0f, 1.0f);

  command.enabled = true;
  command.pwm = static_cast<uint8_t>(config.pwmMin + (config.pwmMax - config.pwmMin) * trimmed);
  return command;
}
