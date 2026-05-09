#include "app_config.h"

TankBlend computeTankBlend(float waterLiters, float methLiters) {
  TankBlend blend{};
  blend.waterLiters = waterLiters >= 0.0f ? waterLiters : 0.0f;
  blend.methLiters = methLiters >= 0.0f ? methLiters : 0.0f;
  blend.totalLiters = blend.waterLiters + blend.methLiters;

  if (blend.totalLiters <= 0.0f) {
    blend.methPercent = 0.0f;
    blend.methFraction = 0.0f;
    return blend;
  }

  // Core requested equation: Meth% = Vm / (Vm + Vw) * 100.
  blend.methPercent = (blend.methLiters / blend.totalLiters) * 100.0f;
  blend.methFraction = blend.methPercent / 100.0f;
  return blend;
}

AppConfig defaultConfig() {
  AppConfig config{};

  // Default to GM 3-bar example calibration.
  config.mapType = MapSensorType::GM3Bar;
  config.map.vMin = 0.50f;
  config.map.vMax = 4.50f;
  config.map.kpaMin = 20.0f;  // Example value for many GM 3-bar sensors.
  config.map.kpaMax = 312.0f; // Example value for many GM 3-bar sensors.
  config.map.baroKpa = 101.325f;

  // Gain chosen to be conservative for cooling/knock margin use.
  // At 25% meth and +4 psi over start, base duty is about 90%.
  // Duty is then clamped and forced full near fullPsi threshold.
  config.gainK = 30.0f;
  config.dutyMinPercent = 18.0f;
  config.dutyMaxPercent = 100.0f;

  // 50-200 Hz suggested pump PWM range; default at 100 Hz.
  config.pwmFrequencyHz = 100;
  config.pwmResolutionBits = 10;

  config.floatActiveLow = true;
  config.floatDebounceMs = 100;

  config.serialBaud = 115200;
  config.debugPeriodMs = 250;
  config.loopPeriodMs = 20;

  return config;
}
