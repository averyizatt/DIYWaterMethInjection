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

  // Common GM MAP sensors through a 47k/94k divider (ratio 0.6667).
  // Sensor outputs 0.5-4.5 V; after divider: 0.333-3.000 V at ESP32 ADC.
  config.mapType = MapSensorType::GM3Bar;
  config.map.vMin = 0.333f;
  config.map.vMax = 3.000f;
  if (config.mapType == MapSensorType::GM2Bar) {
    config.map.kpaMin = 10.0f;
    config.map.kpaMax = 200.0f;
  } else {
    config.map.kpaMin = 20.0f;
    config.map.kpaMax = 312.0f;
  }
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

  config.knock.enabled = true;
  config.knock.minRpmToArm = 0;
  config.knock.minMapKpaToArm = 120.0f;
  config.knock.sampleRateHz = 8000;
  config.knock.samplesPerUpdate = 64;
  config.knock.autoCenterFromBore = true;
  config.knock.boreMm = 96.0f;
  config.knock.centerFreqHz = 6500.0f;
  config.knock.bandwidthHz = 1800.0f;
  config.knock.signalGain = 1.0f;
  config.knock.biasAlpha = 0.002f;
  config.knock.envelopeAlpha = 0.20f;
  config.knock.rmsAlpha = 0.12f;
  config.knock.boostEnableKpa = 120.0f;
  config.knock.thresholdMultiplier = 2.5f;
  config.knock.thresholdOffset = 8.0f;
  config.knock.baselineLearnAlpha = 0.02f;
  config.knock.eventCooldownMs = 250;
  config.knock.warningThresholdCount = 2;
  config.knock.criticalThresholdCount = 4;
  config.knock.baselineLearningEnabled = true;
  config.knock.clipLowAdc = 5;
  config.knock.clipHighAdc = 4090;
  config.knock.clipPercentForFault = 30;
  config.knock.stuckAdcDelta = 3;
  config.knock.faultHoldMs = 1200;
  config.knock.missingSignalRms = 0.8f;
  config.knock.responseMode = KnockResponseMode::WarnOnly;

  return config;
}
