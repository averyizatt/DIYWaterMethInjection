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

  // GM MAP sensor wired directly to the Nano ADC.
  // Sensor outputs 0.5-4.5 V across the Nano's 0-5 V ADC range.
  config.mapType = MapSensorType::GM3Bar;
  config.map.vMin = 0.500f;
  config.map.vMax = 4.500f;
  if (config.mapType == MapSensorType::GM2Bar) {
    config.map.kpaMin = 10.0f;
    config.map.kpaMax = 200.0f;
  } else {
    config.map.kpaMin = 20.0f;
    config.map.kpaMax = 312.0f;
  }
  config.map.baroKpa = 90.0f;
  config.boost.startPsi = 3.5f;
  config.boost.fullPsi = 7.5f;
  config.boost.overboostWarnPsi = 13.5f;
  config.boost.overboostEmergencyPsi = 15.0f;

  // Gain chosen to be conservative for cooling/knock margin use.
  // At 25% meth and +4 psi over start, base duty is about 90%.
  // Duty is then clamped and forced full near fullPsi threshold.
  config.gainK = 30.0f;
  config.dutyMinPercent = 18.0f;
  config.dutyMaxPercent = 100.0f;
  config.overboostWarnDutyPercent = 85.0f;

  // Relay switching period: 1 second per cycle (1 Hz), safe for all mechanical relays.
  config.relayPeriodMs = 1000;

  config.floatActiveLow = true;
  config.floatDebounceMs = 100;
  config.floatLowShutdownDelayMs = 2000;

  config.serialBaud = 115200;
  config.debugPeriodMs = 250;
  config.loopPeriodMs = 20;
  config.benchTestDutyPercent = 100;

  config.knock.enabled = true;
  config.knock.minRpmToArm = 0;
  config.knock.minMapKpaToArm = 120.0f;
  config.knock.minLoadPercentToArm = 35.0f;
  config.knock.sampleRateHz = 20000;
  config.knock.samplesPerUpdate = 64;
  config.knock.autoCenterFromBore = true;
  config.knock.boreMm = 87.5f;
  config.knock.centerFreqHz = 5000.0f;
  config.knock.bandwidthHz = 1800.0f;
  config.knock.multiBandSpread = 0.14f;
  config.knock.fftEnabled = true;
  config.knock.fftMinSnrDb = 3.0f;
  config.knock.fftWeight = 0.40f;
  config.knock.fftShortWeight = 0.45f;
  config.knock.fftHarmonicWeight = 0.30f;
  config.knock.spectralTemplateWeight = 0.25f;
  config.knock.mapRateGateKpaPerSec = 140.0f;
  config.knock.transientThresholdScale = 1.30f;
  config.knock.transientHoldMs = 180;
  config.knock.minDetectConfidence = 0.52f;
  config.knock.warnConfidence = 0.58f;
  config.knock.criticalConfidence = 0.72f;
  config.knock.confidenceWeightSpectral = 0.35f;
  config.knock.confidenceWeightFft = 0.30f;
  config.knock.confidenceWeightHarmonic = 0.20f;
  config.knock.confidenceWeightTemplate = 0.15f;
  config.knock.iatTempCompStartC = 40.0f;
  config.knock.iatTempCompPerC = knock_sensor_specs::kSensitivityTempScalePerC;
  config.knock.bayTempCompStartC = 60.0f;
  config.knock.bayTempCompPerC = knock_sensor_specs::kSensitivityTempScalePerC;
  config.knock.maxTempCompScale = 1.45f;
  config.knock.profileScaleIdle = 1.35f;
  config.knock.profileScaleSpool = 1.20f;
  config.knock.profileScaleSteady = 1.00f;
  config.knock.profileScaleLift = 1.15f;
  config.knock.profileScaleHeatSoak = 1.28f;
  config.knock.longTermBaselineAlpha = 0.003f;
  config.knock.driftWarnPercent = 35.0f;
  config.knock.driftCriticalPercent = 65.0f;
  config.knock.adaptiveMultMin = 1.2f;
  config.knock.adaptiveMultMax = 3.8f;
  config.knock.adaptiveMultLearnAlpha = 0.015f;
  config.knock.riskMapRateWeight = 0.45f;
  config.knock.riskConfidenceWeight = 0.35f;
  config.knock.riskEventWeight = 0.20f;
  config.knock.conservativeHealthThreshold = 65.0f;
  config.knock.failsafeHealthThreshold = 40.0f;
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
  config.knock.clipHighAdc = 1018;
  config.knock.clipPercentForFault = 30;
  config.knock.stuckAdcDelta = 3;
  config.knock.faultHoldMs = 1200;
  config.knock.missingSignalRms = 0.8f;
  config.knock.responseMode = KnockResponseMode::WarnOnly;

  return config;
}
