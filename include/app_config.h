#pragma once

#include "modes.h"

enum class MapSensorType : uint8_t {
  GM2Bar = 0,
  GM3Bar
};

struct MapCalibration {
  // Typical GM linear MAP output region.
  float vMin{0.50f};
  float vMax{4.50f};

  // Select based on sensor type (examples provided in defaultConfig()).
  float kpaMin{10.0f};
  float kpaMax{200.0f};

  // Local atmospheric pressure estimate used for boost conversion.
  float baroKpa{101.325f};
};

struct BoostThresholdsPsi {
  // Conservative defaults for a low-boost street setup.
  // SPRAY_START_PSI
  float startPsi{3.5f};
  // FULL_SPRAY_PSI
  float fullPsi{7.5f};
  // OVERBOOST_WARN_PSI (assist warning/high-duty threshold)
  float overboostWarnPsi{13.5f};
  // OVERBOOST_EMERGENCY_PSI (latched overboost-assist fault threshold)
  float overboostEmergencyPsi{15.0f};
};

enum class KnockResponseMode : uint8_t {
  LogOnly = 0,
  WarnOnly = 1,
  ForceSpray = 2,
  SafetyShutdown = 3,
};

namespace knock_sensor_specs {
constexpr float kFreqMinHz = 3000.0f;
constexpr float kFreqMaxHz = 25000.0f;
// Keep > 2x 3 kHz with Nyquist safety factor margin (0.45f usable band).
constexpr uint16_t kMinSampleRateHz = 7000U;
constexpr uint16_t kMaxSampleRateHz = 25000U;
constexpr float kNyquistSafetyFactor = 0.45f;
constexpr float kResonanceHz = 30000.0f;
constexpr float kSensitivityAt5kMvPerG = 26.0f;
constexpr float kSensitivityTempMvPerGC = 0.04f;
constexpr float kOperatingTempMinC = -40.0f;
constexpr float kOperatingTempMaxC = 150.0f;
constexpr float kSensitivityTempScalePerC = kSensitivityTempMvPerGC / kSensitivityAt5kMvPerG;
} // namespace knock_sensor_specs

struct KnockConfig {
  bool enabled{true};
  // Detection is only active when both arming conditions are met.
  // RPM can remain unused if no reliable crank signal is available.
  uint16_t minRpmToArm{0};
  float minMapKpaToArm{120.0f};
  float minLoadPercentToArm{30.0f};

  // Front-end ADC sampling and analysis block sizing.
  uint16_t sampleRateHz{20000};
  uint8_t samplesPerUpdate{64};

  // Knock band targeting. Sensor operating frequency range is 3-25 kHz.
  // If autoCenterFromBore is true, centerFreqHz is
  // estimated from bore and harmonic assumptions at runtime.
  bool autoCenterFromBore{true};
  float boreMm{87.5f};
  float centerFreqHz{5000.0f};
  float bandwidthHz{1800.0f};
  float multiBandSpread{0.14f};
  bool fftEnabled{true};
  float fftMinSnrDb{3.0f};
  float fftWeight{0.40f};
  float fftShortWeight{0.45f};
  float fftHarmonicWeight{0.30f};
  float spectralTemplateWeight{0.25f};

  // During fast MAP transients, detection thresholds are raised briefly to
  // reject spool/tip-in mechanical bursts that are not true combustion knock.
  float mapRateGateKpaPerSec{140.0f};
  float transientThresholdScale{1.30f};
  uint16_t transientHoldMs{180};

  float minDetectConfidence{0.52f};
  float warnConfidence{0.58f};
  float criticalConfidence{0.72f};
  float confidenceWeightSpectral{0.35f};
  float confidenceWeightFft{0.30f};
  float confidenceWeightHarmonic{0.20f};
  float confidenceWeightTemplate{0.15f};

  float iatTempCompStartC{40.0f};
  float iatTempCompPerC{knock_sensor_specs::kSensitivityTempScalePerC};
  float bayTempCompStartC{60.0f};
  float bayTempCompPerC{knock_sensor_specs::kSensitivityTempScalePerC};
  float maxTempCompScale{1.45f};

  float profileScaleIdle{1.35f};
  float profileScaleSpool{1.20f};
  float profileScaleSteady{1.00f};
  float profileScaleLift{1.15f};
  float profileScaleHeatSoak{1.28f};

  float longTermBaselineAlpha{0.003f};
  float driftWarnPercent{35.0f};
  float driftCriticalPercent{65.0f};
  float adaptiveMultMin{1.2f};
  float adaptiveMultMax{3.8f};
  float adaptiveMultLearnAlpha{0.015f};

  float riskMapRateWeight{0.45f};
  float riskConfidenceWeight{0.35f};
  float riskEventWeight{0.20f};
  float conservativeHealthThreshold{65.0f};
  float failsafeHealthThreshold{40.0f};

  // Sensor and DSP scaling.
  float signalGain{1.0f};
  float biasAlpha{0.002f};
  float envelopeAlpha{0.20f};
  float rmsAlpha{0.12f};

  // Adjustable thresholding.
  float thresholdOffset{8.0f};
  float boostEnableKpa{120.0f};
  float thresholdMultiplier{2.5f};
  bool baselineLearningEnabled{true};
  float baselineLearnAlpha{0.02f};

  // Event timing and severity integration.
  uint16_t eventCooldownMs{250};
  uint8_t warningThresholdCount{2};
  uint8_t criticalThresholdCount{4};

  // Sensor sanity/fault detection controls.
  uint16_t clipLowAdc{5};
  uint16_t clipHighAdc{1018};
  uint8_t clipPercentForFault{30};
  uint16_t stuckAdcDelta{3};
  uint16_t faultHoldMs{1200};
  float missingSignalRms{0.8f};

  KnockResponseMode responseMode{KnockResponseMode::WarnOnly};
};

struct TankBlend {
  float waterLiters{1.5f};
  float methLiters{0.5f};
  float totalLiters{2.0f};
  float methPercent{25.0f};
  float methFraction{0.25f};
};

struct AppConfig {
  InjectionMode mode{InjectionMode::BoostOnly};

  MapSensorType mapType{MapSensorType::GM3Bar};
  MapCalibration map{};
  BoostThresholdsPsi boost{};

  // Duty = K * (1 - M) * (Pboost - Pstart), duty in percent.
  // K unit: percent per psi.
  float gainK{30.0f};

  // Conservative minimum duty once injection is active.
  float dutyMinPercent{18.0f};
  float dutyMaxPercent{100.0f};
  // High duty commanded while OVERBOOST_WARN_PSI is active.
  float overboostWarnDutyPercent{85.0f};

  // Relay time-slice period in ms. 1000 = 1 Hz switching (safe for any relay).
  // Lower = more responsive flow control but more wear. 500ms (2 Hz) is a reasonable max.
  uint16_t relayPeriodMs{1000};

  // true = LOW at pin means low-fluid (switch closes to GND when level is low).
  // With INPUT_PULLUP, a disconnected/open wire reads HIGH and is treated as full.
  bool floatActiveLow{true};
  uint32_t floatDebounceMs{100};
  uint32_t floatLowShutdownDelayMs{2000};

  uint32_t serialBaud{115200};
  uint32_t debugPeriodMs{250};
  uint32_t loopPeriodMs{20};

  // Duty applied when the bench-test button is held down.
  // Keep this at 100% by default so non-serial bench checks are easy to verify with a meter.
  uint8_t benchTestDutyPercent{100};

  KnockConfig knock{};
};

AppConfig defaultConfig();

TankBlend computeTankBlend(float waterLiters, float methLiters);
