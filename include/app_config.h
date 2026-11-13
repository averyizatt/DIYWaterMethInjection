#pragma once

#include "modes.h"

// Manifold pressure comes from the MicroSquirt over CAN (ecu_broadcast.h); only the
// barometer is kept here, for turning it into boost.
struct MapCalibration {
  // Atmospheric pressure for boost: the ECU's barometer when it sends one, else the
  // manifold with the engine stopped. Until then an estimate for about 5,000 ft.
  float baroKpa{90.0f};
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
  // Absolute. The car lives at about 5,000 ft (barometer 85-87 kPa), where 120 kPa is
  // 5 psi of boost and the logs spend only seconds a drive above it; 100 kPa is about
  // 2 psi there. A stock 9.5:1 engine on 91 can knock from light boost on.
  float minMapKpaToArm{100.0f};
  float minLoadPercentToArm{35.0f};

  // Front-end ADC sampling and analysis block sizing.
  uint16_t sampleRateHz{19231};  // Reported on 0x30D only; the ADC's real rate is adc_sampler::knockSamplesPerSecond().
  uint8_t samplesPerUpdate{64};

  // Knock band targeting. Sensor operating frequency range is 3-25 kHz.
  // If autoCenterFromBore is true, centerFreqHz is
  // estimated from bore and harmonic assumptions at runtime.
  bool autoCenterFromBore{true};
  // Ford 2.3 L Lima: 96.0 mm (3.780 in) bore, so knock rings at about 6.0 kHz.
  float boreMm{96.0f};
  float centerFreqHz{6000.0f};
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
  float boostEnableKpa{100.0f};
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

  // Installed float switch closes to GND while the tank has fluid and opens when low,
  // so HIGH at the pin means low-fluid (false here). With INPUT_PULLUP a disconnected
  // or broken wire also reads HIGH and is treated as low: injection stops (fail-safe).
  // Set true for a switch that closes to GND when the level is low.
  bool floatActiveLow{false};
  bool tankProtectionEnabled{true};
  uint32_t floatDebounceMs{100};
  uint32_t floatLowShutdownDelayMs{2000};

  uint32_t serialBaud{115200};
  uint32_t debugPeriodMs{250};
  uint32_t loopPeriodMs{20};

  KnockConfig knock{};
};

// The defaults are the field initializers above.
AppConfig defaultConfig();

TankBlend computeTankBlend(float waterLiters, float methLiters);
