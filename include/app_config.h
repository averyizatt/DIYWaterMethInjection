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
  float startPsi{3.5f};
  float fullPsi{7.5f};
};

enum class KnockResponseMode : uint8_t {
  LogOnly = 0,
  WarnOnly = 1,
  ForceSpray = 2,
  SafetyShutdown = 3,
};

struct KnockConfig {
  bool enabled{true};
  // Detection is only active when both arming conditions are met.
  uint16_t minRpmToArm{0};
  float minMapKpaToArm{120.0f};

  // Front-end ADC sampling and analysis block sizing.
  uint16_t sampleRateHz{8000};
  uint8_t samplesPerUpdate{64};

  // Knock band targeting. If autoCenterFromBore is true, centerFreqHz is
  // estimated from bore and harmonic assumptions at runtime.
  bool autoCenterFromBore{true};
  float boreMm{96.0f};
  float centerFreqHz{6500.0f};
  float bandwidthHz{1800.0f};

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
  uint16_t clipHighAdc{4090};
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

  uint16_t pwmFrequencyHz{100};
  uint8_t pwmResolutionBits{10};

  // false = active-HIGH low signal means empty; i.e. pin HIGH (open) = empty,
  // pin LOW (switch closed to GND) = full.  Matches a normally-open switch that
  // shorts to GND when the tank is full (0 Ω full / open-circuit empty).
  bool floatActiveLow{false};
  uint32_t floatDebounceMs{100};

  uint32_t serialBaud{115200};
  uint32_t debugPeriodMs{250};
  uint32_t loopPeriodMs{20};

  KnockConfig knock{};
};

AppConfig defaultConfig();

TankBlend computeTankBlend(float waterLiters, float methLiters);
