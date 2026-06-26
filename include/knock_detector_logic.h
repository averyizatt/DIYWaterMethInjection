#pragma once

#include <stdint.h>

#include "knock_filter.h"

enum class KnockOperatingProfile : uint8_t {
  Idle = 0,
  Spool = 1,
  SteadyLoad = 2,
  Lift = 3,
  HeatSoak = 4,
};

enum class KnockAnomalyClass : uint8_t {
  None = 0,
  LikelyKnock = 1,
  MechanicalTransient = 2,
  SensorIssue = 3,
  UnknownEnergy = 4,
};

enum class KnockDegradeMode : uint8_t {
  Full = 0,
  Conservative = 1,
  Failsafe = 2,
};

enum KnockReasonFlags : uint16_t {
  REASON_ARM_BLOCKED = 1U << 0,
  REASON_SENSOR_FAULT = 1U << 1,
  REASON_TRANSIENT_GATE = 1U << 2,
  REASON_LOW_CONFIDENCE = 1U << 3,
  REASON_FFT_GATE = 1U << 4,
  REASON_DEGRADE_CONSERVATIVE = 1U << 5,
  REASON_DEGRADE_FAILSAFE = 1U << 6,
  REASON_PROFILE_HEATSOAK = 1U << 7,
};

struct KnockDetectorConfig {
  bool enabled{true};
  uint16_t minRpmToArm{0};
  float minMapKpaToArm{120.0f};
  float minLoadPercentToArm{30.0f};
  uint16_t eventCooldownMs{250};

  bool autoCenterFromBore{true};
  float boreMm{96.0f};
  float centerFreqHz{6500.0f};
  float bandwidthHz{1800.0f};
  float multiBandSpread{0.14f};
  bool fftEnabled{true};
  float fftMinSnrDb{3.0f};
  float fftWeight{0.40f};
  float fftShortWeight{0.45f};
  float fftHarmonicWeight{0.30f};
  float spectralTemplateWeight{0.25f};

  float mapRateGateKpaPerSec{140.0f};
  float transientThresholdScale{1.30f};
  uint16_t transientHoldMs{180};

  // Confidence/risk fusion and actuation tiering.
  float minDetectConfidence{0.52f};
  float warnConfidence{0.58f};
  float criticalConfidence{0.72f};
  float confidenceWeightSpectral{0.35f};
  float confidenceWeightFft{0.30f};
  float confidenceWeightHarmonic{0.20f};
  float confidenceWeightTemplate{0.15f};

  // Temperature-aware threshold adaptation.
  float iatTempCompStartC{40.0f};
  float iatTempCompPerC{0.010f};
  float bayTempCompStartC{60.0f};
  float bayTempCompPerC{0.006f};
  float maxTempCompScale{1.45f};

  // Profile threshold scaling.
  float profileScaleIdle{1.35f};
  float profileScaleSpool{1.20f};
  float profileScaleSteady{1.00f};
  float profileScaleLift{1.15f};
  float profileScaleHeatSoak{1.28f};

  // Long-term drift tracking and adaptation management.
  float longTermBaselineAlpha{0.003f};
  float driftWarnPercent{35.0f};
  float driftCriticalPercent{65.0f};
  float adaptiveMultMin{1.2f};
  float adaptiveMultMax{3.8f};
  float adaptiveMultLearnAlpha{0.015f};

  // Predictive risk and health/degrade policy.
  float riskMapRateWeight{0.45f};
  float riskConfidenceWeight{0.35f};
  float riskEventWeight{0.20f};
  float conservativeHealthThreshold{65.0f};
  float failsafeHealthThreshold{40.0f};

  float thresholdOffset{8.0f};
  float thresholdMultiplier{2.5f};
  bool baselineLearningEnabled{true};
  float baselineLearnAlpha{0.02f};

  float signalGain{1.0f};
  float biasAlpha{0.002f};
  float envelopeAlpha{0.20f};
  float rmsAlpha{0.12f};
  float sampleRateHz{8000.0f};

  uint16_t clipLowAdc{5};
  uint16_t clipHighAdc{1018};
  uint8_t clipPercentForFault{30};
  uint16_t stuckAdcDelta{3};
  uint16_t faultHoldMs{1200};
  float missingSignalRms{0.8f};

  uint8_t warningThresholdCount{2};
  uint8_t criticalThresholdCount{4};
};

struct KnockDetectorFrame {
  bool armed{false};
  bool detected{false};
  bool warningActive{false};
  bool criticalActive{false};
  bool sensorFault{false};
  bool clippingDetected{false};
  bool baselineLearned{false};

  float rawAdc{0.0f};
  float biasAdc{512.0f};
  float centered{0.0f};
  float filtered{0.0f};
  float envelope{0.0f};
  float rms{0.0f};
  float lowBandRms{0.0f};
  float midBandRms{0.0f};
  float highBandRms{0.0f};
  float spectralConfidence{0.0f};
  float fftSnrDb{0.0f};
  float fftShortSnrDb{0.0f};
  float fftLongSnrDb{0.0f};
  float harmonicScore{0.0f};
  float templateDeviation{0.0f};
  float fftTargetMag{0.0f};
  float fftNoiseMag{0.0f};
  float selectedCenterHz{0.0f};
  float expectedCenterHz{0.0f};
  float loadPercent{0.0f};
  float mapRateKpaPerSec{0.0f};
  float transientScale{1.0f};
  float tempScale{1.0f};
  float profileScale{1.0f};
  float adaptiveMultiplier{2.5f};
  float shortBaseline{0.0f};
  float longBaseline{0.0f};
  float driftPercent{0.0f};
  float finalConfidence{0.0f};
  float knockRisk{0.0f};
  float healthScore{100.0f};
  float iatC{20.0f};
  float bayC{20.0f};
  uint16_t reasonFlags{0};
  KnockOperatingProfile profile{KnockOperatingProfile::SteadyLoad};
  KnockAnomalyClass anomalyClass{KnockAnomalyClass::None};
  KnockDegradeMode degradeMode{KnockDegradeMode::Full};
  float baseline{0.0f};
  float threshold{0.0f};

  uint8_t eventCount{0};
};

class KnockDetectorLogic {
public:
  void configure(const KnockDetectorConfig &config);
  void reset();

  // Process a single block of samples.
  KnockDetectorFrame processBlock(const uint16_t *samples,
                                  uint8_t sampleCount,
                                  float loadPercent,
                                  float mapKpa,
                                  float mapRateKpaPerSec,
                                  float iatC,
                                  float bayC,
                                  uint32_t nowMs,
                                  uint16_t minAdc,
                                  uint16_t maxAdc,
                                  uint16_t clipLowCount,
                                  uint16_t clipHighCount);

private:
  static float estimateKnockCenterHz(float boreMm);

  KnockDetectorConfig config_{};
  KnockFilterPipeline filterLow_{};
  KnockFilterPipeline filterMid_{};
  KnockFilterPipeline filterHigh_{};
  float centerLowHz_{0.0f};
  float centerMidHz_{0.0f};
  float centerHighHz_{0.0f};

  static constexpr uint8_t kLoadBins = 8;
  float baselineMap_[kLoadBins]{};
  float longBaselineMap_[kLoadBins]{};
  uint16_t baselineMapSamples_[kLoadBins]{};
  float spectralTemplateMap_[kLoadBins]{};
  uint16_t spectralTemplateSamples_[kLoadBins]{};
  float centerBiasMap_[kLoadBins]{};
  float adaptiveThresholdMultMap_[kLoadBins]{};
  float fallbackBaseline_{0.0f};
  float healthScore_{100.0f};
  KnockDegradeMode degradeMode_{KnockDegradeMode::Full};
  uint32_t transientHoldUntilMs_{0};
  uint32_t lastEventMs_{0};
  uint32_t lowSignalSinceMs_{0};
  uint32_t stuckSinceMs_{0};
  uint32_t totalBaselineSamples_{0};
  uint8_t eventWindow_{0};
  uint8_t eventCount_{0};
};
