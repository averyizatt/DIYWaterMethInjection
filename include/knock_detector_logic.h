#pragma once

#include <stdint.h>

#include "knock_filter.h"

struct KnockDetectorConfig {
  bool enabled{true};
  uint16_t minRpmToArm{0};
  float minMapKpaToArm{120.0f};
  uint16_t eventCooldownMs{250};

  bool autoCenterFromBore{true};
  float boreMm{96.0f};
  float centerFreqHz{6500.0f};
  float bandwidthHz{1800.0f};

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
  uint16_t clipHighAdc{4090};
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
  float biasAdc{2048.0f};
  float centered{0.0f};
  float filtered{0.0f};
  float envelope{0.0f};
  float rms{0.0f};
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
                                  uint16_t rpm,
                                  float mapKpa,
                                  uint32_t nowMs,
                                  uint16_t minAdc,
                                  uint16_t maxAdc,
                                  uint16_t clipLowCount,
                                  uint16_t clipHighCount);

private:
  static float estimateKnockCenterHz(float boreMm);

  KnockDetectorConfig config_{};
  KnockFilterPipeline filter_{};

  float baseline_{0.0f};
  uint32_t lastEventMs_{0};
  uint32_t lowSignalSinceMs_{0};
  uint32_t stuckSinceMs_{0};
  uint32_t baselineSamples_{0};
  uint8_t eventWindow_{0};
  uint8_t eventCount_{0};
};
