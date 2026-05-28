#pragma once

#include <stdint.h>

#include "app_config.h"
#include "can_contract/can_protocol.h"
#include "knock_detector_logic.h"
#include "knock_sampler.h"

struct KnockFaultEvent {
  uint8_t code{0};
  uint8_t severity{0};
  uint8_t data0{0};
  uint8_t data1{0};
};

struct KnockStateSnapshot {
  bool online{false};
  bool signalValid{true};
  bool armed{false};
  bool knockDetected{false};
  bool sensorFault{false};
  bool clippingDetected{false};
  bool warningActive{false};
  bool criticalActive{false};
  bool baselineLearned{false};

  float rawAdc{0.0f};
  float biasAdc{2048.0f};
  float filteredSignal{0.0f};
  float envelope{0.0f};
  float knockLevelRms{0.0f};
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
  uint8_t profile{0};
  uint8_t anomalyClass{0};
  uint8_t degradeMode{0};
  float baseline{12.0f};
  float threshold{32.0f};

  float energy{0.0f};
  uint8_t eventCount{0};
  uint16_t lastEventRpm{0};
  uint8_t lastEventBoostKpa{0};
  bool requestForceSpray{false};
  bool requestSafetyShutdown{false};
};

class KnockMonitor {
public:
  void begin(int adcPin, const KnockConfig &config);
  void setConfig(const KnockConfig &config);
  void clearFaults();
  KnockStateSnapshot update(float mapKpa, float loadPercent, float mapRateKpaPerSec,
                            float iatC, float bayC, uint32_t nowMs);
  KnockStateSnapshot state() const { return state_; }
  bool consumeFault(KnockFaultEvent &eventOut);

private:
  KnockDetectorConfig toDetectorConfig(const KnockConfig &config) const;
  void maybeQueueFault(uint8_t code, uint8_t severity, uint8_t data0, uint8_t data1, uint32_t nowMs);

  int adcPin_{-1};
  KnockConfig config_{};
  KnockStateSnapshot state_{};

  KnockSampler sampler_{};
  KnockDetectorLogic detector_{};
  uint16_t sampleBuffer_[128]{};

  uint32_t lastEventMs_{0};
  uint8_t eventWindowCount_{0};

  uint8_t lastFaultCode_{0};
  uint32_t lastFaultMs_{0};
  bool faultPending_{false};
  KnockFaultEvent pendingFault_{};
};
