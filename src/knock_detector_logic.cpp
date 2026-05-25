#include "knock_detector_logic.h"

#include <math.h>

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr uint32_t kBaselineLearnMinSamples = 400;

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}
} // namespace

void KnockDetectorLogic::configure(const KnockDetectorConfig &config) {
  config_ = config;

  KnockFilterConfig filterCfg{};
  filterCfg.sampleRateHz = config_.sampleRateHz;
  filterCfg.centerFreqHz = config_.autoCenterFromBore ? estimateKnockCenterHz(config_.boreMm)
                                                       : config_.centerFreqHz;
  filterCfg.bandwidthHz = config_.bandwidthHz;
  filterCfg.biasAlpha = config_.biasAlpha;
  filterCfg.envelopeAlpha = config_.envelopeAlpha;
  filterCfg.rmsAlpha = config_.rmsAlpha;
  filterCfg.signalGain = config_.signalGain;
  filter_.configure(filterCfg);
}

void KnockDetectorLogic::reset() {
  baseline_ = 0.0f;
  lastEventMs_ = 0;
  lowSignalSinceMs_ = 0;
  stuckSinceMs_ = 0;
  baselineSamples_ = 0;
  eventWindow_ = 0;
  eventCount_ = 0;
  filter_.reset();
}

KnockDetectorFrame KnockDetectorLogic::processBlock(const uint16_t *samples,
                                                    uint8_t sampleCount,
                                                    uint16_t rpm,
                                                    float mapKpa,
                                                    uint32_t nowMs,
                                                    uint16_t minAdc,
                                                    uint16_t maxAdc,
                                                    uint16_t clipLowCount,
                                                    uint16_t clipHighCount) {
  KnockDetectorFrame frame{};
  if (samples == nullptr || sampleCount == 0) {
    frame.sensorFault = true;
    return frame;
  }

  for (uint8_t i = 0; i < sampleCount; ++i) {
    filter_.processSample(static_cast<float>(samples[i]));
  }

  frame.rawAdc = filter_.rawSample();
  frame.biasAdc = filter_.bias();
  frame.centered = filter_.centered();
  frame.filtered = filter_.bandpassed();
  frame.envelope = filter_.envelope();
  frame.rms = filter_.rms();

  const float clippedPercent = static_cast<float>(clipLowCount + clipHighCount) * 100.0f /
                               static_cast<float>(sampleCount);
  frame.clippingDetected = clippedPercent >= static_cast<float>(config_.clipPercentForFault);

  const bool stuckNow = static_cast<uint16_t>(maxAdc - minAdc) <= config_.stuckAdcDelta;
  if (stuckNow) {
    if (stuckSinceMs_ == 0) stuckSinceMs_ = nowMs;
  } else {
    stuckSinceMs_ = 0;
  }

  const bool missingNow = frame.rms < config_.missingSignalRms;
  if (missingNow) {
    if (lowSignalSinceMs_ == 0) lowSignalSinceMs_ = nowMs;
  } else {
    lowSignalSinceMs_ = 0;
  }

  const bool stuckFault = (stuckSinceMs_ != 0) && ((nowMs - stuckSinceMs_) >= config_.faultHoldMs);
  const bool missingFault = (lowSignalSinceMs_ != 0) && ((nowMs - lowSignalSinceMs_) >= config_.faultHoldMs);
  frame.sensorFault = frame.clippingDetected || stuckFault || missingFault;

  frame.armed = config_.enabled && !frame.sensorFault && (rpm >= config_.minRpmToArm) &&
                (mapKpa >= config_.minMapKpaToArm);

  if (baseline_ <= 0.0f) baseline_ = frame.rms;
  const bool canLearn = config_.baselineLearningEnabled && !frame.sensorFault && !frame.armed;
  if (canLearn) {
    baseline_ += config_.baselineLearnAlpha * (frame.rms - baseline_);
    ++baselineSamples_;
  }

  frame.baselineLearned = baselineSamples_ >= kBaselineLearnMinSamples;
  frame.baseline = baseline_;

  frame.threshold = (baseline_ * config_.thresholdMultiplier) + config_.thresholdOffset;
  frame.threshold = clampFloat(frame.threshold, config_.thresholdOffset, 5000.0f);

  if (eventWindow_ > 0 && (nowMs - lastEventMs_) > 1000U) {
    --eventWindow_;
  }

  const bool overThreshold = frame.rms > frame.threshold;
  const bool debounceElapsed = (lastEventMs_ == 0) || ((nowMs - lastEventMs_) >= config_.eventCooldownMs);
  if (frame.armed && overThreshold && debounceElapsed) {
    frame.detected = true;
    lastEventMs_ = nowMs;
    ++eventCount_;
    if (eventWindow_ < 255) ++eventWindow_;
  }

  frame.warningActive = eventWindow_ >= config_.warningThresholdCount;
  frame.criticalActive = eventWindow_ >= config_.criticalThresholdCount;
  frame.eventCount = eventCount_;

  return frame;
}

float KnockDetectorLogic::estimateKnockCenterHz(float boreMm) {
  // Practical estimate: 2.2x of the classic combustion chamber radial mode
  // puts common 4-cylinder bores in the 5.5-8 kHz range.
  const float boreM = clampFloat(boreMm, 60.0f, 120.0f) / 1000.0f;
  const float fundamentalHz = 900.0f / (kPi * boreM);
  return clampFloat(fundamentalHz * 2.2f, 3500.0f, 9000.0f);
}
