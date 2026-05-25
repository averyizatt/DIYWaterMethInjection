#include "knock_monitor.h"

#include <Arduino.h>

namespace {
inline uint8_t encodeRpmDiv100(uint16_t rpm) {
  const uint16_t rpmDiv100 = rpm / 100U;
  return static_cast<uint8_t>(rpmDiv100 > 255U ? 255U : rpmDiv100);
}
} // namespace

void KnockMonitor::begin(int adcPin, const KnockConfig &config) {
  adcPin_ = adcPin;
  config_ = config;
  state_ = KnockStateSnapshot{};
  state_.online = adcPin_ >= 0;

  if (adcPin_ >= 0) {
    sampler_.begin(adcPin_, config_.sampleRateHz);
    detector_.configure(toDetectorConfig(config_));
    detector_.reset();
  }
}

void KnockMonitor::setConfig(const KnockConfig &config) {
  config_ = config;
  sampler_.setSampleRate(config_.sampleRateHz);
  detector_.configure(toDetectorConfig(config_));
}

void KnockMonitor::clearFaults() {
  state_.warningActive = false;
  state_.criticalActive = false;
  state_.sensorFault = false;
  state_.clippingDetected = false;
  state_.eventCount = 0;
  eventWindowCount_ = 0;
  lastFaultCode_ = 0;
  lastFaultMs_ = 0;
  faultPending_ = false;
  detector_.reset();
}

void KnockMonitor::maybeQueueFault(uint8_t code, uint8_t severity, uint8_t data0, uint8_t data1,
                                   uint32_t nowMs) {
  if (code == lastFaultCode_ && (nowMs - lastFaultMs_) < 2000) return;
  pendingFault_.code = code;
  pendingFault_.severity = severity;
  pendingFault_.data0 = data0;
  pendingFault_.data1 = data1;
  faultPending_ = true;
  lastFaultCode_ = code;
  lastFaultMs_ = nowMs;
}

KnockDetectorConfig KnockMonitor::toDetectorConfig(const KnockConfig &config) const {
  KnockDetectorConfig detectorCfg{};
  detectorCfg.enabled = config.enabled;
  detectorCfg.minRpmToArm = config.minRpmToArm;
  detectorCfg.minMapKpaToArm = config.minMapKpaToArm;
  detectorCfg.eventCooldownMs = config.eventCooldownMs;
  detectorCfg.autoCenterFromBore = config.autoCenterFromBore;
  detectorCfg.boreMm = config.boreMm;
  detectorCfg.centerFreqHz = config.centerFreqHz;
  detectorCfg.bandwidthHz = config.bandwidthHz;
  detectorCfg.thresholdOffset = config.thresholdOffset;
  detectorCfg.thresholdMultiplier = config.thresholdMultiplier;
  detectorCfg.baselineLearningEnabled = config.baselineLearningEnabled;
  detectorCfg.baselineLearnAlpha = config.baselineLearnAlpha;
  detectorCfg.signalGain = config.signalGain;
  detectorCfg.biasAlpha = config.biasAlpha;
  detectorCfg.envelopeAlpha = config.envelopeAlpha;
  detectorCfg.rmsAlpha = config.rmsAlpha;
  detectorCfg.sampleRateHz = static_cast<float>(config.sampleRateHz);
  detectorCfg.clipLowAdc = config.clipLowAdc;
  detectorCfg.clipHighAdc = config.clipHighAdc;
  detectorCfg.clipPercentForFault = config.clipPercentForFault;
  detectorCfg.stuckAdcDelta = config.stuckAdcDelta;
  detectorCfg.faultHoldMs = config.faultHoldMs;
  detectorCfg.missingSignalRms = config.missingSignalRms;
  detectorCfg.warningThresholdCount = config.warningThresholdCount;
  detectorCfg.criticalThresholdCount = config.criticalThresholdCount;
  return detectorCfg;
}

KnockStateSnapshot KnockMonitor::update(float boostKpa, uint16_t rpm, uint32_t nowMs) {
  state_.online = adcPin_ >= 0;
  if (adcPin_ < 0) {
    state_.signalValid = false;
    state_.sensorFault = true;
    maybeQueueFault(can_protocol::knock_fault_code::ADC_FAULT,
                    static_cast<uint8_t>(can_protocol::FaultSeverity::CRITICAL), 0, 0, nowMs);
    return state_;
  }

  const uint8_t requestedSamples =
      config_.samplesPerUpdate > static_cast<uint8_t>(sizeof(sampleBuffer_) / sizeof(sampleBuffer_[0]))
          ? static_cast<uint8_t>(sizeof(sampleBuffer_) / sizeof(sampleBuffer_[0]))
          : config_.samplesPerUpdate;

  KnockSamplerStats stats{};
  const uint8_t captured = sampler_.captureBlock(sampleBuffer_, requestedSamples,
                                                 config_.clipLowAdc, config_.clipHighAdc,
                                                 stats);
  if (captured == 0) {
    state_.signalValid = false;
    state_.sensorFault = true;
    maybeQueueFault(can_protocol::knock_fault_code::ADC_FAULT,
                    static_cast<uint8_t>(can_protocol::FaultSeverity::CRITICAL), 1, 0, nowMs);
    return state_;
  }

  const KnockDetectorFrame frame = detector_.processBlock(sampleBuffer_, captured, rpm, boostKpa, nowMs,
                                                          stats.minAdc, stats.maxAdc,
                                                          stats.clipLowCount, stats.clipHighCount);

  state_.armed = frame.armed;
  state_.knockDetected = frame.detected;
  state_.signalValid = !frame.sensorFault;
  state_.sensorFault = frame.sensorFault;
  state_.clippingDetected = frame.clippingDetected;
  state_.warningActive = frame.warningActive;
  state_.criticalActive = frame.criticalActive;
  state_.baselineLearned = frame.baselineLearned;
  state_.rawAdc = frame.rawAdc;
  state_.biasAdc = frame.biasAdc;
  state_.filteredSignal = frame.filtered;
  state_.envelope = frame.envelope;
  state_.knockLevelRms = frame.rms;
  state_.energy = frame.rms;
  state_.baseline = frame.baseline;
  state_.threshold = frame.threshold;
  state_.eventCount = frame.eventCount;

  if (frame.detected) {
    lastEventMs_ = nowMs;
    state_.lastEventRpm = rpm;
    state_.lastEventBoostKpa = can_protocol::clampU8(static_cast<int>(boostKpa));

    if (frame.criticalActive) {
      maybeQueueFault(can_protocol::knock_fault_code::KNOCK_CRITICAL,
                      static_cast<uint8_t>(can_protocol::FaultSeverity::CRITICAL),
                      encodeRpmDiv100(state_.lastEventRpm), state_.lastEventBoostKpa, nowMs);
    } else if (frame.warningActive) {
      maybeQueueFault(can_protocol::knock_fault_code::KNOCK_WARNING,
                      static_cast<uint8_t>(can_protocol::FaultSeverity::WARNING),
                      encodeRpmDiv100(state_.lastEventRpm), state_.lastEventBoostKpa, nowMs);
    }
  }

  if (frame.sensorFault) {
    maybeQueueFault(can_protocol::knock_fault_code::SENSOR_DISCONNECTED,
                    static_cast<uint8_t>(can_protocol::FaultSeverity::WARNING),
                    can_protocol::clampU8(static_cast<int>(frame.rawAdc / 16.0f)),
                    frame.clippingDetected ? 1 : 0, nowMs);
  }

  state_.requestForceSpray = false;
  state_.requestSafetyShutdown = false;
  switch (config_.responseMode) {
  case KnockResponseMode::ForceSpray:
    state_.requestForceSpray = state_.criticalActive;
    break;
  case KnockResponseMode::SafetyShutdown:
    state_.requestSafetyShutdown = state_.criticalActive;
    break;
  case KnockResponseMode::LogOnly:
  case KnockResponseMode::WarnOnly:
  default:
    break;
  }

  return state_;
}

bool KnockMonitor::consumeFault(KnockFaultEvent &eventOut) {
  if (!faultPending_) return false;
  eventOut = pendingFault_;
  faultPending_ = false;
  return true;
}
