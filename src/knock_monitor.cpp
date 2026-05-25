#include "knock_monitor.h"

#include <Arduino.h>
#include <math.h>

namespace {
constexpr uint16_t kAdcMidpoint = 2048;
constexpr uint16_t kAdcLowClip = 5;
constexpr uint16_t kAdcHighClip = 4090;
constexpr uint8_t kSamplesPerTick = 10;
constexpr float kEnergyAlpha = 0.18f;
constexpr float kActivityAlpha = 0.08f;
constexpr float kBaselineAlphaSlow = 0.01f;
constexpr float kBaselineAlphaFast = 0.06f;
constexpr float kMinimumEnergyFloor = 2.0f;
constexpr float kLowActivityThreshold = 1.6f;
constexpr uint32_t kLowActivityFaultDelayMs = 2500;
constexpr uint32_t kClipWindowMs = 1000;
constexpr uint16_t kClipWindowThreshold = 20;
constexpr uint32_t kBaselineLearnMs = 3000;
constexpr uint32_t kTaskIntervalMs = 20;

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
  state_.signalValid = true;
  state_.baseline = 12.0f;
  state_.threshold = 32.0f;
  state_.baselineLearned = false;
  state_.energy = 0.0f;

  if (adcPin_ >= 0) {
    pinMode(adcPin_, INPUT);
    analogReadResolution(12);
#if defined(ADC_11db)
    analogSetPinAttenuation(adcPin_, ADC_11db);
#endif
  }
}

void KnockMonitor::setConfig(const KnockConfig &config) {
  config_ = config;
}

void KnockMonitor::clearFaults() {
  state_.warningActive = false;
  state_.criticalActive = false;
  state_.sensorFault = false;
  state_.clippingDetected = false;
  state_.eventCount = 0;
  eventWindowCount_ = 0;
  eventCountRolling_ = 0;
  clipHighTotal_ = 0;
  clipLowTotal_ = 0;
  lowActivitySinceMs_ = 0;
  lastFaultCode_ = 0;
  lastFaultMs_ = 0;
  faultPending_ = false;
}

void KnockMonitor::updateSignalHealth(uint16_t sampleRaw, float absCentered, uint32_t nowMs) {
  activityEma_ += kActivityAlpha * (absCentered - activityEma_);

  if (clipWindowStartMs_ == 0) clipWindowStartMs_ = nowMs;
  if (sampleRaw >= kAdcHighClip) {
    ++clipHighWindowCount_;
    ++clipHighTotal_;
  }
  if (sampleRaw <= kAdcLowClip) {
    ++clipLowWindowCount_;
    ++clipLowTotal_;
  }
  if ((nowMs - clipWindowStartMs_) >= kClipWindowMs) {
    state_.clippingDetected = (clipHighWindowCount_ >= kClipWindowThreshold) ||
                              (clipLowWindowCount_ >= kClipWindowThreshold);
    clipHighWindowCount_ = 0;
    clipLowWindowCount_ = 0;
    clipWindowStartMs_ = nowMs;
  }

  if (activityEma_ < kLowActivityThreshold) {
    if (lowActivitySinceMs_ == 0) lowActivitySinceMs_ = nowMs;
    if ((nowMs - lowActivitySinceMs_) >= kLowActivityFaultDelayMs) {
      state_.signalValid = false;
      state_.sensorFault = true;
    }
  } else {
    lowActivitySinceMs_ = 0;
    state_.signalValid = true;
    state_.sensorFault = false;
  }
}

void KnockMonitor::updateBaseline(bool detectActive) {
  if (!config_.baselineLearningEnabled) return;
  // Freeze baseline adaptation during active detection/warning/critical windows
  // so sustained knock activity is not learned as normal background noise.
  if (detectActive || state_.warningActive || state_.criticalActive) return;

  const float alpha = state_.baselineLearned ? kBaselineAlphaSlow : kBaselineAlphaFast;
  state_.baseline += alpha * (state_.energy - state_.baseline);
  if (state_.baseline < kMinimumEnergyFloor) state_.baseline = kMinimumEnergyFloor;
  ++baselineSampleCount_;
  if (!state_.baselineLearned && baselineSampleCount_ > (kBaselineLearnMs / kTaskIntervalMs)) {
    state_.baselineLearned = true;
  }
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

KnockStateSnapshot KnockMonitor::update(float boostKpa, uint16_t rpm, uint32_t nowMs) {
  state_.online = adcPin_ >= 0;
  if (adcPin_ < 0) {
    state_.signalValid = false;
    state_.sensorFault = true;
    maybeQueueFault(can_protocol::knock_fault_code::ADC_FAULT,
                    static_cast<uint8_t>(can_protocol::FaultSeverity::CRITICAL), 0, 0, nowMs);
    return state_;
  }

  float accum = 0.0f;
  for (uint8_t i = 0; i < kSamplesPerTick; ++i) {
    const uint16_t raw = static_cast<uint16_t>(analogRead(adcPin_));
    const int16_t centered = static_cast<int16_t>(raw) - static_cast<int16_t>(kAdcMidpoint);
    const float absCentered = fabsf(static_cast<float>(centered));
    accum += absCentered;
    updateSignalHealth(raw, absCentered, nowMs);
  }
  const float sampled = accum / static_cast<float>(kSamplesPerTick);
  state_.energy += kEnergyAlpha * (sampled - state_.energy);
  if (state_.energy < 0.0f) state_.energy = 0.0f;

  const bool detectActive = config_.enabled && state_.signalValid && !state_.sensorFault &&
                            (boostKpa >= config_.boostEnableKpa);
  updateBaseline(detectActive);

  state_.threshold = (state_.baseline * config_.thresholdMultiplier) + config_.thresholdOffset;
  if (state_.threshold < state_.baseline + 1.0f) state_.threshold = state_.baseline + 1.0f;

  if (lastDecayMs_ == 0) lastDecayMs_ = nowMs;
  if ((nowMs - lastDecayMs_) >= 1000) {
    if (eventWindowCount_ > 0) --eventWindowCount_;
    lastDecayMs_ = nowMs;
  }

  if (state_.sensorFault) {
    maybeQueueFault(can_protocol::knock_fault_code::SENSOR_DISCONNECTED,
                    static_cast<uint8_t>(can_protocol::FaultSeverity::WARNING),
                    can_protocol::clampU8(static_cast<int>(activityEma_)), 0, nowMs);
  } else if (state_.clippingDetected) {
    maybeQueueFault(can_protocol::knock_fault_code::SIGNAL_CLIPPING,
                    static_cast<uint8_t>(can_protocol::FaultSeverity::WARNING),
                    static_cast<uint8_t>(clipHighTotal_ & 0xFF),
                    static_cast<uint8_t>(clipLowTotal_ & 0xFF), nowMs);
  } else if (!state_.baselineLearned && config_.enabled) {
    maybeQueueFault(can_protocol::knock_fault_code::BASELINE_NOT_LEARNED,
                    static_cast<uint8_t>(can_protocol::FaultSeverity::INFO),
                    can_protocol::clampU8(static_cast<int>(state_.baseline)), 0, nowMs);
  } else if (detectActive && (state_.energy > state_.threshold) &&
             ((nowMs - lastEventMs_) >= config_.eventCooldownMs)) {
    lastEventMs_ = nowMs;
    ++eventCountRolling_;
    if (eventWindowCount_ < 255) ++eventWindowCount_;
    state_.lastEventRpm = rpm;
    state_.lastEventBoostKpa = can_protocol::clampU8(static_cast<int>(boostKpa));

    const bool critical = eventWindowCount_ >= config_.criticalThresholdCount;
    const bool warning = eventWindowCount_ >= config_.warningThresholdCount;
    if (critical) {
      maybeQueueFault(can_protocol::knock_fault_code::KNOCK_CRITICAL,
                      static_cast<uint8_t>(can_protocol::FaultSeverity::CRITICAL),
                      encodeRpmDiv100(state_.lastEventRpm), state_.lastEventBoostKpa, nowMs);
    } else if (warning) {
      maybeQueueFault(can_protocol::knock_fault_code::KNOCK_WARNING,
                      static_cast<uint8_t>(can_protocol::FaultSeverity::WARNING),
                      encodeRpmDiv100(state_.lastEventRpm), state_.lastEventBoostKpa, nowMs);
    }
  }

  state_.eventCount = eventCountRolling_;
  state_.warningActive = eventWindowCount_ >= config_.warningThresholdCount;
  state_.criticalActive = eventWindowCount_ >= config_.criticalThresholdCount;
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
