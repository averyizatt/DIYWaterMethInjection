#pragma once

#include <stdint.h>

#include "app_config.h"
#include "can_contract/can_protocol.h"

struct KnockFaultEvent {
  uint8_t code{0};
  uint8_t severity{0};
  uint8_t data0{0};
  uint8_t data1{0};
};

struct KnockStateSnapshot {
  bool online{false};
  bool signalValid{true};
  bool sensorFault{false};
  bool clippingDetected{false};
  bool warningActive{false};
  bool criticalActive{false};
  bool baselineLearned{false};
  float energy{0.0f};
  float baseline{12.0f};
  float threshold{32.0f};
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
  KnockStateSnapshot update(float boostKpa, uint16_t rpm, uint32_t nowMs);
  KnockStateSnapshot state() const { return state_; }
  bool consumeFault(KnockFaultEvent &eventOut);

private:
  void updateSignalHealth(uint16_t sampleRaw, float absCentered, uint32_t nowMs);
  void updateBaseline(bool detectActive);
  void maybeQueueFault(uint8_t code, uint8_t severity, uint8_t data0, uint8_t data1, uint32_t nowMs);

  int adcPin_{-1};
  KnockConfig config_{};
  KnockStateSnapshot state_{};

  float activityEma_{0.0f};
  uint32_t lastEventMs_{0};
  uint32_t lastDecayMs_{0};
  uint32_t lowActivitySinceMs_{0};
  uint32_t clipWindowStartMs_{0};
  uint16_t clipHighWindowCount_{0};
  uint16_t clipLowWindowCount_{0};
  uint16_t clipHighTotal_{0};
  uint16_t clipLowTotal_{0};
  uint32_t baselineSampleCount_{0};
  uint8_t eventWindowCount_{0};
  uint8_t eventCountRolling_{0};

  uint8_t lastFaultCode_{0};
  uint32_t lastFaultMs_{0};
  bool faultPending_{false};
  KnockFaultEvent pendingFault_{};
};
