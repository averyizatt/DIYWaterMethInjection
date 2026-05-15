#include "can_bridge.h"

#include <Arduino.h>
#include "can_contract/can_protocol.h"

#if __has_include(<driver/twai.h>)
#include <driver/twai.h>
#define WMETH_HAS_TWAI 1
#else
#define WMETH_HAS_TWAI 0
#endif

namespace {
// State TX period: 50 ms per shared contract (ID_ENGINE_METH_STATE).
constexpr uint32_t kStateTxIntervalMs = 50;
// If no frame from master in 3 seconds, report master as timed out.
constexpr uint32_t kMasterTimeoutMs = 3000;

// Meth state values mirroring the shared contract MethState enum.
// Redefined locally to avoid a dependency on the main project headers.
constexpr uint8_t kMethStateOff      = 0;
constexpr uint8_t kMethStateArmed    = 1;
constexpr uint8_t kMethStateSpraying = 2;
constexpr uint8_t kMethStateFault    = 3;
constexpr uint8_t kMethStateTest     = 4;

// FlowStatus values (shared contract FlowStatus enum).
constexpr uint8_t kFlowUnknown  = 0;
constexpr uint8_t kFlowOk       = 1;
constexpr uint8_t kFlowLow      = 2;
constexpr uint8_t kFlowNone     = 3;
} // namespace

bool CanBridge::begin(int txPin, int rxPin, uint32_t bitrate) {
  twaiReady_ = false;
#if WMETH_HAS_TWAI
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(txPin),
      static_cast<gpio_num_t>(rxPin),
      TWAI_MODE_NORMAL);

  // Pick timing config matching bitrate (only 500k and 250k supported here).
  twai_timing_config_t t;
  if (bitrate == 250000) {
    t = TWAI_TIMING_CONFIG_250KBITS();
  } else {
    t = TWAI_TIMING_CONFIG_500KBITS();
  }

  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) == ESP_OK) {
    if (twai_start() == ESP_OK) {
      twaiReady_ = true;
    } else {
      twai_driver_uninstall();
    }
  }
#else
  (void)txPin;
  (void)rxPin;
  (void)bitrate;
#endif
  return twaiReady_;
}

void CanBridge::poll() {
#if WMETH_HAS_TWAI
  if (!twaiReady_) return;

  twai_message_t rx{};
  while (twai_receive(&rx, 0) == ESP_OK) {
    if (!rx.extd && rx.data_length_code <= 8) {
      const uint32_t nowMs = millis();
      dispatchFrame(static_cast<uint16_t>(rx.identifier & 0x7FFU),
                    rx.data_length_code, rx.data, nowMs);
    }
  }
#endif
}

void CanBridge::dispatchFrame(uint16_t id, uint8_t dlc, const uint8_t* data, uint32_t nowMs) {
  using namespace can_protocol;

  // Track any frame from the master block (0x200–0x2FF) as a heartbeat.
  if (id >= ID_BLOCK_MASTER_BASE && id < ID_BLOCK_ENGINE_METH_BASE) {
    lastMasterRxMs_ = nowMs;
  }

  if (id == ID_ENGINE_METH_COMMAND && dlc >= 1) {
    const uint8_t cmd = data[0];

    if (cmd == meth_command::ARM && dlc >= 2) {
      armed_ = (data[1] != 0);
      return;
    }
    if (cmd == meth_command::MANUAL_TEST_DUTY && dlc >= 2) {
      manualTestDuty_ = data[1];
      pendingManualTest_ = true;
      return;
    }
    if (cmd == meth_command::STOP_MANUAL_TEST) {
      pendingManualTest_ = false;
      manualTestDuty_ = 0;
      armed_ = false;
      return;
    }
    if (cmd == meth_command::CLEAR_FAULTS) {
      clearFaultsRequested_ = true;
      return;
    }
  }

  if (id == ID_METH_CONFIG_BROADCAST && dlc >= 8) {
    MethConfigBroadcast cfg{};
    can_protocol::CanFrame cfgFrame{};
    cfgFrame.id = id;
    cfgFrame.dlc = dlc;
    for (uint8_t i = 0; i < dlc && i < 8; ++i) cfgFrame.data[i] = data[i];
    if (!unpackMethConfigBroadcast(cfgFrame, cfg)) {
      // Checksum mismatch — request a retransmit.
      sendConfigAck(cfg.version, 1 /*rejected*/, 4 /*checksum*/, lastConfigRatioPercent_);
      return;
    }
    lastConfigVersion_ = cfg.version;
    // Update ratio if it changed — main.cpp will apply it to the blend.
    if (cfg.ratio_percent <= 100 && cfg.ratio_percent != lastConfigRatioPercent_) {
      lastConfigRatioPercent_ = cfg.ratio_percent;
      remoteRatioPercent_ = cfg.ratio_percent;
      hasRemoteRatio_ = true;
    }
    armed_ = (cfg.desired_armed != 0);
    sendConfigAck(cfg.version, 0 /*OK*/, 0, lastConfigRatioPercent_);
    lastMasterRxMs_ = nowMs;
  }
}

void CanBridge::sendStateIfDue(const SensorReadings& readings,
                               const ControlResult& result,
                               const AppConfig& config,
                               uint32_t nowMs) {
#if WMETH_HAS_TWAI
  if (!twaiReady_) return;
  if ((nowMs - lastStateTxMs_) < kStateTxIntervalMs) return;
  lastStateTxMs_ = nowMs;

  // Determine meth state.
  uint8_t methState = kMethStateOff;
  if (result.failsafe != FailsafeReason::None) {
    methState = kMethStateFault;
  } else if (pendingManualTest_) {
    methState = kMethStateTest;
  } else if (result.pump.enabled) {
    methState = kMethStateSpraying;
  } else if (armed_) {
    methState = kMethStateArmed;
  }

  // Tank level: float sensor = LOW means empty (0), HIGH means OK (100).
  const uint8_t tankLevel = readings.tankLow ? 0 : 100;

  // Flow status inferred from pump command and failsafe.
  uint8_t flowStatus = kFlowUnknown;
  if (result.failsafe == FailsafeReason::None && result.pump.enabled) {
    flowStatus = kFlowOk;
  } else if (result.failsafe == FailsafeReason::LowFluid) {
    flowStatus = kFlowNone;
  }

  // Boost kPa clamped to uint8_t (0–255 kPa).
  const uint8_t boostKpa = static_cast<uint8_t>(
      constrain(static_cast<int>(readings.mapKpa), 0, 255));

  // Fault flags byte: bit 0 = low fluid, bit 1 = MAP invalid, bit 2 = bad blend.
  uint8_t faultFlags = 0;
  if (result.failsafe == FailsafeReason::LowFluid)           faultFlags |= 0x01;
  if (result.failsafe == FailsafeReason::MapInvalid)         faultFlags |= 0x02;
  if (result.failsafe == FailsafeReason::InvalidBlend)       faultFlags |= 0x04;
  if (result.failsafe == FailsafeReason::InvalidBoostConfig) faultFlags |= 0x08;

  twai_message_t tx{};
  tx.identifier = can_protocol::ID_ENGINE_METH_STATE;
  tx.extd = 0;
  tx.rtr = 0;
  tx.data_length_code = 8;
  tx.data[0] = methState;
  tx.data[1] = static_cast<uint8_t>(constrain(static_cast<int>(result.finalDutyPercent), 0, 100));
  tx.data[2] = tankLevel;
  tx.data[3] = flowStatus;
  tx.data[4] = boostKpa;
  tx.data[5] = can_protocol::tempToOffset40(0);  // IAT: not sensed; transmit 0 °C placeholder
  tx.data[6] = can_protocol::tempToOffset40(static_cast<int>(temperatureRead()));  // on-die temp
  tx.data[7] = faultFlags;

  twai_transmit(&tx, 0);
#else
  (void)readings;
  (void)result;
  (void)config;
  (void)nowMs;
#endif
}

void CanBridge::sendFault(uint8_t code, uint8_t severity, uint8_t data0, uint8_t data1) {
#if WMETH_HAS_TWAI
  if (!twaiReady_) return;
  twai_message_t tx{};
  tx.identifier = can_protocol::ID_ENGINE_METH_FAULT;
  tx.extd = 0;
  tx.rtr = 0;
  tx.data_length_code = 4;
  tx.data[0] = code;
  tx.data[1] = severity;
  tx.data[2] = data0;
  tx.data[3] = data1;
  twai_transmit(&tx, 0);
#else
  (void)code;
  (void)severity;
  (void)data0;
  (void)data1;
#endif
}

void CanBridge::sendConfigAck(uint8_t version, uint8_t status,
                              uint8_t rejectReason, uint8_t activeRatioPercent) {
#if WMETH_HAS_TWAI
  if (!twaiReady_) return;
  twai_message_t tx{};
  tx.identifier = can_protocol::ID_METH_CONFIG_ACK;
  tx.extd = 0;
  tx.rtr = 0;
  tx.data_length_code = 4;
  tx.data[0] = version;
  tx.data[1] = status;
  tx.data[2] = rejectReason;
  tx.data[3] = activeRatioPercent;
  twai_transmit(&tx, 0);
#else
  (void)version;
  (void)status;
  (void)rejectReason;
  (void)activeRatioPercent;
#endif
}

bool CanBridge::isMasterTimedOut(uint32_t nowMs) const {
  if (lastMasterRxMs_ == 0) return false;  // Never heard from master — not yet connected.
  return (nowMs - lastMasterRxMs_) > kMasterTimeoutMs;
}
