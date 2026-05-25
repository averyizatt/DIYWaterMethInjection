#pragma once

#include <stdint.h>
#include "app_config.h"
#include "can_contract/can_protocol.h"
#include "injection_controller.h"
#include "sensors.h"

// ---------------------------------------------------------------------------
// CanBridge
// Handles TWAI (ESP32-S3 built-in CAN) communication with the CCM master.
//
// TX (ID_ENGINE_METH_STATE = 0x300, DLC 8, every 50 ms):
//   Reports injection state, pump duty, tank level, boost kPa, and fault flags.
//
// RX (ID_ENGINE_METH_COMMAND = 0x301, DLC varies):
//   Receives ARM/DISARM, manual-test duty, boost threshold, IAT threshold,
//   and clear-faults commands from the master.
//
// RX (ID_METH_CONFIG_BROADCAST = 0x304, DLC 8):
//   Receives the master's desired configuration and ACKs it via 0x306.
//
// TX (ID_ENGINE_METH_FAULT = 0x302, DLC 4, on demand):
//   Sent when a new fault condition is detected.
// ---------------------------------------------------------------------------
class CanBridge {
public:
  // Initialise the TWAI peripheral. Returns true on success.
  bool begin(int txPin, int rxPin, uint32_t bitrate = 500000);

  // Must be called from the main loop every iteration.
  // Reads incoming frames and updates the command state.
  void poll();

  // Transmit a state frame if the TX interval has elapsed.
  // Call this every loop after running the injection controller.
  void sendStateIfDue(const SensorReadings& readings,
                      const ControlResult& result,
                      const AppConfig& config,
                      uint32_t nowMs);

  // Transmit a fault frame immediately.
  void sendFault(uint8_t code, uint8_t severity, uint8_t data0, uint8_t data1);
  void sendKnockStateIfDue(const can_protocol::EngineKnockState& state, uint32_t nowMs);
  void sendKnockFault(uint8_t code, uint8_t severity, uint8_t data0, uint8_t data1);

  // ---------- command outputs read by main loop ----------
  bool isArmed() const { return armed_; }
  bool hasPendingManualTest() const { return pendingManualTest_; }
  uint8_t manualTestDuty() const { return manualTestDuty_; }
  void clearManualTest() { pendingManualTest_ = false; manualTestDuty_ = 0; }

  // Ratio received from CCM (% methanol). Applied to the local blend.
  uint8_t remoteRatioPercent() const { return remoteRatioPercent_; }
  bool hasRemoteRatio() const { return hasRemoteRatio_; }
  void clearRemoteRatio() { hasRemoteRatio_ = false; }

  // True if the master requested a fault clear.
  bool hasClearFaultsRequest() const { return clearFaultsRequested_; }
  void clearFaultsRequest() { clearFaultsRequested_ = false; }

  bool isOnline() const { return twaiReady_; }

  // True if no state frame has been received from master within the timeout.
  bool isMasterTimedOut(uint32_t nowMs) const;

private:
  void dispatchFrame(uint16_t id, uint8_t dlc, const uint8_t* data, uint32_t nowMs);
  void sendConfigAck(uint8_t version, uint8_t status, uint8_t rejectReason,
                     uint8_t activeRatioPercent);

  bool twaiReady_ = false;
  uint32_t lastStateTxMs_ = 0;
  uint32_t lastMasterRxMs_ = 0;

  bool armed_ = false;
  bool pendingManualTest_ = false;
  uint8_t manualTestDuty_ = 0;

  uint8_t remoteRatioPercent_ = 50;
  bool hasRemoteRatio_ = false;

  bool clearFaultsRequested_ = false;

  // Last config broadcast accepted from master.
  uint8_t lastConfigVersion_ = 0;
  uint8_t lastConfigRatioPercent_ = 255;  // 255 = unknown/custom
};
