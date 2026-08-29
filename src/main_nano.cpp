#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <mcp_can.h>

#include "actuators.h"
#include "app_config.h"
#include "can_contract/can_protocol.h"
#include "injection_controller.h"
#include "pins.h"
#include "sensors.h"

namespace {
AppConfig config = defaultConfig();
TankBlend blend = computeTankBlend(1.5f, 0.5f);

MapSensor mapSensor;
FloatSensor floatSensor;
PumpDriver pumpDriver;
WarningOutput warningOutput;
InjectionController controller;
MCP_CAN canBus(pins::CAN_SPI_CS);

uint32_t lastLoopMs = 0;
uint32_t lastDebugMs = 0;
uint32_t lastCanStateTxMs = 0;
uint32_t lastCanRetryMs = 0;
uint32_t lastCanDiagMs = 0;
uint32_t lastMasterRxMs = 0;
uint32_t manualTestStartMs = 0;
float knockBiasRaw = 0.0f;
float knockEnvelopeRaw = 0.0f;
bool canOnline = false;
uint16_t canDiagTxFailStreak = 0;
bool desiredArmed = false;
bool manualTestActive = false;
uint8_t manualTestDuty = 0;
uint8_t activeConfigVersion = 0;
SensorReadings lastReadings{};

constexpr float kPsiPerKpa = 0.1450377f;
constexpr float kKnockBiasAlpha = 0.01f;
constexpr float kKnockEnvelopeAlpha = 0.18f;
constexpr float kKnockSoftwareGain = 1.0f;
constexpr float kKnockDetectThresholdRaw = 35.0f;
constexpr uint32_t kCanRetryMs = 2000;
constexpr uint32_t kCanDiagMs = 3000;
constexpr uint32_t kCanStateTxMs = 50;
constexpr uint32_t kMasterTimeoutMs = 3000;
constexpr uint32_t kManualTestTimeoutMs = 5000;

enum MethFaultBits : uint8_t {
  kFaultLowTank = 1U << 0,
  kFaultMapInvalid = 1U << 1,
  kFaultBoostInvalid = 1U << 2,
  kFaultConfigInvalid = 1U << 3,
  kFaultOverboostLatched = 1U << 4,
  kFaultCanTimeout = 1U << 5,
};

inline bool elapsed(uint32_t now, uint32_t start, uint32_t intervalMs) {
  return static_cast<uint32_t>(now - start) >= intervalMs;
}

uint8_t clampPercent(float value) {
  if (value <= 0.0f) return 0;
  if (value >= 100.0f) return 100;
  return static_cast<uint8_t>(value + 0.5f);
}

uint8_t tempOffset40OrZero(float tempC, bool valid) {
  return valid ? can_protocol::tempToOffset40(static_cast<int>(tempC)) : can_protocol::tempToOffset40(0);
}

uint8_t faultBitsFor(const SensorReadings& readings, const ControlResult& control, bool masterTimedOut) {
  uint8_t bits = 0;
  if (readings.tankLow) bits |= kFaultLowTank;
  if (!readings.mapValid) bits |= kFaultMapInvalid;
  if (control.failsafe == FailsafeReason::BoostInvalid) bits |= kFaultBoostInvalid;
  if (control.failsafe == FailsafeReason::InvalidBoostConfig ||
      control.failsafe == FailsafeReason::InvalidBlend) {
    bits |= kFaultConfigInvalid;
  }
  if (control.overboostAssistFaultLatched) bits |= kFaultOverboostLatched;
  if (masterTimedOut) bits |= kFaultCanTimeout;
  return bits;
}

uint8_t methStateFor(const SensorReadings& readings, const ControlResult& control, bool masterTimedOut) {
  const uint8_t faults = faultBitsFor(readings, control, masterTimedOut);
  if (faults != 0U && faults != kFaultCanTimeout) {
    return static_cast<uint8_t>(can_protocol::MethState::FAULT);
  }
  if (manualTestActive) return static_cast<uint8_t>(can_protocol::MethState::TEST);
  if (control.pump.enabled && control.finalDutyPercent > 0.0f) {
    return static_cast<uint8_t>(can_protocol::MethState::SPRAYING);
  }
  return desiredArmed ? static_cast<uint8_t>(can_protocol::MethState::ARMED)
                      : static_cast<uint8_t>(can_protocol::MethState::OFF);
}

bool initCanBus() {
  pinMode(pins::CAN_SPI_INT, INPUT_PULLUP);
  SPI.begin();

  if (canBus.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN: MCP2515 init failed");
    return false;
  }

  canBus.setMode(MCP_NORMAL);
  Serial.println("CAN: MCP2515 online at 500 kbps");
  Serial.print("CAN: CS=D");
  Serial.print(pins::CAN_SPI_CS);
  Serial.print(" INT=D");
  Serial.println(pins::CAN_SPI_INT);
  return true;
}

bool sendCanFrame(const can_protocol::CanFrame& frame) {
  if (!canOnline) return false;
  byte data[8] = {};
  const byte len = frame.dlc > 8 ? 8 : frame.dlc;
  for (byte i = 0; i < len; ++i) data[i] = frame.data[i];
  if (canBus.sendMsgBuf(frame.id, 0, len, data) != CAN_OK) {
    if (canDiagTxFailStreak < UINT16_MAX) ++canDiagTxFailStreak;
    Serial.print("CAN: TX failed, streak=");
    Serial.print(canDiagTxFailStreak);
    Serial.println(", will retry init");
    canOnline = false;
    return false;
  }
  canDiagTxFailStreak = 0;
  return true;
}

void serviceCanDiag(uint32_t now) {
  if (!canOnline || !elapsed(now, lastCanDiagMs, kCanDiagMs)) return;
  lastCanDiagMs = now;

  const byte eflg = canBus.getError();
  const byte rec = canBus.errorCountRX();
  const byte tec = canBus.errorCountTX();
  Serial.print("[CAN] EFLG=0x");
  if (eflg < 0x10U) Serial.print('0');
  Serial.print(eflg, HEX);
  Serial.print(" REC=");
  Serial.print(rec);
  Serial.print(" TEC=");
  Serial.print(tec);
  Serial.print(" INT=");
  Serial.println(digitalRead(pins::CAN_SPI_INT));

  if ((eflg & MCP_EFLG_TXBO) != 0U) {
    Serial.println("[CAN] TXBO detected; resetting MCP2515");
    canOnline = false;
    canBus.mcp2515_reset();
    delay(2);
    canOnline = initCanBus();
    lastCanRetryMs = now;
    Serial.println(canOnline ? "[CAN] TXBO recovery OK" : "[CAN] TXBO recovery FAILED");
  }
}

void sendConfigAck(uint8_t status, uint8_t rejectReason) {
  sendCanFrame(can_protocol::packMethConfigAck(activeConfigVersion, status, rejectReason,
                                               static_cast<uint8_t>(blend.methPercent + 0.5f)));
}

void applyConfigBroadcast(const can_protocol::MethConfigBroadcast& cfg) {
  if (!can_protocol::validateMethConfigChecksum(cfg)) {
    sendConfigAck(1, 1);
    return;
  }

  activeConfigVersion = cfg.version;
  desiredArmed = cfg.desired_armed != 0;
  if (cfg.ratio_percent <= 100U) {
    const float methLiters = static_cast<float>(cfg.ratio_percent);
    const float waterLiters = 100.0f - methLiters;
    blend = computeTankBlend(waterLiters, methLiters);
  }
  if (cfg.boost_trigger_kpa > 0U) {
    const float startPsi = (static_cast<float>(cfg.boost_trigger_kpa) - config.map.baroKpa) * kPsiPerKpa;
    config.boost.startPsi = startPsi > 0.0f ? startPsi : 0.0f;
    if (config.boost.fullPsi <= config.boost.startPsi) {
      config.boost.fullPsi = config.boost.startPsi + 4.0f;
    }
  }
  if (cfg.max_pump_duty > 0U && cfg.max_pump_duty <= 100U) {
    config.dutyMaxPercent = static_cast<float>(cfg.max_pump_duty);
  }
  sendConfigAck(0, 0);
}

void handleMethCommand(const can_protocol::CanFrame& frame) {
  if (frame.dlc < 1) return;
  switch (frame.data[0]) {
    case can_protocol::meth_command::ARM:
      if (frame.dlc >= 2) {
        desiredArmed = frame.data[1] != 0;
        if (!desiredArmed) {
          manualTestActive = false;
          manualTestDuty = 0;
        }
      }
      break;
    case can_protocol::meth_command::MANUAL_TEST_DUTY:
      if (frame.dlc >= 2 && frame.data[1] > 0U && frame.data[1] <= 100U && !lastReadings.tankLow) {
        manualTestActive = true;
        manualTestDuty = frame.data[1];
        manualTestStartMs = millis();
      }
      break;
    case can_protocol::meth_command::STOP_MANUAL_TEST:
      manualTestActive = false;
      manualTestDuty = 0;
      break;
    case can_protocol::meth_command::SET_BOOST_TRIGGER:
      if (frame.dlc >= 2) {
        const float startPsi = (static_cast<float>(frame.data[1]) - config.map.baroKpa) * kPsiPerKpa;
        config.boost.startPsi = startPsi > 0.0f ? startPsi : 0.0f;
      }
      break;
    case can_protocol::meth_command::CLEAR_FAULTS:
      controller.clearLatchedFaults();
      break;
    default:
      break;
  }
}

void serviceCanRx(uint32_t now) {
  byte framesRead = 0;
  while (digitalRead(pins::CAN_SPI_INT) == LOW && framesRead < 8) {
    unsigned long rxId = 0;
    byte len = 0;
    byte data[8] = {};

    if (canBus.readMsgBuf(&rxId, &len, data) != CAN_OK) {
      Serial.println("CAN: read failed");
      break;
    }

    can_protocol::CanFrame frame{};
    frame.id = static_cast<uint16_t>(rxId & 0x7FFU);
    frame.dlc = len > 8 ? 8 : len;
    for (byte i = 0; i < frame.dlc; ++i) frame.data[i] = data[i];

    if (frame.id == can_protocol::ID_MASTER_HEARTBEAT ||
        frame.id == can_protocol::ID_METH_CONFIG_BROADCAST ||
        frame.id == can_protocol::ID_ENGINE_METH_COMMAND) {
      lastMasterRxMs = now;
    }

    if (frame.id == can_protocol::ID_ENGINE_METH_COMMAND) {
      handleMethCommand(frame);
    } else if (frame.id == can_protocol::ID_METH_CONFIG_BROADCAST) {
      can_protocol::MethConfigBroadcast cfg{};
      if (can_protocol::unpackMethConfigBroadcast(frame, cfg)) applyConfigBroadcast(cfg);
    } else if (frame.id == can_protocol::ID_METH_CONFIG_REQUEST) {
      sendConfigAck(0, 0);
    }
    ++framesRead;
  }
}

void sendMethState(uint32_t now, const SensorReadings& readings, const ControlResult& control, bool masterTimedOut) {
  if (!elapsed(now, lastCanStateTxMs, kCanStateTxMs)) return;
  lastCanStateTxMs = now;

  can_protocol::CanFrame frame{};
  frame.id = can_protocol::ID_ENGINE_METH_STATE;
  frame.dlc = 8;
  frame.data[0] = methStateFor(readings, control, masterTimedOut);
  frame.data[1] = manualTestActive ? manualTestDuty : clampPercent(control.finalDutyPercent);
  frame.data[2] = readings.tankLow ? 0U : 100U;
  frame.data[3] = control.pump.enabled ? static_cast<uint8_t>(can_protocol::FlowStatus::OK)
                                       : static_cast<uint8_t>(can_protocol::FlowStatus::UNKNOWN);
  frame.data[4] = can_protocol::clampU8(static_cast<int>(readings.mapKpa + 0.5f));
  frame.data[5] = tempOffset40OrZero(readings.iatC, readings.iatValid);
  frame.data[6] = tempOffset40OrZero(readings.engineBayC, readings.engineBayValid);
  frame.data[7] = faultBitsFor(readings, control, masterTimedOut);
  sendCanFrame(frame);
}

void serviceCanBus(uint32_t now, const SensorReadings& readings, const ControlResult& control) {
  if (!canOnline) {
    if (elapsed(now, lastCanRetryMs, kCanRetryMs)) {
      lastCanRetryMs = now;
      canOnline = initCanBus();
    }
    return;
  }

  serviceCanRx(now);
  serviceCanDiag(now);
  if (!canOnline) return;
  const bool masterTimedOut = lastMasterRxMs != 0U && elapsed(now, lastMasterRxMs, kMasterTimeoutMs);
  if (masterTimedOut) {
    desiredArmed = false;
    manualTestActive = false;
    manualTestDuty = 0;
  }
  sendMethState(now, readings, control, masterTimedOut);
}
} // namespace

void setup() {
  Serial.begin(config.serialBaud);
  delay(100);

  mapSensor.begin(pins::MAP_SENSOR_ADC, config.map);
  floatSensor.begin(pins::FLOAT_SENSOR_DIGITAL,
                    config.floatActiveLow,
                    config.floatDebounceMs,
                    config.floatLowShutdownDelayMs);
  pumpDriver.beginRelay(pins::PUMP_OUT, config.relayPeriodMs);
  warningOutput.begin(pins::WARNING_LED, true);
  pinMode(pins::KNOCK_SENSOR_ADC, INPUT);

  Serial.println("Nano water/meth CAN firmware");
  Serial.print("Knock pin: ");
  Serial.println(pins::KNOCK_SENSOR_ADC);
  canOnline = initCanBus();
  lastCanRetryMs = millis();
}

void loop() {
  const uint32_t now = millis();
  if (!elapsed(now, lastLoopMs, config.loopPeriodMs)) return;
  lastLoopMs = now;

  SensorReadings readings = mapSensor.read();
  readings.tankLow = floatSensor.update();
  lastReadings = readings;

  const int knockRaw = analogRead(pins::KNOCK_SENSOR_ADC);
  if (knockBiasRaw <= 0.0f) {
    knockBiasRaw = static_cast<float>(knockRaw);
  } else {
    knockBiasRaw += (static_cast<float>(knockRaw) - knockBiasRaw) * kKnockBiasAlpha;
  }

  const float knockDeltaRaw = static_cast<float>(knockRaw) - knockBiasRaw;
  const float knockAbsGained = fabs(knockDeltaRaw * kKnockSoftwareGain);
  knockEnvelopeRaw += (knockAbsGained - knockEnvelopeRaw) * kKnockEnvelopeAlpha;
  const bool knockDetected = knockEnvelopeRaw >= kKnockDetectThresholdRaw;

  AppConfig runtimeConfig = config;
  runtimeConfig.mode = desiredArmed ? InjectionMode::BoostOnly : InjectionMode::Off;
  ControlResult control = controller.update(readings, runtimeConfig, blend);

  if (manualTestActive && elapsed(now, manualTestStartMs, kManualTestTimeoutMs)) {
    manualTestActive = false;
    manualTestDuty = 0;
  }
  PumpCommand pump = control.pump;
  if (manualTestActive) {
    pump.enabled = true;
    pump.dutyPercent = manualTestDuty;
    control.finalDutyPercent = manualTestDuty;
  }
  pumpDriver.apply(pump);

  const bool warn = knockDetected || readings.tankLow || control.failsafe != FailsafeReason::None ||
                    control.overboostAssistFaultLatched;
  warningOutput.set(warn);
  serviceCanBus(now, readings, control);

  if (elapsed(now, lastDebugMs, 500)) {
    lastDebugMs = now;
    Serial.print("METH armed=");
    Serial.print(desiredArmed ? "YES" : "no");
    Serial.print(" state=");
    Serial.print(methStateFor(readings, control, false));
    Serial.print(" duty=");
    Serial.print(control.finalDutyPercent, 0);
    Serial.print(" mapKpa=");
    Serial.print(readings.mapKpa, 1);
    Serial.print(" tankLow=");
    Serial.print(readings.tankLow ? "YES" : "no");
    Serial.print(" knockRaw=");
    Serial.print(knockRaw);
    Serial.print(" knockEnv=");
    Serial.print(knockEnvelopeRaw, 1);
    Serial.print(" can=");
    Serial.println(canOnline ? "OK" : "OFF");
  }
}
