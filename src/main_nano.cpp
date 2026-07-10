#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

#include "actuators.h"
#include "app_config.h"
#include "can_contract/can_protocol.h"
#include "injection_controller.h"
#include "pins.h"
#include "sensors.h"

#ifndef CCM_CAN_SPEED
#define CCM_CAN_SPEED CAN_500KBPS
#endif

#ifndef CCM_MCP2515_CLOCK
#define CCM_MCP2515_CLOCK MCP_8MHZ
#endif

namespace {
AppConfig config = defaultConfig();
TankBlend blend = computeTankBlend(1.5f, 0.5f);

MapSensor mapSensor;
FloatSensor floatSensor;
PressureSensor oilPressureSensor;
PressureSensor fuelPressureSensor;
PressureSensor methPressureSensor;
PressureSensor boostRefPressureSensor;
PumpDriver pumpDriver;
WarningOutput warningOutput;
InjectionController controller;
MCP_CAN canBus(pins::CAN_SPI_CS);

uint32_t lastLoopMs = 0;
uint32_t lastDebugMs = 0;
uint32_t lastMethStateTxMs = 0;
uint32_t lastSensorExtTxMs = 0;
uint32_t lastKnockStateTxMs = 0;
uint32_t lastKnockHookTxMs = 0;
uint32_t lastCanConfigTxMs = 0;
uint32_t lastCanRetryMs = 0;
uint32_t lastCanCommandMs = 0;
uint32_t lastCanTxFailureLogMs = 0;
uint32_t lastEngineRuntimeRxMs = 0;
float knockBiasRaw = 0.0f;
float knockEnvelopeRaw = 0.0f;
float knockNoiseBaselineRaw = 0.0f;
float knockThresholdRaw = 0.0f;
uint16_t knockSamplesProcessed = 0;
bool canOnline = false;
bool canCommandSeen = false;
bool canCommandTimedOut = false;
bool manualTestActive = false;
uint8_t manualTestDuty = 0;
uint8_t knockEventCount = 0;
uint8_t lastKnockBoostKpa = 0;
uint8_t lastKnockRpmDiv100 = 0;
uint8_t consecutiveCanTxFailures = 0;
bool lastKnockDetected = false;
bool knockWarningActive = false;
bool knockCriticalActive = false;
bool knockSignalValid = false;
bool knockSensorFault = false;
bool knockClippingDetected = false;
bool knockBaselineLearned = false;
bool lastKnockWarningActive = false;
bool lastKnockCriticalActive = false;
bool lastKnockSensorFault = false;
bool lastKnockClippingDetected = false;
bool knockFaultPending = false;
uint8_t pendingKnockFaultCode = 0;
uint8_t pendingKnockFaultSeverity = 0;
uint8_t pendingKnockFaultData0 = 0;
uint8_t pendingKnockFaultData1 = 0;
uint16_t engineRpm = 0;
uint8_t engineRuntimeMapKpa = 0;
bool engineRpmValid = false;
bool engineRuntimeMapValid = false;

constexpr float kKnockAdcRefVoltage = 5.0f;
constexpr float kKnockAdcMaxCount = 1023.0f;
constexpr float kPsiPerKpa = 0.1450377f;

constexpr float kKnockBiasAlpha = 0.01f;
constexpr float kKnockEnvelopeAlpha = 0.18f;
constexpr float kKnockSoftwareGain = 1.0f;
constexpr float kKnockDetectThresholdRaw = 35.0f;
constexpr float kKnockCriticalThresholdRaw = 55.0f;
constexpr uint16_t kKnockBaselineLearnSamples = 50;
constexpr uint32_t kCanRetryMs = 2000;
constexpr uint32_t kCanCommandTimeoutMs = 1000;
constexpr uint32_t kCanTxFailureLogMs = 1000;
constexpr uint32_t kEngineRuntimeTimeoutMs = 500;
constexpr uint32_t kMethStateTxMs = 50;
constexpr uint32_t kSensorExtTxMs = 250;
constexpr uint32_t kKnockStateTxMs = 50;
constexpr uint32_t kKnockHookTxMs = 50;
constexpr uint32_t kCanConfigTxMs = 1000;
constexpr uint8_t kBaroSampleCount = 16;
constexpr float kMinStartupBaroKpa = 60.0f;
constexpr float kMaxStartupBaroKpa = 110.0f;
constexpr uint16_t kAnalogFaultOilPressure = 1U << 0;
constexpr uint16_t kAnalogFaultFuelPressure = 1U << 1;
constexpr uint16_t kAnalogFaultMethPressure = 1U << 2;
constexpr uint16_t kAnalogFaultBoostRefPressure = 1U << 3;

inline bool elapsed(uint32_t now, uint32_t start, uint32_t intervalMs) {
  return static_cast<uint32_t>(now - start) >= intervalMs;
}

float clampFloatValue(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

float currentKnockThresholdRaw() {
  const float multiplier = clampFloatValue(config.knock.thresholdMultiplier, 0.5f, 10.0f);
  const float offset = clampFloatValue(config.knock.thresholdOffset, 0.0f, 255.0f);
  return clampFloatValue((knockNoiseBaselineRaw * multiplier) + offset, 1.0f, 255.0f);
}

bool initCanBus() {
  pinMode(pins::CAN_SPI_INT, INPUT_PULLUP);
  SPI.begin();

  if (canBus.begin(MCP_ANY, CCM_CAN_SPEED, CCM_MCP2515_CLOCK) != CAN_OK) {
    Serial.println("CAN: MCP2515 init failed");
    return false;
  }

  canBus.setMode(MCP_NORMAL);
  Serial.print("CAN: MCP2515 online speed=");
  Serial.print(CCM_CAN_SPEED);
  Serial.print(" clock=");
  Serial.println(CCM_MCP2515_CLOCK);
  Serial.print("CAN: CS=D");
  Serial.print(pins::CAN_SPI_CS);
  Serial.print(" INT=D");
  Serial.println(pins::CAN_SPI_INT);
  return true;
}

void calibrateStartupBaro() {
  float mapTotalKpa = 0.0f;
  uint8_t validSamples = 0;

  for (uint8_t i = 0; i < kBaroSampleCount; ++i) {
    const SensorReadings sample = mapSensor.read();
    if (sample.mapValid && sample.mapKpa >= kMinStartupBaroKpa && sample.mapKpa <= kMaxStartupBaroKpa) {
      mapTotalKpa += sample.mapKpa;
      ++validSamples;
    }
    delay(5);
  }

  if (validSamples == 0) {
    Serial.print("MAP baro kept at ");
    Serial.print(config.map.baroKpa, 1);
    Serial.println(" kPa");
    return;
  }

  config.map.baroKpa = mapTotalKpa / static_cast<float>(validSamples);
  mapSensor.begin(pins::MAP_SENSOR_ADC, config.map);
  Serial.print("MAP baro calibrated to ");
  Serial.print(config.map.baroKpa, 1);
  Serial.println(" kPa");
}

PressureConfig make100PsiDividerPressureConfig(int pin) {
  PressureConfig pressure{};
  pressure.enabled = pin >= 0;
  pressure.pin = pin;
  pressure.adcVref = 5.0f;
  pressure.adcMaxCount = 1023;
  pressure.oversampleCount = 8;
  pressure.filterAlpha = 0.20f;
  pressure.dividerTopOhms = 10000.0f;
  pressure.dividerBottomOhms = 20000.0f;
  pressure.sensorMinV = 0.5f;
  pressure.sensorMaxV = 4.5f;
  pressure.pressureMinPsi = 0.0f;
  pressure.pressureMaxPsi = 100.0f;
  pressure.openCircuitThresholdV = 4.85f;
  pressure.shortThresholdV = 0.10f;
  pressure.minValidPsi = -5.0f;
  pressure.maxValidPsi = 120.0f;
  return pressure;
}

void updatePressureReadings(SensorReadings &readings, uint32_t now) {
  oilPressureSensor.update(now);
  fuelPressureSensor.update(now);
  methPressureSensor.update(now);
  boostRefPressureSensor.update(now);

  readings.oilPressureValid = oilPressureSensor.valid();
  readings.fuelPressureValid = fuelPressureSensor.valid();
  readings.methPressureValid = methPressureSensor.valid();
  readings.boostRefPressureValid = boostRefPressureSensor.valid();

  readings.oilPressurePsi = readings.oilPressureValid ? oilPressureSensor.valuePsi() : 0.0f;
  readings.fuelPressurePsi = readings.fuelPressureValid ? fuelPressureSensor.valuePsi() : 0.0f;
  readings.methPressurePsi = readings.methPressureValid ? methPressureSensor.valuePsi() : 0.0f;
  readings.boostRefPressurePsi = readings.boostRefPressureValid ? boostRefPressureSensor.valuePsi() : 0.0f;

  if (!readings.oilPressureValid && oilPressureSensor.config().enabled) {
    readings.analogFaultFlags |= kAnalogFaultOilPressure;
  }
  if (!readings.fuelPressureValid && fuelPressureSensor.config().enabled) {
    readings.analogFaultFlags |= kAnalogFaultFuelPressure;
  }
  if (!readings.methPressureValid && methPressureSensor.config().enabled) {
    readings.analogFaultFlags |= kAnalogFaultMethPressure;
  }
  if (!readings.boostRefPressureValid && boostRefPressureSensor.config().enabled) {
    readings.analogFaultFlags |= kAnalogFaultBoostRefPressure;
  }
}

void printCanFrame(unsigned long id, byte len, const byte *data) {
  Serial.print("CAN RX id=0x");
  Serial.print(id, HEX);
  Serial.print(" len=");
  Serial.print(len);
  Serial.print(" data=");
  for (byte i = 0; i < len; ++i) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    if (i + 1 < len) Serial.print(' ');
  }
  Serial.println();
}

void printHexByte(uint8_t value) {
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

void printCanErrorFlags(uint8_t eflg) {
  if (eflg == 0) {
    Serial.print(" none");
    return;
  }
  if (eflg & MCP_EFLG_TXBO) Serial.print(" TXBO");
  if (eflg & MCP_EFLG_TXEP) Serial.print(" TXEP");
  if (eflg & MCP_EFLG_RXEP) Serial.print(" RXEP");
  if (eflg & MCP_EFLG_TXWAR) Serial.print(" TXWAR");
  if (eflg & MCP_EFLG_RXWAR) Serial.print(" RXWAR");
  if (eflg & MCP_EFLG_EWARN) Serial.print(" EWARN");
  if (eflg & MCP_EFLG_RX0OVR) Serial.print(" RX0OVR");
  if (eflg & MCP_EFLG_RX1OVR) Serial.print(" RX1OVR");
}

uint8_t sendCanFrame(const can_protocol::CanFrame &frame) {
  return canBus.sendMsgBuf(frame.id, 0, frame.dlc, const_cast<uint8_t *>(frame.data));
}

bool transmitCanFrame(const can_protocol::CanFrame &frame, const char *label) {
  const uint8_t result = sendCanFrame(frame);
  if (result == CAN_OK) {
    consecutiveCanTxFailures = 0;
    return true;
  }

  if (consecutiveCanTxFailures < 255) ++consecutiveCanTxFailures;
  const uint32_t now = millis();
  if (consecutiveCanTxFailures == 1 || elapsed(now, lastCanTxFailureLogMs, kCanTxFailureLogMs)) {
    lastCanTxFailureLogMs = now;
    const uint8_t eflg = canBus.getError();
    const uint8_t tec = canBus.errorCountTX();
    const uint8_t rec = canBus.errorCountRX();
    Serial.print("CAN: ");
    Serial.print(label);
    Serial.print(" TX failed count=");
    Serial.print(consecutiveCanTxFailures);
    Serial.print(" ret=");
    Serial.print(result);
    Serial.print(" EFLG=0x");
    printHexByte(eflg);
    Serial.print(" TEC=");
    Serial.print(tec);
    Serial.print(" REC=");
    Serial.print(rec);
    Serial.print(" flags:");
    printCanErrorFlags(eflg);
    Serial.println();
  }

  return true;
}

uint8_t scaledU8(float value, float scale) {
  return can_protocol::clampU8(static_cast<int>(value * scale + 0.5f));
}

uint8_t constrainAndReport(uint8_t value, uint8_t minValue, uint8_t maxValue, uint8_t &status) {
  if (value < minValue) {
    status = can_protocol::config_ack_status::VALUE_CLAMPED;
    return minValue;
  }
  if (value > maxValue) {
    status = can_protocol::config_ack_status::VALUE_CLAMPED;
    return maxValue;
  }
  return value;
}

bool sendConfigAck(uint8_t command, uint8_t status, uint8_t value) {
  return transmitCanFrame(can_protocol::packConfigAck(
      command, status, value,
      static_cast<uint8_t>(can_protocol::CAN_PROTOCOL_SCHEMA_VERSION)), "0x306");
}

bool sendKnockConfigPages() {
  can_protocol::KnockConfigPage1 page1{};
  page1.config_flags = (config.knock.enabled ? (1U << 0) : 0U) |
                       (config.knock.autoCenterFromBore ? (1U << 1) : 0U);
  page1.threshold_offset = can_protocol::clampU8(static_cast<int>(config.knock.thresholdOffset + 0.5f));
  page1.adaptive_multiplier_x10 = scaledU8(config.knock.thresholdMultiplier, 10.0f);
  page1.min_rpm_div100 = can_protocol::clampU8(config.knock.minRpmToArm / 100U);
  page1.min_map_kpa = can_protocol::clampU8(static_cast<int>(config.knock.minMapKpaToArm + 0.5f));
  page1.debounce_ms_div10 = can_protocol::clampU8(config.knock.eventCooldownMs / 10U);
  page1.gain_x10 = scaledU8(config.knock.signalGain, 10.0f);
  page1.center_frequency_div100 = can_protocol::clampU8(
      static_cast<int>(config.knock.centerFreqHz / 100.0f + 0.5f));

  can_protocol::KnockConfigPage2 page2{};
  page2.bandwidth_div100 = can_protocol::clampU8(
      static_cast<int>(config.knock.bandwidthHz / 100.0f + 0.5f));
  page2.sample_rate_div100 = can_protocol::clampU8(config.knock.sampleRateHz / 100U);
  page2.samples_per_update = config.knock.samplesPerUpdate;
  page2.bias_alpha_x1000 = scaledU8(config.knock.biasAlpha, 1000.0f);
  page2.rms_alpha_x100 = scaledU8(config.knock.rmsAlpha, 100.0f);
  page2.envelope_alpha_x100 = scaledU8(config.knock.envelopeAlpha, 100.0f);
  page2.bore_mm = can_protocol::clampU8(static_cast<int>(config.knock.boreMm + 0.5f));

  return transmitCanFrame(can_protocol::packKnockConfigPage1(page1), "0x30C") &&
         transmitCanFrame(can_protocol::packKnockConfigPage2(page2), "0x30D");
}

void applyCanCommandTimeout(uint32_t now) {
  if (!canCommandSeen || !elapsed(now, lastCanCommandMs, kCanCommandTimeoutMs)) return;

  if (!canCommandTimedOut) {
    Serial.println("CAN: command timeout, disarming");
  }
  canCommandTimedOut = true;
  manualTestActive = false;
  manualTestDuty = 0;
  config.mode = InjectionMode::Off;
}

void handleEngineRuntimeFrame(byte len, const byte *data) {
  if (len < 2) return;

  engineRpm = static_cast<uint16_t>(data[0]) |
              (static_cast<uint16_t>(data[1]) << 8U);
  engineRpmValid = true;
  lastEngineRuntimeRxMs = millis();

  if (len >= 3) {
    engineRuntimeMapKpa = data[2];
    engineRuntimeMapValid = true;
  }
  if (len >= 4) {
    engineRpmValid = (data[3] & (1U << 0)) != 0;
    engineRuntimeMapValid = (data[3] & (1U << 1)) != 0;
  }
}

uint16_t currentEngineRpm(uint32_t now) {
  if (!engineRpmValid || elapsed(now, lastEngineRuntimeRxMs, kEngineRuntimeTimeoutMs)) {
    engineRpmValid = false;
    return 0;
  }
  return engineRpm;
}

void handleCanFrame(unsigned long id, byte len, const byte *data) {
  if (id == can_protocol::ID_ENGINE_RUNTIME) {
    handleEngineRuntimeFrame(len, data);
    return;
  }

  if (id == can_protocol::ID_METH_CONFIG_REQUEST) {
    if (!sendKnockConfigPages()) {
      Serial.println("CAN: config request TX failed, will retry init");
      canOnline = false;
    }
    return;
  }

  if (id != can_protocol::ID_ENGINE_METH_COMMAND) {
    return;
  }

  if (len < 1) {
    sendConfigAck(0, can_protocol::config_ack_status::INVALID_LENGTH, 0);
    return;
  }

  uint8_t ackStatus = can_protocol::config_ack_status::OK;
  uint8_t ackValue = len >= 2 ? data[1] : 0;
  bool commandAccepted = true;

  switch (data[0]) {
  case can_protocol::meth_command::ARM:
    if (len >= 2) {
      config.mode = data[1] ? InjectionMode::BoostOnly : InjectionMode::Off;
      if (!data[1]) {
        manualTestActive = false;
        manualTestDuty = 0;
      }
      Serial.println(data[1] ? "CAN CMD: meth armed" : "CAN CMD: meth disarmed");
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::MANUAL_TEST_DUTY:
    if (len >= 2) {
      manualTestActive = true;
      manualTestDuty = constrainAndReport(data[1], 0, 100, ackStatus);
      ackValue = manualTestDuty;
      Serial.print("CAN CMD: manual duty=");
      Serial.println(manualTestDuty);
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::STOP_MANUAL_TEST:
    manualTestActive = false;
    manualTestDuty = 0;
    Serial.println("CAN CMD: manual stop");
    break;
  case can_protocol::meth_command::SET_BOOST_TRIGGER:
    if (len >= 2) {
      const uint8_t boostKpa = constrainAndReport(data[1], 0, 250, ackStatus);
      config.boost.startPsi = static_cast<float>(boostKpa) * kPsiPerKpa;
      ackValue = boostKpa;
      Serial.print("CAN CMD: boost trigger kPa=");
      Serial.println(boostKpa);
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::CLEAR_FAULTS:
    controller.clearLatchedFaults();
    canCommandTimedOut = false;
    Serial.println("CAN CMD: clear faults");
    break;
  case can_protocol::meth_command::KNOCK_SET_ENABLE:
    if (len >= 2) {
      config.knock.enabled = data[1] != 0;
      ackValue = config.knock.enabled ? 1 : 0;
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::KNOCK_SET_THRESHOLD_OFFSET:
    if (len >= 2) {
      const uint8_t value = constrainAndReport(data[1], 0, 200, ackStatus);
      config.knock.thresholdOffset = static_cast<float>(value);
      ackValue = value;
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::KNOCK_SET_ADAPTIVE_MULTIPLIER_X10:
    if (len >= 2) {
      const uint8_t value = constrainAndReport(data[1], 12, 38, ackStatus);
      config.knock.thresholdMultiplier = static_cast<float>(value) / 10.0f;
      ackValue = value;
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::KNOCK_SET_MIN_RPM_DIV100:
    if (len >= 2) {
      config.knock.minRpmToArm = static_cast<uint16_t>(data[1]) * 100U;
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::KNOCK_SET_MIN_MAP_KPA:
    if (len >= 2) {
      const uint8_t value = constrainAndReport(data[1], 0, 250, ackStatus);
      config.knock.minMapKpaToArm = static_cast<float>(value);
      config.knock.boostEnableKpa = config.knock.minMapKpaToArm;
      ackValue = value;
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::KNOCK_SET_DEBOUNCE_MS_DIV10:
    if (len >= 2) {
      const uint8_t value = constrainAndReport(data[1], 1, 250, ackStatus);
      config.knock.eventCooldownMs = static_cast<uint16_t>(value) * 10U;
      ackValue = value;
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::KNOCK_SET_GAIN_X10:
    if (len >= 2) {
      const uint8_t value = constrainAndReport(data[1], 1, 200, ackStatus);
      config.knock.signalGain = static_cast<float>(value) / 10.0f;
      ackValue = value;
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::KNOCK_SET_CENTER_FREQ_DIV100:
    if (len >= 2) {
      const uint8_t value = constrainAndReport(data[1], 30, 250, ackStatus);
      config.knock.centerFreqHz = static_cast<float>(value) * 100.0f;
      config.knock.autoCenterFromBore = false;
      ackValue = value;
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::KNOCK_SET_BANDWIDTH_DIV100:
    if (len >= 2) {
      const uint8_t value = constrainAndReport(data[1], 3, 200, ackStatus);
      config.knock.bandwidthHz = static_cast<float>(value) * 100.0f;
      ackValue = value;
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::KNOCK_SET_AUTO_FREQ_FROM_BORE:
    if (len >= 2) {
      config.knock.autoCenterFromBore = data[1] != 0;
      ackValue = config.knock.autoCenterFromBore ? 1 : 0;
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::KNOCK_CLEAR_EVENTS:
    knockEventCount = 0;
    lastKnockBoostKpa = 0;
    lastKnockRpmDiv100 = 0;
    lastKnockDetected = false;
    lastKnockWarningActive = false;
    lastKnockCriticalActive = false;
    lastKnockSensorFault = false;
    lastKnockClippingDetected = false;
    knockFaultPending = false;
    break;
  default:
    Serial.print("CAN CMD: unsupported 0x");
    Serial.println(data[0], HEX);
    ackStatus = can_protocol::config_ack_status::UNSUPPORTED_COMMAND;
    commandAccepted = false;
    break;
  }

  if (ackStatus == can_protocol::config_ack_status::OK ||
      ackStatus == can_protocol::config_ack_status::VALUE_CLAMPED) {
    canCommandSeen = true;
    canCommandTimedOut = false;
    lastCanCommandMs = millis();
  }

  if (!sendConfigAck(data[0], ackStatus, ackValue)) {
    Serial.println("CAN: config ACK TX failed, will retry init");
    canOnline = false;
  } else if (commandAccepted &&
             (ackStatus == can_protocol::config_ack_status::OK ||
              ackStatus == can_protocol::config_ack_status::VALUE_CLAMPED) &&
             !sendKnockConfigPages()) {
    Serial.println("CAN: config page TX failed, will retry init");
    canOnline = false;
  }
}

uint8_t methStateFor(const ControlResult &result, const SensorReadings &readings) {
  if (manualTestActive) return static_cast<uint8_t>(can_protocol::MethState::TEST);
  if (result.failsafe != FailsafeReason::None || result.overboostAssistFaultLatched) {
    return static_cast<uint8_t>(can_protocol::MethState::FAULT);
  }
  if (result.pump.enabled && result.finalDutyPercent > 0.0f) {
    return static_cast<uint8_t>(can_protocol::MethState::SPRAYING);
  }
  if (config.mode != InjectionMode::Off && !readings.tankLow) {
    return static_cast<uint8_t>(can_protocol::MethState::ARMED);
  }
  return static_cast<uint8_t>(can_protocol::MethState::OFF);
}

uint8_t methFaultFlagsFor(const ControlResult &result, const SensorReadings &readings) {
  uint8_t flags = 0;
  if (readings.tankLow || result.failsafe == FailsafeReason::LowFluid) flags |= (1U << 0);
  if (!readings.mapValid || result.failsafe == FailsafeReason::MapInvalid) flags |= (1U << 1);
  if (result.overboostAssistActive) flags |= (1U << 2);
  if (result.overboostEmergencyActive || result.overboostAssistFaultLatched) flags |= (1U << 3);
  if (result.failsafe == FailsafeReason::InvalidBlend ||
      result.failsafe == FailsafeReason::InvalidBoostConfig) {
    flags |= (1U << 4);
  }
  if (canCommandTimedOut) flags |= (1U << 5);
  return flags;
}

uint8_t boostGaugeKpaForCan(const SensorReadings &readings) {
  const float boostGaugeKpa = readings.mapKpa - config.map.baroKpa;
  return can_protocol::clampU8(static_cast<int>(boostGaugeKpa > 0.0f ? boostGaugeKpa + 0.5f : 0.0f));
}

void queueKnockFault(uint8_t code, uint8_t severity, uint8_t data0, uint8_t data1) {
  pendingKnockFaultCode = code;
  pendingKnockFaultSeverity = severity;
  pendingKnockFaultData0 = data0;
  pendingKnockFaultData1 = data1;
  knockFaultPending = true;
}

void serviceCanBus(uint32_t now,
                   const SensorReadings &readings,
                   const ControlResult &result,
                   bool knockDetected,
                   int knockRaw) {
  if (!canOnline) {
    if (elapsed(now, lastCanRetryMs, kCanRetryMs)) {
      lastCanRetryMs = now;
      canOnline = initCanBus();
      if (canOnline) consecutiveCanTxFailures = 0;
    }
    return;
  }

  applyCanCommandTimeout(now);

  byte framesRead = 0;
  while (digitalRead(pins::CAN_SPI_INT) == LOW && framesRead < 8) {
    unsigned long rxId = 0;
    byte len = 0;
    byte data[8] = {};

    if (canBus.readMsgBuf(&rxId, &len, data) == CAN_OK) {
      printCanFrame(rxId, len, data);
      handleCanFrame(rxId, len, data);
    } else {
      Serial.println("CAN: read failed");
      break;
    }
    ++framesRead;
  }

  const uint8_t commandedDuty = manualTestActive ? manualTestDuty :
      can_protocol::clampU8(static_cast<int>(result.finalDutyPercent + 0.5f));

  if (knockDetected && !lastKnockDetected) {
    ++knockEventCount;
    lastKnockBoostKpa = boostGaugeKpaForCan(readings);
    lastKnockRpmDiv100 = can_protocol::clampU8(currentEngineRpm(now) / 100U);
  }
  lastKnockDetected = knockDetected;

  if (elapsed(now, lastMethStateTxMs, kMethStateTxMs)) {
    lastMethStateTxMs = now;
    can_protocol::EngineMethState state{};
    state.meth_state = methStateFor(result, readings);
    state.pump_duty = commandedDuty;
    state.tank_level = readings.tankLow ? 0 : 100;
    state.flow_status = static_cast<uint8_t>(can_protocol::FlowStatus::UNKNOWN);
    state.boost_kpa = boostGaugeKpaForCan(readings);
    state.iat_c = static_cast<int8_t>(readings.iatValid ? readings.iatC : 0.0f);
    state.engine_bay_c = static_cast<int8_t>(readings.engineBayValid ? readings.engineBayC : 0.0f);
    state.fault_flags = methFaultFlagsFor(result, readings);

    if (!transmitCanFrame(can_protocol::packEngineMethState(state), "0x300")) {
      return;
    }
  }

  if (elapsed(now, lastSensorExtTxMs, kSensorExtTxMs)) {
    lastSensorExtTxMs = now;
    can_protocol::EngineSensorExt ext{};
    ext.oil_pressure_psi = can_protocol::clampU8(static_cast<int>(readings.oilPressurePsi + 0.5f));
    ext.fuel_pressure_psi = can_protocol::clampU8(static_cast<int>(readings.fuelPressurePsi + 0.5f));
    ext.meth_pressure_psi = can_protocol::clampU8(static_cast<int>(readings.methPressurePsi + 0.5f));
    ext.boost_ref_pressure_psi = can_protocol::clampU8(static_cast<int>(readings.boostRefPressurePsi + 0.5f));
    ext.ambient_temp_c = static_cast<int8_t>(readings.ambientValid ? readings.ambientC : 0.0f);
    ext.cabin_temp_c = static_cast<int8_t>(readings.cabinValid ? readings.cabinC : 0.0f);
    ext.analog_fault_flags = readings.analogFaultFlags;

    if (!transmitCanFrame(can_protocol::packEngineSensorExt(ext), "0x303")) {
      return;
    }
  }

  if (elapsed(now, lastKnockStateTxMs, kKnockStateTxMs)) {
    lastKnockStateTxMs = now;
    can_protocol::EngineKnockState state{};
    state.status_flags |= config.knock.enabled ? (1U << 0) : 0U;
    state.status_flags |= knockSignalValid ? (1U << 1) : 0U;
    state.status_flags |= knockWarningActive ? (1U << 2) : 0U;
    state.status_flags |= knockCriticalActive ? (1U << 3) : 0U;
    state.status_flags |= knockBaselineLearned ? (1U << 4) : 0U;
    state.status_flags |= knockSensorFault ? (1U << 5) : 0U;
    state.status_flags |= knockClippingDetected ? (1U << 6) : 0U;
    state.energy = can_protocol::clampU8(static_cast<int>(knockEnvelopeRaw + 0.5f));
    state.baseline = can_protocol::clampU8(static_cast<int>(knockNoiseBaselineRaw + 0.5f));
    state.threshold = can_protocol::clampU8(static_cast<int>(knockThresholdRaw + 0.5f));
    state.event_count = knockEventCount;
    state.last_event_rpm_div100 = lastKnockRpmDiv100;
    state.last_event_boost_kpa = lastKnockBoostKpa;
    state.reserved = 0;

    if (!transmitCanFrame(can_protocol::packEngineKnockState(state), "0x307")) {
      return;
    }
  }

  if (knockFaultPending) {
    if (transmitCanFrame(can_protocol::packEngineKnockFault(pendingKnockFaultCode,
                                                            pendingKnockFaultSeverity,
                                                            pendingKnockFaultData0,
                                                            pendingKnockFaultData1),
                         "0x308")) {
      knockFaultPending = false;
    }
  }

  if (elapsed(now, lastKnockHookTxMs, kKnockHookTxMs)) {
    lastKnockHookTxMs = now;
    can_protocol::KnockLiveHook hook{};
    hook.flags |= config.knock.enabled ? (1U << 0) : 0U;
    hook.flags |= (readings.mapKpa >= config.knock.minMapKpaToArm) ? (1U << 1) : 0U;
    hook.flags |= knockWarningActive ? (1U << 2) : 0U;
    hook.flags |= knockSensorFault ? (1U << 3) : 0U;
    hook.flags |= knockClippingDetected ? (1U << 4) : 0U;
    hook.flags |= knockWarningActive ? (1U << 5) : 0U;
    hook.flags |= knockCriticalActive ? (1U << 6) : 0U;
    hook.flags |= knockBaselineLearned ? (1U << 7) : 0U;
    hook.live_knock_rms = can_protocol::clampU8(static_cast<int>(knockEnvelopeRaw + 0.5f));
    hook.adaptive_threshold = can_protocol::clampU8(static_cast<int>(knockThresholdRaw + 0.5f));
    hook.adaptive_baseline = can_protocol::clampU8(static_cast<int>(knockNoiseBaselineRaw + 0.5f));
    hook.event_count = knockEventCount;
    hook.bias_adc_div16 = can_protocol::clampU8(static_cast<int>(knockBiasRaw / 16.0f + 0.5f));
    hook.raw_adc_div16 = can_protocol::clampU8(knockRaw / 16);
    hook.envelope_level = can_protocol::clampU8(static_cast<int>(knockEnvelopeRaw + 0.5f));

    if (!transmitCanFrame(can_protocol::packKnockLiveHook(hook), "0x30B")) {
      return;
    }
  }

  if (elapsed(now, lastCanConfigTxMs, kCanConfigTxMs)) {
    lastCanConfigTxMs = now;
    if (!sendKnockConfigPages()) {
      return;
    }
  }
}
} // namespace

void setup() {
  Serial.begin(config.serialBaud);
  delay(100);

  mapSensor.begin(pins::MAP_SENSOR_ADC, config.map);
  calibrateStartupBaro();
  oilPressureSensor.begin(make100PsiDividerPressureConfig(pins::OIL_PRESSURE_ADC));
  fuelPressureSensor.begin(make100PsiDividerPressureConfig(pins::FUEL_PRESSURE_ADC));
  methPressureSensor.begin(make100PsiDividerPressureConfig(pins::METH_PRESSURE_ADC));
  boostRefPressureSensor.begin(make100PsiDividerPressureConfig(pins::BOOST_REF_PRESSURE_ADC));
  floatSensor.begin(pins::FLOAT_SENSOR_DIGITAL,
                    config.floatActiveLow,
                    config.floatDebounceMs,
                    config.floatLowShutdownDelayMs);
  pumpDriver.beginRelay(pins::PUMP_OUT, config.relayPeriodMs);
  warningOutput.begin(pins::WARNING_LED, true);
  pinMode(pins::KNOCK_SENSOR_ADC, INPUT);

  Serial.println("Nano minimal injection firmware");
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
  updatePressureReadings(readings, now);
  const uint16_t rpm = currentEngineRpm(now);

  const int knockRaw = analogRead(pins::KNOCK_SENSOR_ADC);
  const uint16_t knockRawCount = static_cast<uint16_t>(knockRaw < 0 ? 0 : knockRaw);
  if (knockBiasRaw <= 0.0f) {
    knockBiasRaw = static_cast<float>(knockRaw);
  } else {
    const float biasAlpha = clampFloatValue(config.knock.biasAlpha, 0.0001f, 0.2f);
    knockBiasRaw += (static_cast<float>(knockRaw) - knockBiasRaw) * biasAlpha;
  }
  if (knockSamplesProcessed < 65535U) ++knockSamplesProcessed;

  const float knockDeltaRaw = static_cast<float>(knockRaw) - knockBiasRaw;
  const float knockGain = clampFloatValue(config.knock.signalGain, 0.05f, 20.0f);
  const float knockDeltaGained = knockDeltaRaw * knockGain;
  const float knockAbsGained = fabs(knockDeltaGained);
  const float envelopeAlpha = clampFloatValue(config.knock.envelopeAlpha, 0.01f, 0.8f);
  knockEnvelopeRaw += (knockAbsGained - knockEnvelopeRaw) * envelopeAlpha;

  if (knockNoiseBaselineRaw <= 0.0f) {
    knockNoiseBaselineRaw = knockEnvelopeRaw;
  } else if (!knockWarningActive) {
    const float learnAlpha = clampFloatValue(config.knock.baselineLearnAlpha, 0.001f, 0.2f);
    knockNoiseBaselineRaw += (knockEnvelopeRaw - knockNoiseBaselineRaw) * learnAlpha;
  }
  knockThresholdRaw = currentKnockThresholdRaw();
  knockBaselineLearned = knockSamplesProcessed >= kKnockBaselineLearnSamples;
  knockClippingDetected =
      knockRawCount <= config.knock.clipLowAdc || knockRawCount >= config.knock.clipHighAdc;
  knockSensorFault = knockClippingDetected;
  const bool rpmArmed = config.knock.minRpmToArm == 0 || rpm >= config.knock.minRpmToArm;
  const bool mapArmed = readings.mapKpa >= config.knock.minMapKpaToArm;
  knockSignalValid = config.knock.enabled && knockBaselineLearned && !knockSensorFault &&
                     rpmArmed && mapArmed;
  knockWarningActive = knockSignalValid && knockEnvelopeRaw >= knockThresholdRaw;
  knockCriticalActive = knockSignalValid && knockEnvelopeRaw >= (knockThresholdRaw * 1.6f);
  const bool knockDetected = knockWarningActive;

  if (knockWarningActive && !lastKnockWarningActive) {
    queueKnockFault(can_protocol::knock_fault_code::KNOCK_WARNING,
                    static_cast<uint8_t>(can_protocol::FaultSeverity::WARNING),
                    can_protocol::clampU8(static_cast<int>(knockEnvelopeRaw + 0.5f)),
                    boostGaugeKpaForCan(readings));
  }
  if (knockCriticalActive && !lastKnockCriticalActive) {
    queueKnockFault(can_protocol::knock_fault_code::KNOCK_CRITICAL,
                    static_cast<uint8_t>(can_protocol::FaultSeverity::CRITICAL),
                    can_protocol::clampU8(static_cast<int>(knockEnvelopeRaw + 0.5f)),
                    boostGaugeKpaForCan(readings));
  }
  if (knockSensorFault && !lastKnockSensorFault) {
    queueKnockFault(can_protocol::knock_fault_code::SENSOR_DISCONNECTED,
                    static_cast<uint8_t>(can_protocol::FaultSeverity::WARNING),
                    can_protocol::clampU8(knockRaw / 4),
                    knockClippingDetected ? 1 : 0);
  }
  if (knockClippingDetected && !lastKnockClippingDetected) {
    queueKnockFault(can_protocol::knock_fault_code::SIGNAL_CLIPPING,
                    static_cast<uint8_t>(can_protocol::FaultSeverity::WARNING),
                    can_protocol::clampU8(knockRaw / 4),
                    0);
  }
  lastKnockWarningActive = knockWarningActive;
  lastKnockCriticalActive = knockCriticalActive;
  lastKnockSensorFault = knockSensorFault;
  lastKnockClippingDetected = knockClippingDetected;

  applyCanCommandTimeout(now);
  ControlResult result = controller.update(readings, config, blend);

  if (manualTestActive) {
    result.pump.enabled = manualTestDuty > 0;
    result.pump.dutyPercent = manualTestDuty;
    result.finalDutyPercent = manualTestDuty;
  }

  pumpDriver.apply(result.pump);
  warningOutput.set(result.failsafe != FailsafeReason::None ||
                    result.overboostAssistFaultLatched ||
                    knockDetected);

  serviceCanBus(now, readings, result, knockDetected, knockRaw);

  if (elapsed(now, lastDebugMs, 500)) {
    lastDebugMs = now;
    Serial.print("KNOCK raw=");
    Serial.print(knockRaw);
    Serial.print(" v=");
    Serial.print(static_cast<float>(knockRaw) * kKnockAdcRefVoltage / kKnockAdcMaxCount, 3);
    Serial.print(" biasRaw=");
    Serial.print(knockBiasRaw, 1);
    Serial.print(" biasV=");
    Serial.print(knockBiasRaw * kKnockAdcRefVoltage / kKnockAdcMaxCount, 3);
    Serial.print(" deltaRaw=");
    Serial.print(knockDeltaRaw, 1);
    Serial.print(" deltaGain=");
    Serial.print(knockDeltaGained, 1);
    Serial.print(" envGain=");
    Serial.print(knockEnvelopeRaw, 1);
    Serial.print(" gain=");
    Serial.print(config.knock.signalGain, 1);
    Serial.print(" noiseBase=");
    Serial.print(knockNoiseBaselineRaw, 1);
    Serial.print(" thresh=");
    Serial.print(knockThresholdRaw, 1);
    Serial.print(" detected=");
    Serial.print(knockDetected ? "YES" : "no");
    Serial.print(" rpm=");
    Serial.print(rpm);
    Serial.print(engineRpmValid ? "" : "!");
    Serial.print(" MAP=");
    Serial.print(readings.mapKpa, 1);
    Serial.print("kPa boost=");
    Serial.print(readings.boostPsi, 1);
    Serial.print("psi canBoost=");
    Serial.print(boostGaugeKpaForCan(readings));
    Serial.print("kPa oil=");
    Serial.print(readings.oilPressurePsi, 1);
    Serial.print(readings.oilPressureValid ? "psi" : "psi!");
    Serial.print(" fuel=");
    Serial.print(readings.fuelPressurePsi, 1);
    Serial.print(readings.fuelPressureValid ? "psi" : "psi!");
    Serial.print(" faults=0x");
    Serial.println(readings.analogFaultFlags, HEX);
  }
}
