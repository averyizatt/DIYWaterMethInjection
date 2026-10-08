#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <math.h>
#include <mcp_can.h>

#include "actuators.h"
#include "app_config.h"
#include "can_contract/can_protocol.h"
#include "injection_controller.h"
#include "meth_tune.h"
#include "pins.h"
#include "sensors.h"

#ifndef CCM_CAN_SPEED
#define CCM_CAN_SPEED CAN_500KBPS
#endif

#ifndef CCM_MCP2515_CLOCK
#define CCM_MCP2515_CLOCK MCP_8MHZ
#endif

#ifndef CCM_CAN_RX_TRACE
#define CCM_CAN_RX_TRACE 0
#endif

#ifdef CCM_CAN_PROTOCOL_VERSION
static_assert(CCM_CAN_PROTOCOL_VERSION == can_protocol::CAN_PROTOCOL_SCHEMA_VERSION,
              "platformio.ini CCM_CAN_PROTOCOL_VERSION does not match the shared CAN contract");
#endif

#ifndef CAN_PROTOCOL_HAS_METH_TUNE
#error "Update DIYComfortControlModule: the shared CAN contract needs water/meth tuning (extension 4)."
#endif
static_assert(meth_tune::kCount == can_protocol::meth_tune_setting::COUNT &&
              meth_tune::kPeriodMs == can_protocol::meth_tune_setting::PERIOD_MS &&
              meth_tune::kOverboostAssist == can_protocol::meth_tune_setting::OVERBOOST_ASSIST &&
              meth_tune::kMaxDosePct == can_protocol::meth_tune_setting::MAX_DOSE_PCT &&
              meth_tune::kHotFullC == can_protocol::meth_tune_setting::HOT_FULL_C &&
              meth_tune::kHoldDoseLimit == can_protocol::meth_hold::DOSE_LIMIT &&
              meth_tune::kHoldFault == can_protocol::meth_hold::FAULT &&
              meth_tune::kSetClamped == can_protocol::config_ack_status::VALUE_CLAMPED,
              "meth_tune.h does not match the shared CAN contract");

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

// Pulse tuning (meth_tune.h): adjustable over CAN, saved in EEPROM on request.
meth_tune::Settings tune;
meth_tune::Governor governor;
uint8_t tuneRevision = 0;     // Bumped whenever a setting changes.
bool tuneUnsaved = false;
bool tuneSavePending = false; // SAVE waits until the pump is not in the middle of a pulse.
uint8_t tuneReportNext = 0;   // Next setting to report; 0 = no report running.
uint32_t lastTuneStatusTxMs = 0;
uint32_t lastTuneTempsTxMs = 0;
uint32_t lastIntakeTempMs = 0;
float preTempC = 0.0f;
float postTempC = 0.0f;
bool preTempValid = false;
bool postTempValid = false;
// Intake temperature forwarded from the ECU by the dash, used for the temperature
// features until the sensor before the nozzle is fitted.
float ecuIntakeC = 0.0f;
uint32_t ecuIntakeSeenMs = 0;
bool ecuIntakeSeen = false;
bool usingEcuTemp = false;
bool lastPumpOn = false;
constexpr uint32_t kEcuIntakeTimeoutMs = 1500;
constexpr int kTuneEepromAddress = 0;
constexpr uint8_t kTuneEepromMagic = 0x4D;
constexpr uint8_t kTuneEepromVersion = 2;
constexpr uint32_t kIntakeTempSampleMs = 50;

uint32_t lastLoopMs = 0;
uint32_t lastDebugMs = 0;
uint32_t lastMethStateTxMs = 0;
uint32_t lastSensorExtTxMs = 0;
uint32_t lastKnockStateTxMs = 0;
uint32_t lastKnockHookTxMs = 0;
uint32_t lastCanConfigTxMs = 0;
uint32_t lastCanRetryMs = 0;
uint32_t lastCanDiagMs = 0;
uint32_t lastCanRxPollMs = 0;
uint32_t lastCanCommandMs = 0;
uint32_t lastCanTxFailureLogMs = 0;
uint32_t lastEngineRuntimeRxMs = 0;
uint32_t manualTestStartMs = 0;
uint32_t lastCanHealthLogMs = 0;
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
constexpr uint16_t kKnockBaselineLearnSamples = 50;
constexpr uint32_t kCanRetryMs = 2000;
constexpr uint32_t kCanDiagMs = 3000;
constexpr uint32_t kCanRxFallbackPollMs = 50;
constexpr uint32_t kCanCommandTimeoutMs = 3000;
constexpr uint32_t kManualTestTimeoutMs = 5000;
constexpr uint32_t kCanTxFailureLogMs = 1000;
constexpr uint32_t kCanHealthLogMs = 1000;
constexpr uint32_t kEngineRuntimeTimeoutMs = 500;
constexpr uint32_t kMethStateTxMs = 50;
constexpr uint32_t kSensorExtTxMs = 250;
constexpr uint32_t kKnockStateTxMs = 50;
constexpr uint32_t kKnockHookTxMs = 50;
constexpr uint32_t kCanConfigTxMs = 1000;
constexpr uint32_t kSerialHeartbeatMs = 1000;
constexpr bool kSerialCanVerbose = false;
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

void printHexByte(uint8_t value);

float clampFloatValue(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

const char *pressureFaultName(PressureFault fault) {
  switch (fault) {
  case PressureFault::None:
    return "none";
  case PressureFault::Disabled:
    return "disabled";
  case PressureFault::OpenCircuit:
    return "open";
  case PressureFault::ShortToGround:
    return "short";
  case PressureFault::OutOfRange:
    return "range";
  case PressureFault::InvalidConfig:
    return "cfg";
  case PressureFault::StaleReading:
    return "stale";
  case PressureFault::AdcError:
    return "adc";
  default:
    return "unknown";
  }
}

float currentKnockThresholdRaw() {
  const float multiplier = clampFloatValue(config.knock.thresholdMultiplier, 0.5f, 10.0f);
  const float offset = clampFloatValue(config.knock.thresholdOffset, 0.0f, 255.0f);
  return clampFloatValue((knockNoiseBaselineRaw * multiplier) + offset, 1.0f, 255.0f);
}

const char *canSpeedName(uint8_t speed) {
  switch (speed) {
  case CAN_500KBPS:
    return "500kbps";
  case CAN_250KBPS:
    return "250kbps";
  case CAN_125KBPS:
    return "125kbps";
  case CAN_1000KBPS:
    return "1000kbps";
  default:
    return "custom";
  }
}

const char *canClockName(uint8_t clock) {
  switch (clock) {
  case MCP_8MHZ:
    return "8MHz";
  case MCP_16MHZ:
    return "16MHz";
  case MCP_20MHZ:
    return "20MHz";
  case MCP_10MHZ:
    return "10MHz";
  default:
    return "unknown";
  }
}

const char *canModeName(uint8_t modeBits) {
  switch (modeBits & MODE_MASK) {
  case MCP_NORMAL:
    return "normal";
  case MCP_SLEEP:
    return "sleep";
  case MCP_LOOPBACK:
    return "loopback";
  case MCP_LISTENONLY:
    return "listen-only";
  case MODE_CONFIG:
    return "config";
  default:
    return "unknown";
  }
}

uint8_t mcp2515DebugReadRegister(uint8_t address) {
  digitalWrite(pins::CAN_SPI_CS, LOW);
  SPI.transfer(MCP_READ);
  SPI.transfer(address);
  const uint8_t value = SPI.transfer(0x00);
  digitalWrite(pins::CAN_SPI_CS, HIGH);
  return value;
}

uint8_t mcp2515DebugReadStatus() {
  digitalWrite(pins::CAN_SPI_CS, LOW);
  SPI.transfer(MCP_READ_STATUS);
  const uint8_t value = SPI.transfer(0x00);
  digitalWrite(pins::CAN_SPI_CS, HIGH);
  return value;
}

void runCanLoopbackSelfTest() {
  const uint8_t loopbackModeResult = canBus.setMode(MCP_LOOPBACK);
  uint8_t data[1] = {0xA5};
  const uint8_t txResult = canBus.sendMsgBuf(can_protocol::ID_ENGINE_KNOCK_STATE, 0, 1, data);
  bool rxMatched = false;
  unsigned long rxId = 0;
  byte len = 0;
  byte rxData[8] = {};
  const uint32_t start = millis();

  while (elapsed(millis(), start, 25) == false) {
    if (canBus.checkReceive() == CAN_MSGAVAIL &&
        canBus.readMsgBuf(&rxId, &len, rxData) == CAN_OK) {
      rxMatched = rxId == can_protocol::ID_ENGINE_KNOCK_STATE &&
                  len == 1 &&
                  rxData[0] == data[0];
      break;
    }
  }

  Serial.print(F("CAN SELFTEST loopbackMode="));
  Serial.print(loopbackModeResult);
  Serial.print(F(" tx="));
  Serial.print(txResult);
  Serial.print(F(" rx="));
  Serial.print(rxMatched ? 1 : 0);
  Serial.print(F(" id=0x"));
  Serial.print(rxId, HEX);
  Serial.print(F(" len="));
  Serial.println(len);
}

bool initCanBus() {
  pinMode(pins::CAN_SPI_CS, OUTPUT);
  digitalWrite(pins::CAN_SPI_CS, HIGH);
  pinMode(pins::CAN_SPI_INT, INPUT_PULLUP);
  SPI.begin();

  const uint8_t beginResult = canBus.begin(MCP_ANY, CCM_CAN_SPEED, CCM_MCP2515_CLOCK);
  Serial.print(F("CAN INIT: begin result="));
  Serial.print(beginResult);
  Serial.print(F(" bitrate="));
  Serial.print(canSpeedName(CCM_CAN_SPEED));
  Serial.print(F(" clock="));
  Serial.println(canClockName(CCM_MCP2515_CLOCK));

  if (beginResult != CAN_OK) {
    Serial.println(F("CAN: MCP2515 init failed"));
    return false;
  }

  runCanLoopbackSelfTest();

  const uint8_t modeResult = canBus.setMode(MCP_NORMAL);
  if (modeResult != CAN_OK) {
    Serial.println(F("CAN: MCP2515 normal mode failed"));
    return false;
  }
  const uint8_t canstat = mcp2515DebugReadRegister(MCP_CANSTAT);
  const uint8_t canintf = mcp2515DebugReadRegister(MCP_CANINTF);
  const uint8_t stat = mcp2515DebugReadStatus();
  Serial.print(F("CAN INIT: set normal mode result="));
  Serial.print(modeResult);
  Serial.print(F(" mode="));
  Serial.print(canModeName(canstat));
  Serial.print(F(" CANSTAT=0x"));
  printHexByte(canstat);
  Serial.print(F(" INTF=0x"));
  printHexByte(canintf);
  Serial.print(F(" STAT=0x"));
  printHexByte(stat);
  Serial.println(F(" rxFilter=any txId=standard"));
  Serial.print(F("CAN: CS=D"));
  Serial.print(pins::CAN_SPI_CS);
  Serial.print(F(" INT=D"));
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
    Serial.print(F("MAP baro kept at "));
    Serial.print(config.map.baroKpa, 1);
    Serial.println(F(" kPa"));
    return;
  }

  config.map.baroKpa = mapTotalKpa / static_cast<float>(validSamples);
  mapSensor.begin(pins::MAP_SENSOR_ADC, config.map);
  Serial.print(F("MAP baro calibrated to "));
  Serial.print(config.map.baroKpa, 1);
  Serial.println(F(" kPa"));
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
  pressure.invertedOutput = false;
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

  if (!readings.oilPressureValid) {
    readings.analogFaultFlags |= kAnalogFaultOilPressure;
  }
  if (!readings.fuelPressureValid) {
    readings.analogFaultFlags |= kAnalogFaultFuelPressure;
  }
  if (!readings.methPressureValid) {
    readings.analogFaultFlags |= kAnalogFaultMethPressure;
  }
  if (!readings.boostRefPressureValid) {
    readings.analogFaultFlags |= kAnalogFaultBoostRefPressure;
  }
}

#if CCM_CAN_RX_TRACE
void printCanFrame(unsigned long id, byte len, const byte *data) {
  Serial.print(F("CAN RX id=0x"));
  Serial.print(id, HEX);
  Serial.print(F(" len="));
  Serial.print(len);
  Serial.print(F(" data="));
  for (byte i = 0; i < len; ++i) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    if (i + 1 < len) Serial.print(' ');
  }
  Serial.println();
}
#endif

void printCanTxFrame(const can_protocol::CanFrame &frame, uint8_t result) {
  if (!kSerialCanVerbose) return;
  Serial.print(F("CAN TX id=0x"));
  Serial.print(frame.id, HEX);
  Serial.print(F(" len="));
  Serial.print(frame.dlc);
  Serial.print(F(" result="));
  Serial.print(result);
  Serial.print(F(" data="));
  for (byte i = 0; i < frame.dlc; ++i) {
    if (frame.data[i] < 0x10) Serial.print('0');
    Serial.print(frame.data[i], HEX);
    if (i + 1 < frame.dlc) Serial.print(' ');
  }
  Serial.println();
}

void printHexByte(uint8_t value) {
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
}

void printCanErrorFlags(uint8_t eflg) {
  if (eflg == 0) {
    Serial.print(F(" none"));
    return;
  }
  if (eflg & MCP_EFLG_TXBO) Serial.print(F(" TXBO"));
  if (eflg & MCP_EFLG_TXEP) Serial.print(F(" TXEP"));
  if (eflg & MCP_EFLG_RXEP) Serial.print(F(" RXEP"));
  if (eflg & MCP_EFLG_TXWAR) Serial.print(F(" TXWAR"));
  if (eflg & MCP_EFLG_RXWAR) Serial.print(F(" RXWAR"));
  if (eflg & MCP_EFLG_EWARN) Serial.print(F(" EWARN"));
  if (eflg & MCP_EFLG_RX0OVR) Serial.print(F(" RX0OVR"));
  if (eflg & MCP_EFLG_RX1OVR) Serial.print(F(" RX1OVR"));
}

uint8_t sendCanFrame(const can_protocol::CanFrame &frame) {
  return canBus.sendMsgBuf(frame.id, 0, frame.dlc, const_cast<uint8_t *>(frame.data));
}

bool transmitCanFrame(const can_protocol::CanFrame &frame, const char *label) {
  const uint8_t result = sendCanFrame(frame);
  printCanTxFrame(frame, result);
  if (result == CAN_OK) {
    consecutiveCanTxFailures = 0;
    return true;
  }

  canBus.abortTX();
  if (consecutiveCanTxFailures < 255) ++consecutiveCanTxFailures;
  const uint32_t now = millis();
  if (consecutiveCanTxFailures == 1 || elapsed(now, lastCanTxFailureLogMs, kCanTxFailureLogMs)) {
    lastCanTxFailureLogMs = now;
    const uint8_t eflg = canBus.getError();
    const uint8_t tec = canBus.errorCountTX();
    const uint8_t rec = canBus.errorCountRX();
    Serial.print(F("CAN: "));
    Serial.print(label);
    Serial.print(F(" TX failed count="));
    Serial.print(consecutiveCanTxFailures);
    Serial.print(F(" ret="));
    Serial.print(result);
    Serial.print(F(" EFLG=0x"));
    printHexByte(eflg);
    Serial.print(F(" TEC="));
    Serial.print(tec);
    Serial.print(F(" REC="));
    Serial.print(rec);
    Serial.print(F(" flags:"));
    printCanErrorFlags(eflg);
    Serial.println();
  }

  return false;
}

void logCanHealth(uint32_t now) {
  if (!kSerialCanVerbose) return;
  if (!elapsed(now, lastCanHealthLogMs, kCanHealthLogMs)) return;
  lastCanHealthLogMs = now;

  const uint8_t eflg = canBus.getError();
  const uint8_t tec = canBus.errorCountTX();
  const uint8_t rec = canBus.errorCountRX();
  const uint8_t canstat = mcp2515DebugReadRegister(MCP_CANSTAT);
  const uint8_t canintf = mcp2515DebugReadRegister(MCP_CANINTF);
  const uint8_t stat = mcp2515DebugReadStatus();

  Serial.print(F("CAN HEALTH online="));
  Serial.print(canOnline ? 1 : 0);
  Serial.print(F(" mode="));
  Serial.print(canModeName(canstat));
  Serial.print(F(" INT="));
  Serial.print(digitalRead(pins::CAN_SPI_INT));
  Serial.print(F(" CANSTAT=0x"));
  printHexByte(canstat);
  Serial.print(F(" INTF=0x"));
  printHexByte(canintf);
  Serial.print(F(" STAT=0x"));
  printHexByte(stat);
  Serial.print(F(" EFLG=0x"));
  printHexByte(eflg);
  Serial.print(F(" REC="));
  Serial.print(rec);
  Serial.print(F(" TEC="));
  Serial.print(tec);
  Serial.print(F(" flags:"));
  printCanErrorFlags(eflg);
  Serial.println();
}

bool serviceCanDiag(uint32_t now) {
  if (!canOnline || !elapsed(now, lastCanDiagMs, kCanDiagMs)) return canOnline;
  lastCanDiagMs = now;

  const uint8_t eflg = canBus.getError();
  const uint8_t rec = canBus.errorCountRX();
  const uint8_t tec = canBus.errorCountTX();
  Serial.print(F("[CAN] EFLG=0x"));
  printHexByte(eflg);
  Serial.print(F(" REC="));
  Serial.print(rec);
  Serial.print(F(" TEC="));
  Serial.print(tec);
  Serial.print(F(" INT="));
  Serial.print(digitalRead(pins::CAN_SPI_INT));
  Serial.print(F(" flags:"));
  printCanErrorFlags(eflg);
  Serial.println();

  const bool busOff = (eflg & MCP_EFLG_TXBO) != 0U;
  const bool txErrorState = (eflg & (MCP_EFLG_TXEP | MCP_EFLG_TXWAR | MCP_EFLG_EWARN)) != 0U;
  const bool txPathStuck = consecutiveCanTxFailures >= 32U &&
                           (txErrorState || tec > 0U);
  const bool persistentTxFailure = consecutiveCanTxFailures >= 64U;
  if (!busOff && !txPathStuck && !persistentTxFailure) return true;

  Serial.println(busOff ? F("[CAN] TXBO detected; resetting MCP2515")
                        : F("[CAN] sustained TX failure; resetting MCP2515"));
  canOnline = false;
  // MCP_CAN::begin() performs the controller reset; the lower-level reset
  // helper is private in the pinned MCP_CAN library.
  canOnline = initCanBus();
  lastCanRetryMs = now;
  if (canOnline) consecutiveCanTxFailures = 0;
  Serial.println(canOnline ? F("[CAN] controller recovery OK")
                           : F("[CAN] controller recovery FAILED"));
  return canOnline;
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
      static_cast<uint8_t>(can_protocol::CAN_PROTOCOL_SCHEMA_VERSION)), "0x30A");
}

bool sendMethConfigAck(uint8_t version, uint8_t status, uint8_t rejectReason) {
  return transmitCanFrame(can_protocol::packMethConfigAck(
      version, status, rejectReason,
      can_protocol::clampU8(static_cast<int>(blend.methPercent + 0.5f))), "0x306-meth");
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

  const bool page1Sent = transmitCanFrame(can_protocol::packKnockConfigPage1(page1), "0x30C");
  const bool page2Sent = transmitCanFrame(can_protocol::packKnockConfigPage2(page2), "0x30D");
  return page1Sent && page2Sent;
}

// Boost thresholds and the relay period live in the tune; the controller reads copies.
void syncTuneToConfig() {
  config.boost.startPsi = tune.value[meth_tune::kStartPsiX10] / 10.0f;
  config.boost.fullPsi = tune.value[meth_tune::kFullPsiX10] / 10.0f;
  config.relayPeriodMs = tune.value[meth_tune::kPeriodMs];
  // The tank blend is reported in the config acknowledgement and checked by the controller.
  const float meth = static_cast<float>(tune.value[meth_tune::kMethPercent]);
  blend = computeTankBlend(100.0f - meth, meth);
}

void tuneChanged() {
  ++tuneRevision;
  tuneUnsaved = true;
  syncTuneToConfig();
}

// EEPROM: magic, version, the settings as little-endian u16, XOR checksum.
bool loadTune() {
  meth_tune::Settings saved;
  saved.value[0] = 0;
  int address = kTuneEepromAddress;
  uint8_t checksum = EEPROM.read(address) ^ EEPROM.read(address + 1);
  const bool header = EEPROM.read(address) == kTuneEepromMagic && EEPROM.read(address + 1) == kTuneEepromVersion;
  address += 2;
  for (uint8_t key = 1; key <= meth_tune::kCount; ++key, address += 2) {
    const uint8_t low = EEPROM.read(address);
    const uint8_t high = EEPROM.read(address + 1);
    checksum ^= low ^ high;
    saved.value[key] = static_cast<uint16_t>(low | (static_cast<uint16_t>(high) << 8));
  }
  if (!header || checksum != EEPROM.read(address) || !meth_tune::valid(saved)) return false;
  tune = saved;
  return true;
}

void saveTune() {
  int address = kTuneEepromAddress;
  uint8_t checksum = kTuneEepromMagic ^ kTuneEepromVersion;
  EEPROM.update(address, kTuneEepromMagic);
  EEPROM.update(address + 1, kTuneEepromVersion);
  address += 2;
  for (uint8_t key = 1; key <= meth_tune::kCount; ++key, address += 2) {
    const uint8_t low = static_cast<uint8_t>(tune.value[key] & 0xFF);
    const uint8_t high = static_cast<uint8_t>(tune.value[key] >> 8);
    checksum ^= low ^ high;
    EEPROM.update(address, low);
    EEPROM.update(address + 1, high);
  }
  EEPROM.update(address, checksum);
}

// The older boost-start commands set the same value the tune holds.
// Repeated frames with the same value must not count as a settings change.
void setStartPsiFromLegacy(float startPsi) {
  meth_tune::Settings next = tune;
  uint16_t applied = 0;
  const float x10 = startPsi * 10.0f + 0.5f;
  meth_tune::set(next, meth_tune::kStartPsiX10, x10 < 0.0f ? 0 : static_cast<uint16_t>(x10), applied);
  if (applied == tune.value[meth_tune::kStartPsiX10]) return;
  tune = next;
  tuneChanged();
}

void handleTuneCommand(byte len, const byte *data) {
  namespace ack = can_protocol::config_ack_status;
  uint8_t status = ack::OK;
  uint8_t subject = len >= 2 ? data[1] : 0;
  uint16_t value = 0;
  if (data[0] == can_protocol::meth_tune_command::SET_SETTING) {
    if (len != 4) {
      status = ack::INVALID_LENGTH;
    } else {
      status = meth_tune::set(tune, data[1], static_cast<uint16_t>((data[2] << 8) | data[3]), value);
      if (status != ack::UNSUPPORTED_COMMAND) tuneChanged();
    }
  } else if (len != 2) {
    status = ack::INVALID_LENGTH;
  } else if (data[1] == can_protocol::meth_tune_action::SAVE) {
    // Writing EEPROM blocks for tens of ms; loop() does it between pulses. The status
    // keeps reporting UNSAVED until it has really been written.
    tuneSavePending = true;
  } else if (data[1] == can_protocol::meth_tune_action::REVERT) {
    if (!loadTune()) meth_tune::loadDefaults(tune);
    tuneChanged();
    tuneUnsaved = false;
    tuneSavePending = false;
  } else if (data[1] == can_protocol::meth_tune_action::DEFAULTS) {
    meth_tune::loadDefaults(tune);
    tuneChanged();
  } else if (data[1] == can_protocol::meth_tune_action::REPORT) {
    tuneReportNext = 1;
  } else {
    status = ack::UNSUPPORTED_COMMAND;
  }
  if (!transmitCanFrame(can_protocol::packMethTuneAck(data[0], status, subject, value, tuneRevision), "0x30F-ack")) {
    canOnline = false;
  }
}

void updateIntakeTemps(uint32_t now) {
  if (!elapsed(now, lastIntakeTempMs, kIntakeTempSampleMs)) return;
  lastIntakeTempMs = now;
  float sample = 0.0f;
  // Light smoothing only: these sensors are there to show each pulse cooling the air.
  if (meth_tune::thermistorC(static_cast<uint16_t>(analogRead(pins::PRE_INJECTION_TEMP_ADC)),
                             pins::INTAKE_TEMP_PULLUP_OHMS, sample)) {
    preTempC = preTempValid ? preTempC + (sample - preTempC) * 0.3f : sample;
    preTempValid = true;
  } else {
    preTempValid = false;
  }
  if (meth_tune::thermistorC(static_cast<uint16_t>(analogRead(pins::POST_INJECTION_TEMP_ADC)),
                             pins::INTAKE_TEMP_PULLUP_OHMS, sample)) {
    postTempC = postTempValid ? postTempC + (sample - postTempC) * 0.3f : sample;
    postTempValid = true;
  } else {
    postTempValid = false;
  }
}

// Why the controller is not asking for injection, for the dash's status line.
uint8_t idleHoldFor(const ControlResult &result) {
  if (config.mode == InjectionMode::Off) return meth_tune::kHoldDisarmed;
  if (result.failsafe == FailsafeReason::LowFluid) return meth_tune::kHoldTankLow;
  if (result.failsafe != FailsafeReason::None) return meth_tune::kHoldFault;
  return meth_tune::kHoldBelowBoost;
}

void applyCanCommandTimeout(uint32_t now) {
  if (!canCommandSeen || !elapsed(now, lastCanCommandMs, kCanCommandTimeoutMs)) return;

  // Timeout only cancels remote manual test. Keep normal injection mode intact
  // so periodic/non-periodic CAN traffic cannot force output oscillations.
  if (!manualTestActive) return;

  if (!canCommandTimedOut) {
    Serial.println(F("CAN: command timeout, stopping manual test"));
  }
  canCommandTimedOut = true;
  manualTestActive = false;
  manualTestDuty = 0;
}

void markMasterActivity(uint32_t now) {
  canCommandSeen = true;
  canCommandTimedOut = false;
  lastCanCommandMs = now;
}

void handleMethConfigBroadcast(byte len, const byte *data) {
  if (len < 8) return;

  uint8_t checksum = 0;
  for (byte i = 0; i < 7; ++i) checksum ^= data[i];
  if (checksum != data[7]) {
    sendMethConfigAck(data[0], 1, 1);
    return;
  }

  config.mode = data[1] != 0U ? InjectionMode::BoostOnly : InjectionMode::Off;
  config.tankProtectionEnabled = (data[6] & 0x01U) != 0U;
  // A disarmed periodic config must not cancel a deliberately started test.
  // Arming cancels the test; explicit DISARM/STOP commands still stop it.
  if (config.mode != InjectionMode::Off) {
    manualTestActive = false;
    manualTestDuty = 0;
  }
  if (data[2] <= 100U && data[2] != tune.value[meth_tune::kMethPercent]) {
    tune.value[meth_tune::kMethPercent] = data[2];
    tuneChanged();
  }
  if (data[3] > 0U) {
    const float startPsi = (static_cast<float>(data[3]) - config.map.baroKpa) * kPsiPerKpa;
    setStartPsiFromLegacy(startPsi);
  }
  if (data[5] > 0U && data[5] <= 100U) {
    config.dutyMaxPercent = static_cast<float>(data[5]);
  }
  sendMethConfigAck(data[0], 0, 0);
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
  const uint32_t now = millis();
  if (id == can_protocol::ID_MASTER_HEARTBEAT) {
    markMasterActivity(now);
    return;
  }

  if (id == can_protocol::ID_METH_CONFIG_BROADCAST) {
    markMasterActivity(now);
    handleMethConfigBroadcast(len, data);
    return;
  }

  if (id == can_protocol::ID_ENGINE_RUNTIME) {
    markMasterActivity(now);
    if (kSerialCanVerbose) {
      Serial.println(F("CAN RX: engine runtime 0x309"));
    }
    handleEngineRuntimeFrame(len, data);
    return;
  }

  if (id == can_protocol::ID_METH_CONFIG_REQUEST) {
    if (kSerialCanVerbose) {
      Serial.println(F("CAN RX: config request 0x305, sending 0x30C/0x30D"));
    }
    if (!sendKnockConfigPages()) {
      Serial.println(F("CAN: config request TX failed, will retry init"));
      canOnline = false;
    }
    return;
  }

  if (id != can_protocol::ID_ENGINE_METH_COMMAND) {
    return;
  }
  markMasterActivity(now);

  if (len < 1) {
    if (kSerialCanVerbose) {
      Serial.println(F("CAN RX: command 0x301 invalid length"));
    }
    sendConfigAck(0, can_protocol::config_ack_status::INVALID_LENGTH, 0);
    return;
  }

  if (kSerialCanVerbose) {
    Serial.print(F("CAN RX: command 0x301 cmd=0x"));
    if (data[0] < 0x10) Serial.print('0');
    Serial.println(data[0], HEX);
  }

  if (data[0] == can_protocol::meth_tune_command::LIVE_INTAKE_TEMP) {
    // Live data, sent several times a second: taken silently, no acknowledgement.
    if (len == 3) {
      ecuIntakeC = static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[2]) / 10.0f;
      ecuIntakeSeenMs = now;
      ecuIntakeSeen = true;
    }
    return;
  }

  if (data[0] == can_protocol::meth_tune_command::SET_SETTING ||
      data[0] == can_protocol::meth_tune_command::SETTINGS_ACTION) {
    handleTuneCommand(len, data);
    return;
  }

  uint8_t ackStatus = can_protocol::config_ack_status::OK;
  uint8_t ackValue = len >= 2 ? data[1] : 0;
  bool commandAccepted = true;

  switch (data[0]) {
  case can_protocol::meth_command::ARM:
    if (len >= 2) {
      config.mode = data[1] ? InjectionMode::BoostOnly : InjectionMode::Off;
      manualTestActive = false;
      manualTestDuty = 0;
      if (kSerialCanVerbose) {
        Serial.println(data[1] ? F("CAN CMD: meth armed") : F("CAN CMD: meth disarmed"));
      }
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::MANUAL_TEST_DUTY:
    if (len >= 2) {
      // Repeated requests must not extend the five-second test window.
      if (manualTestActive || config.mode != InjectionMode::Off ||
          (config.tankProtectionEnabled && floatSensor.isLow())) {
        ackStatus = can_protocol::config_ack_status::VALUE_CLAMPED;
        ackValue = 0;
        break;
      }
      manualTestActive = true;
      manualTestDuty = constrainAndReport(data[1], 0, 100, ackStatus);
      manualTestStartMs = now;
      ackValue = manualTestDuty;
      if (kSerialCanVerbose) {
        Serial.print(F("CAN CMD: manual duty="));
        Serial.println(manualTestDuty);
      }
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::STOP_MANUAL_TEST:
    manualTestActive = false;
    manualTestDuty = 0;
    if (kSerialCanVerbose) {
      Serial.println(F("CAN CMD: manual stop"));
    }
    break;
  case can_protocol::meth_command::SET_BOOST_TRIGGER:
    if (len >= 2) {
      const uint8_t boostKpa = constrainAndReport(data[1], 0, 250, ackStatus);
      setStartPsiFromLegacy(static_cast<float>(boostKpa) * kPsiPerKpa);
      ackValue = boostKpa;
      if (kSerialCanVerbose) {
        Serial.print(F("CAN CMD: boost trigger kPa="));
        Serial.println(boostKpa);
      }
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::CLEAR_FAULTS:
    controller.clearLatchedFaults();
    canCommandTimedOut = false;
    if (kSerialCanVerbose) {
      Serial.println(F("CAN CMD: clear faults"));
    }
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
    if (kSerialCanVerbose) {
      Serial.print(F("CAN CMD: unsupported 0x"));
      Serial.println(data[0], HEX);
    }
    ackStatus = can_protocol::config_ack_status::UNSUPPORTED_COMMAND;
    commandAccepted = false;
    break;
  }

  if (!sendConfigAck(data[0], ackStatus, ackValue)) {
    Serial.println(F("CAN: config ACK TX failed, will retry init"));
    canOnline = false;
  } else if (commandAccepted &&
             (ackStatus == can_protocol::config_ack_status::OK ||
              ackStatus == can_protocol::config_ack_status::VALUE_CLAMPED) &&
             !sendKnockConfigPages()) {
    Serial.println(F("CAN: config page TX failed, will retry init"));
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
  if (config.mode != InjectionMode::Off && (!config.tankProtectionEnabled || !readings.tankLow)) {
    return static_cast<uint8_t>(can_protocol::MethState::ARMED);
  }
  return static_cast<uint8_t>(can_protocol::MethState::OFF);
}

uint8_t methFaultFlagsFor(const ControlResult &result, const SensorReadings &readings) {
  uint8_t flags = 0;
  if ((config.tankProtectionEnabled && readings.tankLow) || result.failsafe == FailsafeReason::LowFluid) flags |= (1U << 0);
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

uint8_t pressurePsiX2ForCan(float pressurePsi) {
  return can_protocol::clampU8(static_cast<int>((pressurePsi * 2.0f) + 0.5f));
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

  if (!serviceCanDiag(now)) return;
  logCanHealth(now);
  applyCanCommandTimeout(now);

  const bool interruptActive = digitalRead(pins::CAN_SPI_INT) == LOW;
  const bool fallbackPollDue = elapsed(now, lastCanRxPollMs, kCanRxFallbackPollMs);
  byte framesRead = 0;
  if (interruptActive || fallbackPollDue) lastCanRxPollMs = now;
  // Poll the controller periodically even when INT is stuck high or its wire
  // is open. checkReceive() also avoids treating an error-only interrupt as a
  // failed frame read.
  while ((interruptActive || fallbackPollDue) && framesRead < 8 &&
         canBus.checkReceive() == CAN_MSGAVAIL) {
    unsigned long rxId = 0;
    byte len = 0;
    byte data[8] = {};

    if (canBus.readMsgBuf(&rxId, &len, data) == CAN_OK) {
#if CCM_CAN_RX_TRACE
      printCanFrame(rxId, len, data);
#endif
      handleCanFrame(rxId, len, data);
    } else {
      Serial.println(F("CAN: read failed"));
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
    ext.oil_pressure_psi_x2 = pressurePsiX2ForCan(readings.oilPressurePsi);
    ext.fuel_pressure_psi_x2 = pressurePsiX2ForCan(readings.fuelPressurePsi);
    ext.meth_pressure_psi_x2 = pressurePsiX2ForCan(readings.methPressurePsi);
    ext.boost_ref_pressure_psi_x2 = pressurePsiX2ForCan(readings.boostRefPressurePsi);
    ext.ambient_temp_c = static_cast<int8_t>(readings.ambientValid ? readings.ambientC : 0.0f);
    ext.cabin_temp_c = static_cast<int8_t>(readings.cabinValid ? readings.cabinC : 0.0f);
    ext.analog_fault_flags = readings.analogFaultFlags;
    if (!readings.iatValid) ext.analog_fault_flags |= can_protocol::analog_sensor_fault::IAT;
    if (!readings.engineBayValid) ext.analog_fault_flags |= can_protocol::analog_sensor_fault::ENGINE_BAY;
    if (!readings.ambientValid) ext.analog_fault_flags |= can_protocol::analog_sensor_fault::AMBIENT;
    if (!readings.cabinValid) ext.analog_fault_flags |= can_protocol::analog_sensor_fault::CABIN;

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

  if (elapsed(now, lastTuneStatusTxMs, can_protocol::METH_TUNE_STATUS_TX_MS)) {
    lastTuneStatusTxMs = now;
    namespace flag = can_protocol::meth_tune_flag;
    uint8_t flags = 0;
    if (tuneUnsaved) flags |= flag::UNSAVED;
    if (engineRpmValid && engineRpm >= tune.value[meth_tune::kMinRpm]) flags |= flag::RPM_OK;
    if (preTempValid) flags |= flag::PRE_VALID;
    if (postTempValid) flags |= flag::POST_VALID;
    if (digitalRead(pins::PUMP_OUT) == HIGH) flags |= flag::PUMP_ON;
    if (governor.earlyStart()) flags |= flag::EARLY_START;
    if (governor.hotAir()) flags |= flag::HOT_AIR;
    if (usingEcuTemp) flags |= flag::ECU_TEMP;
    if (!transmitCanFrame(can_protocol::packMethTuneStatus(tuneRevision, flags, governor.hold(), governor.onMs(),
                                                           tune.value[meth_tune::kPeriodMs]), "0x30F-status")) {
      return;
    }
  }

  if (elapsed(now, lastTuneTempsTxMs, can_protocol::METH_TUNE_TEMPS_TX_MS)) {
    lastTuneTempsTxMs = now;
    const uint8_t validBits = (preTempValid ? 1U : 0U) | (postTempValid ? 2U : 0U);
    if (!transmitCanFrame(can_protocol::packMethTuneTemps(
            static_cast<int16_t>(preTempValid ? preTempC * 10.0f : 0.0f),
            static_cast<int16_t>(postTempValid ? postTempC * 10.0f : 0.0f), validBits), "0x30F-temps")) {
      return;
    }
  }

  // One setting per pass, so a full report never delays the pump timing.
  if (tuneReportNext != 0) {
    const uint8_t key = tuneReportNext;
    if (transmitCanFrame(can_protocol::packMethTuneSettingReport(
            key, tune.value[key], tuneRevision,
            tuneUnsaved ? can_protocol::meth_tune_flag::UNSAVED : 0), "0x30F-setting")) {
      tuneReportNext = key < meth_tune::kCount ? key + 1 : 0;
    }
  }
}
} // namespace

void setup() {
  Serial.begin(config.serialBaud);
  delay(250);
  // Fail safe until the comfort module explicitly arms this controller.
  config.mode = InjectionMode::Off;
  Serial.println();
  Serial.println(F("BOOT: Nano water-meth/knock firmware"));
  Serial.print(F("BOOT: serial baud="));
  Serial.println(config.serialBaud);

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
  // Saved pulse tuning, or the conservative defaults on a new or corrupt EEPROM.
  if (!loadTune()) meth_tune::loadDefaults(tune);
  syncTuneToConfig();
  pinMode(pins::PRE_INJECTION_TEMP_ADC, INPUT);
  pinMode(pins::POST_INJECTION_TEMP_ADC, INPUT);
  pumpDriver.beginRelay(pins::PUMP_OUT, config.relayPeriodMs);
  warningOutput.begin(pins::WARNING_LED, true);
  pinMode(pins::KNOCK_SENSOR_ADC, INPUT);

  Serial.println(F("Nano water/meth CAN firmware"));
  Serial.print(F("Knock pin: "));
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
  updateIntakeTemps(now);
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
  if (manualTestActive && (elapsed(now, manualTestStartMs, kManualTestTimeoutMs) ||
                           (config.tankProtectionEnabled && readings.tankLow) || !readings.mapValid ||
                           config.mode != InjectionMode::Off)) {
    manualTestActive = false;
    manualTestDuty = 0;
  }
  ControlResult result = controller.update(readings, config, blend);

  if (manualTestActive && !manualPumpTestSafe(readings, config, result)) {
    manualTestActive = false;
    manualTestDuty = 0;
  }

  // The controller decides whether injection is wanted; the governor turns that into
  // relay pulses within the tuned limits (RPM, air temperature, spray time, ramp).
  bool pumpOn = false;
  if (manualTestActive) {
    result.pump.enabled = manualTestDuty > 0;
    result.pump.dutyPercent = manualTestDuty;
    result.finalDutyPercent = manualTestDuty;
    pumpOn = governor.manual(now, tune, manualTestDuty);
  } else {
    meth_tune::Inputs inputs;
    inputs.wantSpray = result.pump.enabled;
    inputs.idleHold = idleHoldFor(result);
    inputs.boostPsi = readings.boostPsi;
    inputs.mapKpa = readings.mapKpa;
    inputs.rpmValid = engineRpmValid;
    inputs.rpm = rpm;
    // The controller's own sensor before the nozzle when it is fitted; otherwise the
    // ECU's intake temperature while the dash keeps sending it.
    const bool ecuTempFresh = ecuIntakeSeen && !elapsed(now, ecuIntakeSeenMs, kEcuIntakeTimeoutMs);
    usingEcuTemp = !preTempValid && ecuTempFresh;
    inputs.preValid = preTempValid || ecuTempFresh;
    inputs.preC = preTempValid ? preTempC : ecuIntakeC;
    inputs.assistActive = result.overboostAssistActive;
    inputs.assistDutyPercent = result.finalDutyPercent;
    inputs.capDutyPercent = config.dutyMaxPercent;
    pumpOn = governor.update(now, tune, inputs);
    // Telemetry shows what the pump is really doing, not the old duty model.
    result.pump.enabled = governor.onMs() > 0;
    result.finalDutyPercent = governor.onMs() * 100.0f / tune.value[meth_tune::kPeriodMs];
    result.pump.dutyPercent = result.finalDutyPercent;
  }

  pumpDriver.setRelay(pumpOn);
  if (pumpOn != lastPumpOn) {
    // Send the status frame on this pass, not up to half a second later.
    lastPumpOn = pumpOn;
    lastTuneStatusTxMs = now - can_protocol::METH_TUNE_STATUS_TX_MS;
  }
  // A pulse ends on a loop pass, so anything slow would stretch it. Defer the EEPROM
  // write and the long serial line while a timed pulse is running.
  const bool pulseRunning = pumpOn && governor.onMs() < tune.value[meth_tune::kPeriodMs];
  if (tuneSavePending && !pulseRunning) {
    saveTune();
    tuneSavePending = false;
    tuneUnsaved = false;
  }
  warningOutput.set(result.failsafe != FailsafeReason::None ||
                    result.overboostAssistFaultLatched ||
                    knockDetected);

  serviceCanBus(now, readings, result, knockDetected, knockRaw);

  if (!pulseRunning && elapsed(now, lastDebugMs, kSerialHeartbeatMs)) {
    lastDebugMs = now;
    const uint8_t commandedDuty = manualTestActive ? manualTestDuty :
        can_protocol::clampU8(static_cast<int>(result.finalDutyPercent + 0.5f));
    Serial.print(F("HB t="));
    Serial.print(now);
    Serial.print(F(" map="));
    Serial.print(readings.mapKpa, 1);
    Serial.print(F(" boostPsi="));
    Serial.print(readings.boostPsi, 2);
    Serial.print(F("psi canBoost="));
    Serial.print(boostGaugeKpaForCan(readings));
    Serial.print(F("kPa baro="));
    Serial.print(config.map.baroKpa, 1);
    Serial.print(F(" raw="));
    Serial.print(readings.mapRaw);
    Serial.print(F(" duty="));
    Serial.print(commandedDuty);
    Serial.print(F("% spray="));
    Serial.print((result.pump.enabled && result.finalDutyPercent > 0.0f) ? 1 : 0);
    Serial.print(F(" pumpOut="));
    Serial.print(digitalRead(pins::PUMP_OUT));
    Serial.print(F(" mode="));
    Serial.print(static_cast<uint8_t>(config.mode));
    Serial.print(F(" startPsi="));
    Serial.print(config.boost.startPsi, 2);
    Serial.print(F(" canTO="));
    Serial.print(canCommandTimedOut ? 1 : 0);
    Serial.print(F("% tankLow="));
    Serial.print(readings.tankLow ? 1 : 0);
    Serial.print(F(" floatD3="));
    Serial.print(digitalRead(pins::FLOAT_SENSOR_DIGITAL));
    Serial.print(F(" lowWhen="));
    Serial.print(config.floatActiveLow ? F("LOW") : F("HIGH"));
    Serial.print(F(" can="));
    Serial.print(canOnline ? 1 : 0);
    Serial.print(F(" rpm="));
    Serial.print(rpm);
    Serial.print(engineRpmValid ? "" : "!");
    Serial.print(F(" knock="));
    Serial.print(knockDetected ? 1 : 0);
    Serial.print(F(" oilAdcV="));
    Serial.print(oilPressureSensor.adcNodeVoltage(), 3);
    Serial.print(F(" oilV="));
    Serial.print(oilPressureSensor.sensorVoltage(), 3);
    Serial.print(F(" oilPsi="));
    Serial.print(readings.oilPressurePsi, 1);
    Serial.print(F(" oilOk="));
    Serial.print(readings.oilPressureValid ? 1 : 0);
    Serial.print(F(" oilFault="));
    Serial.print(pressureFaultName(oilPressureSensor.fault()));
    Serial.print('(');
    Serial.print(static_cast<uint8_t>(oilPressureSensor.fault()));
    Serial.print(')');
    Serial.print(F(" fuelAdcV="));
    Serial.print(fuelPressureSensor.adcNodeVoltage(), 3);
    Serial.print(F(" fuelV="));
    Serial.print(fuelPressureSensor.sensorVoltage(), 3);
    Serial.print(F(" fuelPsi="));
    Serial.print(readings.fuelPressurePsi, 1);
    Serial.print(F(" fuelOk="));
    Serial.print(readings.fuelPressureValid ? 1 : 0);
    Serial.print(F(" fuelFault="));
    Serial.print(pressureFaultName(fuelPressureSensor.fault()));
    Serial.print('(');
    Serial.print(static_cast<uint8_t>(fuelPressureSensor.fault()));
    Serial.print(')');
    Serial.print(F(" faults=0x"));
    Serial.println(readings.analogFaultFlags, HEX);
  }
}
