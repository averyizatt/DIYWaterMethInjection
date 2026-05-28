#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include <mcp_can.h>
#include <stdlib.h>

#include "actuators.h"
#include "app_config.h"
#include "injection_controller.h"
#include "knock_monitor.h"
#include "knock_ui.h"
#include "pins.h"
#include "sensors.h"
#include "can_contract/can_protocol.h"

namespace {
AppConfig config = defaultConfig();
TankBlend blend = computeTankBlend(1.5f, 0.5f);

MapSensor mapSensor;
FloatSensor floatSensor;
ThermistorSensor iatSensor;
ThermistorSensor engineBaySensor;
ThermistorSensor cabinSensor;
ThermistorSensor ambientSensor;
PressureSensor oilPressure;
PressureSensor fuelPressure;
PressureSensor methPressure;
PressureSensor boostRefPressure;
PressureSensor sparePressure1;
PressureSensor sparePressure2;
PumpDriver pumpDriver;
WarningOutput warningOutput;
InjectionController controller;
KnockMonitor knockMonitor;
Preferences preferences;

MCP_CAN canBus(pins::CAN_SPI_CS);
bool canOnline = false;
bool canRecoveryEnabled = true;

uint32_t lastLoopMs = 0;
uint32_t lastDebugMs = 0;
uint32_t lastStateTxMs = 0;
uint32_t lastExtTxMs = 0;
uint32_t lastKnockTxMs = 0;
uint32_t lastMasterRxMs = 0;
FailsafeReason lastFailsafe = FailsafeReason::None;
FailsafeReason lastReportedCanFault = FailsafeReason::None;
String serialLine;

bool armed = false;
bool pendingManualTest = false;
uint8_t manualTestDuty = 0;
bool benchButtonActive = false;       // true while physical button is held
bool benchButtonOwnsTest = false;     // true when the button (not CAN) started pendingManualTest
uint32_t benchButtonDebounceMs = 0;   // last edge time
uint8_t remoteRatioPercent = 50;
bool hasRemoteRatio = false;
bool clearFaultsRequested = false;
uint8_t lastConfigVersion = 0;
uint8_t lastConfigRatioPercent = 255;
uint32_t lastCanRecoveryAttemptMs = 0;
uint16_t canTxFailStreak = 0;
bool analogSafetyLatched = false;

constexpr char kPrefsNamespace[] = "wmix";
constexpr char kPrefsKeyWater[] = "water_l";
constexpr char kPrefsKeyMeth[] = "meth_l";
constexpr uint8_t kFaultSeverityWarning = 1;

constexpr uint32_t kStateTxIntervalMs = 50;
constexpr uint32_t kExtTxIntervalMs = 250;
constexpr uint32_t kKnockTxIntervalMs = 50;
constexpr uint32_t kKnockApiTxIntervalMs = 100;
constexpr uint32_t kKnockCfgTxIntervalMs = 500;
constexpr uint32_t kCanCommandTimeoutMs = 2000;
constexpr uint32_t kCanRecoveryIntervalMs = 1000;
constexpr uint16_t kCanTxFailStreakBeforeOffline = 6;

constexpr uint16_t kCanIdKnockUiState = 0x30B;
constexpr uint16_t kCanIdKnockUiConfig1 = 0x30C;
constexpr uint16_t kCanIdKnockUiConfig2 = 0x30D;
constexpr uint16_t kCanIdKnockUiBands = 0x30E;
constexpr uint16_t kCanIdKnockUiDsp = 0x30F;
constexpr uint16_t kCanIdKnockUiExplain = 0x310;

constexpr uint8_t kCmdKnockSetEnable = 0x40;
constexpr uint8_t kCmdKnockSetThresholdOffset = 0x41;
constexpr uint8_t kCmdKnockSetMultiplierX10 = 0x42;
constexpr uint8_t kCmdKnockSetMinRpmDiv100 = 0x43;
constexpr uint8_t kCmdKnockSetMinMapKpa = 0x44;
constexpr uint8_t kCmdKnockSetDebounceDiv10 = 0x45;
constexpr uint8_t kCmdKnockSetGainX10 = 0x46;
constexpr uint8_t kCmdKnockSetCenterHzDiv100 = 0x47;
constexpr uint8_t kCmdKnockSetBandwidthHzDiv100 = 0x48;
constexpr uint8_t kCmdKnockSetAutoFreq = 0x49;
constexpr uint8_t kCmdKnockResetEvents = 0x4A;
constexpr uint8_t kCmdKnockSetMinLoadPct = 0x4B;
constexpr uint8_t kCmdKnockSetSpreadX100 = 0x4C;
constexpr uint8_t kCmdKnockSetFftEnable = 0x4D;
constexpr uint8_t kCmdKnockSetFftSnrDb = 0x4E;
constexpr uint8_t kCmdKnockSetFftShortWeightX100 = 0x4F;
constexpr uint8_t kCmdKnockSetFftHarmonicWeightX100 = 0x50;
constexpr uint8_t kCmdKnockSetTemplateWeightX100 = 0x51;
constexpr uint8_t kCmdKnockSetMapRateGateKpaPerSec = 0x52;
constexpr uint8_t kCmdKnockSetTransientScaleX100 = 0x53;
constexpr uint8_t kCmdKnockSetTransientHoldDiv10 = 0x54;
constexpr uint8_t kCmdKnockSetDetectConfidenceX100 = 0x55;
constexpr uint8_t kCmdKnockSetWarnConfidenceX100 = 0x56;
constexpr uint8_t kCmdKnockSetCriticalConfidenceX100 = 0x57;
constexpr uint8_t kCmdKnockSetIatCompStartC = 0x58;
constexpr uint8_t kCmdKnockSetIatCompPerCx1000 = 0x59;
constexpr uint8_t kCmdKnockSetBayCompStartC = 0x5A;
constexpr uint8_t kCmdKnockSetBayCompPerCx1000 = 0x5B;

constexpr uint16_t kFaultIat = 1U << 0;
constexpr uint16_t kFaultEngineBay = 1U << 1;
constexpr uint16_t kFaultCabin = 1U << 2;
constexpr uint16_t kFaultAmbient = 1U << 3;
constexpr uint16_t kFaultOil = 1U << 4;
constexpr uint16_t kFaultFuel = 1U << 5;
constexpr uint16_t kFaultMeth = 1U << 6;
constexpr uint16_t kFaultBoostRef = 1U << 7;
constexpr uint16_t kFaultSpare1 = 1U << 8;
constexpr uint16_t kFaultSpare2 = 1U << 9;

constexpr uint8_t kKnockStatusOnline = 1U << 0;
constexpr uint8_t kKnockStatusSignalValid = 1U << 1;
constexpr uint8_t kKnockStatusWarning = 1U << 2;
constexpr uint8_t kKnockStatusCritical = 1U << 3;
constexpr uint8_t kKnockStatusSensorFault = 1U << 4;
constexpr uint8_t kKnockStatusClipping = 1U << 5;
constexpr uint8_t kKnockStatusBaselineLearned = 1U << 6;

constexpr uint8_t kMethStateOff = 0;
constexpr uint8_t kMethStateArmed = 1;
constexpr uint8_t kMethStateSpraying = 2;
constexpr uint8_t kMethStateFault = 3;
constexpr uint8_t kMethStateTest = 4;

constexpr uint8_t kFlowUnknown = 0;
constexpr uint8_t kFlowOk = 1;
constexpr uint8_t kFlowLow = 2;
constexpr uint8_t kFlowNone = 3;

inline bool elapsed(uint32_t now, uint32_t start, uint32_t intervalMs) {
  return static_cast<uint32_t>(now - start) >= intervalMs;
}

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

uint8_t toU8(float value, float scale = 1.0f) {
  const int scaled = static_cast<int>(value * scale);
  return can_protocol::clampU8(scaled);
}

bool initCanBus() {
  if (canBus.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    canOnline = false;
    return false;
  }
  canBus.setMode(MCP_NORMAL);
  canOnline = true;
  canRecoveryEnabled = true;
  canTxFailStreak = 0;
  return true;
}

void sanitizeKnockConfig(KnockConfig &knock) {
  knock.minRpmToArm = constrain(knock.minRpmToArm, static_cast<uint16_t>(0), static_cast<uint16_t>(12000));
  knock.minMapKpaToArm = clampFloat(knock.minMapKpaToArm, 20.0f, 320.0f);
  knock.minLoadPercentToArm = clampFloat(knock.minLoadPercentToArm, 0.0f, 100.0f);
  knock.sampleRateHz = constrain(knock.sampleRateHz, static_cast<uint16_t>(2000), static_cast<uint16_t>(20000));
  knock.samplesPerUpdate = constrain(knock.samplesPerUpdate, static_cast<uint8_t>(16), static_cast<uint8_t>(128));
  knock.boreMm = clampFloat(knock.boreMm, 60.0f, 120.0f);
  knock.centerFreqHz = clampFloat(knock.centerFreqHz, 1000.0f, 9000.0f);
  knock.bandwidthHz = clampFloat(knock.bandwidthHz, 300.0f, 5000.0f);
  knock.multiBandSpread = clampFloat(knock.multiBandSpread, 0.05f, 0.35f);
  knock.fftMinSnrDb = clampFloat(knock.fftMinSnrDb, -6.0f, 20.0f);
  knock.fftWeight = clampFloat(knock.fftWeight, 0.0f, 1.0f);
  knock.fftShortWeight = clampFloat(knock.fftShortWeight, 0.0f, 1.0f);
  knock.fftHarmonicWeight = clampFloat(knock.fftHarmonicWeight, 0.0f, 1.0f);
  knock.spectralTemplateWeight = clampFloat(knock.spectralTemplateWeight, 0.0f, 1.0f);
  knock.mapRateGateKpaPerSec = clampFloat(knock.mapRateGateKpaPerSec, 20.0f, 400.0f);
  knock.transientThresholdScale = clampFloat(knock.transientThresholdScale, 1.0f, 2.5f);
  knock.transientHoldMs = constrain(knock.transientHoldMs, static_cast<uint16_t>(10),
                                    static_cast<uint16_t>(1000));
  knock.minDetectConfidence = clampFloat(knock.minDetectConfidence, 0.20f, 0.95f);
  knock.warnConfidence = clampFloat(knock.warnConfidence, 0.25f, 0.98f);
  knock.criticalConfidence = clampFloat(knock.criticalConfidence, 0.30f, 0.99f);
  if (knock.warnConfidence < knock.minDetectConfidence) knock.warnConfidence = knock.minDetectConfidence;
  if (knock.criticalConfidence < knock.warnConfidence) knock.criticalConfidence = knock.warnConfidence;
  knock.confidenceWeightSpectral = clampFloat(knock.confidenceWeightSpectral, 0.0f, 1.0f);
  knock.confidenceWeightFft = clampFloat(knock.confidenceWeightFft, 0.0f, 1.0f);
  knock.confidenceWeightHarmonic = clampFloat(knock.confidenceWeightHarmonic, 0.0f, 1.0f);
  knock.confidenceWeightTemplate = clampFloat(knock.confidenceWeightTemplate, 0.0f, 1.0f);
  knock.iatTempCompStartC = clampFloat(knock.iatTempCompStartC, -20.0f, 120.0f);
  knock.iatTempCompPerC = clampFloat(knock.iatTempCompPerC, 0.0f, 0.05f);
  knock.bayTempCompStartC = clampFloat(knock.bayTempCompStartC, -20.0f, 150.0f);
  knock.bayTempCompPerC = clampFloat(knock.bayTempCompPerC, 0.0f, 0.05f);
  knock.maxTempCompScale = clampFloat(knock.maxTempCompScale, 1.0f, 2.5f);
  knock.profileScaleIdle = clampFloat(knock.profileScaleIdle, 0.8f, 2.0f);
  knock.profileScaleSpool = clampFloat(knock.profileScaleSpool, 0.8f, 2.0f);
  knock.profileScaleSteady = clampFloat(knock.profileScaleSteady, 0.8f, 2.0f);
  knock.profileScaleLift = clampFloat(knock.profileScaleLift, 0.8f, 2.0f);
  knock.profileScaleHeatSoak = clampFloat(knock.profileScaleHeatSoak, 0.8f, 2.0f);
  knock.longTermBaselineAlpha = clampFloat(knock.longTermBaselineAlpha, 0.0001f, 0.05f);
  knock.driftWarnPercent = clampFloat(knock.driftWarnPercent, 5.0f, 150.0f);
  knock.driftCriticalPercent = clampFloat(knock.driftCriticalPercent, 10.0f, 200.0f);
  if (knock.driftCriticalPercent < knock.driftWarnPercent) {
    knock.driftCriticalPercent = knock.driftWarnPercent;
  }
  knock.adaptiveMultMin = clampFloat(knock.adaptiveMultMin, 0.8f, 5.0f);
  knock.adaptiveMultMax = clampFloat(knock.adaptiveMultMax, 1.0f, 8.0f);
  if (knock.adaptiveMultMax < knock.adaptiveMultMin) knock.adaptiveMultMax = knock.adaptiveMultMin;
  knock.adaptiveMultLearnAlpha = clampFloat(knock.adaptiveMultLearnAlpha, 0.0005f, 0.1f);
  knock.riskMapRateWeight = clampFloat(knock.riskMapRateWeight, 0.0f, 1.0f);
  knock.riskConfidenceWeight = clampFloat(knock.riskConfidenceWeight, 0.0f, 1.0f);
  knock.riskEventWeight = clampFloat(knock.riskEventWeight, 0.0f, 1.0f);
  knock.conservativeHealthThreshold = clampFloat(knock.conservativeHealthThreshold, 10.0f, 95.0f);
  knock.failsafeHealthThreshold = clampFloat(knock.failsafeHealthThreshold, 5.0f, 90.0f);
  if (knock.failsafeHealthThreshold > knock.conservativeHealthThreshold) {
    knock.failsafeHealthThreshold = knock.conservativeHealthThreshold;
  }
  knock.signalGain = clampFloat(knock.signalGain, 0.1f, 20.0f);
  knock.biasAlpha = clampFloat(knock.biasAlpha, 0.0001f, 0.05f);
  knock.envelopeAlpha = clampFloat(knock.envelopeAlpha, 0.01f, 0.8f);
  knock.rmsAlpha = clampFloat(knock.rmsAlpha, 0.01f, 0.6f);
  knock.thresholdOffset = clampFloat(knock.thresholdOffset, 0.0f, 200.0f);
  knock.thresholdMultiplier = clampFloat(knock.thresholdMultiplier, 0.5f, 8.0f);
  knock.baselineLearnAlpha = clampFloat(knock.baselineLearnAlpha, 0.001f, 0.3f);
  knock.eventCooldownMs = constrain(knock.eventCooldownMs, static_cast<uint16_t>(20), static_cast<uint16_t>(5000));
}

void attemptCanRecoveryIfNeeded(uint32_t nowMs) {
  if (!canRecoveryEnabled) return;
  if (canOnline) return;
  if (!elapsed(nowMs, lastCanRecoveryAttemptMs, kCanRecoveryIntervalMs)) return;
  lastCanRecoveryAttemptMs = nowMs;

  if (initCanBus()) {
    Serial.println("CAN: MCP2515 recovered");
  }
}

void sendRawCanFrame(uint16_t id, const uint8_t *data, uint8_t dlc) {
  if (!canOnline) return;
  const byte tx = canBus.sendMsgBuf(id, 0, dlc, const_cast<uint8_t *>(data));
  if (tx == CAN_OK) {
    canTxFailStreak = 0;
    return;
  }

  if (canTxFailStreak < 65535) ++canTxFailStreak;
  if (canTxFailStreak >= kCanTxFailStreakBeforeOffline) {
    canOnline = false;
    canRecoveryEnabled = true;
  }
}

void sendCanFrame(const can_protocol::CanFrame &frame) {
  sendRawCanFrame(frame.id, frame.data, frame.dlc);
}

void sendConfigAck(uint8_t version, uint8_t status, uint8_t rejectReason, uint8_t activeRatioPercent) {
  const uint8_t payload[4] = {version, status, rejectReason, activeRatioPercent};
  sendRawCanFrame(can_protocol::ID_METH_CONFIG_ACK, payload, 4);
}

void sendFault(uint8_t code, uint8_t severity, uint8_t data0, uint8_t data1) {
  const uint8_t payload[4] = {code, severity, data0, data1};
  sendRawCanFrame(can_protocol::ID_ENGINE_METH_FAULT, payload, 4);
}

void sendKnockApiHooksIfDue(const KnockStateSnapshot &knockState, uint32_t nowMs) {
  static uint32_t lastKnockApiTxMs = 0;
  static uint32_t lastKnockCfgTxMs = 0;

  if (elapsed(nowMs, lastKnockApiTxMs, kKnockApiTxIntervalMs)) {
    uint8_t flags = 0;
    if (config.knock.enabled) flags |= 1U << 0;
    if (knockState.armed) flags |= 1U << 1;
    if (knockState.knockDetected) flags |= 1U << 2;
    if (knockState.sensorFault) flags |= 1U << 3;
    if (knockState.clippingDetected) flags |= 1U << 4;
    if (knockState.warningActive) flags |= 1U << 5;
    if (knockState.criticalActive) flags |= 1U << 6;
    if (knockState.baselineLearned) flags |= 1U << 7;

    const uint8_t payload[8] = {
        flags,
        toU8(knockState.knockLevelRms),
        toU8(knockState.threshold),
        toU8(knockState.baseline),
        knockState.eventCount,
        toU8(knockState.biasAdc, 1.0f / 16.0f),
        toU8(knockState.rawAdc, 1.0f / 16.0f),
        toU8(knockState.envelope)};
    sendRawCanFrame(kCanIdKnockUiState, payload, 8);
    lastKnockApiTxMs = nowMs;
  }

  if (elapsed(nowMs, lastKnockCfgTxMs, kKnockCfgTxIntervalMs)) {
    const uint8_t payloadCfg1[8] = {
        static_cast<uint8_t>((config.knock.enabled ? 1U : 0U) |
                             (config.knock.autoCenterFromBore ? 2U : 0U)),
        toU8(config.knock.thresholdOffset),
        toU8(config.knock.thresholdMultiplier * 10.0f),
        toU8(static_cast<float>(config.knock.minRpmToArm) / 100.0f),
        toU8(config.knock.minMapKpaToArm),
        toU8(static_cast<float>(config.knock.eventCooldownMs) / 10.0f),
        toU8(config.knock.signalGain * 10.0f),
        toU8(config.knock.centerFreqHz / 100.0f)};
    sendRawCanFrame(kCanIdKnockUiConfig1, payloadCfg1, 8);

    const uint8_t payloadCfg2[8] = {
        toU8(config.knock.bandwidthHz / 100.0f),
        toU8(static_cast<float>(config.knock.sampleRateHz) / 100.0f),
        config.knock.samplesPerUpdate,
        toU8(config.knock.biasAlpha * 1000.0f),
        toU8(config.knock.rmsAlpha * 100.0f),
        toU8(config.knock.envelopeAlpha * 100.0f),
        toU8(config.knock.boreMm),
        toU8(config.knock.minLoadPercentToArm)};
    sendRawCanFrame(kCanIdKnockUiConfig2, payloadCfg2, 8);

    const uint8_t payloadBands[8] = {
        toU8(knockState.lowBandRms),
        toU8(knockState.midBandRms),
        toU8(knockState.highBandRms),
        toU8(knockState.spectralConfidence * 100.0f),
      toU8(knockState.selectedCenterHz / 100.0f),
      toU8(knockState.loadPercent),
      toU8((knockState.fftSnrDb + 6.0f) * 8.0f),
      toU8(config.knock.multiBandSpread * 100.0f)};
    sendRawCanFrame(kCanIdKnockUiBands, payloadBands, 8);

    const uint8_t payloadDsp[8] = {
      toU8((knockState.fftLongSnrDb + 6.0f) * 8.0f),
      toU8((knockState.fftShortSnrDb + 6.0f) * 8.0f),
      toU8(knockState.harmonicScore * 100.0f),
      toU8(knockState.templateDeviation * 100.0f),
      toU8(knockState.mapRateKpaPerSec + 127.0f),
      toU8(knockState.transientScale * 100.0f),
      toU8(config.knock.fftShortWeight * 100.0f),
      toU8(config.knock.spectralTemplateWeight * 100.0f)};
    sendRawCanFrame(kCanIdKnockUiDsp, payloadDsp, 8);

    const uint8_t payloadExplain[8] = {
      static_cast<uint8_t>(knockState.profile),
      static_cast<uint8_t>(knockState.anomalyClass),
      static_cast<uint8_t>(knockState.degradeMode),
      toU8(knockState.finalConfidence * 100.0f),
      toU8(knockState.knockRisk * 100.0f),
      toU8(knockState.healthScore),
      static_cast<uint8_t>(knockState.reasonFlags & 0xFFU),
      static_cast<uint8_t>((knockState.reasonFlags >> 8) & 0xFFU)};
    sendRawCanFrame(kCanIdKnockUiExplain, payloadExplain, 8);

    lastKnockCfgTxMs = nowMs;
  }
}

void dispatchCanFrame(uint16_t id, uint8_t dlc, const uint8_t *data, uint32_t nowMs) {
  using namespace can_protocol;

  if (id >= ID_BLOCK_MASTER_BASE && id < ID_BLOCK_ENGINE_METH_BASE) {
    lastMasterRxMs = nowMs;
  }

  if (id == ID_ENGINE_METH_COMMAND && dlc >= 1) {
    const uint8_t cmd = data[0];
    if (cmd == meth_command::ARM && dlc >= 2) {
      armed = (data[1] != 0);
      return;
    }
    if (cmd == meth_command::MANUAL_TEST_DUTY && dlc >= 2) {
      manualTestDuty = data[1];
      pendingManualTest = true;
      return;
    }
    if (cmd == meth_command::STOP_MANUAL_TEST) {
      pendingManualTest = false;
      manualTestDuty = 0;
      benchButtonOwnsTest = false;
      armed = false;
      return;
    }
    if (cmd == meth_command::CLEAR_FAULTS) {
      clearFaultsRequested = true;
      return;
    }

    if (cmd == kCmdKnockSetEnable && dlc >= 2) {
      config.knock.enabled = data[1] != 0;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetThresholdOffset && dlc >= 2) {
      config.knock.thresholdOffset = static_cast<float>(data[1]);
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetMultiplierX10 && dlc >= 2) {
      config.knock.thresholdMultiplier = static_cast<float>(data[1]) / 10.0f;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetMinRpmDiv100 && dlc >= 2) {
      config.knock.minRpmToArm = static_cast<uint16_t>(data[1]) * 100U;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetMinMapKpa && dlc >= 2) {
      config.knock.minMapKpaToArm = static_cast<float>(data[1]);
      config.knock.boostEnableKpa = config.knock.minMapKpaToArm;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetDebounceDiv10 && dlc >= 2) {
      config.knock.eventCooldownMs = static_cast<uint16_t>(data[1]) * 10U;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetGainX10 && dlc >= 2) {
      config.knock.signalGain = static_cast<float>(data[1]) / 10.0f;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetCenterHzDiv100 && dlc >= 2) {
      config.knock.centerFreqHz = static_cast<float>(data[1]) * 100.0f;
      config.knock.autoCenterFromBore = false;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetBandwidthHzDiv100 && dlc >= 2) {
      config.knock.bandwidthHz = static_cast<float>(data[1]) * 100.0f;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetAutoFreq && dlc >= 2) {
      config.knock.autoCenterFromBore = data[1] != 0;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetMinLoadPct && dlc >= 2) {
      config.knock.minLoadPercentToArm = static_cast<float>(data[1]);
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetSpreadX100 && dlc >= 2) {
      config.knock.multiBandSpread = static_cast<float>(data[1]) / 100.0f;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetFftEnable && dlc >= 2) {
      config.knock.fftEnabled = data[1] != 0;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetFftSnrDb && dlc >= 2) {
      config.knock.fftMinSnrDb = static_cast<float>(data[1]) - 20.0f;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetFftShortWeightX100 && dlc >= 2) {
      config.knock.fftShortWeight = static_cast<float>(data[1]) / 100.0f;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetFftHarmonicWeightX100 && dlc >= 2) {
      config.knock.fftHarmonicWeight = static_cast<float>(data[1]) / 100.0f;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetTemplateWeightX100 && dlc >= 2) {
      config.knock.spectralTemplateWeight = static_cast<float>(data[1]) / 100.0f;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetMapRateGateKpaPerSec && dlc >= 2) {
      config.knock.mapRateGateKpaPerSec = static_cast<float>(data[1]);
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetTransientScaleX100 && dlc >= 2) {
      config.knock.transientThresholdScale = static_cast<float>(data[1]) / 100.0f;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetTransientHoldDiv10 && dlc >= 2) {
      config.knock.transientHoldMs = static_cast<uint16_t>(data[1]) * 10U;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetDetectConfidenceX100 && dlc >= 2) {
      config.knock.minDetectConfidence = static_cast<float>(data[1]) / 100.0f;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetWarnConfidenceX100 && dlc >= 2) {
      config.knock.warnConfidence = static_cast<float>(data[1]) / 100.0f;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetCriticalConfidenceX100 && dlc >= 2) {
      config.knock.criticalConfidence = static_cast<float>(data[1]) / 100.0f;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetIatCompStartC && dlc >= 2) {
      config.knock.iatTempCompStartC = static_cast<float>(data[1]) - 40.0f;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetIatCompPerCx1000 && dlc >= 2) {
      config.knock.iatTempCompPerC = static_cast<float>(data[1]) / 1000.0f;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetBayCompStartC && dlc >= 2) {
      config.knock.bayTempCompStartC = static_cast<float>(data[1]) - 40.0f;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockSetBayCompPerCx1000 && dlc >= 2) {
      config.knock.bayTempCompPerC = static_cast<float>(data[1]) / 1000.0f;
      sanitizeKnockConfig(config.knock);
      return;
    }
    if (cmd == kCmdKnockResetEvents) {
      knockMonitor.clearFaults();
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
      sendConfigAck(cfg.version, 1, 4, lastConfigRatioPercent);
      return;
    }

    lastConfigVersion = cfg.version;
    if (cfg.ratio_percent <= 100 && cfg.ratio_percent != lastConfigRatioPercent) {
      lastConfigRatioPercent = cfg.ratio_percent;
      remoteRatioPercent = cfg.ratio_percent;
      hasRemoteRatio = true;
    }
    armed = (cfg.desired_armed != 0);
    sendConfigAck(cfg.version, 0, 0, lastConfigRatioPercent);
    lastMasterRxMs = nowMs;
  }
}

void pollCan(uint32_t nowMs) {
  if (!canOnline) return;

  while (digitalRead(pins::CAN_SPI_INT) == LOW) {
    unsigned long id = 0;
    uint8_t dlc = 0;
    uint8_t data[8] = {0};
    if (canBus.readMsgBuf(&id, &dlc, data) != CAN_OK) break;
    dispatchCanFrame(static_cast<uint16_t>(id & 0x7FFU), dlc, data, nowMs);
  }
}

void sendStateIfDue(const SensorReadings &readings, const ControlResult &result, uint32_t nowMs) {
  if (!canOnline) return;
  if (!elapsed(nowMs, lastStateTxMs, kStateTxIntervalMs)) return;
  lastStateTxMs = nowMs;

  uint8_t methState = kMethStateOff;
  if (result.failsafe != FailsafeReason::None) {
    methState = kMethStateFault;
  } else if (pendingManualTest) {
    methState = kMethStateTest;
  } else if (result.pump.enabled) {
    methState = kMethStateSpraying;
  } else if (armed) {
    methState = kMethStateArmed;
  }

  const uint8_t tankLevel = readings.tankLow ? 0 : 100;

  uint8_t flowStatus = kFlowUnknown;
  if (result.failsafe == FailsafeReason::None && result.pump.enabled) {
    flowStatus = kFlowOk;
  } else if (result.failsafe == FailsafeReason::LowFluid) {
    flowStatus = kFlowNone;
  }

  const uint8_t boostKpa = static_cast<uint8_t>(constrain(static_cast<int>(readings.mapKpa), 0, 255));

  uint8_t faultFlags = 0;
  if (result.failsafe == FailsafeReason::LowFluid) faultFlags |= 0x01;
  if (result.failsafe == FailsafeReason::MapInvalid) faultFlags |= 0x02;
  if (result.failsafe == FailsafeReason::InvalidBlend) faultFlags |= 0x04;
  if (result.failsafe == FailsafeReason::InvalidBoostConfig) faultFlags |= 0x08;

  uint8_t payload[8] = {0};
  payload[0] = methState;
  payload[1] = static_cast<uint8_t>(constrain(static_cast<int>(result.finalDutyPercent), 0, 100));
  payload[2] = tankLevel;
  payload[3] = flowStatus;
  payload[4] = boostKpa;
  payload[5] = can_protocol::tempToOffset40(static_cast<int>(readings.iatC));
  payload[6] = can_protocol::tempToOffset40(static_cast<int>(readings.engineBayC));
  payload[7] = faultFlags;
  sendRawCanFrame(can_protocol::ID_ENGINE_METH_STATE, payload, 8);

  if (elapsed(nowMs, lastExtTxMs, kExtTxIntervalMs)) {
    lastExtTxMs = nowMs;
    can_protocol::EngineSensorExt ext{};
    ext.oil_pressure_psi = can_protocol::clampU8(static_cast<int>(readings.oilPressurePsi));
    ext.fuel_pressure_psi = can_protocol::clampU8(static_cast<int>(readings.fuelPressurePsi));
    ext.meth_pressure_psi = can_protocol::clampU8(static_cast<int>(readings.methPressurePsi));
    ext.boost_ref_pressure_psi = can_protocol::clampU8(static_cast<int>(readings.boostRefPressurePsi));
    ext.ambient_temp_c = static_cast<int8_t>(readings.ambientC);
    ext.cabin_temp_c = static_cast<int8_t>(readings.cabinC);
    ext.analog_fault_flags = readings.analogFaultFlags;
    sendCanFrame(can_protocol::packEngineSensorExt(ext));
  }
}

void sendKnockStateIfDue(const KnockStateSnapshot &knockState, uint32_t nowMs) {
  if (!canOnline) return;
  if (!elapsed(nowMs, lastKnockTxMs, kKnockTxIntervalMs)) return;
  lastKnockTxMs = nowMs;

  can_protocol::EngineKnockState knockCan{};
  if (knockState.online) knockCan.status_flags |= kKnockStatusOnline;
  if (knockState.signalValid) knockCan.status_flags |= kKnockStatusSignalValid;
  if (knockState.warningActive) knockCan.status_flags |= kKnockStatusWarning;
  if (knockState.criticalActive) knockCan.status_flags |= kKnockStatusCritical;
  if (knockState.sensorFault) knockCan.status_flags |= kKnockStatusSensorFault;
  if (knockState.clippingDetected) knockCan.status_flags |= kKnockStatusClipping;
  if (knockState.baselineLearned) knockCan.status_flags |= kKnockStatusBaselineLearned;
  knockCan.energy = can_protocol::clampU8(static_cast<int>(knockState.energy));
  knockCan.baseline = can_protocol::clampU8(static_cast<int>(knockState.baseline));
  knockCan.threshold = can_protocol::clampU8(static_cast<int>(knockState.threshold));
  knockCan.event_count = knockState.eventCount;
  knockCan.last_event_rpm_div100 = can_protocol::clampU8(static_cast<int>(knockState.lastEventRpm / 100));
  knockCan.last_event_boost_kpa = knockState.lastEventBoostKpa;
  sendCanFrame(can_protocol::packEngineKnockState(knockCan));
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  WATER <liters>   - set water volume");
  Serial.println("  METH <liters>    - set methanol volume");
  Serial.println("  SHOW             - print current config/blend");
  Serial.println("  MODE OFF|BOOST|PRIME");
  Serial.println("  CAN RETRY        - re-enable MCP2515 recovery attempts");
  Serial.println("  TEST <0-100>     - fire pump at duty% (bench test)");
  Serial.println("  TEST OFF         - stop bench test");
  KnockUi::printHelp(Serial);
  Serial.println("  HELP");
}

const char *modeName(InjectionMode mode) {
  switch (mode) {
  case InjectionMode::Off:
    return "OFF";
  case InjectionMode::BoostOnly:
    return "BOOST";
  case InjectionMode::Prime:
    return "PRIME";
  default:
    return "UNKNOWN";
  }
}

const char *failsafeName(FailsafeReason reason) {
  switch (reason) {
  case FailsafeReason::None:
    return "NONE";
  case FailsafeReason::LowFluid:
    return "LOW_FLUID";
  case FailsafeReason::MapInvalid:
    return "MAP_INVALID";
  case FailsafeReason::InvalidBlend:
    return "INVALID_BLEND";
  case FailsafeReason::InvalidBoostConfig:
    return "INVALID_BOOST_CONFIG";
  default:
    return "UNKNOWN";
  }
}

void saveBlend() {
  if (!preferences.begin(kPrefsNamespace, false)) return;
  preferences.putFloat(kPrefsKeyWater, blend.waterLiters);
  preferences.putFloat(kPrefsKeyMeth, blend.methLiters);
  preferences.end();
}

void loadBlend() {
  if (!preferences.begin(kPrefsNamespace, true)) return;
  const float water = preferences.getFloat(kPrefsKeyWater, blend.waterLiters);
  const float meth = preferences.getFloat(kPrefsKeyMeth, blend.methLiters);
  preferences.end();
  blend = computeTankBlend(water, meth);
}

void printSetupSummary() {
  Serial.println("---- Water/Meth Controller ----");
  Serial.print("Mode: ");
  Serial.println(modeName(config.mode));
  Serial.print("MAP calibration (V): ");
  Serial.print(config.map.vMin, 3);
  Serial.print(" to ");
  Serial.print(config.map.vMax, 3);
  Serial.print(" -> (kPa): ");
  Serial.print(config.map.kpaMin, 1);
  Serial.print(" to ");
  Serial.println(config.map.kpaMax, 1);
  Serial.print("Boost start/full (psi): ");
  Serial.print(config.boost.startPsi, 1);
  Serial.print(" / ");
  Serial.println(config.boost.fullPsi, 1);
  Serial.print("Blend water/meth (L): ");
  Serial.print(blend.waterLiters, 2);
  Serial.print(" / ");
  Serial.print(blend.methLiters, 2);
  Serial.print("  Meth%: ");
  Serial.println(blend.methPercent, 1);
  Serial.print("Knock ADC pin: ");
  Serial.println(pins::KNOCK_SENSOR_ADC);
  Serial.print("Bench test button: pin ");
  Serial.print(pins::BENCH_TEST_BUTTON);
  Serial.print(" (D9/GPIO18) - hold to fire pump at ");
  Serial.print(config.benchTestDutyPercent);
  Serial.println("%");
  Serial.println("CAN: MCP2515 SPI");
  Serial.println("--------------------------------");
}

void printKnockSummary() {
  KnockUi::printSummary(Serial, config.knock, knockMonitor.state());
}

void configureAnalogSensors() {
  ThermistorConfig iatCfg{};
  iatCfg.enabled = true;
  iatCfg.pin = pins::IAT_THERM_PIN;
  iatCfg.pullupOhms = 10000.0f;
  iatCfg.filterAlpha = 0.2f;
  iatCfg.minValidTempC = -40.0f;
  iatCfg.maxValidTempC = 180.0f;
  iatSensor.begin(iatCfg);

  ThermistorConfig bayCfg = iatCfg;
  bayCfg.pin = pins::ENGINE_BAY_THERM_PIN;
  bayCfg.maxValidTempC = 200.0f;
  engineBaySensor.begin(bayCfg);

  ThermistorConfig cabinCfg = iatCfg;
  cabinCfg.pin = pins::CABIN_THERM_PIN;
  cabinCfg.maxValidTempC = 120.0f;
  cabinSensor.begin(cabinCfg);

  ThermistorConfig ambientCfg = iatCfg;
  ambientCfg.pin = pins::AMBIENT_THERM_PIN;
  ambientCfg.maxValidTempC = 100.0f;
  ambientSensor.begin(ambientCfg);

  PressureConfig pressureCfg{};
  pressureCfg.enabled = true;
  pressureCfg.sensorMinV = 0.5f;
  pressureCfg.sensorMaxV = 4.5f;
  pressureCfg.dividerTopOhms = 10000.0f;
  pressureCfg.dividerBottomOhms = 20000.0f;
  pressureCfg.pressureMinPsi = 0.0f;
  pressureCfg.pressureMaxPsi = 100.0f;
  pressureCfg.maxValidPsi = 250.0f;

  PressureConfig oilCfg = pressureCfg;
  oilCfg.pin = pins::OIL_PRESSURE_ADC;
  oilCfg.pressureMaxPsi = 150.0f;
  oilCfg.maxValidPsi = 200.0f;
  oilPressure.begin(oilCfg);

  PressureConfig fuelCfg = pressureCfg;
  fuelCfg.pin = pins::FUEL_PRESSURE_ADC;
  fuelPressure.begin(fuelCfg);

  PressureConfig methCfg = pressureCfg;
  methCfg.pin = pins::METH_PRESSURE_ADC;
  methPressure.begin(methCfg);

  PressureConfig boostRefCfg = pressureCfg;
  boostRefCfg.pin = pins::BOOST_REF_PRESSURE_ADC;
  boostRefPressure.begin(boostRefCfg);

  PressureConfig spare1Cfg = pressureCfg;
  spare1Cfg.enabled = false;
  spare1Cfg.pin = pins::SPARE_PRESSURE_1_ADC;
  sparePressure1.begin(spare1Cfg);

  PressureConfig spare2Cfg = pressureCfg;
  spare2Cfg.enabled = false;
  spare2Cfg.pin = pins::SPARE_PRESSURE_2_ADC;
  sparePressure2.begin(spare2Cfg);
}

void updateAnalogReadings(SensorReadings &r, uint32_t nowMs) {
  iatSensor.update(nowMs);
  engineBaySensor.update(nowMs);
  cabinSensor.update(nowMs);
  ambientSensor.update(nowMs);
  oilPressure.update(nowMs);
  fuelPressure.update(nowMs);
  methPressure.update(nowMs);
  boostRefPressure.update(nowMs);
  sparePressure1.update(nowMs);
  sparePressure2.update(nowMs);

  r.analogFaultFlags = 0;
  r.iatValid = iatSensor.valid();
  r.engineBayValid = engineBaySensor.valid();
  r.cabinValid = cabinSensor.valid();
  r.ambientValid = ambientSensor.valid();
  r.oilPressureValid = oilPressure.valid();
  r.fuelPressureValid = fuelPressure.valid();
  r.methPressureValid = methPressure.valid();
  r.boostRefPressureValid = boostRefPressure.valid();
  r.sparePressure1Valid = sparePressure1.valid();
  r.sparePressure2Valid = sparePressure2.valid();

  if (r.iatValid) r.iatC = iatSensor.valueC();
  else if (iatSensor.config().enabled) r.analogFaultFlags |= kFaultIat;

  if (r.engineBayValid) r.engineBayC = engineBaySensor.valueC();
  else if (engineBaySensor.config().enabled) r.analogFaultFlags |= kFaultEngineBay;

  if (r.cabinValid) r.cabinC = cabinSensor.valueC();
  else if (cabinSensor.config().enabled) r.analogFaultFlags |= kFaultCabin;

  if (r.ambientValid) r.ambientC = ambientSensor.valueC();
  else if (ambientSensor.config().enabled) r.analogFaultFlags |= kFaultAmbient;

  if (r.oilPressureValid) r.oilPressurePsi = oilPressure.valuePsi();
  else if (oilPressure.config().enabled) r.analogFaultFlags |= kFaultOil;

  if (r.fuelPressureValid) r.fuelPressurePsi = fuelPressure.valuePsi();
  else if (fuelPressure.config().enabled) r.analogFaultFlags |= kFaultFuel;

  if (r.methPressureValid) r.methPressurePsi = methPressure.valuePsi();
  else if (methPressure.config().enabled) r.analogFaultFlags |= kFaultMeth;

  if (r.boostRefPressureValid) r.boostRefPressurePsi = boostRefPressure.valuePsi();
  else if (boostRefPressure.config().enabled) r.analogFaultFlags |= kFaultBoostRef;

  if (r.sparePressure1Valid) r.sparePressure1Psi = sparePressure1.valuePsi();
  else if (sparePressure1.config().enabled) r.analogFaultFlags |= kFaultSpare1;

  if (r.sparePressure2Valid) r.sparePressure2Psi = sparePressure2.valuePsi();
  else if (sparePressure2.config().enabled) r.analogFaultFlags |= kFaultSpare2;
}

bool parsePositiveFloat(const String &token, float &valueOut) {
  char buffer[32];
  token.toCharArray(buffer, sizeof(buffer));
  char *endPtr = nullptr;
  const float parsed = strtof(buffer, &endPtr);
  if (endPtr == buffer) return false;
  valueOut = parsed;
  return true;
}

void parseCommand(const String &line) {
  String cmd = line;
  cmd.trim();
  if (cmd.length() == 0) return;
  String cmdUpper = cmd;
  cmdUpper.toUpperCase();

  if (cmd.equalsIgnoreCase("HELP")) {
    printHelp();
    return;
  }
  if (cmd.equalsIgnoreCase("SHOW")) {
    printSetupSummary();
    return;
  }

  if (cmd.startsWith("WATER ") || cmd.startsWith("water ")) {
    float value = 0.0f;
    if (parsePositiveFloat(cmd.substring(6), value) && value >= 0.0f) {
      blend = computeTankBlend(value, blend.methLiters);
      saveBlend();
      Serial.print("Water updated. Meth% = ");
      Serial.println(blend.methPercent, 1);
    }
    return;
  }

  if (cmd.startsWith("METH ") || cmd.startsWith("meth ")) {
    float value = 0.0f;
    if (parsePositiveFloat(cmd.substring(5), value) && value >= 0.0f) {
      blend = computeTankBlend(blend.waterLiters, value);
      saveBlend();
      Serial.print("Meth updated. Meth% = ");
      Serial.println(blend.methPercent, 1);
    }
    return;
  }

  if (cmd.startsWith("MODE ") || cmd.startsWith("mode ")) {
    const String value = cmd.substring(5);
    if (value.equalsIgnoreCase("OFF")) config.mode = InjectionMode::Off;
    else if (value.equalsIgnoreCase("BOOST")) config.mode = InjectionMode::BoostOnly;
    else if (value.equalsIgnoreCase("PRIME")) config.mode = InjectionMode::Prime;
    else return;
    Serial.print("Mode set to ");
    Serial.println(modeName(config.mode));
    return;
  }

  if (cmdUpper == "CAN RETRY") {
    canRecoveryEnabled = true;
    lastCanRecoveryAttemptMs = 0;
    Serial.println("CAN recovery re-enabled.");
    return;
  }

  if (cmdUpper.startsWith("KNOCK")) {
    if (!KnockUi::handleCommand(cmd, config.knock, knockMonitor, Serial)) {
      Serial.println("Invalid KNOCK command. Use KNOCK SHOW for current settings.");
    } else {
      sanitizeKnockConfig(config.knock);
    }
    return;
  }

  if (cmdUpper.startsWith("TEST")) {
    const String arg = cmd.length() > 5 ? cmd.substring(5) : String("");
    if (arg.equalsIgnoreCase("OFF") || arg.length() == 0) {
      pendingManualTest = false;
      manualTestDuty = 0;
      Serial.println("Manual test stopped.");
    } else {
      float duty = 0.0f;
      if (parsePositiveFloat(arg, duty) && duty <= 100.0f) {
        manualTestDuty = static_cast<uint8_t>(duty);
        pendingManualTest = true;
        Serial.print("Manual test: pump at ");
        Serial.print(manualTestDuty);
        Serial.println("%");
      } else {
        Serial.println("Usage: TEST <0-100> | TEST OFF");
      }
    }
    return;
  }
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      parseCommand(serialLine);
      serialLine = "";
      continue;
    }
    if (serialLine.length() < 120) serialLine += c;
  }
}

// Poll the physical bench-test button (active LOW, internal pull-up).
// Holding the button sets pendingManualTest; releasing clears it only if
// the button itself started the test (not a CAN-commanded manual test).
void pollBenchButton(uint32_t nowMs) {
  constexpr uint32_t kDebounceMs = 30;
  const bool rawPressed = (digitalRead(pins::BENCH_TEST_BUTTON) == LOW);

  if (rawPressed != benchButtonActive) {
    if ((nowMs - benchButtonDebounceMs) >= kDebounceMs) {
      benchButtonActive = rawPressed;
      benchButtonDebounceMs = nowMs;

      if (benchButtonActive) {
        pendingManualTest = true;
        benchButtonOwnsTest = true;
        manualTestDuty = config.benchTestDutyPercent;
        Serial.print("Bench test button pressed – pump duty ");
        Serial.print(manualTestDuty);
        Serial.println("%");
      } else if (benchButtonOwnsTest) {
        pendingManualTest = false;
        manualTestDuty = 0;
        benchButtonOwnsTest = false;
        Serial.println("Bench test button released – pump off");
      }
    }
  } else {
    benchButtonDebounceMs = nowMs;
  }
}
} // namespace

void setup() {
  Serial.begin(config.serialBaud);
  delay(200);

  sanitizeKnockConfig(config.knock);

  loadBlend();
  mapSensor.begin(pins::MAP_SENSOR_ADC, config.map);
  floatSensor.begin(pins::FLOAT_SENSOR_DIGITAL,
                    config.floatActiveLow,
                    config.floatDebounceMs,
                    config.floatLowShutdownDelayMs);
  configureAnalogSensors();
  knockMonitor.begin(pins::KNOCK_SENSOR_ADC, config.knock);
  pumpDriver.begin(pins::PUMP_PWM, config.pwmFrequencyHz, config.pwmResolutionBits);
  warningOutput.begin(pins::WARNING_LED, true);
  pinMode(pins::BENCH_TEST_BUTTON, INPUT_PULLUP);

  SPI.begin();
  pinMode(pins::CAN_SPI_INT, INPUT);
  if (initCanBus()) {
    Serial.println("CAN: MCP2515 online");
    lastMasterRxMs = millis();
  } else {
    Serial.println("CAN: MCP2515 init failed - running serial-only");
    canRecoveryEnabled = false;
    Serial.println("CAN: auto-recovery paused (use 'CAN RETRY' after wiring MCP2515)");
  }

  printSetupSummary();
  printHelp();
}

void loop() {
  handleSerialCommands();

  const uint32_t now = millis();
  pollBenchButton(now);

  if (!elapsed(now, lastLoopMs, config.loopPeriodMs)) return;
  lastLoopMs = now;

  attemptCanRecoveryIfNeeded(now);
  pollCan(now);

  if (canOnline && elapsed(now, lastMasterRxMs, kCanCommandTimeoutMs)) {
    // Fail safe to OFF if the master controller stops sending commands.
    armed = false;
    pendingManualTest = false;
    manualTestDuty = 0;
  }

  if (hasRemoteRatio) {
    const uint8_t pct = remoteRatioPercent;
    blend = computeTankBlend(static_cast<float>(100 - pct), static_cast<float>(pct));
    hasRemoteRatio = false;
  }
  if (clearFaultsRequested) {
    lastFailsafe = FailsafeReason::None;
    lastReportedCanFault = FailsafeReason::None;
    analogSafetyLatched = false;
    clearFaultsRequested = false;
  }

  SensorReadings readings = mapSensor.read();
  readings.tankLow = floatSensor.update();
  updateAnalogReadings(readings, now);

  float loadPercent = 0.0f;
  static float lastMapKpaForRate = 0.0f;
  static uint32_t lastMapRateMs = 0;
  const float mapSpan = config.map.kpaMax - config.map.baroKpa;
  if (mapSpan > 1.0f) {
    loadPercent = ((readings.mapKpa - config.map.baroKpa) / mapSpan) * 100.0f;
  }
  loadPercent = clampFloat(loadPercent, 0.0f, 100.0f);

  float mapRateKpaPerSec = 0.0f;
  if (lastMapRateMs != 0 && now > lastMapRateMs) {
    const float dt = static_cast<float>(now - lastMapRateMs) / 1000.0f;
    if (dt > 0.0005f) {
      mapRateKpaPerSec = (readings.mapKpa - lastMapKpaForRate) / dt;
    }
  }
  lastMapKpaForRate = readings.mapKpa;
  lastMapRateMs = now;

  knockMonitor.setConfig(config.knock);
  const KnockStateSnapshot knockState = knockMonitor.update(readings.mapKpa, loadPercent,
                                                            mapRateKpaPerSec,
                                                            readings.iatC,
                                                            readings.engineBayC,
                                                            now);

  AppConfig effectiveConfig = config;
  if (canOnline && !armed) effectiveConfig.mode = InjectionMode::Off;

  ControlResult result{};
  if (pendingManualTest) {
    result.pump.enabled = true;
    result.pump.dutyPercent = constrain(manualTestDuty, 0, 100);
    result.finalDutyPercent = result.pump.dutyPercent;
  } else {
    result = controller.update(readings, effectiveConfig, blend);
  }

  bool knockSafetyShutdown = false;
  if (knockState.requestSafetyShutdown) {
    result.pump.enabled = false;
    result.pump.dutyPercent = 0.0f;
    result.finalDutyPercent = 0.0f;
    knockSafetyShutdown = true;
  } else if (knockState.requestForceSpray && result.failsafe == FailsafeReason::None && !result.pump.enabled) {
    result.pump.enabled = true;
    result.pump.dutyPercent = effectiveConfig.dutyMinPercent;
    result.finalDutyPercent = effectiveConfig.dutyMinPercent;
  }

  const uint16_t analogCriticalMask = static_cast<uint16_t>(kFaultIat | kFaultEngineBay | kFaultMeth);
  const bool analogCriticalFault = (readings.analogFaultFlags & analogCriticalMask) != 0;
  if (analogCriticalFault) {
    result.pump.enabled = false;
    result.pump.dutyPercent = 0.0f;
    result.finalDutyPercent = 0.0f;
  }

  pumpDriver.apply(result.pump);
  warningOutput.set(result.failsafe != FailsafeReason::None || knockSafetyShutdown ||
                    knockState.warningActive || knockState.criticalActive);

  if (result.failsafe != lastReportedCanFault) {
    if (result.failsafe != FailsafeReason::None) {
      uint8_t code = 0x09;
      if (result.failsafe == FailsafeReason::LowFluid) code = 0x01;
      if (result.failsafe == FailsafeReason::MapInvalid) code = 0x05;
      if (result.failsafe == FailsafeReason::InvalidBlend) code = 0x08;
      if (result.failsafe == FailsafeReason::InvalidBoostConfig) code = 0x08;
      sendFault(code, kFaultSeverityWarning, 0, 0);
    }
    lastReportedCanFault = result.failsafe;
  }

  if (knockSafetyShutdown) {
    sendFault(can_protocol::meth_fault_code::SAFETY_SHUTDOWN, kFaultSeverityWarning, 0, 0);
  }

  if (analogCriticalFault && !analogSafetyLatched) {
    sendFault(0x0B, kFaultSeverityWarning,
              static_cast<uint8_t>(readings.analogFaultFlags & 0xFFU),
              static_cast<uint8_t>((readings.analogFaultFlags >> 8) & 0xFFU));
    analogSafetyLatched = true;
  } else if (!analogCriticalFault) {
    analogSafetyLatched = false;
  }

  KnockFaultEvent knockFault{};
  if (knockMonitor.consumeFault(knockFault)) {
    sendCanFrame(can_protocol::packEngineKnockFault(knockFault.code, knockFault.severity,
                                                     knockFault.data0, knockFault.data1));
  }

  sendKnockStateIfDue(knockState, now);
  sendKnockApiHooksIfDue(knockState, now);
  sendStateIfDue(readings, result, now);

  if (result.failsafe != lastFailsafe) {
    Serial.print("Failsafe state changed: ");
    Serial.println(failsafeName(result.failsafe));
    lastFailsafe = result.failsafe;
  }

  if (!elapsed(now, lastDebugMs, config.debugPeriodMs)) return;
  lastDebugMs = now;

  Serial.print("MAPkPa=");
  Serial.print(readings.mapKpa, 1);
  Serial.print(" boostPsi=");
  Serial.print(readings.boostPsi, 2);
  Serial.print(" iatC=");
  Serial.print(readings.iatC, 1);
  Serial.print(" oil/fuel/meth=");
  Serial.print(readings.oilPressurePsi, 1);
  Serial.print("/");
  Serial.print(readings.fuelPressurePsi, 1);
  Serial.print("/");
  Serial.print(readings.methPressurePsi, 1);
  Serial.print(" dutyOut=");
  Serial.print(result.finalDutyPercent, 1);
  Serial.print(" fs=");
  Serial.print(failsafeName(result.failsafe));
  Serial.print(" knockRaw=");
  Serial.print(knockState.rawAdc, 1);
  Serial.print(" knockBias=");
  Serial.print(knockState.biasAdc, 1);
  Serial.print(" knockFilt=");
  Serial.print(knockState.filteredSignal, 2);
  Serial.print(" knockEnv=");
  Serial.print(knockState.envelope, 2);
  Serial.print(" knockRMS=");
  Serial.print(knockState.knockLevelRms, 2);
  Serial.print(" knockL/M/H=");
  Serial.print(knockState.lowBandRms, 2);
  Serial.print("/");
  Serial.print(knockState.midBandRms, 2);
  Serial.print("/");
  Serial.print(knockState.highBandRms, 2);
  Serial.print(" specC=");
  Serial.print(knockState.spectralConfidence, 3);
  Serial.print(" fftSNR=");
  Serial.print(knockState.fftSnrDb, 2);
  Serial.print(" fftL/S=");
  Serial.print(knockState.fftLongSnrDb, 2);
  Serial.print("/");
  Serial.print(knockState.fftShortSnrDb, 2);
  Serial.print(" harm=");
  Serial.print(knockState.harmonicScore, 2);
  Serial.print(" tdev=");
  Serial.print(knockState.templateDeviation, 3);
  Serial.print(" mapRate=");
  Serial.print(knockState.mapRateKpaPerSec, 1);
  Serial.print(" conf/risk=");
  Serial.print(knockState.finalConfidence, 2);
  Serial.print("/");
  Serial.print(knockState.knockRisk, 2);
  Serial.print(" hlth=");
  Serial.print(knockState.healthScore, 1);
  Serial.print(" cls=");
  Serial.print(knockState.anomalyClass);
  Serial.print(" dg=");
  Serial.print(knockState.degradeMode);
  Serial.print(" ts=");
  Serial.print(knockState.transientScale, 2);
  Serial.print(" knockThr=");
  Serial.print(knockState.threshold, 2);
  Serial.print(" knockDet=");
  Serial.print(knockState.knockDetected ? "1" : "0");
  Serial.print(" knockArmed=");
  Serial.print(knockState.armed ? "1" : "0");
  Serial.print(" knockEvt=");
  Serial.print(knockState.eventCount);
  Serial.print(" analogFault=0x");
  Serial.print(readings.analogFaultFlags, HEX);
  Serial.println();
}
