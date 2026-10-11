// ESP32-S3 water/meth and knock controller (replaces the Arduino Nano board).
//
// What changed with the hardware, and nothing else:
//   - The pump runs through a MOSFET with PWM (meth_tune::PwmGovernor), not a relay.
//   - Manifold pressure, engine speed and intake air temperature come straight from
//     the MicroSquirt's CAN broadcast (ecu_broadcast.h). There is no MAP sensor here,
//     and no fresh ECU data means no injection.
//   - Knock, oil pressure, fuel pressure and the air temperature before the nozzle are
//     read by the ADC's own timer (adc_sampler).
//   - New firmware arrives from the dash over CAN (can_contract/firmware_update.h).
// Every CAN message, command and setting is the same as the Nano firmware's: the
// functions that speak them are that firmware's own code. Wiring: docs/ESP32_HARDWARE.md.
#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include <esp_ota_ops.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <math.h>
#include <mcp2515.h>

#include "adc_sampler.h"
#include "analog_inputs.h"
#include "app_config.h"
#include "can_contract/can_protocol.h"
#include "can_contract/firmware_update.h"
#include "ecu_broadcast.h"
#include "injection_controller.h"
#include "knock_band.h"
#include "meth_tune.h"
#include "pins_esp32.h"

// Identifies this build to the dash. The published firmware is built with the first
// eight hex digits of its commit; a build from the editor reports 0.
#ifndef METH_FW_BUILD
#define METH_FW_BUILD 0
#endif

// A firmware received over CAN stays on trial until the dash confirms it (see loop()).
extern "C" bool verifyRollbackLater() { return true; }

#ifdef CCM_CAN_PROTOCOL_VERSION
static_assert(CCM_CAN_PROTOCOL_VERSION == can_protocol::CAN_PROTOCOL_SCHEMA_VERSION,
              "platformio.ini CCM_CAN_PROTOCOL_VERSION does not match the shared CAN contract");
#endif
#ifndef CAN_PROTOCOL_HAS_METH_PWM
#error "Update DIYComfortControlModule: the shared CAN contract needs the PWM duty settings (extension 5)."
#endif
static_assert(meth_tune::kCount == can_protocol::meth_tune_setting::COUNT &&
              meth_tune::kPeriodMs == can_protocol::meth_tune_setting::PERIOD_MS &&
              meth_tune::kMaxDosePct == can_protocol::meth_tune_setting::MAX_DOSE_PCT &&
              meth_tune::kHotFullC == can_protocol::meth_tune_setting::HOT_FULL_C &&
              meth_tune::kMinDutyPct == can_protocol::meth_tune_setting::MIN_DUTY_PCT &&
              meth_tune::kMaxDutyPct == can_protocol::meth_tune_setting::MAX_DUTY_PCT &&
              meth_tune::kHoldDoseLimit == can_protocol::meth_hold::DOSE_LIMIT &&
              meth_tune::kHoldFault == can_protocol::meth_hold::FAULT &&
              meth_tune::kSetClamped == can_protocol::config_ack_status::VALUE_CLAMPED,
              "meth_tune.h does not match the shared CAN contract");

namespace {
namespace fw = can_protocol::firmware;

AppConfig config = defaultConfig();
TankBlend blend = computeTankBlend(1.5f, 0.5f);
InjectionController controller;
analog_inputs::FloatSwitch floatSensor;
analog_inputs::Pressure oilPressure;
analog_inputs::Pressure fuelPressure;
ecu::Data ecuData;
float pumpDutyPercent = 0.0f;   // What the pump is being driven at right now.
bool adcReady = false;

// Tuning (meth_tune.h): adjustable over CAN, saved in flash on request.
meth_tune::Settings tune;
meth_tune::PwmGovernor governor;
uint8_t tuneRevision = 0;     // Bumped whenever a setting changes.
bool tuneUnsaved = false;
bool tuneSavePending = false; // SAVE is written from loop().
uint8_t tuneReportNext = 0;   // Next setting to report; 0 = no report running.
uint32_t lastTuneStatusTxMs = 0;
uint32_t lastTuneTempsTxMs = 0;
uint32_t lastIntakeTempMs = 0;
float preTempC = 0.0f;
float postTempC = 0.0f;
bool preTempValid = false;
bool postTempValid = false;
// The ECU's intake temperature (from its broadcast, or forwarded by the dash), used
// for the temperature features while the sensor before the nozzle is not reading.
float ecuIntakeC = 0.0f;
uint32_t ecuIntakeSeenMs = 0;
bool ecuIntakeSeen = false;
bool usingEcuTemp = false;
bool lastPumpOn = false;
constexpr uint32_t kEcuIntakeTimeoutMs = 1500;
// A tune saved by the firmware before that (17 settings) is still read, so flashing
// this firmware does not reset the tune.
constexpr uint8_t kTuneEepromVersionBeforeDuty = 2;
constexpr uint8_t kTuneCountBeforeDuty = 17;
constexpr uint32_t kIntakeTempSampleMs = 50;

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
uint32_t manualTestStartMs = 0;
// Knock listening (knock_band.h): the newest burst each loop pass, judged by the detector.
knock_band::Detector knockDetector;
knock_band::Burst knockBurst;
uint16_t knockSamples[knock_band::kSamples];
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

constexpr float kPsiPerKpa = 0.1450377f;
// The amplifier should rest near mid-scale; far from it the sensor is not being heard.
constexpr uint16_t kKnockBiasLowAdc = 150;
constexpr uint16_t kKnockBiasHighAdc = 870;
constexpr uint32_t kCanRetryMs = 2000;
constexpr uint32_t kCanCommandTimeoutMs = 3000;
constexpr uint32_t kManualTestTimeoutMs = 5000;
constexpr uint32_t kCanTxFailureLogMs = 1000;
constexpr uint32_t kEngineRuntimeTimeoutMs = 500;
constexpr uint32_t kMethStateTxMs = 50;
constexpr uint32_t kSensorExtTxMs = 250;
constexpr uint32_t kKnockStateTxMs = 50;
constexpr uint32_t kKnockHookTxMs = 50;
constexpr uint32_t kCanConfigTxMs = 1000;
constexpr uint32_t kSerialHeartbeatMs = 1000;
constexpr bool kSerialCanVerbose = false;
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

uint16_t knockCenterHz() {
  if (config.knock.autoCenterFromBore) {
    return knock_band::knockHzForBore(static_cast<uint8_t>(clampFloatValue(config.knock.boreMm, 40.0f, 200.0f) + 0.5f));
  }
  return static_cast<uint16_t>(clampFloatValue(config.knock.centerFreqHz, 2000.0f, 9000.0f));
}

knock_band::Settings knockSettings() {
  knock_band::Settings s;
  s.offset = static_cast<uint8_t>(clampFloatValue(config.knock.thresholdOffset, 0.0f, 255.0f));
  s.multiplierX10 = static_cast<uint8_t>(clampFloatValue(config.knock.thresholdMultiplier, 0.5f, 10.0f) * 10.0f + 0.5f);
  s.gainX16 = static_cast<uint8_t>(clampFloatValue(config.knock.signalGain, 0.1f, 15.9f) * 16.0f + 0.5f);
  s.holdMs = config.knock.eventCooldownMs < 100 ? 100 : config.knock.eventCooldownMs;
  return s;
}

// ---------------------------------------------------------------------------
// Pump: a logic-level MOSFET on a PWM output. Off is the state of a reset or
// unpowered ESP32 too, through the gate pull-down on the board.
// ---------------------------------------------------------------------------
//
// A MOSFET that fails usually fails switched on. The optional master relay
// (pins::PUMP_RELAY_OUT) feeds the pump its 12 V: it closes when spraying starts and
// opens a few seconds after it ends, so a failed MOSFET cannot keep the pump running.
// It never follows the PWM, so it clicks once per spray, not many times a second.
volatile uint32_t pumpSetMs = 0;
volatile bool pumpRunning = false;
volatile bool relayClosed = false;
uint32_t lastSprayMs = 0;
constexpr int32_t kPumpKeeperMs = 250;
constexpr uint32_t kRelayHoldMs = 3000;

void setPump(float dutyPercent) {
  if (!(dutyPercent > 0.0f)) dutyPercent = 0.0f;
  if (dutyPercent > 100.0f) dutyPercent = 100.0f;
  pumpDutyPercent = dutyPercent;
  const uint32_t top = (1UL << PUMP_PWM_BITS) - 1;
  const uint32_t now = millis();
  pumpSetMs = now;
  pumpRunning = dutyPercent > 0.0f;
  if (pumpRunning) {
    lastSprayMs = now;
    relayClosed = true;
  } else if (relayClosed && static_cast<uint32_t>(now - lastSprayMs) >= kRelayHoldMs) {
    relayClosed = false;
  }
  digitalWrite(pins::PUMP_RELAY_OUT, relayClosed ? HIGH : LOW);
  ledcWrite(pins::PUMP_OUT, static_cast<uint32_t>(dutyPercent * top / 100.0f + 0.5f));
}

// The PWM hardware keeps running whatever the program does, so the main loop has to
// keep asking for it: it sets the pump every 20 ms, and if that stops, the CAN task
// (another task, on the other processor core) switches the pump and the relay off
// through here - seconds before the watchdog restarts the board.
void keepPumpSafe() {
  if ((pumpRunning || relayClosed) && static_cast<int32_t>(millis() - pumpSetMs) > kPumpKeeperMs) {
    pumpRunning = false;
    relayClosed = false;
    ledcWrite(pins::PUMP_OUT, 0);
    digitalWrite(pins::PUMP_RELAY_OUT, LOW);
  }
}

// ---------------------------------------------------------------------------
// CAN: MCP2515 on SPI. A small task reads the controller every millisecond into a
// queue, so its two receive buffers are never left full while the main loop works;
// a lock keeps that task and the main loop's transmissions apart.
// ---------------------------------------------------------------------------
struct RxFrame {
  uint16_t id;
  uint8_t len;
  uint8_t data[8];
};
constexpr uint32_t kCanSpiHz = 1000000;   // Slow enough for jumper wires to the module.
MCP2515 mcp(pins::CAN_SPI_CS, kCanSpiHz, &SPI);
SemaphoreHandle_t canLock = nullptr;
QueueHandle_t canRxQueue = nullptr;
bool spiStarted = false;
bool txStuck = false;                      // The frame before never left: nobody is acknowledging.
volatile uint32_t canRxOverflows = 0;

uint8_t readRegister(uint8_t address) {
  SPI.beginTransaction(SPISettings(kCanSpiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(pins::CAN_SPI_CS, LOW);
  SPI.transfer(0x03);
  SPI.transfer(address);
  const uint8_t value = SPI.transfer(0);
  digitalWrite(pins::CAN_SPI_CS, HIGH);
  SPI.endTransaction();
  return value;
}

void modifyRegister(uint8_t address, uint8_t mask, uint8_t value) {
  SPI.beginTransaction(SPISettings(kCanSpiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(pins::CAN_SPI_CS, LOW);
  SPI.transfer(0x05);
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(value);
  digitalWrite(pins::CAN_SPI_CS, HIGH);
  SPI.endTransaction();
}

// Only what this controller acts on gets through the filters:
//   0x300-0x30F  commands, configuration and the dash's engine runtime
//   0x680-0x68F  firmware update
//   0x5E8-0x5EB, 0x5F0-0x5F3, 0x700-0x703  the ECU's broadcast (ecu_broadcast.h)
bool initCanBus() {
  xSemaphoreTake(canLock, portMAX_DELAY);
  if (!spiStarted) {
    SPI.begin(pins::CAN_SPI_SCK, pins::CAN_SPI_MISO, pins::CAN_SPI_MOSI, pins::CAN_SPI_CS);
    spiStarted = true;
  }
  bool ok = mcp.reset() == MCP2515::ERROR_OK && mcp.setBitrate(CAN_500KBPS, MCP_8MHZ) == MCP2515::ERROR_OK &&
            mcp.setFilterMask(MCP2515::MASK0, false, 0x7F0) == MCP2515::ERROR_OK &&
            mcp.setFilterMask(MCP2515::MASK1, false, 0x7FC) == MCP2515::ERROR_OK &&
            mcp.setFilter(MCP2515::RXF0, false, 0x300) == MCP2515::ERROR_OK &&
            mcp.setFilter(MCP2515::RXF1, false, fw::ID_COMMAND) == MCP2515::ERROR_OK &&
            mcp.setFilter(MCP2515::RXF2, false, 0x5F0) == MCP2515::ERROR_OK &&
            mcp.setFilter(MCP2515::RXF3, false, 0x5E8) == MCP2515::ERROR_OK &&
            mcp.setFilter(MCP2515::RXF4, false, 0x700) == MCP2515::ERROR_OK &&
            mcp.setFilter(MCP2515::RXF5, false, 0x5F0) == MCP2515::ERROR_OK &&
            mcp.setNormalMode() == MCP2515::ERROR_OK;
  // Read back: normal mode, and frames retried when the bus is lost to another module.
  ok = ok && (readRegister(0x0F) & 0xE8) == 0x00;
  txStuck = false;
  xSemaphoreGive(canLock);
  Serial.println(ok ? F("CAN: MCP2515 ready, 500 kbit/s") : F("CAN: MCP2515 not answering; will retry"));
  return ok;
}

void canReceiveTask(void *) {
  for (;;) {
    keepPumpSafe();
    if (canOnline && xSemaphoreTake(canLock, pdMS_TO_TICKS(5)) == pdTRUE) {
      if (mcp.getErrorFlags() & 0xC0) {
        mcp.clearRXnOVRFlags();
        canRxOverflows = canRxOverflows + 1;
      }
      can_frame frame;
      for (uint8_t n = 0; n < 8 && mcp.readMessage(&frame) == MCP2515::ERROR_OK; ++n) {
        if (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) continue;
        RxFrame rx;
        rx.id = static_cast<uint16_t>(frame.can_id & 0x7FF);
        rx.len = frame.can_dlc > 8 ? 8 : frame.can_dlc;
        memcpy(rx.data, frame.data, sizeof(rx.data));
        xQueueSend(canRxQueue, &rx, 0);
      }
      xSemaphoreGive(canLock);
    }
    vTaskDelay(1);
  }
}

// One transmit buffer, one frame at a time, in order. The frame before this one gets
// up to 3 ms to leave (the controller retries it by itself when another module wins the
// bus). Still waiting after that means nobody is acknowledging: it is dropped, this call
// reports the failure, and later frames are not waited for until one gets out again.
bool transmitCanFrame(const can_protocol::CanFrame &frame, const char *label) {
  if (!canOnline || xSemaphoreTake(canLock, pdMS_TO_TICKS(10)) != pdTRUE) return false;
  bool left = true;
  const uint32_t start = micros();
  while (readRegister(0x30) & 0x08) {
    if (txStuck || static_cast<uint32_t>(micros() - start) > 3000) {
      modifyRegister(0x30, 0x08, 0);
      left = false;
      break;
    }
  }
  txStuck = !left;
  can_frame out{};
  out.can_id = frame.id;
  out.can_dlc = frame.dlc > 8 ? 8 : frame.dlc;
  memcpy(out.data, frame.data, out.can_dlc);
  // The library's result only says whether the frame had already lost the bus once.
  mcp.sendMessage(MCP2515::TXB0, &out);
  xSemaphoreGive(canLock);
  if (left) {
    consecutiveCanTxFailures = 0;
    return true;
  }
  if (consecutiveCanTxFailures < 255) ++consecutiveCanTxFailures;
  const uint32_t now = millis();
  if (consecutiveCanTxFailures == 1 || elapsed(now, lastCanTxFailureLogMs, kCanTxFailureLogMs)) {
    lastCanTxFailureLogMs = now;
    Serial.printf("CAN: %s TX failed, count=%u\n", label, consecutiveCanTxFailures);
  }
  return false;
}

// A supply dip can reset the MCP2515 alone, leaving it silent in configuration mode.
// Its control register tells; so does bus-off. Either way it is set up again.
bool serviceCanHealth(uint32_t now) {
  static uint32_t lastCheckMs = 0;
  if (!elapsed(now, lastCheckMs, 1000)) return true;
  lastCheckMs = now;
  if (xSemaphoreTake(canLock, pdMS_TO_TICKS(10)) != pdTRUE) return true;
  const uint8_t control = readRegister(0x0F);
  const uint8_t errors = mcp.getErrorFlags();
  xSemaphoreGive(canLock);
  if ((control & 0xE8) == 0x00 && !(errors & 0x20) && consecutiveCanTxFailures < 64) return true;
  Serial.println(F("CAN: controller lost its settings or went bus-off; setting it up again"));
  canOnline = false;
  lastCanRetryMs = now - kCanRetryMs;
  return false;
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
  page1.center_frequency_div100 = can_protocol::clampU8((knockCenterHz() + 50U) / 100U);

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

// ---------------------------------------------------------------------------
// Saved tuning: one block in flash (NVS), checked before it is used.
// ---------------------------------------------------------------------------
constexpr uint8_t kTuneStoreVersion = 1;

bool loadTune() {
  Preferences store;
  if (!store.begin("methtune", true)) return false;
  meth_tune::Settings saved;
  const bool read = store.getUChar("version", 0) == kTuneStoreVersion &&
                    store.getBytes("values", saved.value, sizeof(saved.value)) == sizeof(saved.value);
  store.end();
  if (!read) return false;
  saved.value[0] = 0;
  if (!meth_tune::valid(saved)) return false;
  tune = saved;
  return true;
}

void saveTune() {
  Preferences store;
  if (!store.begin("methtune", false)) return;
  store.putUChar("version", kTuneStoreVersion);
  store.putBytes("values", tune.value, sizeof(tune.value));
  store.end();
}

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
    // Written from loop(), not from here. The status keeps reporting UNSAVED until it
    // has really been written.
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

// Engine speed: the ECU's own broadcast when it is arriving, else the dash's 0x309.
uint16_t currentEngineRpm(uint32_t now) {
  if (ecuData.rpmFresh(now)) {
    engineRpm = ecuData.rpm;
    engineRpmValid = true;
    lastEngineRuntimeRxMs = now;
    return engineRpm;
  }
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

// ---------------------------------------------------------------------------
// Firmware update over CAN. The image goes into the spare application slot; the
// running firmware is untouched until the restart. Injection comes first: no
// transfer starts while the pump runs or boost is near, and one under way is dropped.
// ---------------------------------------------------------------------------
struct OtaFlash {
  esp_ota_handle_t handle = 0;
  const esp_partition_t *slot = nullptr;
  bool open = false;
  bool begin(uint32_t size) {
    slot = esp_ota_get_next_update_partition(nullptr);
    if (!slot || size > slot->size) return false;
    open = esp_ota_begin(slot, OTA_WITH_SEQUENTIAL_WRITES, &handle) == ESP_OK;
    return open;
  }
  bool write(const uint8_t *data, uint32_t length) { return open && esp_ota_write(handle, data, length) == ESP_OK; }
  bool finish() {
    if (!open) return false;
    open = false;
    return esp_ota_end(handle) == ESP_OK && esp_ota_set_boot_partition(slot) == ESP_OK;
  }
  void abort() {
    if (open) esp_ota_abort(handle);
    open = false;
  }
  bool confirm() { return esp_ota_mark_app_valid_cancel_rollback() == ESP_OK; }
};
OtaFlash otaFlash;
fw::Receiver<OtaFlash> updater(fw::TARGET_WATER_METH, METH_FW_BUILD, otaFlash);
uint32_t restartAtMs = 0;
constexpr uint32_t kTrialMs = 90000;   // Unconfirmed after this long: back to the previous firmware.

bool injectionNeeded(uint32_t now) {
  if (pumpDutyPercent > 0.0f || manualTestActive) return true;
  if (config.mode == InjectionMode::Off || !ecuData.mapFresh(now)) return false;
  const float boostPsi = (ecuData.mapKpa - config.map.baroKpa) * kPsiPerKpa;
  return boostPsi >= config.boost.startPsi - 1.0f;
}

void handleUpdateFrame(const RxFrame &rx, uint32_t now) {
  if (rx.len != 8) return;
  can_protocol::CanFrame in{}, reply{};
  in.id = rx.id;
  in.dlc = 8;
  memcpy(in.data, rx.data, 8);
  if (in.data[0] == fw::BEGIN && in.data[1] == fw::TARGET_WATER_METH && in.data[5] == fw::MAGIC_0 &&
      in.data[6] == fw::MAGIC_1 && injectionNeeded(now)) {
    transmitCanFrame(fw::packAck(fw::TARGET_WATER_METH, fw::BEGIN, fw::STATUS_BUSY), "0x681");
    return;
  }
  fw::Action action = fw::Action::NONE;
  if (!updater.handle(in, reply, now, action)) return;
  transmitCanFrame(reply, "0x681");
  if (action == fw::Action::STARTED) Serial.printf("UPDATE: transfer started, %lu bytes\n", static_cast<unsigned long>(updater.size()));
  if (action == fw::Action::STOPPED) Serial.println(F("UPDATE: transfer stopped; running firmware kept"));
  if (action == fw::Action::CONFIRMED) Serial.println(F("UPDATE: new firmware confirmed by the dash"));
  if (action == fw::Action::RESTART) {
    Serial.println(F("UPDATE: firmware received and checked; restarting into it"));
    restartAtMs = (now + 300) | 1;
  }
}

// Everything the receive task has queued: ECU values, update frames, commands.
void drainCanFrames(uint32_t now) {
  RxFrame rx;
  for (uint8_t n = 0; n < 64 && xQueueReceive(canRxQueue, &rx, 0) == pdTRUE; ++n) {
    if (ecu::accept(rx.id, rx.data, rx.len, now, ecuData)) continue;
    if (rx.id == fw::ID_COMMAND) {
      handleUpdateFrame(rx, now);
      continue;
    }
    handleCanFrame(rx.id, rx.len, rx.data);
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

  if (!serviceCanHealth(now)) return;
  applyCanCommandTimeout(now);
  // Received frames were already handled by drainCanFrames() at the top of loop().

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
    state.energy = knockDetector.level();
    state.baseline = knockDetector.baseline();
    state.threshold = knockDetector.threshold();
    state.event_count = knockEventCount;
    state.last_event_rpm_div100 = lastKnockRpmDiv100;
    state.last_event_boost_kpa = lastKnockBoostKpa;
    // Spare byte: how much of the signal sits at the knock frequency, 0..255.
    state.reserved = knockDetector.share();

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
    hook.live_knock_rms = knockDetector.level();
    hook.adaptive_threshold = knockDetector.threshold();
    hook.adaptive_baseline = knockDetector.baseline();
    hook.event_count = knockEventCount;
    hook.bias_adc_div16 = can_protocol::clampU8(knockBurst.bias / 16);
    hook.raw_adc_div16 = can_protocol::clampU8(knockRaw / 16);
    // Largest swing in the burst, in half counts: for setting the amplifier gain.
    hook.envelope_level = can_protocol::clampU8(knockBurst.peak / 2);

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
    if (pumpDutyPercent > 0.0f) flags |= flag::PUMP_ON;
    if (governor.earlyStart()) flags |= flag::EARLY_START;
    if (governor.hotAir()) flags |= flag::HOT_AIR;
    if (usingEcuTemp) flags |= flag::ECU_TEMP;
    // PWM drive: the duty is reported as on-time out of a period of 1000 (per mille).
    if (!transmitCanFrame(can_protocol::packMethTuneStatus(tuneRevision, flags, governor.hold(),
                                                           static_cast<uint16_t>(pumpDutyPercent * 10.0f + 0.5f), 1000),
                          "0x30F-status")) {
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

  // One setting per pass. Each report says this controller drives the pump with PWM,
  // which is how the dash knows to show duty settings.
  if (tuneReportNext != 0) {
    const uint8_t key = tuneReportNext;
    if (transmitCanFrame(can_protocol::packMethTuneSettingReport(
            key, tune.value[key], tuneRevision,
            tuneUnsaved ? can_protocol::meth_tune_flag::UNSAVED : 0, can_protocol::meth_drive::PWM), "0x30F-setting")) {
      tuneReportNext = key < meth_tune::kCount ? key + 1 : 0;
    }
  }
}

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------
bool rawFloatLow() {
  // Default wiring: the switch closes to ground while there is fluid, so HIGH means low
  // (and so does a broken wire). config.floatActiveLow reverses it.
  return digitalRead(pins::FLOAT_SENSOR_DIGITAL) == (config.floatActiveLow ? LOW : HIGH);
}

void updatePressureReadings(SensorReadings &readings) {
  float millivolts = 0.0f;
  if (adc_sampler::averageMv(adc_sampler::kOil, millivolts)) oilPressure.update(millivolts);
  if (adc_sampler::averageMv(adc_sampler::kFuel, millivolts)) fuelPressure.update(millivolts);
  readings.oilPressureValid = adcReady && oilPressure.valid;
  readings.fuelPressureValid = adcReady && fuelPressure.valid;
  readings.oilPressurePsi = readings.oilPressureValid ? oilPressure.psi : 0.0f;
  readings.fuelPressurePsi = readings.fuelPressureValid ? fuelPressure.psi : 0.0f;
  if (!readings.oilPressureValid) readings.analogFaultFlags |= kAnalogFaultOilPressure;
  if (!readings.fuelPressureValid) readings.analogFaultFlags |= kAnalogFaultFuelPressure;
  // Not fitted on this board; reported as the Nano firmware reports an unfitted sensor.
  readings.analogFaultFlags |= kAnalogFaultMethPressure | kAnalogFaultBoostRefPressure;
}

void updateIntakeTemps(uint32_t now) {
  if (!elapsed(now, lastIntakeTempMs, kIntakeTempSampleMs)) return;
  lastIntakeTempMs = now;
  float millivolts = 0.0f, sample = 0.0f;
  // Light smoothing only: these sensors are there to show the spray cooling the air.
  if (adc_sampler::averageMv(adc_sampler::kPre, millivolts) &&
      analog_inputs::thermistorC(millivolts, pins::INTAKE_TEMP_SUPPLY_MV, pins::INTAKE_TEMP_PULLUP_OHMS, sample)) {
    preTempC = preTempValid ? preTempC + (sample - preTempC) * 0.3f : sample;
    preTempValid = true;
  } else {
    preTempValid = false;
  }
  if (adc_sampler::averageMv(adc_sampler::kPost, millivolts) &&
      analog_inputs::thermistorC(millivolts, pins::INTAKE_TEMP_SUPPLY_MV, pins::INTAKE_TEMP_PULLUP_OHMS, sample)) {
    postTempC = postTempValid ? postTempC + (sample - postTempC) * 0.3f : sample;
    postTempValid = true;
  } else {
    postTempValid = false;
  }
}
} // namespace

void setup() {
  // The pump is off before anything else happens.
  pinMode(pins::PUMP_RELAY_OUT, OUTPUT);
  digitalWrite(pins::PUMP_RELAY_OUT, LOW);
  pinMode(pins::PUMP_OUT, OUTPUT);
  digitalWrite(pins::PUMP_OUT, LOW);
  const bool pumpReady = ledcAttach(pins::PUMP_OUT, PUMP_PWM_HZ, PUMP_PWM_BITS);
  ledcWrite(pins::PUMP_OUT, 0);

  Serial.begin(config.serialBaud);
  if (!pumpReady) Serial.println(F("BOOT: pump PWM did not start; the pump will not run"));
  // Fail safe until the dash explicitly arms this controller.
  config.mode = InjectionMode::Off;
  Serial.println();
  Serial.printf("BOOT: ESP32-S3 water/meth and knock firmware, build %08lX\n", static_cast<unsigned long>(METH_FW_BUILD));

  pinMode(pins::WARNING_LED, OUTPUT);
  digitalWrite(pins::WARNING_LED, LOW);
  pinMode(pins::FLOAT_SENSOR_DIGITAL, INPUT_PULLUP);
  floatSensor.begin(rawFloatLow(), millis(), config.floatDebounceMs, config.floatLowShutdownDelayMs);

  // Saved tuning, or the conservative defaults on a new or damaged store.
  if (!loadTune()) meth_tune::loadDefaults(tune);
  syncTuneToConfig();

  // A firmware that hangs is restarted by the watchdog; if it was on trial, that
  // restart goes back to the previous one.
  enableLoopWDT();
  esp_ota_img_states_t imageState;
  const esp_partition_t *running = esp_ota_get_running_partition();
  updater.setOnTrial(running && esp_ota_get_state_partition(running, &imageState) == ESP_OK &&
                     imageState == ESP_OTA_IMG_PENDING_VERIFY);
  if (updater.onTrial()) Serial.println(F("BOOT: new firmware, on trial until the dash confirms it"));

  adcReady = adc_sampler::begin(pins::KNOCK_SENSOR_ADC, pins::OIL_PRESSURE_ADC, pins::FUEL_PRESSURE_ADC,
                                pins::PRE_INJECTION_TEMP_ADC, pins::POST_INJECTION_TEMP_ADC);
  if (!adcReady) Serial.println(F("BOOT: analog inputs did not start; knock and pressures unavailable"));

  canLock = xSemaphoreCreateMutex();
  canRxQueue = xQueueCreate(128, sizeof(RxFrame));
  canOnline = initCanBus();
  lastCanRetryMs = millis();
  xTaskCreatePinnedToCore(canReceiveTask, "can-rx", 4096, nullptr, 6, nullptr, 0);
}

void loop() {
  // Received frames first, so every time they record is at or before this pass's "now".
  drainCanFrames(millis());
  const uint32_t now = millis();
  if (restartAtMs && static_cast<int32_t>(now - restartAtMs) >= 0) {
    setPump(0.0f);
    ESP.restart();
  }

  // ---- Firmware update: nothing but the transfer, with the pump held off ----
  if (updater.active()) {
    setPump(0.0f);
    if (injectionNeeded(now)) {
      updater.cancel();
      transmitCanFrame(fw::packAck(fw::TARGET_WATER_METH, fw::ABORT, fw::STATUS_BUSY), "0x681");
      Serial.println(F("UPDATE: boost is near; transfer dropped, running firmware kept"));
    } else {
      if (updater.expired(now)) Serial.println(F("UPDATE: transfer abandoned: the dash went quiet"));
      delay(1);
      return;
    }
  }
  if (updater.onTrial() && now >= kTrialMs) {
    Serial.println(F("UPDATE: new firmware not confirmed by the dash; going back to the previous one"));
    setPump(0.0f);
    esp_ota_mark_app_invalid_rollback_and_reboot();
    updater.setOnTrial(false);   // Only reached if there is nothing to go back to.
  }

  if (!elapsed(now, lastLoopMs, config.loopPeriodMs)) {
    delay(1);
    return;
  }
  lastLoopMs = now;

  // ---- Manifold pressure and engine speed, from the ECU ----
  SensorReadings readings;
  const bool mapFresh = ecuData.mapFresh(now);
  if (ecuData.baroKnown()) {
    config.map.baroKpa = ecuData.baroKpa;
  } else if (mapFresh && ecuData.rpmFresh(now) && ecuData.rpm == 0 &&
             ecuData.mapKpa >= ecu::kMinBaroKpa && ecuData.mapKpa <= ecu::kMaxBaroKpa) {
    config.map.baroKpa = ecuData.mapKpa;   // Key on, engine stopped: the manifold is at atmospheric pressure.
  }
  readings.mapValid = mapFresh;
  readings.mapKpa = mapFresh ? ecuData.mapKpa : config.map.baroKpa;
  readings.boostPsi = (readings.mapKpa - config.map.baroKpa) * kPsiPerKpa;
  if (ecuData.intakeFresh(now)) {
    ecuIntakeC = ecuData.intakeC;
    ecuIntakeSeenMs = now;
    ecuIntakeSeen = true;
  }
  readings.tankLow = floatSensor.update(rawFloatLow(), now);
  updatePressureReadings(readings);
  updateIntakeTemps(now);
  const uint16_t rpm = currentEngineRpm(now);

  // ---- Knock: the newest burst from the ADC, judged as the Nano firmware judged it ----
  if (adc_sampler::latestBurst(knockSamples)) {
    knockBurst = knock_band::analyze(knockSamples, knock_band::centerBinFor(knockCenterHz()),
                                     config.knock.bandwidthHz < 1000.0f ? 1 : 2);
  }
  const bool knockInputDead = !adcReady || adc_sampler::silentForMs(now) > 500;
  const int knockRaw = knockSamples[knock_band::kSamples - 1];
  knockClippingDetected = knockBurst.railed >= knock_band::kSamples / 3;
  knockSensorFault = knockInputDead || knockClippingDetected || knockBurst.bias < kKnockBiasLowAdc ||
                     knockBurst.bias > kKnockBiasHighAdc;
  const bool rpmArmed = config.knock.minRpmToArm == 0 || rpm >= config.knock.minRpmToArm;
  const bool mapArmed = readings.mapKpa >= config.knock.minMapKpaToArm;
  const bool knockArmed = config.knock.enabled && rpmArmed && mapArmed && !knockSensorFault;
  knockDetector.update(now, knockBurst, knockSettings(), knockArmed, engineRpmValid ? rpm : 0);
  knockBaselineLearned = knockDetector.learned();
  knockSignalValid = knockArmed && knockBaselineLearned;
  knockWarningActive = knockDetector.warning();
  knockCriticalActive = knockDetector.critical();
  const bool knockDetected = knockWarningActive;

  if (knockWarningActive && !lastKnockWarningActive) {
    queueKnockFault(can_protocol::knock_fault_code::KNOCK_WARNING,
                    static_cast<uint8_t>(can_protocol::FaultSeverity::WARNING),
                    knockDetector.level(),
                    boostGaugeKpaForCan(readings));
  }
  if (knockCriticalActive && !lastKnockCriticalActive) {
    queueKnockFault(can_protocol::knock_fault_code::KNOCK_CRITICAL,
                    static_cast<uint8_t>(can_protocol::FaultSeverity::CRITICAL),
                    knockDetector.level(),
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

  // ---- Pump test and injection ----
  applyCanCommandTimeout(now);
  if (manualTestActive && (elapsed(now, manualTestStartMs, kManualTestTimeoutMs) ||
                           (config.tankProtectionEnabled && readings.tankLow) ||
                           config.mode != InjectionMode::Off)) {
    manualTestActive = false;
    manualTestDuty = 0;
  }
  ControlResult result = controller.update(readings, config, blend);
  // A pump test runs disarmed and for five seconds at most. It does not need the ECU
  // (it is for the bench and for measuring the nozzle), but any other fault stops it.
  if (manualTestActive && ((result.failsafe != FailsafeReason::None && result.failsafe != FailsafeReason::MapInvalid) ||
                           result.overboostAssistFaultLatched)) {
    manualTestActive = false;
    manualTestDuty = 0;
  }

  float duty = 0.0f;
  if (manualTestActive) {
    result.pump.enabled = manualTestDuty > 0;
    result.pump.dutyPercent = manualTestDuty;
    result.finalDutyPercent = manualTestDuty;
    duty = governor.manual(now, tune, manualTestDuty);
  } else {
    meth_tune::Inputs inputs;
    inputs.wantSpray = result.pump.enabled;
    inputs.idleHold = idleHoldFor(result);
    inputs.boostPsi = readings.boostPsi;
    inputs.mapKpa = readings.mapKpa;
    inputs.rpmValid = engineRpmValid;
    inputs.rpm = rpm;
    // The sensor before the nozzle when it is reading; otherwise the ECU's intake temperature.
    const bool ecuTempFresh = ecuIntakeSeen && !elapsed(now, ecuIntakeSeenMs, kEcuIntakeTimeoutMs);
    usingEcuTemp = !preTempValid && ecuTempFresh;
    inputs.preValid = preTempValid || ecuTempFresh;
    inputs.preC = preTempValid ? preTempC : ecuIntakeC;
    inputs.assistActive = result.overboostAssistActive;
    inputs.assistDutyPercent = result.finalDutyPercent;
    inputs.capDutyPercent = config.dutyMaxPercent;
    duty = governor.update(now, tune, inputs);
    // Telemetry shows what the pump is really doing.
    result.pump.enabled = duty > 0.0f;
    result.finalDutyPercent = duty;
    result.pump.dutyPercent = duty;
  }
  setPump(duty);
  const bool pumpOn = duty > 0.0f;
  if (pumpOn != lastPumpOn) {
    // Send the status frame on this pass, not up to half a second later.
    lastPumpOn = pumpOn;
    lastTuneStatusTxMs = now - can_protocol::METH_TUNE_STATUS_TX_MS;
  }
  if (tuneSavePending) {
    saveTune();
    tuneSavePending = false;
    tuneUnsaved = false;
  }
  digitalWrite(pins::WARNING_LED, (result.failsafe != FailsafeReason::None || result.overboostAssistFaultLatched ||
                                   knockDetected) ? HIGH : LOW);

  serviceCanBus(now, readings, result, knockDetected, knockRaw);

  // The ADC feeds knock and every pressure: if it has gone quiet, start again from reset.
  if (adcReady && adc_sampler::silentForMs(now) > 3000) {
    Serial.println(F("FAULT: no data from the ADC for 3 s; restarting"));
    setPump(0.0f);
    ESP.restart();
  }

  if (elapsed(now, lastDebugMs, kSerialHeartbeatMs)) {
    lastDebugMs = now;
    Serial.printf("HB t=%lu map=%.1f%s boostPsi=%.2f baro=%.1f rpm=%u%s duty=%.0f%% hold=%u mode=%u tankLow=%d can=%d "
                  "oilPsi=%.1f%s fuelPsi=%.1f%s pre=%.1f%s knock=%d bias=%u knockHz=%lu rxOvf=%lu\n",
                  static_cast<unsigned long>(now), readings.mapKpa, readings.mapValid ? "" : "!", readings.boostPsi,
                  config.map.baroKpa, rpm, engineRpmValid ? "" : "!", pumpDutyPercent, governor.hold(),
                  static_cast<unsigned>(config.mode), readings.tankLow ? 1 : 0, canOnline ? 1 : 0,
                  readings.oilPressurePsi, readings.oilPressureValid ? "" : "!", readings.fuelPressurePsi,
                  readings.fuelPressureValid ? "" : "!", preTempC, preTempValid ? "" : "!", knockDetected ? 1 : 0,
                  knockBurst.bias, static_cast<unsigned long>(adc_sampler::knockSamplesPerSecond()),
                  static_cast<unsigned long>(canRxOverflows));
  }
}
