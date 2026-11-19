// Water/meth controller PCB (rev D, Seeed XIAO ESP32-S3): docs/PCB.md. Two builds of this file:
//
//   WaterMethPCBFrogDash    (PCB_FROGDASH=1) the Frogdash dash's CAN protocol, as the car's DevKitC-1
//                   board speaks it (main_esp32.cpp): armed, tuned, tested and updated from the
//                   dash, MAP and RPM from the MegaSquirt broadcast.
//   WaterMethPCBStandalone  (PCB_FROGDASH=0) for any car, no dash: engine data from an ECU's CAN
//                   broadcast, OBD-II or a MAP sensor (engine_source.h), set up and watched
//                   from a phone over the board's own Wi-Fi (web_ui.h), or over USB serial.
//
// Both run the same pump logic as the car's board (meth_tune::PulseGovernor: full on or off
// in timed pulses, each run soft-started), with the same safety layers (a keeper on the other
// core, watchdogs, crash hold, armed again after an unasked-for reset), plus what this board
// adds: the solenoid valve, open only while the pump runs (flow_estimate.h, with an optional
// lead time), the pump's own checks from its current, the 12 V supply and the line pressure
// (pump_guard.h), flow estimated from line pressure (flow_estimate.h), software watchdogs
// (watchdogs.h) and diagnostic trouble codes kept in flash (diag.h).
#include <Arduino.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_rom_gpio.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <hal/gpio_ll.h>
#include <math.h>
#include <soc/gpio_sig_map.h>
#include <string.h>

#include "analog_inputs.h"
#include "app_config.h"
#include "diag.h"
#include "ecu_broadcast.h"
#include "flow_estimate.h"
#include "injection_controller.h"
#include "meth_tune.h"
#include "pcb_board.h"
#include "pcb_can.h"
#include "pcb_led.h"
#include "pcb_sensors.h"
#include "pump_guard.h"
#include "reset_guard.h"
#include "status_light.h"
#include "watchdogs.h"

#ifndef PCB_FROGDASH
#error "Build an environment from platformio.ini: WaterMethPCBFrogDash or WaterMethPCBStandalone"
#endif

#if PCB_FROGDASH
#include "black_box.h"
#include "can_contract/can_protocol.h"
#include "can_contract/firmware_update.h"
#else
#include <WebServer.h>
#include <WiFi.h>
#include "engine_source.h"
#include "pwm_drive.h"
#include "standalone_settings.h"
#include "web_ui.h"
#endif

#ifndef METH_FW_BUILD
#define METH_FW_BUILD 0
#endif

// New firmware stays on trial until confirmed (by the dash, or by running a minute: below).
extern "C" bool verifyRollbackLater() { return true; }

#if PCB_FROGDASH
#ifndef CAN_PROTOCOL_HAS_METH_PULSE_DRIVE
#error "Update DIYComfortControlModule: the shared CAN contract needs the pulse pump drive (extension 10)."
#endif
#ifndef CAN_PROTOCOL_HAS_METH_BLACK_BOX
#error "Update DIYComfortControlModule: the shared CAN contract needs the water/meth black box (extension 9)."
#endif
static_assert(meth_tune::kCount == can_protocol::meth_tune_setting::COUNT, "meth_tune.h does not match the shared CAN contract");
static_assert(black_box::kSampleBytes == can_protocol::meth_black_box::SAMPLE_BYTES &&
              black_box::kSlots == can_protocol::meth_black_box::MAX_SAMPLES, "black_box.h does not match the shared CAN contract");
#endif

namespace {

#if PCB_FROGDASH
const char *const kImageMark = pcb::kImageMarkFrogdash;
const char *const kBuildName = "Frogdash CAN";
#else
const char *const kImageMark = pcb::kImageMarkStandalone;
const char *const kBuildName = "stand-alone";
#endif

inline bool elapsed(uint32_t now, uint32_t start, uint32_t intervalMs) { return static_cast<uint32_t>(now - start) >= intervalMs; }
constexpr float kPsiPerKpa = 0.1450377f;
constexpr uint32_t kLoopMs = 20;
constexpr uint32_t kWatchdogMs = 2000;
constexpr int32_t kPumpKeeperMs = 250;
constexpr uint32_t kManualTestTimeoutMs = 5000;
constexpr uint32_t kTrialMs = 90000;

AppConfig config = defaultConfig();
TankBlend blend = computeTankBlend(1.5f, 0.5f);
InjectionController controller;
meth_tune::Settings tune;
meth_tune::PulseGovernor governor;
analog_inputs::FloatSwitch floatSensor;
analog_inputs::IntakeInputs intakeInputs;
pump_guard::Guard guard;
ecu::Data ecuData;
uint8_t tuneRevision = 0;
bool tuneUnsaved = false;
bool tuneSavePending = false;

// ---------------------------------------------------------------------------
// The pump and the valve. The valve opens with the pump and shuts the moment it stops, so
// the line's pressure does not dribble out of the nozzle after a pulse, and a pump switch
// that fails shorted still cannot spray. Both outputs have pull-downs on the board: off
// whenever the ESP32 is unpowered, in reset or booting.
// ---------------------------------------------------------------------------
volatile uint32_t pumpSetMs = 0;
volatile bool pumpRunning = false;
volatile uint32_t pumpKeeperTrips = 0;
float pumpDuty = 0.0f;
bool valveOpen = false;
uint32_t askedPumpCount = 0;   // What the pump PWM and the valve were last set to, for the read-back
bool askedValve = false;
RTC_NOINIT_ATTR reset_guard::Record resetRecord;
RTC_NOINIT_ATTR uint32_t crashHookMark;
constexpr uint32_t kCrashHookRan = 0x50554D50UL;   // "PUMP"
constexpr uint32_t kPumpTop = (1UL << pcb::PUMP_PWM_BITS) - 1;

// The valve opens only with the pump (never without it), and shuts the moment it stops.
void setPump(float duty, bool valve) {
  if (!(duty > 0.0f)) duty = 0.0f;
  if (duty > 100.0f) duty = 100.0f;
  pumpDuty = duty;
  pumpSetMs = millis();
  pumpRunning = duty > 0.0f;
  valveOpen = valve && pumpRunning;
  reset_guard::notePump(resetRecord, pumpRunning);
  // The top count is full on (the framework holds the pin high for it).
  askedPumpCount = static_cast<uint32_t>(duty * kPumpTop / 100.0f + 0.5f);
  askedValve = valveOpen;
  ledcWrite(pcb::PUMP, askedPumpCount);
  digitalWrite(pcb::SOLENOID, valveOpen ? HIGH : LOW);
}
void setPump(float duty) { setPump(duty, duty > 0.0f); }

// What the hardware holds now: the PWM's duty register and the valve pin's level (an OUTPUT
// pin's input stays connected, so it reads the pin itself).
uint32_t readPumpCount() {
  const uint32_t d = ledcRead(pcb::PUMP);
  return d == kPumpTop + 1 ? kPumpTop : d;   // The framework's full on
}
bool readValve() { return digitalRead(pcb::SOLENOID) == HIGH; }

// The main loop sets the pump every 20 ms; if that stops, the CAN task, on the other core,
// switches pump and valve off after kPumpKeeperMs.
void keepPumpSafe() {
  if (pumpRunning && static_cast<int32_t>(millis() - pumpSetMs) > kPumpKeeperMs) {
    pumpRunning = false;
    ledcWrite(pcb::PUMP, 0);
    gpio_ll_set_level(&GPIO, pcb::SOLENOID, 0);
    askedPumpCount = 0;
    askedValve = false;
    pumpKeeperTrips = pumpKeeperTrips + 1;
  }
}

// A crash or restart: pump and valve off first, from registers only.
void IRAM_ATTR outputsOffNow() {
  gpio_ll_set_level(&GPIO, pcb::PUMP, 0);
  esp_rom_gpio_connect_out_signal(pcb::PUMP, SIG_GPIO_OUT_IDX, false, false);
  gpio_ll_set_level(&GPIO, pcb::SOLENOID, 0);
}
void IRAM_ATTR outputsOffAtCrash(arduino_panic_info_t *, void *) {
  outputsOffNow();
  crashHookMark = kCrashHookRan;
}
void outputsOffAtRestart() { outputsOffNow(); }

// ---------------------------------------------------------------------------
// Resets (reset_guard.h): crash count across resets, the hold after repeated crashes, armed
// again after a reset nobody asked for.
// ---------------------------------------------------------------------------
reset_guard::Cause bootCause = reset_guard::Cause::Other;
bool crashHold = false;
uint32_t crashHoldEndsMs = 0;
bool armed = false;

reset_guard::Cause reportedResetCause(esp_reset_reason_t reason) {
  using reset_guard::Cause;
  switch (reason) {
  case ESP_RST_POWERON: return Cause::PowerOn;
  case ESP_RST_SW: case ESP_RST_EXT: case ESP_RST_USB: case ESP_RST_JTAG: return Cause::Requested;
  case ESP_RST_PANIC: case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: case ESP_RST_CPU_LOCKUP: return Cause::Crash;
  case ESP_RST_BROWNOUT: case ESP_RST_PWR_GLITCH: return Cause::Brownout;
  default: return Cause::Other;
  }
}

// ---------------------------------------------------------------------------
// Diagnostic trouble codes (diag.h) and the software watchdogs (watchdogs.h).
// ---------------------------------------------------------------------------
diag::Store codes;
uint16_t powerUps = 0;
uint32_t codesWrittenMs = 0;
watchdogs::PassTimer passTimer;
watchdogs::Heartbeat canTaskBeat;
watchdogs::RunLimit runLimit;
watchdogs::OutputCheck outputCheck;
watchdogs::Persist railBad, lineSensorBad, mapSensorBad, dataLost, heapLow;
volatile uint32_t canTaskCount = 0;
uint8_t watchdogFault = diag::kNone;   // A watchdog's latched fault: pump off until cleared
bool taskHold = false;                 // The CAN task has stopped: held, not latched
uint32_t heapMin = 0xFFFFFFFFUL;
uint32_t lastSlowChecksMs = 0;
bool engineDataSeen = false;
bool engineImplausible = false;
bool mapSensorFailed = false;
TaskHandle_t canTaskHandle = nullptr;
flow::Valve valve;
flow::Meter meter;
flow::PumpCurrent pumpCurrent;
float testPsiSum = 0.0f;
uint16_t testPsiCount = 0;
float testPsi = 0.0f;   // Mean line pressure in the last pump test, at full drive
bool flowFromLine = false;

void printCodes() {
  Serial.printf("CODES (power-up %u):\n", powerUps);
  bool any = false;
  for (uint8_t c = 1; c < diag::kCodeCount; ++c) {
    const diag::Entry &e = codes.entry(c);
    if (e.count == 0 && !e.active) continue;
    any = true;
    const diag::Info i = diag::infoOf(c);
    Serial.printf("  %s %s%s x%u, last at power-up %u %lu s: boost %.1f psi, %u rpm, %.2f A, line %.1f psi, %.2f V, duty %u%%\n",
                  i.id, i.text, e.active ? " (now)" : "", e.count, e.lastPowerUp, static_cast<unsigned long>(e.lastS),
                  e.freeze.boostPsiX10 / 10.0f, e.freeze.rpm, e.freeze.ampsX100 / 100.0f, e.freeze.linePsiX10 / 10.0f,
                  e.freeze.supplyMv / 1000.0f, e.freeze.dutyPct);
  }
  if (!any) Serial.println(F("  none"));
}

void clearFaults() {
  controller.clearLatchedFaults();
  guard.clear();
  watchdogFault = diag::kNone;
  runLimit.reset();
  outputCheck.reset();
  if (crashHold) Serial.println(F("SAFETY: crash hold cleared"));
  reset_guard::clearCrashes(resetRecord);
  crashHold = false;
}

void endCrashHoldWhenDue(uint32_t now) {
  if (!crashHold || static_cast<int32_t>(now - crashHoldEndsMs) < 0) return;
  reset_guard::endHold(resetRecord);
  crashHold = false;
  Serial.println(F("SAFETY: crash hold over; injection allowed again"));
}

void setArmedState(bool on);   // Below: each build keeps it its own way

// ---------------------------------------------------------------------------
// CAN: a task on core 0 carries the pump keeper and moves received frames into a queue.
// ---------------------------------------------------------------------------
QueueHandle_t canRxQueue = nullptr;
volatile bool canUp = false;
uint32_t canInits = 0;
uint32_t canSendFailures = 0;
uint32_t lastCanRxMs = 0;
uint32_t lastCanInitMs = 0;
bool canHeardSinceBoot = false;
uint32_t canBitrate = 500000;
bool canListenOnly = false;
volatile bool canRestartWanted = false;

bool startCan() {
  // The receive task waits at most 1 ms in the driver: let it come out before the driver is
  // set up again.
  canUp = false;
  delay(3);
  canUp = pcb_can::begin(canBitrate, canListenOnly);
  canInits = canInits + 1;
  lastCanInitMs = millis();
  Serial.printf("CAN: %s at %lu kbit/s%s\n", canUp ? "running" : "did not start", static_cast<unsigned long>(canBitrate / 1000),
                canListenOnly ? ", listening only" : "");
  return canUp;
}

void canTask(void *) {
  esp_task_wdt_add(nullptr);
  for (;;) {
    esp_task_wdt_reset();
    canTaskCount = canTaskCount + 1;
    keepPumpSafe();
    pcb_can::Frame f;
    if (canUp && pcb_can::receive(f, 1)) {
      xQueueSend(canRxQueue, &f, 0);
      for (uint8_t n = 0; n < 16 && pcb_can::receive(f, 0); ++n) xQueueSend(canRxQueue, &f, 0);
    } else {
      vTaskDelay(1);
    }
  }
}

bool sendFrame(uint32_t id, bool extended, const uint8_t *data, uint8_t len) {
  if (!canUp) return false;
  pcb_can::Frame f;
  f.id = id;
  f.extended = extended;
  f.len = len > 8 ? 8 : len;
  memset(f.data, 0, 8);
  memcpy(f.data, data, f.len);
  // A few milliseconds at most: the transmit queue holds 16, and a bus nobody acknowledges
  // on fills it; frames are dropped then, never waited for.
  if (pcb_can::send(f, 2)) return true;
  canSendFailures = canSendFailures + 1;
  return false;
}

// ---------------------------------------------------------------------------
// The board's inputs, read every pass (each the mean of a few conversions).
// ---------------------------------------------------------------------------
struct Board {
  float railMv{0.0f};
  bool railOk{false};
  float supplyV{0.0f};
  float amps{0.0f};
  float pressPinMv{0.0f};
  bool lineValid{false};
  float linePsi{0.0f};
  bool tankLow{true};
};
Board board;
pcb_sensors::Smooth railSmooth, supplySmooth, ampsSmooth, lineSmooth;
float preC = 0.0f, postC = 0.0f;
bool preValid = false, postValid = false;
uint32_t lastIntakeMs = 0;

float readMv(int pin, uint8_t n) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < n; ++i) sum += analogReadMilliVolts(pin);
  return static_cast<float>(sum) / n;
}

bool levelPinHigh() { return digitalRead(pcb::LEVEL) == HIGH; }

void readBoard(uint32_t now, float lineFullPsi, bool closedWhenLow) {
  board.railMv = railSmooth.take(pcb_sensors::inputMv(readMv(pcb::VSENS_ADC, 4)), 0.3f);
  board.railOk = pcb_sensors::railOk(board.railMv);
  board.supplyV = supplySmooth.take(pcb_sensors::batteryV(readMv(pcb::VBAT_ADC, 4)), 0.3f);   // Pump ripple averaged out
  // The pump's current: light smoothing, so a short circuit shows within a pass or two.
  board.amps = ampsSmooth.take(pcb_sensors::pumpAmps(readMv(pcb::ISENSE_ADC, 8)), 0.5f);
  board.pressPinMv = readMv(pcb::PRESS_ADC, 4);
  float psi = 0.0f;
  board.lineValid = lineFullPsi > 0.0f && pcb_sensors::linePsi(board.pressPinMv, board.railMv, lineFullPsi, psi);
  if (board.lineValid) board.linePsi = lineSmooth.take(psi, 0.3f); else lineSmooth.forget();
  board.tankLow = floatSensor.update(pcb_sensors::tankLow(levelPinHigh(), closedWhenLow), now);
  if (elapsed(now, lastIntakeMs, 50)) {
    lastIntakeMs = now;
    float a = 0.0f, b = 0.0f;
    const bool okA = pcb_sensors::intakeC(readMv(pcb::IAT1_ADC, 4), board.railMv, a);
    const bool okB = pcb_sensors::intakeC(readMv(pcb::IAT2_ADC, 4), board.railMv, b);
    intakeInputs.update(okA, a, okB, b);
    preValid = intakeInputs.preValid();
    preC = intakeInputs.preC();
    postValid = intakeInputs.postValid();
    postC = intakeInputs.postC();
  }
}

// ---------------------------------------------------------------------------
// Firmware into the spare application slot. Refused unless the image carries this build's
// mark: an image for the DevKitC-1 board, or for the other PCB build, never gets in.
// ---------------------------------------------------------------------------
bool imageHasMark(const esp_partition_t *slot, uint32_t size, const char *mark) {
  const size_t markLen = strlen(mark);
  static uint8_t chunk[4096 + 64];
  size_t carried = 0;
  for (uint32_t at = 0; at < size; at += 4096) {
    const uint32_t take = size - at < 4096 ? size - at : 4096;
    if (esp_partition_read(slot, at, chunk + carried, take) != ESP_OK) return false;
    const size_t have = carried + take;
    for (size_t i = 0; i + markLen <= have; ++i) {
      if (chunk[i] == static_cast<uint8_t>(mark[0]) && memcmp(chunk + i, mark, markLen) == 0) return true;
    }
    carried = have < markLen ? have : markLen - 1;
    memmove(chunk, chunk + have - carried, carried);
    esp_task_wdt_reset();
  }
  return false;
}

struct OtaFlash {
  esp_ota_handle_t handle = 0;
  const esp_partition_t *slot = nullptr;
  uint32_t written = 0;
  bool open = false;
  bool wrongImage = false;
  bool begin(uint32_t size) {
    slot = esp_ota_get_next_update_partition(nullptr);
    written = 0;
    wrongImage = false;
    if (!slot || size > slot->size) return false;
    open = esp_ota_begin(slot, OTA_WITH_SEQUENTIAL_WRITES, &handle) == ESP_OK;
    return open;
  }
  bool write(const uint8_t *data, uint32_t length) {
    if (!open || esp_ota_write(handle, data, length) != ESP_OK) return false;
    written += length;
    return true;
  }
  bool finish() {
    if (!open) return false;
    open = false;
    if (esp_ota_end(handle) != ESP_OK) return false;
    if (!imageHasMark(slot, written, kImageMark)) {
      wrongImage = true;
      Serial.printf("UPDATE: refused: not firmware for this board and build (no \"%s\" in it)\n", kImageMark);
      return false;
    }
    return esp_ota_set_boot_partition(slot) == ESP_OK;
  }
  void abort() {
    if (open) esp_ota_abort(handle);
    open = false;
  }
  bool confirm() { return esp_ota_mark_app_valid_cancel_rollback() == ESP_OK; }
};
OtaFlash otaFlash;
bool onTrial = false;
uint32_t restartAtMs = 0;

// ---------------------------------------------------------------------------
// The LED.
// ---------------------------------------------------------------------------
uint32_t ledStartMs = 0;
void showLed(uint32_t now, const status_light::Facts &facts) {
  const uint32_t since = now - ledStartMs;
  digitalWrite(pcb::LED, pcb_led::lit(status_light::classify(since, facts), since) ? LOW : HIGH);
}

// ---------------------------------------------------------------------------
// The pump test: disarmed, full pressure, for a few seconds at most.
// ---------------------------------------------------------------------------
bool manualTestActive = false;
uint8_t manualTestDuty = 0;
uint32_t manualTestStartMs = 0;
uint32_t manualTestLengthMs = kManualTestTimeoutMs;

uint8_t idleHoldFor(const ControlResult &result) {
  if (config.mode == InjectionMode::Off) return meth_tune::kHoldDisarmed;
  if (crashHold || !guard.allows() || watchdogFault != diag::kNone || taskHold) return meth_tune::kHoldFault;
  if (result.failsafe == FailsafeReason::LowFluid) return meth_tune::kHoldTankLow;
  if (result.failsafe != FailsafeReason::None) return meth_tune::kHoldFault;
  return meth_tune::kHoldBelowBoost;
}

void syncTuneToConfig() {
  config.boost.startPsi = tune.value[meth_tune::kStartPsiX10] / 10.0f;
  config.boost.fullPsi = tune.value[meth_tune::kFullPsiX10] / 10.0f;
  const float meth = static_cast<float>(tune.value[meth_tune::kMethPercent]);
  blend = computeTankBlend(100.0f - meth, meth);
}
void tuneChanged() {
  ++tuneRevision;
  tuneUnsaved = true;
  syncTuneToConfig();
}

// Settings that fail their check when loaded (a CRC that does not match, a value out of
// range): defaults are used, the board starts disarmed and a code is set (diag::kSettingsCorrupt).
bool settingsCorruptAtBoot = false;

// Store version 2, as main_esp32.cpp's (the nozzle figure), with a CRC beside the values.
bool loadTune() {
  Preferences store;
  if (!store.begin("methtune", true)) return false;
  meth_tune::Settings saved;
  const bool read = store.getUChar("version", 0) == 2 && store.getBytes("values", saved.value, sizeof(saved.value)) == sizeof(saved.value);
  const bool hasCrc = store.isKey("crc");
  const uint32_t crc = store.getULong("crc", 0);
  store.end();
  if (!read) return false;
  if ((hasCrc && crc != diag::crc32(saved.value, sizeof(saved.value))) || (saved.value[0] = 0, !meth_tune::valid(saved))) {
    settingsCorruptAtBoot = true;
    return false;
  }
  tune = saved;
  return true;
}
void saveTune() {
  Preferences store;
  if (!store.begin("methtune", false)) return;
  tune.value[0] = 0;
  store.putUChar("version", 2);
  store.putBytes("values", tune.value, sizeof(tune.value));
  store.putULong("crc", diag::crc32(tune.value, sizeof(tune.value)));
  store.end();
}

// What one control pass found, for the protocol, the web page and the LED.
struct PassView {
  float mapKpa{0.0f};
  bool mapValid{false};
  float boostPsi{0.0f};
  uint16_t rpm{0};
  bool rpmValid{false};
  ControlResult result{};
  float duty{0.0f};
};
PassView view;

#if PCB_FROGDASH
// ===========================================================================
// WaterMethPCBFrogDash: the dash's CAN protocol (shared contract, DIYComfortControlModule), as
// main_esp32.cpp speaks it. Knock is not on this board: knock commands are refused and
// 0x307 reports knock not enabled. Added from this board: line pressure in 0x303 and the
// flow status in 0x300, and the pump's own faults as 0x302 events.
// ===========================================================================
namespace fw = can_protocol::firmware;
fw::Receiver<OtaFlash, fw::RomInflate> updater(fw::TARGET_WATER_METH, METH_FW_BUILD, otaFlash);

uint8_t tuneReportNext = 0;
uint32_t lastStateTxMs = 0, lastExtTxMs = 0, lastKnockTxMs = 0, lastTuneStatusTxMs = 0, lastTuneTempsTxMs = 0;
uint32_t lastCommandMs = 0;
bool commandSeen = false;
uint16_t runtimeRpm = 0;
bool runtimeRpmValid = false;
uint32_t runtimeRpmMs = 0;
float dashIntakeC = 0.0f;
uint32_t dashIntakeMs = 0;
bool dashIntakeSeen = false;
bool usingDashTemp = false;
uint8_t faultReported = pump_guard::kNone;
bool startReportPending = true;
uint8_t startReportFlags = 0;
constexpr uint32_t kCanSilentMs = 1500;
constexpr uint32_t kCanRestartAfterMs = 15000;
constexpr float kFlowOkPsi = 20.0f;   // Line pressure that counts as flowing, at full drive

RTC_NOINIT_ATTR black_box::Box blackBox;
black_box::Recorder blackBoxRecorder;
uint8_t blackBoxLeft[black_box::kSlots][black_box::kSampleBytes];
uint8_t blackBoxLeftCount = 0;
bool blackBoxToSend = false, blackBoxSending = false;
uint8_t blackBoxFrame = 0, blackBoxTries = 0;
uint32_t blackBoxTriedMs = 0, blackBoxCanInitsSeen = 0, blackBoxSendFailuresSeen = 0, blackBoxKeeperTripsSeen = 0;

void setArmedState(bool on) {
  armed = on;
  config.mode = on ? InjectionMode::BoostOnly : InjectionMode::Off;
  reset_guard::noteArmed(resetRecord, on);
}

bool send(const can_protocol::CanFrame &f) { return sendFrame(f.id, false, f.data, f.dlc); }
bool canHeardOnce() { return canHeardSinceBoot || bootCause == reset_guard::Cause::CanRestart; }

bool injectionNeeded(uint32_t now) {
  if (pumpDuty > 0.0f || manualTestActive) return true;
  if (config.mode == InjectionMode::Off || !ecuData.mapFresh(now)) return false;
  return (ecuData.mapKpa - config.map.baroKpa) * kPsiPerKpa >= config.boost.startPsi - 1.0f;
}
bool flashWriteAllowed(uint32_t now) { return !injectionNeeded(now) && !updater.active(); }

uint16_t currentRpm(uint32_t now, bool &valid) {
  if (ecuData.rpmFresh(now)) { valid = true; return ecuData.rpm; }
  valid = runtimeRpmValid && !elapsed(now, runtimeRpmMs, 500);
  return valid ? runtimeRpm : 0;
}

void sendAck(uint8_t command, uint8_t status, uint8_t value) {
  send(can_protocol::packConfigAck(command, status, value, static_cast<uint8_t>(can_protocol::CAN_PROTOCOL_SCHEMA_VERSION)));
}

void handleTuneCommand(uint8_t len, const uint8_t *data) {
  namespace ack = can_protocol::config_ack_status;
  uint8_t status = ack::OK;
  const uint8_t subject = len >= 2 ? data[1] : 0;
  uint16_t value = 0;
  if (data[0] == can_protocol::meth_tune_command::SET_SETTING) {
    if (len != 4) status = ack::INVALID_LENGTH;
    else {
      status = meth_tune::set(tune, data[1], static_cast<uint16_t>((data[2] << 8) | data[3]), value);
      if (status != ack::UNSUPPORTED_COMMAND) tuneChanged();
    }
  } else if (len != 2) {
    status = ack::INVALID_LENGTH;
  } else if (data[1] == can_protocol::meth_tune_action::SAVE) {
    tuneSavePending = true;
  } else if (data[1] == can_protocol::meth_tune_action::REVERT) {
    if (!loadTune()) meth_tune::loadDefaults(tune);
    tuneChanged();
    tuneUnsaved = tuneSavePending = false;
  } else if (data[1] == can_protocol::meth_tune_action::DEFAULTS) {
    meth_tune::loadDefaults(tune);
    tuneChanged();
  } else if (data[1] == can_protocol::meth_tune_action::REPORT) {
    tuneReportNext = 1;
  } else {
    status = ack::UNSUPPORTED_COMMAND;
  }
  send(can_protocol::packMethTuneAck(data[0], status, subject, value, tuneRevision));
}

void handleConfigBroadcast(uint8_t len, const uint8_t *data) {
  if (len < 8) return;
  uint8_t sum = 0;
  for (uint8_t i = 0; i < 7; ++i) sum ^= data[i];
  if (sum != data[7]) {
    send(can_protocol::packMethConfigAck(data[0], 1, 1, can_protocol::clampU8(static_cast<int>(blend.methPercent + 0.5f))));
    return;
  }
  setArmedState(data[1] != 0U);
  config.tankProtectionEnabled = (data[6] & 0x01U) != 0U;
  if (config.mode != InjectionMode::Off) { manualTestActive = false; manualTestDuty = 0; }
  if (data[2] <= 100U && data[2] != tune.value[meth_tune::kMethPercent]) {
    tune.value[meth_tune::kMethPercent] = data[2];
    tuneChanged();
  }
  if (data[3] > 0U) {
    uint16_t applied = 0;
    const float startPsi = (static_cast<float>(data[3]) - config.map.baroKpa) * kPsiPerKpa;
    const float x10 = startPsi * 10.0f + 0.5f;
    meth_tune::Settings next = tune;
    meth_tune::set(next, meth_tune::kStartPsiX10, x10 < 0.0f ? 0 : static_cast<uint16_t>(x10), applied);
    if (applied != tune.value[meth_tune::kStartPsiX10]) { tune = next; tuneChanged(); }
  }
  send(can_protocol::packMethConfigAck(data[0], 0, 0, can_protocol::clampU8(static_cast<int>(blend.methPercent + 0.5f))));
}

void handleCommand(uint8_t len, const uint8_t *data, uint32_t now) {
  namespace ack = can_protocol::config_ack_status;
  if (len < 1) { sendAck(0, ack::INVALID_LENGTH, 0); return; }
  const uint8_t cmd = data[0];
  if (cmd == can_protocol::meth_tune_command::LIVE_INTAKE_TEMP) {
    if (len == 3) {
      dashIntakeC = static_cast<int16_t>((static_cast<uint16_t>(data[1]) << 8) | data[2]) / 10.0f;
      dashIntakeMs = now;
      dashIntakeSeen = true;
    }
    return;
  }
  if (cmd == can_protocol::meth_tune_command::BLACK_BOX_RECEIVED) {
    blackBoxToSend = blackBoxSending = false;
    return;
  }
  if (cmd == can_protocol::meth_tune_command::SET_SETTING || cmd == can_protocol::meth_tune_command::SETTINGS_ACTION) {
    handleTuneCommand(len, data);
    return;
  }
  uint8_t status = ack::OK;
  uint8_t value = len >= 2 ? data[1] : 0;
  switch (cmd) {
  case can_protocol::meth_command::ARM:
    if (len >= 2) { setArmedState(data[1] != 0U); manualTestActive = false; manualTestDuty = 0; }
    else status = ack::INVALID_LENGTH;
    break;
  case can_protocol::meth_command::MANUAL_TEST_DUTY:
    if (len < 2) { status = ack::INVALID_LENGTH; break; }
    if (manualTestActive || config.mode != InjectionMode::Off || crashHold || !guard.allows() || watchdogFault != diag::kNone || taskHold ||
        (config.tankProtectionEnabled && floatSensor.isLow())) {
      status = ack::VALUE_CLAMPED;
      value = 0;
      break;
    }
    manualTestActive = true;
    manualTestDuty = data[1] > 100 ? 100 : data[1];
    if (data[1] > 100) status = ack::VALUE_CLAMPED;
    manualTestStartMs = now;
    manualTestLengthMs = kManualTestTimeoutMs;
    value = manualTestDuty;
    break;
  case can_protocol::meth_command::STOP_MANUAL_TEST:
    manualTestActive = false;
    manualTestDuty = 0;
    break;
  case can_protocol::meth_command::SET_BOOST_TRIGGER:
    if (len >= 2) {
      uint16_t applied = 0;
      meth_tune::Settings next = tune;
      meth_tune::set(next, meth_tune::kStartPsiX10, static_cast<uint16_t>(data[1] * kPsiPerKpa * 10.0f + 0.5f), applied);
      if (applied != tune.value[meth_tune::kStartPsiX10]) { tune = next; tuneChanged(); }
    } else {
      status = ack::INVALID_LENGTH;
    }
    break;
  case can_protocol::meth_command::CLEAR_FAULTS:
    clearFaults();
    faultReported = pump_guard::kNone;
    break;
  default:
    status = ack::UNSUPPORTED_COMMAND;   // Knock commands too: this board has no knock input.
    break;
  }
  sendAck(cmd, status, value);
}

void handleUpdateFrame(const pcb_can::Frame &rx, uint32_t now) {
  if (rx.len != 8 || rx.extended) return;
  can_protocol::CanFrame in{}, reply{};
  in.id = static_cast<uint16_t>(rx.id);
  in.dlc = 8;
  memcpy(in.data, rx.data, 8);
  if ((in.data[0] == fw::BEGIN || in.data[0] == fw::RESTART) && in.data[1] == fw::TARGET_WATER_METH && injectionNeeded(now)) {
    send(fw::packAck(fw::TARGET_WATER_METH, in.data[0], fw::STATUS_BUSY));
    return;
  }
  fw::Action action = fw::Action::NONE;
  if (!updater.handle(in, reply, now, action)) return;
  send(reply);
  if (action == fw::Action::STARTED) Serial.printf("UPDATE: transfer started, %lu bytes\n", static_cast<unsigned long>(updater.size()));
  if (action == fw::Action::CONFIRMED) Serial.println(F("UPDATE: new firmware confirmed by the dash"));
  if (action == fw::Action::RESTART) restartAtMs = (now + 300) | 1;
}

void drainCan(uint32_t now) {
  pcb_can::Frame rx;
  for (uint8_t n = 0; n < 64 && xQueueReceive(canRxQueue, &rx, 0) == pdTRUE; ++n) {
    lastCanRxMs = now;
    canHeardSinceBoot = true;
    if (rx.extended) continue;
    const uint16_t id = static_cast<uint16_t>(rx.id);
    if (ecu::accept(id, rx.data, rx.len, now, ecuData)) continue;
    if (id == fw::ID_COMMAND) { handleUpdateFrame(rx, now); continue; }
    if (id == can_protocol::ID_MASTER_HEARTBEAT) { commandSeen = true; lastCommandMs = now; continue; }
    if (id == can_protocol::ID_METH_CONFIG_BROADCAST) { commandSeen = true; lastCommandMs = now; handleConfigBroadcast(rx.len, rx.data); continue; }
    if (id == can_protocol::ID_ENGINE_RUNTIME && rx.len >= 2) {
      runtimeRpm = static_cast<uint16_t>(rx.data[0] | (rx.data[1] << 8));
      runtimeRpmValid = rx.len < 4 || (rx.data[3] & 1U);
      runtimeRpmMs = now;
      continue;
    }
    if (id == can_protocol::ID_ENGINE_METH_COMMAND) { commandSeen = true; lastCommandMs = now; handleCommand(rx.len, rx.data, now); }
  }
}

// Silent bus (the controller or its wiring stopped): set up again after 1.5 s, the board
// restarted after 15 s, three times per ignition at most; only once it has worked.
void serviceCanHealth(uint32_t now) {
  const bool silent = canHeardOnce() && elapsed(now, lastCanRxMs, kCanSilentMs) && elapsed(now, lastCanInitMs, 2000);
  if (!pcb_can::service() && elapsed(now, lastCanInitMs, 2000)) {
    if (!canUp || !pcb_can::running()) startCan();
  } else if (silent) {
    Serial.printf("CAN: nothing heard for %lu ms; starting the controller again\n", static_cast<unsigned long>(now - lastCanRxMs));
    startCan();
  }
  if (canHeardOnce() && elapsed(now, lastCanRxMs, kCanRestartAfterMs) && reset_guard::mayRestartForCan(resetRecord)) {
    Serial.println(F("FAULT: nothing on CAN for 15 s; restarting the board"));
    setPump(0.0f);
    reset_guard::markCanRestart(resetRecord);
    ESP.restart();
  }
}

uint8_t methState(const ControlResult &r) {
  using can_protocol::MethState;
  if (manualTestActive) return static_cast<uint8_t>(MethState::TEST);
  // A low supply (cranking) only holds: it is not a fault to show on the dash.
  if (crashHold || guard.fault() != pump_guard::kNone || watchdogFault != diag::kNone || taskHold || r.failsafe != FailsafeReason::None ||
      r.overboostAssistFaultLatched) return static_cast<uint8_t>(MethState::FAULT);
  if (pumpDuty > 0.0f) return static_cast<uint8_t>(MethState::SPRAYING);
  if (config.mode != InjectionMode::Off && (!config.tankProtectionEnabled || !board.tankLow)) return static_cast<uint8_t>(MethState::ARMED);
  return static_cast<uint8_t>(MethState::OFF);
}

uint8_t flowStatus() {
  using can_protocol::FlowStatus;
  if (guard.fault() == pump_guard::kNoPressure || guard.fault() == pump_guard::kNotConnected) return static_cast<uint8_t>(FlowStatus::NO_FLOW);
  if (guard.fault() == pump_guard::kOverPressure) return static_cast<uint8_t>(FlowStatus::LOW_FLOW);
  if (!board.lineValid || pumpDuty < 99.5f) return static_cast<uint8_t>(FlowStatus::UNKNOWN);
  // Full drive and the line holding pressure: fluid is going through the nozzle.
  return board.linePsi >= kFlowOkPsi ? static_cast<uint8_t>(FlowStatus::OK) : static_cast<uint8_t>(FlowStatus::UNKNOWN);
}

// `fault` is a diag::Code: the pump's own (1-6, pump_guard::Fault's numbers) or a watchdog's,
// which goes as SAFETY_SHUTDOWN with the code in byte 2.
void reportPumpFault(uint8_t fault) {
  namespace code = can_protocol::meth_fault_code;
  uint8_t c = code::SAFETY_SHUTDOWN;
  if (fault == pump_guard::kShort || fault == pump_guard::kOverCurrent) c = code::PUMP_OVERCURRENT;
  if (fault == pump_guard::kNotConnected || fault == pump_guard::kNoPressure) c = code::NO_FLOW;
  if (fault == pump_guard::kOverPressure) c = code::LOW_FLOW;
  can_protocol::CanFrame f{};
  f.id = can_protocol::ID_ENGINE_METH_FAULT;
  f.dlc = 4;
  f.data[0] = c;
  f.data[1] = static_cast<uint8_t>(can_protocol::FaultSeverity::CRITICAL);
  f.data[2] = fault;   // pump_guard::Fault
  f.data[3] = can_protocol::clampU8(static_cast<int>(fault == pump_guard::kNoPressure || fault == pump_guard::kOverPressure
                                                         ? board.linePsi * 0.5f : board.amps * 10.0f));
  if (send(f)) faultReported = fault;
}

void serviceBlackBox(uint32_t now) {
  if (!blackBoxToSend || now < 2000) return;
  if (!blackBoxSending) {
    if (blackBoxTries >= 6) { blackBoxToSend = false; return; }
    if (blackBoxTries > 0 && !elapsed(now, blackBoxTriedMs, 5000)) return;
    blackBoxSending = true;
    blackBoxFrame = 0;
  }
  const uint16_t total = static_cast<uint16_t>(blackBoxLeftCount * black_box::kSampleBytes);
  const uint8_t frames = static_cast<uint8_t>((total + can_protocol::meth_black_box::BYTES_PER_FRAME - 1) / can_protocol::meth_black_box::BYTES_PER_FRAME);
  for (uint8_t n = 0; n < 4 && blackBoxSending; ++n) {
    if (blackBoxFrame == 0) {
      send(can_protocol::packMethBlackBoxHeader(blackBoxLeftCount, black_box::kWindowMs / 10, static_cast<uint8_t>(esp_reset_reason()), frames,
                                                black_box::sumOf(blackBoxLeft, blackBoxLeftCount)));
    } else {
      send(can_protocol::packMethBlackBoxData(static_cast<uint8_t>(blackBoxFrame - 1), &blackBoxLeft[0][0], total));
    }
    if (++blackBoxFrame > frames) { blackBoxSending = false; ++blackBoxTries; blackBoxTriedMs = now; }
  }
}

void recordBlackBox(uint32_t now, uint32_t gapMs) {
  black_box::Pass p;
  p.dutyPct = pumpDuty;
  p.ecuFresh = view.mapValid;
  p.mapKpa = view.mapKpa;
  p.rpmValid = view.rpmValid;
  p.rpm = view.rpm;
  p.hold = manualTestActive ? static_cast<uint8_t>(meth_tune::kHoldNone) : governor.hold();
  p.armed = config.mode != InjectionMode::Off;
  p.softStarting = governor.softStarting(now);
  p.manualTest = manualTestActive;
  p.crashHold = crashHold;
  p.tankLow = board.tankLow;
  p.canOnline = canUp;
  p.canSetUpAgain = canInits != blackBoxCanInitsSeen;
  p.canSendFailed = canSendFailures != blackBoxSendFailuresSeen;
  p.keeperTripped = pumpKeeperTrips != blackBoxKeeperTripsSeen;
  blackBoxCanInitsSeen = canInits;
  blackBoxSendFailuresSeen = canSendFailures;
  blackBoxKeeperTripsSeen = pumpKeeperTrips;
  p.canRxAgeMs = canHeardSinceBoot ? now - lastCanRxMs : now;
  p.loopGapMs = gapMs;
  p.preValid = preValid;
  p.preC = preC;
  p.freeHeapBytes = ESP.getFreeHeap();
  blackBoxRecorder.record(blackBox, now, p);
}

void sendTelemetry(uint32_t now) {
  if (startReportPending && now >= 1000 && send(can_protocol::packMethStartReport(static_cast<uint8_t>(esp_reset_reason()), startReportFlags))) {
    startReportPending = false;
  }
  serviceBlackBox(now);
  if (guard.fault() != pump_guard::kNone && guard.fault() != faultReported) reportPumpFault(guard.fault());
  else if (guard.fault() == pump_guard::kNone && watchdogFault != diag::kNone && watchdogFault != faultReported) reportPumpFault(watchdogFault);
  const ControlResult &r = view.result;
  if (elapsed(now, lastStateTxMs, 50)) {
    lastStateTxMs = now;
    can_protocol::EngineMethState s{};
    s.meth_state = methState(r);
    s.pump_duty = manualTestActive ? manualTestDuty : can_protocol::clampU8(static_cast<int>(pumpDuty + 0.5f));
    s.tank_level = board.tankLow ? 0 : 100;
    s.flow_status = flowStatus();
    const float gauge = view.mapKpa - config.map.baroKpa;
    s.boost_kpa = can_protocol::clampU8(static_cast<int>(gauge > 0.0f ? gauge + 0.5f : 0.0f));
    uint8_t flags = 0;
    if ((config.tankProtectionEnabled && board.tankLow) || r.failsafe == FailsafeReason::LowFluid) flags |= 1U << 0;
    if (!view.mapValid) flags |= 1U << 1;
    if (r.overboostAssistActive) flags |= 1U << 2;
    if (r.overboostEmergencyActive || r.overboostAssistFaultLatched) flags |= 1U << 3;
    if (r.failsafe == FailsafeReason::InvalidBlend || r.failsafe == FailsafeReason::InvalidBoostConfig) flags |= 1U << 4;
    if (crashHold) flags |= can_protocol::meth_fault_flag::CRASH_HOLD;
    s.fault_flags = flags;
    send(can_protocol::packEngineMethState(s));
  }
  if (elapsed(now, lastExtTxMs, 250)) {
    lastExtTxMs = now;
    can_protocol::EngineSensorExt ext{};
    ext.meth_pressure_psi_x2 = board.lineValid ? can_protocol::clampU8(static_cast<int>(board.linePsi * 2.0f + 0.5f)) : 0;
    namespace af = can_protocol::analog_sensor_fault;
    ext.analog_fault_flags = af::OIL | af::FUEL | af::BOOST_REF | af::IAT | af::ENGINE_BAY | af::AMBIENT | af::CABIN;
    if (!board.lineValid) ext.analog_fault_flags |= af::METH;
    send(can_protocol::packEngineSensorExt(ext));
  }
  if (elapsed(now, lastKnockTxMs, 50)) {
    lastKnockTxMs = now;
    can_protocol::EngineKnockState k{};   // Not enabled: no knock input on this board
    send(can_protocol::packEngineKnockState(k));
  }
  if (elapsed(now, lastTuneStatusTxMs, can_protocol::METH_TUNE_STATUS_TX_MS)) {
    lastTuneStatusTxMs = now;
    namespace flag = can_protocol::meth_tune_flag;
    uint8_t flags = 0;
    if (tuneUnsaved) flags |= flag::UNSAVED;
    if (view.rpmValid && view.rpm >= tune.value[meth_tune::kMinRpm]) flags |= flag::RPM_OK;
    if (preValid) flags |= flag::PRE_VALID;
    if (postValid) flags |= flag::POST_VALID;
    if (pumpDuty > 0.0f) flags |= flag::PUMP_ON;
    if (governor.earlyStart()) flags |= flag::EARLY_START;
    if (governor.hotAir()) flags |= flag::HOT_AIR;
    if (usingDashTemp) flags |= flag::ECU_TEMP;
    send(can_protocol::packMethTuneStatus(tuneRevision, flags, governor.hold(), governor.onMs(), tune.value[meth_tune::kPeriodMs]));
  }
  if (elapsed(now, lastTuneTempsTxMs, can_protocol::METH_TUNE_TEMPS_TX_MS)) {
    lastTuneTempsTxMs = now;
    send(can_protocol::packMethTuneTemps(static_cast<int16_t>(preValid ? preC * 10.0f : 0.0f), static_cast<int16_t>(postValid ? postC * 10.0f : 0.0f),
                                         static_cast<uint8_t>((preValid ? 1U : 0U) | (postValid ? 2U : 0U))));
  }
  if (tuneReportNext != 0) {
    const uint8_t key = tuneReportNext;
    if (send(can_protocol::packMethTuneSettingReport(key, tune.value[key], tuneRevision, tuneUnsaved ? can_protocol::meth_tune_flag::UNSAVED : 0,
                                                     can_protocol::meth_drive::PULSE))) {
      tuneReportNext = key < meth_tune::kCount ? key + 1 : 0;
    }
  }
}

// Engine data for this pass: the MegaSquirt broadcast, with the dash's 0x309 as the RPM backup.
void engineData(uint32_t now, meth_tune::Inputs &in) {
  if (ecuData.baroKnown()) config.map.baroKpa = ecuData.baroKpa;
  else if (ecuData.mapFresh(now) && ecuData.rpmFresh(now) && ecuData.rpm == 0 && ecuData.mapKpa >= ecu::kMinBaroKpa &&
           ecuData.mapKpa <= ecu::kMaxBaroKpa) config.map.baroKpa = ecuData.mapKpa;
  view.mapValid = ecuData.mapFresh(now);
  view.mapKpa = view.mapValid ? ecuData.mapKpa : config.map.baroKpa;
  bool rpmValid = false;
  view.rpm = currentRpm(now, rpmValid);
  view.rpmValid = rpmValid;
  const bool ecuTemp = ecuData.intakeFresh(now);
  const bool dashTemp = dashIntakeSeen && !elapsed(now, dashIntakeMs, 1500);
  usingDashTemp = !preValid && (ecuTemp || dashTemp);
  in.preValid = preValid || ecuTemp || dashTemp;
  in.preC = preValid ? preC : ecuTemp ? ecuData.intakeC : dashIntakeC;
}

void setupBuild() {
  // Disarmed until the dash arms it, but armed again after a reset nobody asked for.
  if (!loadTune()) meth_tune::loadDefaults(tune);
  syncTuneToConfig();
  canBitrate = 500000;
  canListenOnly = false;
  esp_ota_img_states_t state;
  const esp_partition_t *running = esp_ota_get_running_partition();
  onTrial = running && esp_ota_get_state_partition(running, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY;
  updater.setOnTrial(onTrial);
}

void blackBoxAtStart(bool pumpWasOn, bool armedAgain, uint32_t softStartMs) {
  using reset_guard::Cause;
  const bool unasked = bootCause == Cause::Crash || bootCause == Cause::Brownout || bootCause == Cause::FaultRestart ||
                       bootCause == Cause::CanRestart || bootCause == Cause::Other;
  blackBoxLeftCount = unasked ? black_box::take(blackBox, blackBoxLeft) : 0;
  blackBoxToSend = blackBoxLeftCount > 0;
  black_box::forget(blackBox);
  blackBoxRecorder.start(blackBox, millis());
  namespace sf = can_protocol::meth_start_flag;
  startReportFlags = static_cast<uint8_t>((pumpWasOn ? sf::PUMP_WAS_ON : 0U) | (armedAgain ? sf::ARMED_AGAIN : 0U) |
                                          (crashHold ? sf::CRASH_HOLD : 0U) | (onTrial ? sf::ON_TRIAL : 0U) |
                                          (bootCause == Cause::CanRestart ? sf::RESTART_SILENT_CAN : 0U) |
                                          (softStartMs > meth_tune::kSoftStartMs ? sf::GENTLE_START : 0U));
}

// The transfer, with the pump held off. Returns true while it runs (the loop does nothing else).
bool serviceUpdate(uint32_t now) {
  if (!updater.active()) return false;
  setPump(0.0f);
  if (injectionNeeded(now)) {
    updater.cancel();
    send(fw::packAck(fw::TARGET_WATER_METH, fw::ABORT, fw::STATUS_BUSY));
    return false;
  }
  updater.expired(now);
  return true;
}

void trialCheck(uint32_t now) {
  if (updater.onTrial() && now >= kTrialMs) {
    Serial.println(F("UPDATE: not confirmed by the dash; going back to the previous firmware"));
    setPump(0.0f);
    esp_ota_mark_app_invalid_rollback_and_reboot();
    updater.setOnTrial(false);
  }
}

void serviceBuild(uint32_t now, uint32_t gapMs) {
  if (tuneSavePending && flashWriteAllowed(now)) { saveTune(); tuneSavePending = tuneUnsaved = false; }
  serviceCanHealth(now);
  sendTelemetry(now);
  recordBlackBox(now, gapMs);
}

bool tankProtection() { return config.tankProtectionEnabled; }
bool closedWhenLow() { return config.floatActiveLow; }
float lineFullScale() { return 250.0f; }   // The board's 250 psi sensor (README)
pump_guard::Pass guardLimits() { pump_guard::Pass p; p.pressureFitted = true; p.minPsi = 0.0f; p.maxPsi = 0.0f; return p; }
// The dash's protocol has no settings for these: the valve opens with the pump, the flow
// is the nozzle figure while it is open, and nothing is counted against a tank.
uint32_t valveLeadMs() { return 0; }
bool ownSettingsValid() { return true; }
void reloadSettings() {
  if (!loadTune()) meth_tune::loadDefaults(tune);
  syncTuneToConfig();
}
float nozzleRatedPsi() { return 0.0f; }
bool pwmDriveChosen() { return false; }
void persistUsedMl(uint32_t) {}
bool rpmRequired() { return true; }
float fuelScale() { return 1.0f; }
// The dash's protocol has the pulse drive only (meth_drive::PULSE).
float driveDuty(uint32_t, const meth_tune::Inputs &, float pulseDuty) { return pulseDuty; }
float steadyFromPct() { return 99.5f; }
void setDriveSoftStart(uint32_t) {}
bool setupFault() { return false; }
bool waitingForData(uint32_t now) { return !canHeardOnce() || (canHeardOnce() && elapsed(now, lastCanRxMs, kCanSilentMs)); }

#else
// ===========================================================================
// WaterMethPCBStandalone: any car. Settings in standalone_settings.h, kept in flash; the web page
// and USB serial commands change them (web_ui.h). Armed state is a setting, kept through
// power cycles: there is no dash to arm it.
// ===========================================================================
standalone::Settings own;
bool ownUnsaved = false;
bool ownSavePending = false;
engine::Obd2 obd;
engine::Barometer barometer;
const engine::Profile *profile = nullptr;
uint32_t ecuFrames = 0;            // Frames from the engine source since start-up
uint32_t lastEcuFrameMs = 0;
bool mapFromSensor = false;
WebServer server(80);
bool wifiOn = false;
uint32_t wifiClientSeenMs = 0;
String wifiPassword;
String ssid;
bool uploadRefused = false;
const char *uploadProblem = nullptr;
constexpr uint32_t kWifiIdleOffMs = 15UL * 60UL * 1000UL;
constexpr uint32_t kConfirmAfterMs = 60000;   // Web-updated firmware: kept once it has run a minute

void setArmedState(bool on) {
  armed = on;
  config.mode = on ? InjectionMode::BoostOnly : InjectionMode::Off;
  reset_guard::noteArmed(resetRecord, on);
  if (own.value[standalone::kArmed] != (on ? 1 : 0)) {
    own.value[standalone::kArmed] = on ? 1 : 0;
    ownSavePending = true;   // Kept through power cycles; written away from boost
  }
}

bool injectionNeeded(uint32_t now) {
  if (pumpDuty > 0.0f || manualTestActive) return true;
  return config.mode != InjectionMode::Off && view.mapValid && view.boostPsi >= config.boost.startPsi - 1.0f;
}
bool flashWriteAllowed(uint32_t now) { return !injectionNeeded(now); }

void applyOwn(bool restartCan);   // Below
float usedMlSaved = 0.0f;         // The fluid count as last written
uint32_t usedMlSavedMs = 0;

// Version 1, with a CRC beside the values. Values saved before settings 20-23 existed are
// taken, and the new ones set to their defaults.
bool loadOwn() {
  Preferences store;
  if (!store.begin("standalone", true)) return false;
  standalone::Settings saved;
  standalone::loadDefaults(saved);
  const size_t stored = store.getBytesLength("values");
  const bool sizeOk = stored == sizeof(saved.value) || stored == (standalone::kCountV1 + 1) * sizeof(uint16_t);
  const bool read = store.getUChar("version", 0) == 1 && sizeOk && store.getBytes("values", saved.value, stored) == stored;
  const bool hasCrc = store.isKey("crc");
  const uint32_t crc = store.getULong("crc", 0);
  wifiPassword = store.getString("wifipass", "");
  meter.setUsed(store.getFloat("usedml", 0.0f));
  usedMlSaved = meter.usedMl();
  store.end();
  if (!read) return false;
  if ((hasCrc && crc != diag::crc32(saved.value, stored)) || (saved.value[0] = 0, !standalone::valid(saved))) {
    settingsCorruptAtBoot = true;
    return false;
  }
  own = saved;
  return true;
}
void saveOwn() {
  Preferences store;
  if (!store.begin("standalone", false)) return;
  own.value[0] = 0;
  store.putUChar("version", 1);
  store.putBytes("values", own.value, sizeof(own.value));
  store.putULong("crc", diag::crc32(own.value, sizeof(own.value)));
  store.end();
}

// The fluid count, kept away from boost every few minutes (and at a refill), not each pass.
void saveUsedMl() {
  Preferences store;
  if (!store.begin("standalone", false)) return;
  store.putFloat("usedml", meter.usedMl());
  store.end();
  usedMlSaved = meter.usedMl();
}
void persistUsedMl(uint32_t now) {
  if (fabsf(meter.usedMl() - usedMlSaved) < 5.0f || !elapsed(now, usedMlSavedMs, 300000UL) || !flashWriteAllowed(now)) return;
  usedMlSavedMs = now;
  saveUsedMl();
}

// ---- Tune profiles: four named slots, each the whole pulse tuning plus the settings
// standalone::inProfile() picks (how it sprays, not how the car is wired), with a CRC. ----
constexpr uint8_t kProfileSlots = 4;
struct ProfileSlot {
  uint8_t version;
  char name[21];
  uint16_t tune[meth_tune::kCount + 1];
  uint16_t own[standalone::kCount + 1];
  uint32_t crc;
};
const char *const kSlotDefaultNames[kProfileSlots] = {"Street", "Track", "Race", "Winter"};

bool readSlot(uint8_t n, ProfileSlot &slot) {
  if (n >= kProfileSlots) return false;
  Preferences store;
  if (!store.begin("profiles", true)) return false;
  char key[4] = {'p', static_cast<char>('0' + n), 0};
  const bool read = store.getBytes(key, &slot, sizeof(slot)) == sizeof(slot);
  store.end();
  return read && slot.version == 1 && slot.crc == diag::crc32(&slot, sizeof(slot) - sizeof(slot.crc));
}

const char *saveSlot(uint8_t n, const char *name) {
  if (n >= kProfileSlots) return "no such profile";
  ProfileSlot slot;
  memset(&slot, 0, sizeof(slot));
  slot.version = 1;
  strncpy(slot.name, name && name[0] ? name : kSlotDefaultNames[n], sizeof(slot.name) - 1);
  for (char *c = slot.name; *c; ++c) if (*c == '"' || *c == '\\' || static_cast<uint8_t>(*c) < 32) *c = ' ';
  memcpy(slot.tune, tune.value, sizeof(slot.tune));
  memcpy(slot.own, own.value, sizeof(slot.own));
  slot.crc = diag::crc32(&slot, sizeof(slot) - sizeof(slot.crc));
  Preferences store;
  if (!store.begin("profiles", false)) return "flash not available";
  char key[4] = {'p', static_cast<char>('0' + n), 0};
  const bool ok = store.putBytes(key, &slot, sizeof(slot)) == sizeof(slot);
  store.end();
  return ok ? nullptr : "flash write failed";
}

const char *loadSlot(uint8_t n, uint32_t now) {
  if (pumpDuty > 0.0f || manualTestActive) return "not while spraying";
  ProfileSlot slot;
  if (!readSlot(n, slot)) return "that profile is empty or damaged";
  meth_tune::Settings next;
  memcpy(next.value, slot.tune, sizeof(next.value));
  next.value[0] = 0;
  if (!meth_tune::valid(next)) return "that profile is damaged";
  tune = next;
  tuneChanged();
  for (uint8_t key = 1; key <= standalone::kCount; ++key) {
    uint16_t applied = 0;
    if (standalone::inProfile(key)) standalone::set(own, key, slot.own[key], applied);
  }
  ownUnsaved = true;
  tuneSavePending = ownSavePending = true;   // Loading a profile keeps it
  applyOwn(true);
  Serial.printf("PROFILE: \"%s\" loaded\n", slot.name);
  (void)now;
  return nullptr;
}
void saveWifiPassword(const String &pass) {
  Preferences store;
  if (!store.begin("standalone", false)) return;
  store.putString("wifipass", pass);
  store.end();
  wifiPassword = pass;
}

// Applies the settings that decide how the board listens: the ECU, the CAN speed and mode.
void applyOwn(bool restartCan) {
  using namespace standalone;
  profile = &engine::kProfiles[own.value[kProfile] < engine::kProfileCount ? own.value[kProfile] : 0];
  obd.setExtended(own.value[kSource] == kSourceObd29);
  mapFromSensor = own.value[kMapFrom] == 1;
  barometer.setFallback(own.value[kBaroX10] / 10.0f);
  config.tankProtectionEnabled = own.value[kTankProtection] != 0;
  config.floatActiveLow = own.value[kFloatClosedWhenLow] != 0;
  const uint32_t bitrate = bitrateOf(own);
  const bool listen = listenOnlyOf(own);
  if (restartCan && (bitrate != canBitrate || listen != canListenOnly || !canUp)) {
    canBitrate = bitrate;
    canListenOnly = listen;
    startCan();
  }
  canBitrate = bitrate;
  canListenOnly = listen;
}

void drainCan(uint32_t now) {
  pcb_can::Frame rx;
  const uint8_t src = static_cast<uint8_t>(own.value[standalone::kSource]);
  for (uint8_t n = 0; n < 64 && xQueueReceive(canRxQueue, &rx, 0) == pdTRUE; ++n) {
    lastCanRxMs = now;
    canHeardSinceBoot = true;
    bool mine = false;
    if (src == standalone::kSourceEcu) mine = engine::accept(*profile, own.value[standalone::kBaseId], rx.id, rx.extended, rx.data, rx.len, now, ecuData);
    else if (src == standalone::kSourceObd11 || src == standalone::kSourceObd29) mine = obd.accept(rx.id, rx.extended, rx.data, rx.len, now, ecuData);
    if (mine) { ++ecuFrames; lastEcuFrameMs = now; }
  }
  if ((src == standalone::kSourceObd11 || src == standalone::kSourceObd29) && canUp) {
    uint32_t id = 0;
    uint8_t data[8];
    if (obd.due(now, id, data)) sendFrame(id, obd.extended(), data, 8);
  }
}

// Manifold pressure, engine speed and barometer for this pass.
void engineData(uint32_t now, meth_tune::Inputs &in) {
  mapSensorFailed = false;
  if (mapFromSensor) {
    float kpa = 0.0f;
    if (board.railOk && engine::mapFromShare(pcb_sensors::shareOf(board.pressPinMv, board.railMv), own.value[standalone::kMapAt05X10] / 10.0f,
                                             own.value[standalone::kMapAt45X10] / 10.0f, kpa)) {
      ecu::takeMap(ecuData, kpa, now);
    } else {
      mapSensorFailed = true;
    }
  }
  config.map.baroKpa = barometer.update(now, ecuData, standalone::hasRpmSource(own));
  view.mapValid = ecuData.mapFresh(now);
  view.mapKpa = view.mapValid ? ecuData.mapKpa : config.map.baroKpa;
  view.rpmValid = ecuData.rpmFresh(now);
  view.rpm = view.rpmValid ? ecuData.rpm : 0;
  in.preValid = preValid || ecuData.intakeFresh(now);
  in.preC = preValid ? preC : ecuData.intakeC;
}

bool tankProtection() { return config.tankProtectionEnabled; }
bool closedWhenLow() { return config.floatActiveLow; }
// With a MAP sensor on the pressure input there is no line pressure sensor.
float lineFullScale() { return mapFromSensor ? 0.0f : static_cast<float>(own.value[standalone::kLineFullPsi]); }
pump_guard::Pass guardLimits() {
  pump_guard::Pass p;
  p.pressureFitted = lineFullScale() > 0.0f;
  p.minPsi = own.value[standalone::kLineMinPsi];
  p.maxPsi = own.value[standalone::kLineMaxPsi];
  p.overAmps = own.value[standalone::kPumpAmpsMaxX10] / 10.0f;
  return p;
}
uint32_t valveLeadMs() { return own.value[standalone::kValveLeadMs]; }
bool ownSettingsValid() { return own.value[0] == 0 && standalone::valid(own); }
void reloadSettings() {
  if (!loadTune()) meth_tune::loadDefaults(tune);
  if (!loadOwn()) standalone::loadDefaults(own);
  tuneChanged();
  tuneUnsaved = false;
  applyOwn(true);
}
float nozzleRatedPsi() { return lineFullScale() > 0.0f ? static_cast<float>(own.value[standalone::kNozzleRatedPsi]) : 0.0f; }
bool rpmRequired() { return standalone::hasRpmSource(own); }
// The pump drive chosen: the pulse governor's duty as it is, or the PWM drive's, which
// sprays whenever the governor's gates let it (pwm_drive.h).
pwm_drive::Drive pwmDrive;
bool pwmChosen() { return own.value[standalone::kDrive] == 1; }
bool pwmDriveChosen() { return pwmChosen(); }
float driveDuty(uint32_t now, const meth_tune::Inputs &in, float pulseDuty) {
  return pwmChosen() ? pwmDrive.update(now, tune, in, governor.hold()) : pulseDuty;
}
float steadyFromPct() { return pwmChosen() ? static_cast<float>(tune.value[meth_tune::kMinDutyPct]) : 99.5f; }
void setDriveSoftStart(uint32_t ms) { pwmDrive.setSoftStartMs(ms); }
float fuelScale() { return own.value[standalone::kEngineDl] / 23.0f; }
bool setupFault() { return !standalone::hasMapSource(own); }
bool waitingForData(uint32_t now) { return !ecuData.mapFresh(now) || (rpmRequired() && !ecuData.rpmFresh(now)); }

// ---- Pump test, settings and actions: shared by the web page and serial commands ----
const char *startTest(uint8_t seconds, uint32_t now) {
  if (config.mode != InjectionMode::Off) return "disarm first";
  if (crashHold || !guard.allows() || watchdogFault != diag::kNone || taskHold) return "clear the fault first";
  if (config.tankProtectionEnabled && floatSensor.isLow()) return "the tank is low";
  if (ecuData.rpmFresh(now) && ecuData.rpm > 0) return "not with the engine running";
  manualTestActive = true;
  manualTestDuty = 100;
  manualTestStartMs = now;
  manualTestLengthMs = (seconds < 1 ? 1 : seconds > 5 ? 5 : seconds) * 1000UL;
  return nullptr;
}

// Sets a setting by name from the value as shown (with its decimals). nullptr on success.
const char *setByName(const char *name, float shown, uint16_t &applied) {
  bool isTune = false;
  const uint8_t key = standalone::keyNamed(name, isTune);
  if (key == 0) return "no such setting";
  const standalone::Info info = isTune ? standalone::tuneInfoOf(key) : standalone::infoOf(key);
  const float raw = shown * info.divisor + 0.5f;
  const uint16_t value = raw < 0.0f ? 0 : raw > 65535.0f ? 65535 : static_cast<uint16_t>(raw);
  if (isTune) {
    meth_tune::set(tune, key, value, applied);
    tuneChanged();
  } else if (key == standalone::kArmed) {
    setArmedState(value != 0);
    applied = value != 0;
  } else {
    standalone::set(own, key, value, applied);
    ownUnsaved = true;
    applyOwn(true);
  }
  return nullptr;
}

const char *doAction(const char *action, uint8_t seconds, uint32_t now) {
  if (strcmp(action, "arm") == 0) { setArmedState(true); manualTestActive = false; return nullptr; }
  if (strcmp(action, "disarm") == 0) { setArmedState(false); return nullptr; }
  if (strcmp(action, "test") == 0) return startTest(seconds, now);
  if (strcmp(action, "stop") == 0) { manualTestActive = false; manualTestDuty = 0; return nullptr; }
  if (strcmp(action, "clear") == 0) { clearFaults(); return nullptr; }
  if (strcmp(action, "clear_codes") == 0) { codes.clear(); return nullptr; }
  if (strcmp(action, "refilled") == 0) {
    meter.filled();
    if (flashWriteAllowed(now)) saveUsedMl(); else usedMlSavedMs = 0;
    return nullptr;
  }
  if (strcmp(action, "save") == 0) { tuneSavePending = ownSavePending = true; return nullptr; }
  if (strcmp(action, "revert") == 0) {
    if (!loadTune()) meth_tune::loadDefaults(tune);
    const bool wasArmed = armed;
    if (!loadOwn()) standalone::loadDefaults(own);
    own.value[standalone::kArmed] = wasArmed ? 1 : 0;
    tuneChanged();
    tuneUnsaved = ownUnsaved = false;
    applyOwn(true);
    return nullptr;
  }
  if (strcmp(action, "defaults") == 0) {
    meth_tune::loadDefaults(tune);
    const bool wasArmed = armed;
    standalone::loadDefaults(own);
    own.value[standalone::kArmed] = wasArmed ? 1 : 0;
    tuneChanged();
    ownUnsaved = true;
    applyOwn(true);
    return nullptr;
  }
  return "no such action";
}

const char *holdText(uint8_t hold) {
  switch (hold) {
  case meth_tune::kHoldNone: return "spraying";
  case meth_tune::kHoldDisarmed: return "not armed";
  case meth_tune::kHoldBelowBoost: return "waiting for boost";
  case meth_tune::kHoldRpmLow: return "engine speed below the minimum";
  case meth_tune::kHoldRpmMissing: return "no engine speed";
  case meth_tune::kHoldAirCold: return "air too cold";
  case meth_tune::kHoldTempSensor: return "no air temperature";
  case meth_tune::kHoldResting: return "resting after its longest spray";
  case meth_tune::kHoldTankLow: return "tank low";
  case meth_tune::kHoldFault: return "fault";
  case meth_tune::kHoldDoseLimit: return "dose limit";
  default: return "?";
  }
}

// ---- The web page (web_ui.h holds the page itself) ----
void addInfo(String &out, const standalone::Info &i) {
  out += "{\"name\":\"";
  out += i.name;
  out += "\",\"label\":\"";
  out += i.label;
  out += "\",\"unit\":\"";
  out += i.unit;
  out += "\",\"min\":";
  out += i.min;
  out += ",\"max\":";
  out += i.max;
  out += ",\"div\":";
  out += i.divisor;
  out += "}";
}

void sendMeta() {
  String out = "{\"tune\":[";
  for (uint8_t k = 1; k <= standalone::kTuneShown; ++k) { if (k > 1) out += ','; addInfo(out, standalone::tuneInfoOf(k)); }
  out += "],\"own\":[";
  for (uint8_t k = 1; k <= standalone::kCount; ++k) { if (k > 1) out += ','; addInfo(out, standalone::infoOf(k)); }
  out += "],\"ecus\":[";
  for (uint8_t i = 0; i < engine::kProfileCount; ++i) {
    if (i) out += ',';
    out += "{\"label\":\"";
    out += engine::kProfiles[i].label;
    out += "\",\"status\":\"";
    out += engine::kProfiles[i].status;
    out += "\",\"kbit\":";
    out += engine::kProfiles[i].bitrate / 1000;
    out += ",\"base\":";
    out += engine::kProfiles[i].defaultBase;
    out += "}";
  }
  out += "],\"maps\":[";
  for (uint8_t i = 0; i < engine::kMapSensorCount; ++i) {
    if (i) out += ',';
    out += "\"";
    out += engine::kMapSensors[i].name;
    out += "\"";
  }
  out += "],\"ssid\":\"";
  out += ssid;
  out += "\",\"build\":\"";
  char build[12];
  snprintf(build, sizeof(build), "%08lX", static_cast<unsigned long>(METH_FW_BUILD));
  out += build;
  out += "\"}";
  server.send(200, "application/json", out);
}

void sendState() {
  const uint32_t now = millis();
  char buf[1400];
  const pcb_can::Health h = pcb_can::health();
  const char *state = manualTestActive ? "pump test" : pumpDuty > 0.0f ? "spraying" : !guard.allows() || crashHold || watchdogFault || taskHold ? "fault"
                    : config.mode == InjectionMode::Off ? "not armed" : "armed";
  const char *fault = guard.fault() != pump_guard::kNone ? pump_guard::name(guard.fault())
                    : guard.hold() == pump_guard::kSupplyLow ? "12 V supply low" : guard.hold() == pump_guard::kSupplyHigh ? "12 V supply too high"
                    : watchdogFault ? diag::infoOf(watchdogFault).text : taskHold ? "CAN task stopped: held"
                    : crashHold ? "held off after repeated crashes" : setupFault() ? "no manifold pressure source set up" : "";
  snprintf(buf, sizeof(buf),
           "{\"state\":\"%s\",\"hold\":\"%s\",\"fault\":\"%s\",\"armed\":%d,\"duty\":%.0f,\"boost_psi\":%.2f,\"map_kpa\":%.1f,"
           "\"map_ok\":%d,\"baro_kpa\":%.1f,\"baro_learned\":%d,\"rpm\":%u,\"rpm_ok\":%d,\"iat1\":%s,\"iat2\":%s,\"pre_c\":%s,"
           "\"ecu_iat\":%s,\"line_psi\":%s,\"amps\":%.2f,\"supply_v\":%.2f,\"rail_v\":%.2f,\"tank_low\":%d,"
           "\"can\":{\"up\":%d,\"kbit\":%lu,\"listen\":%d,\"bus_off\":%d,\"tx_err\":%lu,\"rx_err\":%lu,\"frames\":%lu,\"age_ms\":%lu},"
           "\"on_ms\":%u,\"cycle_ms\":%u,\"unsaved\":%d,\"uptime_s\":%lu,\"reset\":\"%s\",\"crashes\":%u,\"trips\":%u,"
           "\"wifi_default\":%d,\"upload\":\"%s\",",
           state, manualTestActive ? "pump test" : holdText(governor.hold()), fault, armed ? 1 : 0, pumpDuty, view.boostPsi,
           view.mapKpa, view.mapValid ? 1 : 0, config.map.baroKpa, barometer.learned() ? 1 : 0, view.rpm, view.rpmValid ? 1 : 0,
           intakeInputs.preInput() == 0 || intakeInputs.postInput() == 0 ? String(intakeInputs.preInput() == 0 ? preC : postC, 1).c_str() : "null",
           intakeInputs.preInput() == 1 || intakeInputs.postInput() == 1 ? String(intakeInputs.preInput() == 1 ? preC : postC, 1).c_str() : "null",
           preValid ? String(preC, 1).c_str() : "null", ecuData.intakeFresh(now) ? String(ecuData.intakeC, 1).c_str() : "null",
           board.lineValid ? String(board.linePsi, 1).c_str() : "null", board.amps, board.supplyV, board.railMv / 1000.0f,
           board.tankLow ? 1 : 0, canUp ? 1 : 0, static_cast<unsigned long>(canBitrate / 1000), canListenOnly ? 1 : 0, h.busOff ? 1 : 0,
           static_cast<unsigned long>(h.txErrors), static_cast<unsigned long>(h.rxErrors), static_cast<unsigned long>(ecuFrames),
           static_cast<unsigned long>(ecuFrames ? now - lastEcuFrameMs : 0), governor.onMs(), tune.value[meth_tune::kPeriodMs],
           tuneUnsaved || ownUnsaved || tuneSavePending || ownSavePending ? 1 : 0, static_cast<unsigned long>(now / 1000),
           reset_guard::name(bootCause), static_cast<unsigned>(resetRecord.crashes), guard.trips(),
           wifiPassword.length() == 0 ? 1 : 0, uploadProblem ? uploadProblem : "");
  String out = buf;
  // Flow, the fluid count, the valve and the pump's current; the watchdogs; the codes.
  const float tankMl = own.value[standalone::kTankDl] * 100.0f;
  snprintf(buf, sizeof(buf),
           "\"flow\":{\"ml_min\":%.0f,\"from_line\":%d,\"used_ml\":%.0f,\"drive_ml\":%.0f,\"tank_ml\":%.0f,\"left_ml\":%.0f,"
           "\"test_psi\":%s},\"valve_open\":%d,\"amps_avg\":%s,\"amps_peak\":%.2f,"
           "\"wd\":{\"late\":%lu,\"worst_ms\":%lu,\"keeper\":%lu,\"task_ok\":%d,\"run_ms\":%lu,\"run_limit_ms\":%lu,"
           "\"heap\":%lu,\"heap_min\":%lu,\"stack_loop\":%u,\"stack_can\":%u,\"fault\":\"%s\"},"
           "\"codes\":{\"faults\":%u,\"warnings\":%u,\"events\":%lu,\"any\":%d,\"power_ups\":%u},",
           meter.flowMlMin(), flowFromLine ? 1 : 0, meter.usedMl(), meter.driveMl(), tankMl, meter.leftMl(tankMl),
           testPsi > 0.0f ? String(testPsi, 1).c_str() : "null", valveOpen ? 1 : 0,
           pumpCurrent.known() ? String(pumpCurrent.average(), 2).c_str() : "null", pumpCurrent.peak(),
           static_cast<unsigned long>(passTimer.late()), static_cast<unsigned long>(passTimer.worstMs()),
           static_cast<unsigned long>(pumpKeeperTrips), taskHold ? 0 : 1, static_cast<unsigned long>(runLimit.sprayMs()),
           static_cast<unsigned long>(runLimit.limit()), static_cast<unsigned long>(ESP.getFreeHeap()), static_cast<unsigned long>(heapMin),
           static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)),
           static_cast<unsigned>(canTaskHandle ? uxTaskGetStackHighWaterMark(canTaskHandle) : 0),
           watchdogFault ? diag::infoOf(watchdogFault).text : "", codes.activeOfSeverity(diag::kFault),
           codes.activeOfSeverity(diag::kWarning), static_cast<unsigned long>(codes.events()), codes.any() ? 1 : 0, powerUps);
  out += buf;
  out += "\"values\":{";
  bool first = true;
  for (uint8_t k = 1; k <= standalone::kTuneShown; ++k) {
    const standalone::Info i = standalone::tuneInfoOf(k);
    out += first ? "\"" : ",\"";
    first = false;
    out += i.name;
    out += "\":";
    out += String(static_cast<float>(tune.value[k]) / i.divisor, i.divisor > 1 ? 1 : 0);
  }
  for (uint8_t k = 1; k <= standalone::kCount; ++k) {
    const standalone::Info i = standalone::infoOf(k);
    out += ",\"";
    out += i.name;
    out += "\":";
    out += String(static_cast<float>(own.value[k]) / i.divisor, i.divisor > 1 ? 1 : 0);
  }
  out += "}}";
  server.send(200, "application/json", out);
}

// The trouble codes with any occurrences, each with its freeze frame.
void sendCodes() {
  String out = "{\"power_up\":";
  out += powerUps;
  out += ",\"uptime_s\":";
  out += millis() / 1000;
  out += ",\"codes\":[";
  bool first = true;
  char buf[420];
  for (uint8_t c = 1; c < diag::kCodeCount; ++c) {
    const diag::Entry &e = codes.entry(c);
    if (e.count == 0 && !e.active) continue;
    const diag::Info i = diag::infoOf(c);
    snprintf(buf, sizeof(buf),
             "%s{\"id\":\"%s\",\"text\":\"%s\",\"action\":\"%s\",\"severity\":%u,\"active\":%u,\"count\":%u,"
             "\"first\":[%u,%lu],\"last\":[%u,%lu],\"freeze\":{\"boost_psi\":%.1f,\"rpm\":%u,\"amps\":%.2f,\"line_psi\":%.1f,"
             "\"supply_v\":%.2f,\"duty\":%u,\"air_c\":%d}}",
             first ? "" : ",", i.id, i.text, i.action, i.severity, e.active, e.count, e.firstPowerUp, static_cast<unsigned long>(e.firstS),
             e.lastPowerUp, static_cast<unsigned long>(e.lastS), e.freeze.boostPsiX10 / 10.0f, e.freeze.rpm, e.freeze.ampsX100 / 100.0f,
             e.freeze.linePsiX10 / 10.0f, e.freeze.supplyMv / 1000.0f, e.freeze.dutyPct, e.freeze.airC);
    out += buf;
    first = false;
  }
  out += "]}";
  server.send(200, "application/json", out);
}

void sendProfiles() {
  String out = "[";
  for (uint8_t n = 0; n < kProfileSlots; ++n) {
    ProfileSlot slot;
    const bool used = readSlot(n, slot);
    if (n) out += ',';
    out += "{\"name\":\"";
    out += used ? slot.name : kSlotDefaultNames[n];
    out += "\",\"used\":";
    out += used ? 1 : 0;
    if (used) {
      char buf[96];
      snprintf(buf, sizeof(buf), ",\"start_psi\":%.1f,\"full_psi\":%.1f,\"meth_pct\":%u,\"drive\":%u",
               slot.tune[meth_tune::kStartPsiX10] / 10.0f, slot.tune[meth_tune::kFullPsiX10] / 10.0f, slot.tune[meth_tune::kMethPercent],
               slot.own[standalone::kDrive]);
      out += buf;
    }
    out += '}';
  }
  out += ']';
  server.send(200, "application/json", out);
}

void replyResult(const char *problem, uint16_t applied = 0) {
  String out = problem ? String("{\"ok\":0,\"problem\":\"") + problem + "\"}" : String("{\"ok\":1,\"applied\":") + applied + "}";
  server.send(problem ? 400 : 200, "application/json", out);
}

// Firmware from the web page: refused while injecting, and unless it is this build's image.
void handleUpload() {
  HTTPUpload &up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    uploadProblem = nullptr;
    uploadRefused = injectionNeeded(millis());
    if (uploadRefused) { uploadProblem = "not while injecting"; return; }
    setPump(0.0f);
    if (!otaFlash.begin(0)) { uploadRefused = true; uploadProblem = "no room for new firmware"; }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (uploadRefused) return;
    setPump(0.0f);
    if (!otaFlash.write(up.buf, up.currentSize)) { uploadRefused = true; uploadProblem = "flash write failed"; otaFlash.abort(); }
  } else if (up.status == UPLOAD_FILE_END) {
    if (uploadRefused) return;
    if (!otaFlash.finish()) { uploadProblem = otaFlash.wrongImage ? "not firmware for this board (WaterMethPCBStandalone)" : "the image is damaged"; return; }
    uploadProblem = nullptr;
    restartAtMs = (millis() + 1000) | 1;
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    otaFlash.abort();
    uploadProblem = "upload interrupted";
  }
}

void startWifi() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char name[24];
  snprintf(name, sizeof(name), "WaterMeth-%02X%02X", mac[4], mac[5]);
  ssid = name;
  WiFi.mode(WIFI_AP);
  // Until one is set on the web page: "watermeth" (the page asks for a new one).
  const String pass = wifiPassword.length() >= 8 ? wifiPassword : String("watermeth");
  wifiOn = WiFi.softAP(ssid.c_str(), pass.c_str());
  Serial.printf("WIFI: %s \"%s\"; the page is at http://%s\n", wifiOn ? "access point" : "did not start", ssid.c_str(),
                WiFi.softAPIP().toString().c_str());
  server.on("/", HTTP_GET, [] { server.send_P(200, "text/html", web_ui::kPage); });
  server.on("/api/meta", HTTP_GET, sendMeta);
  server.on("/api/state", HTTP_GET, sendState);
  server.on("/api/set", HTTP_POST, [] {
    uint16_t applied = 0;
    const char *problem = setByName(server.arg("name").c_str(), server.arg("value").toFloat(), applied);
    replyResult(problem, applied);
  });
  server.on("/api/action", HTTP_POST, [] {
    replyResult(doAction(server.arg("a").c_str(), static_cast<uint8_t>(server.arg("s").toInt()), millis()));
  });
  server.on("/api/wifi", HTTP_POST, [] {
    const String pass = server.arg("password");
    if (pass.length() < 8 || pass.length() > 63) { replyResult("8 to 63 characters"); return; }
    saveWifiPassword(pass);
    replyResult(nullptr);
  });
  server.on("/api/codes", HTTP_GET, sendCodes);
  server.on("/api/profiles", HTTP_GET, sendProfiles);
  server.on("/api/profile", HTTP_POST, [] {
    const uint8_t slot = static_cast<uint8_t>(server.arg("slot").toInt());
    const String op = server.arg("op");
    if (op == "save") replyResult(saveSlot(slot, server.arg("name").c_str()));
    else if (op == "load") replyResult(loadSlot(slot, millis()));
    else replyResult("no such operation");
  });
  server.on("/update", HTTP_POST, [] { replyResult(uploadProblem); }, handleUpload);
  server.begin();
  wifiClientSeenMs = millis();
}

void stopWifi(const __FlashStringHelper *why) {
  Serial.println(why);
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiOn = false;
}

void serviceWifi(uint32_t now) {
  if (!wifiOn) return;
  // Control comes first: with memory nearly gone the page goes, not the pump's checks.
  if (ESP.getFreeHeap() < watchdogs::kHeapCriticalBytes) {
    stopWifi(F("WIFI: memory nearly gone; Wi-Fi off until the next power-up"));
    return;
  }
  server.handleClient();
  if (WiFi.softAPgetStationNum() > 0) wifiClientSeenMs = now;
  if (own.value[standalone::kWifiAlways] == 0 && elapsed(now, wifiClientSeenMs, kWifiIdleOffMs)) {
    stopWifi(F("WIFI: nobody connected for 15 minutes; off until the next power-up"));
  }
}

// ---- USB serial: the same settings and actions as the web page ----
void serialCommands(uint32_t now) {
  static char line[96];
  static uint8_t len = 0;
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c != '\n' && c != '\r') { if (len < sizeof(line) - 1) line[len++] = c; continue; }
    line[len] = 0;
    len = 0;
    if (line[0] == 0) continue;
    char *word = strtok(line, " ");
    char *arg1 = strtok(nullptr, " ");
    char *arg2 = strtok(nullptr, " ");
    if (strcmp(word, "set") == 0 && arg1 && arg2) {
      uint16_t applied = 0;
      const char *problem = setByName(arg1, atof(arg2), applied);
      Serial.printf("%s\n", problem ? problem : "ok (\"save\" keeps it)");
    } else if (strcmp(word, "show") == 0) {
      for (uint8_t k = 1; k <= standalone::kCount; ++k) {
        const standalone::Info i = standalone::infoOf(k);
        Serial.printf("  %-28s %g %s  (%s)\n", i.name, static_cast<double>(own.value[k]) / i.divisor, i.unit, i.label);
      }
      for (uint8_t k = 1; k <= standalone::kTuneShown; ++k) {
        const standalone::Info i = standalone::tuneInfoOf(k);
        Serial.printf("  %-28s %g %s  (%s)\n", i.name, static_cast<double>(tune.value[k]) / i.divisor, i.unit, i.label);
      }
      for (uint8_t i = 0; i < engine::kProfileCount; ++i) Serial.printf("  ecu %u = %s\n", i, engine::kProfiles[i].label);
    } else if (strcmp(word, "wifi") == 0 && arg1) {
      if (strlen(arg1) < 8) Serial.println(F("at least 8 characters"));
      else { saveWifiPassword(arg1); Serial.println(F("saved: used from the next power-up")); }
    } else if (strcmp(word, "codes") == 0) {
      printCodes();
    } else if (strcmp(word, "profile") == 0 && arg1 && arg2) {
      const uint8_t n = static_cast<uint8_t>(atoi(arg2));
      const char *problem = strcmp(arg1, "save") == 0 ? saveSlot(n, strtok(nullptr, ""))
                          : strcmp(arg1, "load") == 0 ? loadSlot(n, now) : "profile save|load <0-3> [name]";
      Serial.printf("%s\n", problem ? problem : "ok");
    } else if (strcmp(word, "help") == 0) {
      Serial.println(F("show | set <name> <value> | arm | disarm | test <s> | stop | clear | clear_codes | codes | refilled | save | revert |"));
      Serial.println(F("defaults | profile save|load <0-3> [name] | wifi <password>"));
    } else {
      const char *problem = doAction(word, arg1 ? static_cast<uint8_t>(atoi(arg1)) : 3, now);
      Serial.printf("%s\n", problem ? problem : "ok");
    }
  }
}

void setupBuild() {
  if (!loadTune()) meth_tune::loadDefaults(tune);
  if (!loadOwn()) standalone::loadDefaults(own);
  syncTuneToConfig();
  applyOwn(false);
  esp_ota_img_states_t state;
  const esp_partition_t *running = esp_ota_get_running_partition();
  onTrial = running && esp_ota_get_state_partition(running, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY;
}

void blackBoxAtStart(bool, bool, uint32_t) {}

bool serviceUpdate(uint32_t) { return false; }

// New firmware from the web page: kept once it has run a minute without trouble.
void trialCheck(uint32_t now) {
  if (onTrial && now >= kConfirmAfterMs) {
    otaFlash.confirm();
    onTrial = false;
    Serial.println(F("UPDATE: new firmware has run a minute; kept"));
  }
}

void serviceBuild(uint32_t now, uint32_t) {
  if (flashWriteAllowed(now)) {
    if (tuneSavePending) { saveTune(); tuneSavePending = tuneUnsaved = false; }
    if (ownSavePending) { saveOwn(); ownSavePending = ownUnsaved = false; }
  }
  if (!pcb_can::service() && elapsed(now, lastCanInitMs, 2000) && (!canUp || !pcb_can::running())) startCan();
  serviceWifi(now);
  serialCommands(now);
}
#endif

// ---------------------------------------------------------------------------
// Codes and watchdogs, both builds.
// ---------------------------------------------------------------------------
diag::Freeze freezeNow() {
  diag::Freeze f;
  const float boost = view.boostPsi * 10.0f;
  f.boostPsiX10 = static_cast<int16_t>(boost < -32000.0f ? -32000.0f : boost > 32000.0f ? 32000.0f : boost);
  f.rpm = view.rpm;
  f.ampsX100 = static_cast<uint16_t>(board.amps > 0.0f ? (board.amps < 600.0f ? board.amps * 100.0f : 60000.0f) : 0.0f);
  f.linePsiX10 = static_cast<uint16_t>(board.lineValid && board.linePsi > 0.0f ? (board.linePsi < 6000.0f ? board.linePsi * 10.0f : 60000.0f) : 0.0f);
  f.supplyMv = static_cast<uint16_t>(board.supplyV > 0.0f ? (board.supplyV < 60.0f ? board.supplyV * 1000.0f : 60000.0f) : 0.0f);
  f.dutyPct = static_cast<uint8_t>(pumpDuty + 0.5f);
  f.airC = static_cast<int8_t>(preValid ? (preC < -100.0f ? -100.0f : preC > 120.0f ? 120.0f : preC) : 0.0f);
  return f;
}

void loadCodes() {
  Preferences store;
  if (store.begin("diag", false)) {
    powerUps = static_cast<uint16_t>(store.getUShort("powerups", 0) + 1);
    store.putUShort("powerups", powerUps);
    diag::Store::Image *img = new diag::Store::Image;
    if (store.getBytes("codes", img, sizeof(*img)) == sizeof(*img) && !codes.load(*img)) {
      Serial.println(F("CODES: the stored codes failed their check; starting a fresh list"));
    }
    delete img;
    store.end();
  }
  codes.begin(powerUps);
}

// Written away from boost, at most once a minute.
void saveCodesWhenDue(uint32_t now) {
  if (!codes.dirty() || !elapsed(now, codesWrittenMs, 60000UL) || !flashWriteAllowed(now)) return;
  codesWrittenMs = now;
  diag::Store::Image *img = new diag::Store::Image;
  codes.save(*img);
  Preferences store;
  if (store.begin("diag", false)) {
    store.putBytes("codes", img, sizeof(*img));
    store.end();
    codes.written();
  }
  delete img;
}

// Engine data no engine gives is not used: a garbled frame, a wrong ECU or scale.
void checkEngineData() {
  engineImplausible = false;
  if (view.mapValid && !watchdogs::plausibleMap(view.mapKpa)) {
    view.mapValid = false;
    view.mapKpa = config.map.baroKpa;
    engineImplausible = true;
  }
  if (view.rpmValid && !watchdogs::plausibleRpm(view.rpm)) {
    view.rpmValid = false;
    view.rpm = 0;
    engineImplausible = true;
  }
  if (view.mapValid) engineDataSeen = true;
}

// Each pass: the conditions behind the codes.
void recordCodes(uint32_t now, bool late, uint32_t keeperTripsNew) {
  const uint32_t s = now / 1000;
  const diag::Freeze f = freezeNow();
  for (uint8_t c = pump_guard::kShort; c <= pump_guard::kOverPressure; ++c) codes.report(c, guard.fault() == c, s, f);
  codes.report(diag::kSupplyLow, guard.hold() == pump_guard::kSupplyLow, s, f);
  codes.report(diag::kSupplyHigh, guard.hold() == pump_guard::kSupplyHigh, s, f);
  codes.report(diag::kSensorRail, railBad.update(now, !board.railOk, 1000), s, f);
  codes.report(diag::kLineSensor, lineSensorBad.update(now, lineFullScale() > 0.0f && board.railOk && !board.lineValid, 1000), s, f);
  codes.report(diag::kMapSensor, mapSensorBad.update(now, mapSensorFailed && board.railOk, 1000), s, f);
  codes.report(diag::kEngineDataLost, dataLost.update(now, armed && engineDataSeen && waitingForData(now), 2000), s, f);
  codes.report(diag::kEngineDataImplausible, engineImplausible, s, f);
  codes.report(diag::kCanBusOff, pcb_can::health().busOff, s, f);
  codes.report(diag::kTaskStalled, taskHold, s, f);
  codes.report(diag::kPumpRunTooLong, watchdogFault == diag::kPumpRunTooLong, s, f);
  codes.report(diag::kOutputMismatch, watchdogFault == diag::kOutputMismatch, s, f);
  codes.report(diag::kTankLow, tankProtection() && board.tankLow, s, f);
  if (late) codes.event(diag::kLoopOverrun, s, f);
  for (uint32_t i = 0; i < keeperTripsNew && i < 4; ++i) codes.event(diag::kPumpKeeper, s, f);
}

// Once a second: the settings in memory still pass their check, and memory is not running out.
void slowChecks(uint32_t now) {
  if (!elapsed(now, lastSlowChecksMs, 1000)) return;
  lastSlowChecksMs = now;
  tune.value[0] = 0;
  if (!meth_tune::valid(tune) || !ownSettingsValid()) {
    setPump(0.0f);
    codes.event(diag::kSettingsCorrupt, now / 1000, freezeNow());
    Serial.println(F("SAFETY: settings in memory failed their check; reloaded from flash, disarmed"));
    reloadSettings();
    setArmedState(false);
  }
  const uint32_t heap = ESP.getFreeHeap();
  if (heap < heapMin) heapMin = heap;
  codes.report(diag::kLowMemory, heapLow.update(now, heap < watchdogs::kHeapLowBytes, 3000), now / 1000, freezeNow());
}

status_light::Facts ledFacts(uint32_t now) {
  status_light::Facts f;
  f.dutyPct = pumpDuty;
  f.pumpTest = manualTestActive;
  f.armed = config.mode != InjectionMode::Off;
  const ControlResult &r = view.result;
  f.fault = (tankProtection() && board.tankLow) || !canUp || crashHold || guard.fault() != pump_guard::kNone || watchdogFault != diag::kNone ||
            taskHold || r.overboostAssistFaultLatched ||
            r.failsafe == FailsafeReason::InvalidBlend || r.failsafe == FailsafeReason::InvalidBoostConfig || setupFault();
  const uint8_t hold = governor.hold();
  f.waiting = waitingForData(now) || guard.hold() != pump_guard::kHoldNone || (f.armed && (hold == meth_tune::kHoldRpmMissing || hold == meth_tune::kHoldTempSensor ||
                                                  hold == meth_tune::kHoldResting));
  return f;
}

uint32_t lastLoopMs = 0;
uint32_t loopGapMaxMs = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t keeperTripsReported = 0;
uint32_t keeperTripsCoded = 0;
bool testWasActive = false;
bool selfTestDone = false;

// Shortly after power-up, with the pump never on yet: the current sense should read about
// nothing. A reading over 1 A is the pump guard's "stuck on"; between that and 0.3 A the
// current sense itself is off (its amplifier or shunt), and the current checks may be wrong.
void selfTest(uint32_t now) {
  if (selfTestDone || now < 600) return;
  selfTestDone = true;
  const bool ampsOk = board.amps < pump_guard::kOpenAmps;
  if (!ampsOk && board.amps < pump_guard::kStuckAmps) codes.event(diag::kCurrentSenseOffset, now / 1000, freezeNow());
  Serial.printf("SELFTEST: pump current at rest %.2f A %s; sensor supply %.2f V %s; 12 V %.2f V; outputs read back %s\n", board.amps,
                ampsOk ? "ok" : "HIGH", board.railMv / 1000.0f, board.railOk ? "ok" : "OUT OF RANGE", board.supplyV,
                readPumpCount() == 0 && !readValve() ? "off: ok" : "NOT OFF");
}
} // namespace

void setup() {
  // Pump and valve off before anything else.
  pinMode(pcb::PUMP, OUTPUT);
  digitalWrite(pcb::PUMP, LOW);
  pinMode(pcb::SOLENOID, OUTPUT);
  digitalWrite(pcb::SOLENOID, LOW);
  const bool pumpReady = ledcAttach(pcb::PUMP, pcb::PUMP_PWM_HZ, pcb::PUMP_PWM_BITS);
  ledcWrite(pcb::PUMP, 0);
  set_arduino_panic_handler(outputsOffAtCrash, nullptr);
  esp_register_shutdown_handler(outputsOffAtRestart);

  bootCause = reset_guard::boot(resetRecord, reportedResetCause(esp_reset_reason()));
  crashHold = reset_guard::injectionHeld(resetRecord);
  crashHoldEndsMs = reset_guard::holdMs(resetRecord);
  const bool crashHookRan = crashHookMark == kCrashHookRan;
  crashHookMark = 0;
  const bool pumpWasOn = resetRecord.pumpOn != 0;
  const bool armedAgain = reset_guard::comesBackArmed(resetRecord, bootCause);
  reset_guard::notePump(resetRecord, false);
  const uint32_t softStartMs = reset_guard::softStartMs(resetRecord, meth_tune::kSoftStartMs);
  governor.setSoftStartMs(softStartMs);
  setDriveSoftStart(softStartMs);

  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  Serial.setTxTimeoutMs(0);   // Never stall control for a computer that is not reading
#endif
  Serial.println();
  Serial.printf("BOOT: water/meth PCB rev D, %s build %08lX (%s)\n", kBuildName, static_cast<unsigned long>(METH_FW_BUILD), kImageMark);
  Serial.printf("BOOT: last reset: %s%s; crashes this ignition cycle: %u%s\n", reset_guard::name(bootCause),
                pumpWasOn ? ", with the pump running" : "", static_cast<unsigned>(resetRecord.crashes),
                bootCause == reset_guard::Cause::Crash ? (crashHookRan ? " (pump and valve switched off first)" : "") : "");
  if (!pumpReady) Serial.println(F("BOOT: pump PWM did not start; the pump will not run"));

  pinMode(pcb::LED, OUTPUT);
  digitalWrite(pcb::LED, LOW);
  // The level input is the ROM's boot log pin: an input only from here, after start-up.
  pinMode(pcb::LEVEL, INPUT);
  analogReadResolution(12);
  for (int pin : {pcb::IAT1_ADC, pcb::IAT2_ADC, pcb::VBAT_ADC, pcb::PRESS_ADC, pcb::VSENS_ADC, pcb::ISENSE_ADC}) {
    analogSetPinAttenuation(pin, ADC_11db);
  }

  loadCodes();
  setupBuild();
  if (bootCause == reset_guard::Cause::Crash) codes.event(diag::kCrashReset, 0, diag::Freeze{});
  if (bootCause == reset_guard::Cause::Brownout) codes.event(diag::kBrownoutReset, 0, diag::Freeze{});
  if (settingsCorruptAtBoot) {
    codes.event(diag::kSettingsCorrupt, 0, diag::Freeze{});
    Serial.println(F("BOOT: stored settings failed their check; defaults used, disarmed"));
  }
#if PCB_FROGDASH
  setArmedState(armedAgain && !settingsCorruptAtBoot);
#else
  // Armed as saved; after a reset nobody asked for, as it was just before.
  setArmedState(!settingsCorruptAtBoot && (armedAgain || own.value[standalone::kArmed] != 0));
#endif
  if (codes.any()) printCodes();
  floatSensor.begin(pcb_sensors::tankLow(levelPinHigh(), closedWhenLow()), millis(), config.floatDebounceMs, config.floatLowShutdownDelayMs);
  blackBoxAtStart(pumpWasOn, armedAgain, softStartMs);
  if (crashHold) Serial.printf("SAFETY: %u crashes: injection held off for %lu s\n", static_cast<unsigned>(resetRecord.crashes),
                               static_cast<unsigned long>(crashHoldEndsMs / 1000));
  if (onTrial) Serial.println(F("BOOT: new firmware, on trial"));

  esp_task_wdt_config_t watchdog = {};
  watchdog.timeout_ms = kWatchdogMs;
  watchdog.idle_core_mask = 1U << 0;
  watchdog.trigger_panic = true;
  esp_task_wdt_reconfigure(&watchdog);
  enableLoopWDT();

  canRxQueue = xQueueCreate(128, sizeof(pcb_can::Frame));
  startCan();
  xTaskCreatePinnedToCore(canTask, "can-rx", 4096, nullptr, 6, &canTaskHandle, 0);
#if !PCB_FROGDASH
  startWifi();
#endif
  ledStartMs = millis();
}

void loop() {
  drainCan(millis());
  const uint32_t now = millis();
  if (restartAtMs && static_cast<int32_t>(now - restartAtMs) >= 0) {
    setPump(0.0f);
    ESP.restart();
  }
  if (serviceUpdate(now)) {
    status_light::Facts updating;
    updating.updating = true;
    showLed(now, updating);
    delay(1);
    return;
  }
  trialCheck(now);
  if (!elapsed(now, lastLoopMs, kLoopMs)) {
    delay(1);
    return;
  }
  const uint32_t gapMs = lastLoopMs ? now - lastLoopMs : 0;
  if (gapMs > loopGapMaxMs) loopGapMaxMs = gapMs;
  lastLoopMs = now;
  const bool late = passTimer.pass(now);
  endCrashHoldWhenDue(now);

  // What the outputs hold now must be what the last pass set (the PWM's duty register
  // takes a new value at its next period, 2 ms: long before this pass).
  if (outputCheck.update(askedPumpCount, readPumpCount(), askedValve, readValve()) && watchdogFault == diag::kNone) {
    watchdogFault = diag::kOutputMismatch;
    setPump(0.0f);
    Serial.println(F("SAFETY: pump or valve output does not match what was set; off, latched"));
  }
  // The CAN task (which carries the pump keeper) is still running.
  const bool taskWasHeld = taskHold;
  taskHold = !canTaskBeat.check(now, canTaskCount);
  if (taskHold && !taskWasHeld) Serial.println(F("SAFETY: the CAN task has stopped; injection held"));

  readBoard(now, lineFullScale(), closedWhenLow());
  selfTest(now);
  meth_tune::Inputs in;
  engineData(now, in);
  checkEngineData();
  view.boostPsi = (view.mapKpa - config.map.baroKpa) * kPsiPerKpa;

  SensorReadings readings;
  readings.mapValid = view.mapValid;
  readings.mapKpa = view.mapKpa;
  readings.boostPsi = view.boostPsi;
  readings.tankLow = board.tankLow;
  readings.methPressureValid = board.lineValid;
  readings.methPressurePsi = board.linePsi;

  if (manualTestActive && (elapsed(now, manualTestStartMs, manualTestLengthMs) || (config.tankProtectionEnabled && board.tankLow) ||
                           config.mode != InjectionMode::Off || !guard.allows())) {
    manualTestActive = false;
    manualTestDuty = 0;
  }
  ControlResult result = controller.update(readings, config, blend);
  float duty = 0.0f;
  if (manualTestActive) {
    duty = governor.manual(now, tune, manualTestDuty);
  } else {
    in.wantSpray = result.pump.enabled && !crashHold && guard.allows() && watchdogFault == diag::kNone && !taskHold;
    in.idleHold = idleHoldFor(result);
    in.boostPsi = view.boostPsi;
    in.mapKpa = view.mapKpa;
    in.rpmValid = view.rpmValid;
    in.rpm = view.rpm;
    in.assistActive = result.overboostAssistActive;
    in.assistDutyPercent = result.finalDutyPercent;
    in.capDutyPercent = config.dutyMaxPercent;
    in.fuelScale = fuelScale();
    in.rpmRequired = rpmRequired();
    duty = driveDuty(now, in, governor.update(now, tune, in));
  }
  // The board's own checks come last: a fault stops the pump this very pass.
  pump_guard::Pass gp = guardLimits();
  gp.dutyPct = duty;
  gp.ampsValid = true;
  gp.amps = board.amps;
  gp.supplyValid = true;
  gp.supplyV = board.supplyV;
  gp.pressureValid = board.lineValid;
  gp.pressurePsi = board.linePsi;
  gp.pressureFitted = gp.pressureFitted && lineFullScale() > 0.0f;
  gp.steadyFromPct = manualTestActive ? 99.5f : steadyFromPct();
  guard.update(now, gp);
  // No spray lasts longer than the pulse timing allows, whatever the pulse timing does.
  if (runLimit.update(now, duty > 0.0f, tune.value[meth_tune::kMaxSprayS], tune.value[meth_tune::kRestS]) && watchdogFault == diag::kNone) {
    watchdogFault = diag::kPumpRunTooLong;
    Serial.printf("SAFETY: the pump's on-time passed %lu s without its rest; off, latched\n", static_cast<unsigned long>(runLimit.limit() / 1000));
  }
  if (!guard.allows() || watchdogFault != diag::kNone || taskHold) {
    duty = 0.0f;
    if (manualTestActive) { manualTestActive = false; manualTestDuty = 0; }
  }
  setPump(duty, valve.update(now, duty > 0.0f, valveLeadMs()));

  // Flow, from the line pressure when the nozzle was measured at a known pressure.
  flow::Pass fp;
  fp.valveOpen = valveOpen;
  fp.dutyPct = duty;
  fp.pwm = pwmDriveChosen() && !manualTestActive;
  fp.nozzleMlMin = tune.value[meth_tune::kNozzleMlMin];
  fp.ratedPsi = nozzleRatedPsi();
  fp.pressureValid = board.lineValid;
  fp.linePsi = board.linePsi;
  meter.update(now, flow::estimate(fp, flowFromLine));
  pumpCurrent.update(now, duty, board.amps);
  // The line pressure during a pump test, for measuring the nozzle at a known pressure.
  if (manualTestActive && !testWasActive) { testPsiSum = 0.0f; testPsiCount = 0; testPsi = 0.0f; }
  testWasActive = manualTestActive;
  if (manualTestActive && duty >= 99.5f && elapsed(now, manualTestStartMs, 1000) && board.lineValid && testPsiCount < 60000) {
    testPsiSum += board.linePsi;
    ++testPsiCount;
    testPsi = testPsiSum / testPsiCount;
  }
  result.finalDutyPercent = duty;
  result.pump.enabled = duty > 0.0f;
  view.result = result;
  view.duty = duty;
  showLed(now, ledFacts(now));
  serviceBuild(now, gapMs);

  const uint32_t keeperTrips = pumpKeeperTrips;
  recordCodes(now, late, keeperTrips - keeperTripsCoded);
  keeperTripsCoded = keeperTrips;
  slowChecks(now);
  saveCodesWhenDue(now);
  persistUsedMl(now);

  if (pumpKeeperTrips != keeperTripsReported) {
    keeperTripsReported = pumpKeeperTrips;
    Serial.printf("SAFETY: the main loop went %ld ms without setting the pump; it was switched off (%lu times)\n",
                  static_cast<long>(kPumpKeeperMs), static_cast<unsigned long>(keeperTripsReported));
  }
  if (elapsed(now, lastHeartbeatMs, 1000)) {
    lastHeartbeatMs = now;
    Serial.printf("HB t=%lu map=%.1f%s boost=%.2f baro=%.1f rpm=%u%s duty=%.0f hold=%u armed=%d tankLow=%d pre=%.1f%s "
                  "line=%.1f%s amps=%.2f supply=%.2f rail=%.2f guard=%s%s can=%d loopMax=%lu keeper=%lu valve=%d flow=%.0f%s wd=%s heap=%lu\n",
                  static_cast<unsigned long>(now), view.mapKpa, view.mapValid ? "" : "!", view.boostPsi, config.map.baroKpa, view.rpm,
                  view.rpmValid ? "" : "!", pumpDuty, governor.hold(), armed ? 1 : 0, board.tankLow ? 1 : 0, preC, preValid ? "" : "!",
                  board.linePsi, board.lineValid ? "" : "!", board.amps, board.supplyV, board.railMv / 1000.0f,
                  pump_guard::name(guard.fault()), guard.hold() == pump_guard::kSupplyLow ? "(supply low)" : guard.hold() ? "(supply high)" : "",
                  canUp ? 1 : 0, static_cast<unsigned long>(loopGapMaxMs), static_cast<unsigned long>(pumpKeeperTrips), valveOpen ? 1 : 0,
                  meter.flowMlMin(), flowFromLine ? "(line)" : "", watchdogFault ? diag::infoOf(watchdogFault).id : taskHold ? "task" : "ok",
                  static_cast<unsigned long>(ESP.getFreeHeap()));
    loopGapMaxMs = 0;
  }
}
