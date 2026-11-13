// ESP32-S3 water/meth and knock controller (replaces the Arduino Nano board).
//
// What changed with the hardware, and nothing else:
//   - The pump runs through a MOSFET with PWM (meth_tune::PwmGovernor), not a relay.
//   - Manifold pressure, engine speed and intake air temperature come straight from
//     the MicroSquirt's CAN broadcast (ecu_broadcast.h). There is no MAP sensor here,
//     and no fresh ECU data means no injection.
//   - Knock, oil pressure, fuel pressure and the air temperature before the nozzle are
//     read by the ADC's own timer (adc_sampler).
//   - Knock is listened for on every sample, 38 thousand a second, with band-pass
//     filters on the cylinder's two resonances (knock_dsp.h); the Nano could look at one
//     short burst in six. The amplifier's gain is set automatically (knock_agc.h).
//   - New firmware arrives from the dash over CAN (can_contract/firmware_update.h).
// Every CAN message, command and setting is the same as the Nano firmware's: the
// functions that speak them are that firmware's own code. Wiring: docs/ESP32_HARDWARE.md.
#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include <esp_ota_ops.h>
#include <esp_rom_gpio.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <hal/gpio_ll.h>
#include <math.h>
#include <mcp2515.h>
#include <soc/gpio_sig_map.h>

#include "adc_sampler.h"
#include "analog_inputs.h"
#include "app_config.h"
#include "can_contract/can_protocol.h"
#include "can_contract/firmware_update.h"
#include "ecu_broadcast.h"
#include "injection_controller.h"
#include "knock_agc.h"
#include "knock_band.h"
#include "knock_dsp.h"
#include "knock_spectrum.h"
#include "meth_tune.h"
#include "pins_esp32.h"
#include "reset_guard.h"
#include "x9c.h"

// The knock amplifier's gain is set by an X9C104 potentiometer and controlled
// automatically (knock_agc.h). Build with -DKNOCK_GAIN_POT=0 for an amplifier with a
// fixed gain; the firmware also stops by itself if changing the gain shows no effect.
#ifndef KNOCK_GAIN_POT
#define KNOCK_GAIN_POT 1
#endif

// Knock listening: the filters' verdicts, the amplifier's automatic gain and the knock
// frequency finder. Off for now at the owner's request (November 2026), until the knock
// amplifier's input works on the bench, as on the Nano (NANO_KNOCK). The knock state
// frame (0x307) still goes out, reporting knock not enabled, so the dash shows it
// unavailable; knock commands are refused as unsupported; the knock detail (0x30B,
// 0x30C/0x30D, 0x30E) is not sent; the gain potentiometer is left alone. Injection never
// depended on knock. The ADC still samples the knock input with the others: the
// pressures and air temperatures come from the same conversions. Build with
// -DESP32_KNOCK=1 to bring it all back.
#ifndef ESP32_KNOCK
#define ESP32_KNOCK 0
#endif
constexpr bool kKnockOn = ESP32_KNOCK != 0;

// Bench only (environment esp32s3_safety_test): serial commands that make this firmware
// fail on purpose, one way at a time, to watch it recover. Never in the car's build.
#ifndef METH_SAFETY_TEST
#define METH_SAFETY_TEST 0
#endif

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
#ifndef CAN_PROTOCOL_HAS_KNOCK_DETAIL
#error "Update DIYComfortControlModule: the shared CAN contract needs the knock detail messages (extension 6)."
#endif
#ifndef CAN_PROTOCOL_HAS_METH_PWM
#error "Update DIYComfortControlModule: the shared CAN contract needs the PWM duty settings (extension 5)."
#endif
#ifndef CAN_PROTOCOL_HAS_METH_START_REPORT
#error "Update DIYComfortControlModule: the shared CAN contract needs the water/meth start report (extension 8)."
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
uint32_t loopGapMaxMs = 0;          // Longest gap between control passes since the last heartbeat
uint32_t keeperTripsReported = 0;
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
// Knock listening (knock_dsp.h): the ADC task filters every sample; each loop pass
// judges the 20 ms just gone.
knock_dsp::Detector knockDetector;
knock_dsp::PassAnalyser knockAnalyser;
knock_dsp::Summary knockHeard;
knock_dsp::Frame knockFrames[knock_dsp::kMaxFrames];
adc_sampler::KnockStats knockStats;
// The engine's real knock frequency, found from the spectrum (knock_spectrum.h). 0 until
// it has been learned; then the knock filter sits there instead of on the bore's figure.
knock_spectrum::Finder knockFinder;
knock_spectrum::Snapshot knockSpectrum;
float knockLearnedHz = 0.0f;
float knockSavedHz = 0.0f;
float knockToneHz = 0.0f;
uint32_t knockSavedMs = 0;
uint32_t knockSpectrumPrintMs = 0;
constexpr uint32_t kKnockSaveEveryMs = 120000;
constexpr uint32_t kKnockSpectrumPrintMs = 10000;
// What the dash is told about all this (0x30E, can_protocol.h extension 6).
namespace kd = can_protocol::knock_detail;
uint32_t lastKnockDetailTxMs = 0;
uint32_t lastKnockListeningTxMs = 0;
uint32_t lastKnockSpectrumTxMs = 0;
uint8_t knockSpectrumNext = 0;                      // Next chunk to send: the normal spectrum, then the burst
uint8_t knockSpectrumNormal[kd::SPECTRUM_POINTS];   // As sent: (dB + 10) * 3
uint8_t knockSpectrumBurst[kd::SPECTRUM_POINTS];
float knockBurstHoldDb[kd::SPECTRUM_POINTS];        // Loudest block since the burst spectrum was last sent
bool knockBurstHeld = false;
constexpr uint32_t kKnockDetailTxMs = 500;
constexpr uint32_t kKnockListeningTxMs = 200;
constexpr uint32_t kKnockSpectrumTxMs = 80;         // One of twelve chunks: both spectra once a second
// Knock settings changed from the dash are written to flash a few seconds after the last change.
bool knockSettingsDirty = false;
uint32_t knockSettingsChangedMs = 0;
constexpr uint32_t kKnockSettingsSaveMs = 3000;
int16_t knockGainFixedStep = -1;                    // -1: the controller sets the gain; else held here
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

// What the bore suggests the knock frequency is.
uint16_t knockBoreHz() {
  return knock_band::knockHzForBore(static_cast<uint8_t>(clampFloatValue(config.knock.boreMm, 40.0f, 200.0f) + 0.5f));
}

// Where the knock filter sits: on automatic, where the spectrum has shown this engine
// rings, or the bore's figure until that is known; otherwise the frequency set by hand.
uint16_t knockCenterHz() {
  if (config.knock.autoCenterFromBore) {
    return knockLearnedHz > 0.0f ? static_cast<uint16_t>(knockLearnedHz + 0.5f) : knockBoreHz();
  }
  return static_cast<uint16_t>(clampFloatValue(config.knock.centerFreqHz, 2000.0f, 9000.0f));
}

// The learned frequency is kept in flash with the bore it belongs to.
void loadKnockFrequency() {
  Preferences store;
  if (!store.begin("knockfreq", true)) return;
  const uint16_t bore = store.getUShort("bore", 0), hz = store.getUShort("hz", 0);
  store.end();
  const float expected = knockBoreHz();
  if (bore != knockBoreHz() || hz < expected * knock_spectrum::kSearchBelow || hz > expected * knock_spectrum::kSearchAbove) return;
  knockLearnedHz = knockSavedHz = hz;
}

// Setting the frequency by hand discards what was learned: going back to automatic
// then starts again from the bore's figure. It leaves the flash from loop(), away from
// boost (flashWriteAllowed()).
bool knockFrequencyErasePending = false;

void forgetKnockFrequency() {
  knockLearnedHz = knockSavedHz = 0.0f;
  knockFinder.begin(knockBoreHz());
  knockFrequencyErasePending = true;
}

void eraseKnockFrequency() {
  knockFrequencyErasePending = false;
  Preferences store;
  if (!store.begin("knockfreq", false)) return;
  store.remove("hz");
  store.end();
}

void saveKnockFrequency() {
  knockFrequencyErasePending = false;   // Overwritten anyway
  Preferences store;
  if (!store.begin("knockfreq", false)) return;
  store.putUShort("bore", knockBoreHz());
  store.putUShort("hz", static_cast<uint16_t>(knockLearnedHz + 0.5f));
  store.end();
  knockSavedHz = knockLearnedHz;
}

knock_dsp::Settings knockSettings() {
  knock_dsp::Settings s;
  s.offset = clampFloatValue(config.knock.thresholdOffset, 0.0f, 255.0f);
  s.multiplier = clampFloatValue(config.knock.thresholdMultiplier, 0.5f, 10.0f);
  s.gain = clampFloatValue(config.knock.signalGain, 0.1f, 15.9f);
  s.holdMs = config.knock.eventCooldownMs < 100 ? 100 : config.knock.eventCooldownMs;
  return s;
}

// Two band widths, as on the Nano: the bandwidth setting chooses the narrow one below 1 kHz.
float knockBandwidthHz() { return config.knock.bandwidthHz < 1000.0f ? 900.0f : 1500.0f; }

// ---------------------------------------------------------------------------
// Knock amplifier gain (X9C104 potentiometer, x9c.h) under automatic control
// (knock_agc.h). The chip cannot be asked where its wiper is, so it is walked to the
// bottom at start-up and again whenever the engine is stopped.
// ---------------------------------------------------------------------------
struct KnockGainPins {
  void select(bool on) { digitalWrite(pins::KNOCK_GAIN_CS, on ? LOW : HIGH); }
  void inc(bool high) { digitalWrite(pins::KNOCK_GAIN_INC, high ? HIGH : LOW); }
  void up(bool up) { digitalWrite(pins::KNOCK_GAIN_UD, up ? HIGH : LOW); }
  void wait() { delayMicroseconds(5); }
};
KnockGainPins knockGainPins;
x9c::Pot knockPot;
knock_agc::Controller knockGain;
knock_agc::Circuit knockCircuit;
bool knockGainAuto = false;         // False: fixed-gain amplifier, held from the dash, or the pot shows no effect
bool knockGainGaveUp = false;       // Gain changes showed no effect
uint32_t knockGainResyncMs = 0;
constexpr uint32_t kKnockGainResyncMs = 60000;

void beginKnockGain() {
  pinMode(pins::KNOCK_GAIN_CS, OUTPUT);
  pinMode(pins::KNOCK_GAIN_INC, OUTPUT);
  pinMode(pins::KNOCK_GAIN_UD, OUTPUT);
  digitalWrite(pins::KNOCK_GAIN_CS, HIGH);
  digitalWrite(pins::KNOCK_GAIN_INC, HIGH);
  digitalWrite(pins::KNOCK_GAIN_UD, LOW);
  knockCircuit.lowGain = KNOCK_GAIN_AT_BOTTOM;
  knockCircuit.highGain = KNOCK_GAIN_AT_TOP;
  knockCircuit.topStep = KNOCK_GAIN_TOP_STEP;
  knockCircuit.railLow = KNOCK_ADC_LOW_MV;
  knockCircuit.railHigh = KNOCK_ADC_HIGH_MV;
  knockPot.setTopStep(KNOCK_GAIN_TOP_STEP);
  knockGain.begin(knockCircuit, 0);  // Lowest gain first: it cannot clip there.
  knockPot.home(knockGainPins);
  knockGainAuto = KNOCK_GAIN_POT != 0 && knockGainFixedStep < 0;
}

// Knock settings changed from the dash, kept in flash so they are still there after
// the ignition has been off. Version 1.
struct KnockSaved {
  uint8_t version;
  uint8_t enabled;
  uint8_t autoCentre;
  int8_t fixedStep;        // -1 = automatic gain
  uint16_t minRpm;
  uint16_t holdMs;
  float offset, multiplier, minMapKpa, levelGain, bandwidthHz, centreHz, boreMm;
};

void loadKnockSettings() {
  Preferences store;
  if (!store.begin("knockcfg", true)) return;
  KnockSaved saved;
  const bool read = store.getBytes("settings", &saved, sizeof(saved)) == sizeof(saved) && saved.version == 1;
  store.end();
  if (!read) return;
  config.knock.enabled = saved.enabled != 0;
  config.knock.autoCenterFromBore = saved.autoCentre != 0;
  config.knock.minRpmToArm = saved.minRpm > 8000 ? 8000 : saved.minRpm;
  config.knock.eventCooldownMs = saved.holdMs;
  config.knock.thresholdOffset = clampFloatValue(saved.offset, 0.0f, 200.0f);
  config.knock.thresholdMultiplier = clampFloatValue(saved.multiplier, 1.2f, 3.8f);
  config.knock.minMapKpaToArm = clampFloatValue(saved.minMapKpa, 0.0f, 250.0f);
  config.knock.boostEnableKpa = config.knock.minMapKpaToArm;
  config.knock.signalGain = clampFloatValue(saved.levelGain, 0.1f, 20.0f);
  config.knock.bandwidthHz = clampFloatValue(saved.bandwidthHz, 300.0f, 20000.0f);
  config.knock.centerFreqHz = clampFloatValue(saved.centreHz, 3000.0f, 25000.0f);
  config.knock.boreMm = clampFloatValue(saved.boreMm, 40.0f, 200.0f);
  knockGainFixedStep = saved.fixedStep < 0 ? -1 : saved.fixedStep > KNOCK_GAIN_TOP_STEP ? KNOCK_GAIN_TOP_STEP : saved.fixedStep;
}

void saveKnockSettings() {
  KnockSaved saved = {};
  saved.version = 1;
  saved.enabled = config.knock.enabled ? 1 : 0;
  saved.autoCentre = config.knock.autoCenterFromBore ? 1 : 0;
  saved.fixedStep = static_cast<int8_t>(knockGainFixedStep);
  saved.minRpm = config.knock.minRpmToArm;
  saved.holdMs = config.knock.eventCooldownMs;
  saved.offset = config.knock.thresholdOffset;
  saved.multiplier = config.knock.thresholdMultiplier;
  saved.minMapKpa = config.knock.minMapKpaToArm;
  saved.levelGain = config.knock.signalGain;
  saved.bandwidthHz = config.knock.bandwidthHz;
  saved.centreHz = config.knock.centerFreqHz;
  saved.boreMm = config.knock.boreMm;
  Preferences store;
  if (!store.begin("knockcfg", false)) return;
  store.putBytes("settings", &saved, sizeof(saved));
  store.end();
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

// What a reset leaves behind (reset_guard.h), in memory a reset does not clear: the crash
// count, and whether the board was armed and the pump running when it came.
RTC_NOINIT_ATTR reset_guard::Record resetRecord;

void setPump(float dutyPercent) {
  if (!(dutyPercent > 0.0f)) dutyPercent = 0.0f;
  if (dutyPercent > 100.0f) dutyPercent = 100.0f;
  pumpDutyPercent = dutyPercent;
  const uint32_t top = (1UL << PUMP_PWM_BITS) - 1;
  const uint32_t now = millis();
  pumpSetMs = now;
  pumpRunning = dutyPercent > 0.0f;
  reset_guard::notePump(resetRecord, pumpRunning);
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
volatile uint32_t pumpKeeperTrips = 0;   // Times the keeper had to step in (shown on the serial heartbeat)

void keepPumpSafe() {
  if ((pumpRunning || relayClosed) && static_cast<int32_t>(millis() - pumpSetMs) > kPumpKeeperMs) {
    pumpRunning = false;
    relayClosed = false;
    ledcWrite(pins::PUMP_OUT, 0);
    digitalWrite(pins::PUMP_RELAY_OUT, LOW);
    pumpKeeperTrips = pumpKeeperTrips + 1;
  }
}

// When the firmware itself fails, the pump stops before anything else happens. A crash
// (an exception, the task or interrupt watchdog) runs this first, then prints its report
// and writes a core dump to flash before the chip resets: that takes long enough to
// matter, and the PWM hardware would go on driving the pump through all of it. Any
// restart runs it too. Registers only: nothing here may need a lock, the flash or
// another task. After the reset the pins float and the board's pull-downs hold the
// MOSFET and the relay off.
void IRAM_ATTR pumpOffNow() {
  gpio_ll_set_level(&GPIO, pins::PUMP_OUT, 0);
  gpio_ll_set_level(&GPIO, pins::PUMP_RELAY_OUT, 0);
  // The pump pin belongs to the PWM peripheral: give it back to the plain output, now low.
  esp_rom_gpio_connect_out_signal(pins::PUMP_OUT, SIG_GPIO_OUT_IDX, false, false);
}
// Left by the crash handler in memory a reset does not clear, so the next start can say
// whether the pump was switched off before the reset.
RTC_NOINIT_ATTR uint32_t crashHookMark;
constexpr uint32_t kCrashHookRan = 0x50554D50UL;   // "PUMP"
void IRAM_ATTR pumpOffAtCrash(arduino_panic_info_t *, void *) {
  pumpOffNow();
  crashHookMark = kCrashHookRan;
}
void pumpOffAtRestart() { pumpOffNow(); }

// ---------------------------------------------------------------------------
// Crash loops (reset_guard.h): counted across resets in memory a reset does not clear.
// ---------------------------------------------------------------------------
constexpr uint32_t kWatchdogMs = 2000;   // Main loop or CAN task silent this long: restart
reset_guard::Cause bootCause = reset_guard::Cause::Other;
bool crashHold = false;   // Too many crashes since the ignition came on: injection held off
uint32_t crashHoldEndsMs = 0;   // ...until this long after the start (reset_guard::holdMs)

// The dash cleared faults: crashes so far are forgiven and injection is allowed again.
void clearCrashHold() {
  if (crashHold) Serial.println(F("SAFETY: crash hold cleared; injection allowed again"));
  reset_guard::clearCrashes(resetRecord);
  crashHold = false;
}

// The hold has rested its time: injection is tried again (the next hold is longer).
void endCrashHoldWhenDue(uint32_t now) {
  if (!crashHold || static_cast<int32_t>(now - crashHoldEndsMs) < 0) return;
  reset_guard::endHold(resetRecord);
  crashHold = false;
  Serial.printf("SAFETY: crash hold over after %lu s; injection allowed again (a next hold would last %lu s)\n",
                static_cast<unsigned long>(crashHoldEndsMs / 1000), static_cast<unsigned long>(reset_guard::holdMs(resetRecord) / 1000));
}

// Armed or not, kept where a reset finds it: after a crash, a supply dip or a restart
// over a fault the board comes back armed by itself (reset_guard::comesBackArmed).
void setArmed(bool armed) {
  config.mode = armed ? InjectionMode::BoostOnly : InjectionMode::Off;
  reset_guard::noteArmed(resetRecord, armed);
}

reset_guard::Cause reportedResetCause(esp_reset_reason_t reason) {
  using reset_guard::Cause;
  switch (reason) {
  case ESP_RST_POWERON: return Cause::PowerOn;
  case ESP_RST_SW:
  case ESP_RST_EXT:
  case ESP_RST_USB:
  case ESP_RST_JTAG: return Cause::Requested;
  case ESP_RST_PANIC:
  case ESP_RST_INT_WDT:
  case ESP_RST_TASK_WDT:
  case ESP_RST_WDT:
  case ESP_RST_CPU_LOCKUP: return Cause::Crash;
  case ESP_RST_BROWNOUT:
  case ESP_RST_PWR_GLITCH: return Cause::Brownout;
  default: return Cause::Other;
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
// Hearing nothing. With the ignition on the ECU broadcasts many times a second and the
// dash sends engine data, so a bus with nothing on it means the CAN controller or its
// wiring has stopped working, even when its registers look right (a stuck SPI line reads
// back as "normal mode"). The Nano went that way on its tenth spray in November 2026 and
// stayed silent until the ignition was cycled. So: silent for kCanSilentMs, set the
// controller up again (every kCanSilentRetryMs while it lasts); still silent after
// kCanRestartAfterMs, restart the whole board (reset_guard::kCanRestartsAllowed per
// ignition cycle). Only once the bus has worked since the ignition came on: a board on
// the bench with nothing connected is left alone.
uint32_t lastCanRxMs = 0;
uint32_t lastCanInitMs = 0;
bool canHeardSinceBoot = false;
uint32_t canSilentResets = 0;              // Times it was set up again for silence (heartbeat)
constexpr uint32_t kCanSilentMs = 1500;
constexpr uint32_t kCanSilentRetryMs = 2000;
constexpr uint32_t kCanRestartAfterMs = 15000;

// The bus has worked this ignition cycle: frames since this start, or the last start was
// a restart over a silent bus (it had worked before that).
bool canHeardOnce() { return canHeardSinceBoot || bootCause == reset_guard::Cause::CanRestart; }

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
  // Never waits for ever: if the lock is not free, this is tried again in two seconds.
  if (xSemaphoreTake(canLock, pdMS_TO_TICKS(50)) != pdTRUE) return false;
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
  lastCanInitMs = millis();
  xSemaphoreGive(canLock);
  Serial.println(ok ? F("CAN: MCP2515 ready, 500 kbit/s") : F("CAN: MCP2515 not answering; will retry"));
  return ok;
}

#if METH_SAFETY_TEST
volatile bool safetyTestHangCanTask = false;
#endif

void canReceiveTask(void *) {
  // This task carries the pump keeper, so it is watched like the main loop: silent for
  // kWatchdogMs and the board restarts (through pumpOffAtCrash).
  esp_task_wdt_add(nullptr);
  for (;;) {
#if METH_SAFETY_TEST
    while (safetyTestHangCanTask) {
    }
#endif
    esp_task_wdt_reset();
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
  const bool silent = canHeardOnce() && elapsed(now, lastCanRxMs, kCanSilentMs) && elapsed(now, lastCanInitMs, kCanSilentRetryMs);
  if ((control & 0xE8) == 0x00 && !(errors & 0x20) && consecutiveCanTxFailures < 64 && !silent) return true;
  if (silent) {
    canSilentResets = canSilentResets + 1;
    Serial.printf("CAN: nothing heard for %lu ms; setting the controller up again\n", static_cast<unsigned long>(now - lastCanRxMs));
  } else {
    Serial.println(F("CAN: controller lost its settings or went bus-off; setting it up again"));
  }
  canOnline = false;
  lastCanRetryMs = now - kCanRetryMs;
  return false;
}

// Silent however often the controller is set up again: restart the board, which resets
// the SPI hardware and everything else too. Injection is impossible meanwhile anyway: the
// manifold pressure comes over CAN. Comes back armed if it was (reset_guard.h).
void restartIfCanStaysSilent(uint32_t now) {
  if (!canHeardOnce() || !elapsed(now, lastCanRxMs, kCanRestartAfterMs) || !reset_guard::mayRestartForCan(resetRecord)) return;
  Serial.printf("FAULT: nothing on CAN for %lu s however often it was set up again; restarting the board\n",
                static_cast<unsigned long>((now - lastCanRxMs) / 1000));
  setPump(0.0f);
  reset_guard::markCanRestart(resetRecord);
  ESP.restart();
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
  if (!kKnockOn) return true;   // Nothing to report: knock is not built in.
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
  if (crashHold) return meth_tune::kHoldFault;
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

  setArmed(data[1] != 0U);
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
  const bool knockCommand = data[0] >= can_protocol::meth_command::KNOCK_SET_ENABLE &&
                            data[0] <= can_protocol::knock_detail_command::RELEARN_FREQUENCY;

  if (!kKnockOn && knockCommand) {
    // Knock is not built in (ESP32_KNOCK): refused, as the Nano refuses them.
    ackStatus = can_protocol::config_ack_status::UNSUPPORTED_COMMAND;
    commandAccepted = false;
  } else switch (data[0]) {
  case can_protocol::meth_command::ARM:
    if (len >= 2) {
      setArmed(data[1] != 0U);
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
      if (manualTestActive || config.mode != InjectionMode::Off || crashHold ||
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
    clearCrashHold();
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
      forgetKnockFrequency();
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
  case can_protocol::knock_detail_command::SET_AMPLIFIER_GAIN:
    if (len >= 2) {
      // 0 hands the gain back to the controller; n holds the potentiometer at position n - 1.
      const uint8_t value = constrainAndReport(data[1], 0, KNOCK_GAIN_TOP_STEP + 1, ackStatus);
      knockGainFixedStep = value == 0 ? -1 : value - 1;
      if (value == 0) knockGainGaveUp = false;   // Asked for again: it gets another try.
      ackValue = value;
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::knock_detail_command::SET_BORE_MM:
    if (len >= 2) {
      const uint8_t value = constrainAndReport(data[1], 40, 200, ackStatus);
      config.knock.boreMm = static_cast<float>(value);   // The loop restarts the frequency search for it.
      ackValue = value;
    } else {
      ackStatus = can_protocol::config_ack_status::INVALID_LENGTH;
    }
    break;
  case can_protocol::knock_detail_command::RELEARN_FREQUENCY:
    forgetKnockFrequency();
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

  // Any knock setting that was taken is written to flash shortly afterwards.
  if (commandAccepted && data[0] >= can_protocol::meth_command::KNOCK_SET_ENABLE &&
      data[0] <= can_protocol::knock_detail_command::RELEARN_FREQUENCY && data[0] != can_protocol::meth_command::KNOCK_CLEAR_EVENTS &&
      (ackStatus == can_protocol::config_ack_status::OK || ackStatus == can_protocol::config_ack_status::VALUE_CLAMPED)) {
    knockSettingsDirty = true;
    knockSettingsChangedMs = now;
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

// Settings are written to flash only away from boost. While flash is written both
// processors stop running from it, for tens of milliseconds and occasionally more: the
// pump holds its duty meanwhile, but the CAN task's keeper can find the main loop late
// afterwards and cut the pump mid-spray. What is waiting is written once boost is gone
// (the dash keeps showing "unsaved" until then).
bool flashWriteAllowed(uint32_t now) { return !injectionNeeded(now) && !updater.active(); }

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
  if (in.data[0] == fw::RESTART && in.data[1] == fw::TARGET_WATER_METH && injectionNeeded(now)) {
    transmitCanFrame(fw::packAck(fw::TARGET_WATER_METH, fw::RESTART, fw::STATUS_BUSY), "0x681");
    return;   // Not while injecting: asked again once the car is parked
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
    lastCanRxMs = now;            // Anything at all: the bus and the controller are working.
    canHeardSinceBoot = true;
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
  if (crashHold || result.failsafe != FailsafeReason::None || result.overboostAssistFaultLatched) {
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
  // Held off after repeated crashes (reset_guard.h), so the dash can say why and that
  // clearing faults releases it (contract extension 8). The Nano never sets it.
  if (crashHold) flags |= can_protocol::meth_fault_flag::CRASH_HOLD;
  return flags;
}

// Why this start happened, told to the dash once the bus is up (contract extension 8),
// so a restart in the middle of a drive is in its event log with the reason.
bool startReportPending = true;
uint8_t startReportFlags = 0;

void sendStartReportOnce(uint32_t now) {
  if (!startReportPending || now < 1000) return;   // Let the bus settle after a start
  if (transmitCanFrame(can_protocol::packMethStartReport(static_cast<uint8_t>(esp_reset_reason()), startReportFlags), "0x302-start")) {
    startReportPending = false;
  }
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
  restartIfCanStaysSilent(now);
  if (!canOnline) {
    if (elapsed(now, lastCanRetryMs, kCanRetryMs)) {
      lastCanRetryMs = now;
      canOnline = initCanBus();
      if (canOnline) consecutiveCanTxFailures = 0;
    }
    return;
  }

  if (!serviceCanHealth(now)) return;
  sendStartReportOnce(now);
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
    state.status_flags |= (kKnockOn && config.knock.enabled) ? (1U << 0) : 0U;
    state.status_flags |= knockSignalValid ? (1U << 1) : 0U;
    state.status_flags |= knockWarningActive ? (1U << 2) : 0U;
    state.status_flags |= knockCriticalActive ? (1U << 3) : 0U;
    state.status_flags |= knockBaselineLearned ? (1U << 4) : 0U;
    state.status_flags |= knockSensorFault ? (1U << 5) : 0U;
    state.status_flags |= knockClippingDetected ? (1U << 6) : 0U;
    state.energy = knock_dsp::toByte(knockDetector.level());
    state.baseline = knock_dsp::toByte(knockDetector.baseline());
    state.threshold = knock_dsp::toByte(knockDetector.threshold());
    state.event_count = knockEventCount;
    state.last_event_rpm_div100 = lastKnockRpmDiv100;
    state.last_event_boost_kpa = lastKnockBoostKpa;
    // Spare byte: how much of the signal sits at the knock frequency, 0..255.
    state.reserved = knock_dsp::toByte(knockDetector.share() * 255.0f);

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

  if (kKnockOn && elapsed(now, lastKnockHookTxMs, kKnockHookTxMs)) {
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
    hook.live_knock_rms = knock_dsp::toByte(knockDetector.level());
    hook.adaptive_threshold = knock_dsp::toByte(knockDetector.threshold());
    hook.adaptive_baseline = knock_dsp::toByte(knockDetector.baseline());
    hook.event_count = knockEventCount;
    // The dash turns these two into volts as the Nano's ADC counts them (5 V over 1024
    // counts). They are sent in that scale, from the calibrated voltage at the pin, so
    // "amplifier rests at" reads the true voltage on this board too.
    const float nanoCountsPerMv = 1024.0f / 5000.0f;
    hook.bias_adc_div16 = can_protocol::clampU8(static_cast<int>(knockStats.restMv * nanoCountsPerMv) / 16);
    hook.raw_adc_div16 = can_protocol::clampU8(knockRaw / 16);
    // Largest swing in the burst, in half counts: for setting the amplifier gain.
    hook.envelope_level = can_protocol::clampU8(static_cast<int>((knockStats.highMv - knockStats.lowMv) * nanoCountsPerMv) / 4);

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

  // ---- Knock detail for the dash's knock page (0x30E); none when knock is not built in ----
  if (!kKnockOn) return;
  if (elapsed(now, lastKnockDetailTxMs, kKnockDetailTxMs)) {
    lastKnockDetailTxMs = now;
    namespace amp = can_protocol::knock_amplifier_flag;
    uint8_t ampFlags = 0;
    if (knockGainAuto) ampFlags |= amp::AUTOMATIC;
    if (knockGainGaveUp) ampFlags |= amp::INEFFECTIVE;
    if (knockGainAuto && knockGain.overloaded()) ampFlags |= amp::OVERLOADED;
    if (knockGainAuto && knockGain.silent()) ampFlags |= amp::SILENT;
    if (knockGainFixedStep >= 0) ampFlags |= amp::FIXED;
    const float swingMv = knockGainAuto ? knockGain.typicalPeak() : (knockStats.highMv - knockStats.lowMv) * 0.5f;
    if (!transmitCanFrame(can_protocol::packKnockAmplifier(
            ampFlags, can_protocol::clampU8(static_cast<int>(knockGain.gain() * 10.0f + 0.5f)), knockPot.step(), KNOCK_GAIN_TOP_STEP,
            static_cast<uint16_t>(knockStats.restMv > 0.0f ? knockStats.restMv : 0.0f), static_cast<uint16_t>(swingMv > 0.0f ? swingMv : 0.0f),
            static_cast<uint16_t>(knockGain.room())), "0x30E-amp")) {
      return;
    }
    namespace freq = can_protocol::knock_frequency_flag;
    uint8_t freqFlags = 0;
    if (config.knock.autoCenterFromBore) freqFlags |= freq::LEARNING;
    if (config.knock.autoCenterFromBore && knockLearnedHz > 0.0f) freqFlags |= freq::LEARNED;
    if (knockHeard.secondActive) freqFlags |= freq::SECOND_ACTIVE;
    const uint16_t votes = knockFinder.votes();
    if (!transmitCanFrame(can_protocol::packKnockFrequency(
            freqFlags, knockCenterHz(), static_cast<uint16_t>(knockFinder.estimateHz() + 0.5f),
            can_protocol::clampU8(static_cast<int>(knockFinder.agreement() * 100.0f + 0.5f)), can_protocol::clampU8(votes)), "0x30E-freq")) {
      return;
    }
    const uint32_t rate = adc_sampler::knockSamplesPerSecond();
    if (!transmitCanFrame(can_protocol::packKnockEngine(knockBoreHz(), static_cast<uint16_t>(knockToneHz + 0.5f),
                                                        static_cast<uint16_t>(rate > 65535 ? 65535 : rate)), "0x30E-engine")) {
      return;
    }
  }
  if (elapsed(now, lastKnockListeningTxMs, kKnockListeningTxMs)) {
    lastKnockListeningTxMs = now;
    namespace listening = can_protocol::knock_listening_flag;
    uint8_t flags = 0;
    if (knockDetector.notable()) flags |= listening::NOTABLE;
    if (knockDetector.confirmed()) flags |= listening::CONFIRMED;
    const float levelGain = clampFloatValue(config.knock.signalGain, 0.1f, 15.9f);
    const float crest = knockHeard.background > 0.0f ? knockHeard.level / knockHeard.background : 0.0f;
    if (!transmitCanFrame(can_protocol::packKnockListening(
            flags, knock_dsp::toByte(knockDetector.second()), knock_dsp::toByte(knockDetector.secondBaseline()),
            knock_dsp::toByte(knockHeard.background * 0.5f * levelGain), knock_dsp::toByte(crest * 10.0f)), "0x30E-listen")) {
      return;
    }
  }
  if (knockSpectrum.valid && elapsed(now, lastKnockSpectrumTxMs, kKnockSpectrumTxMs)) {
    lastKnockSpectrumTxMs = now;
    const uint8_t chunks = kd::SPECTRUM_POINTS / kd::SPECTRUM_PER_FRAME;
    if (knockSpectrumNext == 0) {
      // A new round: fix what both spectra are, so the chunks of one round belong together.
      for (uint8_t point = 0; point < kd::SPECTRUM_POINTS; ++point) {
        const float hz = kd::SPECTRUM_START_HZ + static_cast<float>(point) * kd::SPECTRUM_STEP_HZ;
        const float normal = knock_spectrum::valueAtHz(knockSpectrum.floor, knockSpectrum.binHz, hz);
        const float normalDb = 10.0f * log10f(normal > 1e-3f ? normal : 1e-3f);
        knockSpectrumNormal[point] = knock_dsp::toByte((normalDb + 10.0f) * 3.0f);
        knockSpectrumBurst[point] = knock_dsp::toByte(((knockBurstHeld ? knockBurstHoldDb[point] : normalDb) + 10.0f) * 3.0f);
      }
      knockBurstHeld = false;
    }
    const bool burst = knockSpectrumNext >= chunks;
    const uint8_t first = static_cast<uint8_t>((knockSpectrumNext % chunks) * kd::SPECTRUM_PER_FRAME);
    if (!transmitCanFrame(can_protocol::packKnockSpectrum(burst ? kd::SPECTRUM_BURST : kd::SPECTRUM_NORMAL, first,
                                                          (burst ? knockSpectrumBurst : knockSpectrumNormal) + first), "0x30E-spectrum")) {
      return;
    }
    knockSpectrumNext = static_cast<uint8_t>((knockSpectrumNext + 1) % (2 * chunks));
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
  // An unplugged sensor reads at the ADC's top and counts as missing (analog_inputs.h):
  // the temperature features then use the ECU's intake temperature instead.
  if (adc_sampler::averageMv(adc_sampler::kPre, millivolts) &&
      analog_inputs::thermistorC(millivolts, pins::INTAKE_TEMP_SUPPLY_MV, pins::INTAKE_TEMP_PULLUP_OHMS,
                                 adc_sampler::topMv(adc_sampler::kPre), sample)) {
    preTempC = preTempValid ? preTempC + (sample - preTempC) * 0.3f : sample;
    preTempValid = true;
  } else {
    preTempValid = false;
  }
  // After the nozzle: not fitted on this board (pins::POST_INJECTION_TEMP_FITTED); the
  // MicroSquirt's MAT stands for it on the dash.
  if (pins::POST_INJECTION_TEMP_FITTED && adc_sampler::averageMv(adc_sampler::kPost, millivolts) &&
      analog_inputs::thermistorC(millivolts, pins::INTAKE_TEMP_SUPPLY_MV, pins::INTAKE_TEMP_PULLUP_OHMS,
                                 adc_sampler::topMv(adc_sampler::kPost), sample)) {
    postTempC = postTempValid ? postTempC + (sample - postTempC) * 0.3f : sample;
    postTempValid = true;
  } else {
    postTempValid = false;
  }
}

// ---------------------------------------------------------------------------
// Knock, one pass every 20 ms from loop(); only when built in (ESP32_KNOCK).
// ---------------------------------------------------------------------------
bool knockDetected = false;   // A knock warning on this pass
int knockRaw = 0;             // The largest knock sample of the pass

void serviceKnock(uint32_t now, const SensorReadings &readings, uint16_t rpm) {
  // ---- Knock: the ADC task has filtered every sample of the last 20 ms (knock_dsp.h) ----
  adc_sampler::setKnockBand(knockCenterHz(), knockBandwidthHz());
  const size_t knockFrameCount = adc_sampler::takeKnockFrames(knockFrames, knock_dsp::kMaxFrames);
  for (size_t n = 0; n < knockFrameCount; ++n) knockAnalyser.add(knockFrames[n]);
  // The window that fits one knock gets shorter as the engine turns faster.
  const knock_dsp::Summary knockPass =
      knockAnalyser.finish(adc_sampler::knockSecondActive(), knock_dsp::windowFramesFor(engineRpmValid ? rpm : 0));
  if (knockPass.valid) knockHeard = knockPass;
  adc_sampler::KnockStats knockFresh;
  const bool knockRangeFresh = adc_sampler::takeKnockStats(knockFresh);
  if (knockRangeFresh) knockStats = knockFresh;
  const bool knockInputDead = !adcReady || adc_sampler::silentForMs(now) > 500;
  knockRaw = knockStats.high;   // The largest sample of the pass
  // With automatic gain a clipped burst is the gain control's business (it backs off at
  // once); it is a fault only when it clips with no gain left to take off, or when
  // nothing is heard at full gain with the engine at speed.
  knockClippingDetected = knockGainAuto ? knockGain.overloaded()
                                        : knockStats.samples > 0 && knockStats.railed * 3 >= knockStats.samples;
  // The amplifier should rest well inside what it can reach; far from that, the sensor
  // is not being heard.
  const float knockReach = KNOCK_ADC_HIGH_MV - KNOCK_ADC_LOW_MV;
  const bool knockRestWrong = knockStats.restMv < KNOCK_ADC_LOW_MV + 0.2f * knockReach ||
                              knockStats.restMv > KNOCK_ADC_HIGH_MV - 0.2f * knockReach;
  knockSensorFault = knockInputDead || knockClippingDetected || knockRestWrong || (knockGainAuto && knockGain.silent());
  const bool rpmArmed = config.knock.minRpmToArm == 0 || rpm >= config.knock.minRpmToArm;
  const bool mapArmed = readings.mapKpa >= config.knock.minMapKpaToArm;
  const bool knockArmed = config.knock.enabled && rpmArmed && mapArmed && !knockSensorFault;
  // The pass after a gain change spans both gains: it is not judged and not learned from.
  const bool knockJudged = knockPass.valid && !(knockGainAuto && knockGain.settling());
  if (knockJudged) knockDetector.update(now, knockHeard, knockSettings(), knockArmed, engineRpmValid ? rpm : 0);

  // ---- The knock frequency itself, from the spectrum (knock_spectrum.h) ----
  if (knockFinder.expectedHz() != knockBoreHz()) {
    // The bore setting changed: what was learned belonged to the other bore.
    knockFinder.begin(knockBoreHz());
    adc_sampler::setKnockExpected(knockBoreHz());
    knockLearnedHz = 0.0f;
  }
  knock_spectrum::Snapshot knockBlock;
  if (adc_sampler::takeKnockSpectrum(knockBlock)) {
    knockSpectrum = knockBlock;
    knockToneHz = knock_spectrum::steadyToneHz(knockSpectrum, knockBoreHz());
    // For the dash: the loudest any frequency has been in a block since the burst
    // spectrum was last sent.
    for (uint8_t point = 0; point < kd::SPECTRUM_POINTS; ++point) {
      const float hz = kd::SPECTRUM_START_HZ + static_cast<float>(point) * kd::SPECTRUM_STEP_HZ;
      const float power = knock_spectrum::valueAtHz(knockSpectrum.power, knockSpectrum.binHz, hz);
      const float db = 10.0f * log10f(power > 1e-3f ? power : 1e-3f);
      if (!knockBurstHeld || db > knockBurstHoldDb[point]) knockBurstHoldDb[point] = db;
    }
    knockBurstHeld = true;
    // Learned only from passes that are notable under load: more than normal at the
    // knock frequency, narrow-band, a burst, with the engine in boost.
    const bool underLoad = engineRpmValid && rpm >= 1500 && readings.mapValid && rpmArmed && mapArmed && !knockSensorFault;
    knockFinder.add(knockSpectrum, knockJudged && underLoad && knockDetector.notable());
    float moved = 0.0f;
    if (config.knock.autoCenterFromBore && knockFinder.decide(knockCenterHz(), moved)) {
      Serial.printf("KNOCK: filter moved from %u to %.0f Hz: that is where this engine rings (%u votes, %.0f %% agree)\n",
                    knockCenterHz(), moved, knockFinder.votes(), knockFinder.agreement() * 100.0f);
      knockLearnedHz = moved;
      knockDetector.relearn();   // Levels learned through the old filter no longer apply.
    }
  }
  const bool flashOk = flashWriteAllowed(now);
  if (flashOk && knockLearnedHz > 0.0f && fabsf(knockLearnedHz - knockSavedHz) >= 50.0f &&
      elapsed(now, knockSavedMs, kKnockSaveEveryMs)) {
    knockSavedMs = now;
    saveKnockFrequency();
  } else if (flashOk && knockFrequencyErasePending) {
    eraseKnockFrequency();
  }
  // Who sets the gain: the controller, or a position held from the dash.
  if (KNOCK_GAIN_POT != 0) {
    if (knockGainFixedStep >= 0) {
      if (knockGainAuto || knockPot.step() != knockGainFixedStep) {
        const float before = knockGain.gain();
        knockPot.moveTo(knockGainPins, static_cast<uint8_t>(knockGainFixedStep));
        knockGain.begin(knockCircuit, knockPot.step());
        knockDetector.rescale(knockGain.gain() / before);
        knockGainAuto = false;
      }
    } else if (!knockGainAuto && !knockGainGaveUp) {
      knockGain.begin(knockCircuit, knockPot.step());
      knockGainAuto = true;
    }
  }
  if (flashOk && knockSettingsDirty && elapsed(now, knockSettingsChangedMs, kKnockSettingsSaveMs)) {
    knockSettingsDirty = false;
    saveKnockSettings();
  }
  if (knockGainAuto) {
    const bool engineStopped = !engineRpmValid || rpm == 0;
    if (engineStopped && elapsed(now, knockGainResyncMs, kKnockGainResyncMs)) {
      // Nothing to listen to: a good moment to make sure the wiper is where it is thought to be.
      knockGainResyncMs = now;
      knockPot.resync(knockGainPins, knockGain.step());
    } else if (knockRangeFresh) {
      knock_agc::Sample sample;
      sample.low = knockStats.lowMv;
      sample.high = knockStats.highMv;
      sample.rest = knockStats.restMv;
      sample.rpm = rpm;
      sample.rpmValid = engineRpmValid;
      sample.knocking = knockDetector.learned() && knockDetector.level() >= knockDetector.threshold();
      if (knockGain.update(sample)) {
        knockPot.moveTo(knockGainPins, knockGain.step());
        knockDetector.rescale(knockGain.ratioX256() / 256.0f);
      }
      if (knockGain.ineffective()) {
        knockGainAuto = false;
        knockGainGaveUp = true;
        Serial.println(F("KNOCK: changing the amplifier gain shows no effect (potentiometer not fitted or not "
                         "working); gain control is off until the next start or until it is asked for again"));
      }
    }
  }
  knockBaselineLearned = knockDetector.learned();
  knockSignalValid = knockArmed && knockBaselineLearned;
  knockWarningActive = knockDetector.warning();
  knockCriticalActive = knockDetector.critical();
  knockDetected = knockWarningActive;

  if (knockWarningActive && !lastKnockWarningActive) {
    queueKnockFault(can_protocol::knock_fault_code::KNOCK_WARNING,
                    static_cast<uint8_t>(can_protocol::FaultSeverity::WARNING),
                    knock_dsp::toByte(knockDetector.level()),
                    boostGaugeKpaForCan(readings));
  }
  if (knockCriticalActive && !lastKnockCriticalActive) {
    queueKnockFault(can_protocol::knock_fault_code::KNOCK_CRITICAL,
                    static_cast<uint8_t>(can_protocol::FaultSeverity::CRITICAL),
                    knock_dsp::toByte(knockDetector.level()),
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
}

#if METH_SAFETY_TEST
// One letter on the serial monitor, then Enter. Each failure starts with the pump on at
// 30 % (have nothing connected to it, or no fluid in the line), so the layer that
// switches it off has something to do.
void serviceSafetyTest() {
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == 's') {
      Serial.println(F("TEST s: pump on, then the main loop stalls for 600 ms"));
      setPump(30.0f);
      const uint32_t start = millis();
      uint32_t offAfterMs = 0;
      while (millis() - start < 600) {
        if (!offAfterMs && !pumpRunning) offAfterMs = millis() - start;
        delay(1);
      }
      Serial.printf("TEST s: the CAN task switched the pump off after %lu ms; PWM duty now %lu, relay %s\n",
                    static_cast<unsigned long>(offAfterMs), static_cast<unsigned long>(ledcRead(pins::PUMP_OUT)),
                    digitalRead(pins::PUMP_RELAY_OUT) ? "CLOSED" : "open");
    } else if (c == 'h') {
      Serial.println(F("TEST h: pump on, then the main loop hangs: the keeper at 250 ms, the watchdog at 2 s"));
      setPump(30.0f);
      delay(20);
      for (;;) {
      }
    } else if (c == 'c') {
      Serial.println(F("TEST c: the CAN task hangs, pump keeper and all: the watchdog at 2 s"));
      safetyTestHangCanTask = true;
    } else if (c == 'x') {
      Serial.println(F("TEST x: pump on, then a crash (a write through a null pointer)"));
      setPump(30.0f);
      delay(20);
      volatile int *nowhere = nullptr;
      *nowhere = 1;
    } else if (c == 'i') {
      Serial.println(F("TEST i: pump on, then this core stops with interrupts off: the interrupt watchdog at 300 ms"));
      setPump(30.0f);
      delay(20);
      portDISABLE_INTERRUPTS();
      for (;;) {
      }
    } else if (c == 'f') {
      Serial.println(F("TEST f: clear faults, as the dash does"));
      clearCrashHold();
    } else if (c == '?') {
      Serial.println(F("TEST: s stall 600 ms | h hang loop | c hang CAN task | x crash | i interrupts off | f clear faults"));
    }
  }
}
#endif
} // namespace

void setup() {
  // The pump is off before anything else happens.
  pinMode(pins::PUMP_RELAY_OUT, OUTPUT);
  digitalWrite(pins::PUMP_RELAY_OUT, LOW);
  pinMode(pins::PUMP_OUT, OUTPUT);
  digitalWrite(pins::PUMP_OUT, LOW);
  const bool pumpReady = ledcAttach(pins::PUMP_OUT, PUMP_PWM_HZ, PUMP_PWM_BITS);
  ledcWrite(pins::PUMP_OUT, 0);
  // From here on, a crash or a restart switches the pump off before anything else.
  set_arduino_panic_handler(pumpOffAtCrash, nullptr);
  esp_register_shutdown_handler(pumpOffAtRestart);

  // What the last reset was, and whether this ignition cycle has seen too many crashes.
  bootCause = reset_guard::boot(resetRecord, reportedResetCause(esp_reset_reason()));
  crashHold = reset_guard::injectionHeld(resetRecord);
  crashHoldEndsMs = reset_guard::holdMs(resetRecord);
  const bool crashHookRan = crashHookMark == kCrashHookRan;
  crashHookMark = 0;
  // What was going on when the reset came, then the pump recorded as off (it is).
  const bool pumpWasOn = resetRecord.pumpOn != 0;
  const bool armedAgain = reset_guard::comesBackArmed(resetRecord, bootCause);
  reset_guard::notePump(resetRecord, false);
  const uint32_t softStartMs = reset_guard::softStartMs(resetRecord, meth_tune::kSoftStartMs);
  governor.setSoftStartMs(softStartMs);

  Serial.begin(config.serialBaud);
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  // The USB serial waits up to 100 ms per print for a computer that is plugged in but
  // not reading (the serial monitor closed). The once-a-second report and the spectrum
  // are dozens of prints: seconds of stalled control. Prints are dropped instead.
  Serial.setTxTimeoutMs(0);
#endif
  if (!pumpReady) Serial.println(F("BOOT: pump PWM did not start; the pump will not run"));
  // Disarmed (fail safe) until the dash arms this controller; but after a reset nobody
  // asked for (a crash, a supply dip, a restart over a fault) armed again if it was, so
  // injection carries on without waiting for the dash.
  setArmed(armedAgain);
  Serial.println();
  Serial.printf("BOOT: ESP32-S3 water/meth and knock firmware, build %08lX\n", static_cast<unsigned long>(METH_FW_BUILD));
  Serial.printf("BOOT: last reset: %s%s; crashes since the ignition came on: %u\n", reset_guard::name(bootCause),
                pumpWasOn ? ", with the pump running" : "", static_cast<unsigned>(resetRecord.crashes));
  if (bootCause == reset_guard::Cause::Crash) {
    Serial.println(crashHookRan ? F("BOOT: the crash handler switched the pump off before the reset")
                                : F("BOOT: the crash handler did not run (the pull-downs held the pump off once the pins floated)"));
  }
  if (armedAgain) Serial.println(F("BOOT: armed again, as before the reset"));
  if (softStartMs > meth_tune::kSoftStartMs) {
    Serial.printf("BOOT: %u resets with the pump running this ignition cycle: soft start %lu ms\n",
                  static_cast<unsigned>(resetRecord.pumpResets), static_cast<unsigned long>(softStartMs));
  }
  if (crashHold) {
    Serial.printf("SAFETY: %u crashes since the ignition came on: injection held off for %lu s, then tried again "
                  "(at once if the dash clears faults)\n", static_cast<unsigned>(resetRecord.crashes),
                  static_cast<unsigned long>(crashHoldEndsMs / 1000));
  }

  pinMode(pins::WARNING_LED, OUTPUT);
  digitalWrite(pins::WARNING_LED, LOW);
  pinMode(pins::FLOAT_SENSOR_DIGITAL, INPUT_PULLUP);
  floatSensor.begin(rawFloatLow(), millis(), config.floatDebounceMs, config.floatLowShutdownDelayMs);

  // Saved tuning, or the conservative defaults on a new or damaged store.
  if (!loadTune()) meth_tune::loadDefaults(tune);
  syncTuneToConfig();

  // Watchdogs. The task watchdog (a hardware timer) restarts the board if the main loop
  // or the CAN task, which carries the pump keeper, goes silent for kWatchdogMs. The
  // interrupt watchdog (300 ms) catches a processor stuck with interrupts off; the
  // brownout detector resets on a supply dip. Each goes through pumpOffAtCrash first.
  // A firmware that hangs while on trial goes back to the previous one.
  esp_task_wdt_config_t watchdog = {};
  watchdog.timeout_ms = kWatchdogMs;
  watchdog.idle_core_mask = 1U << 0;   // Core 0's idle task too, as the framework has it
  watchdog.trigger_panic = true;
  if (esp_task_wdt_reconfigure(&watchdog) != ESP_OK) Serial.println(F("BOOT: watchdog kept at its default (5 s)"));
  enableLoopWDT();
  esp_ota_img_states_t imageState;
  const esp_partition_t *running = esp_ota_get_running_partition();
  updater.setOnTrial(running && esp_ota_get_state_partition(running, &imageState) == ESP_OK &&
                     imageState == ESP_OTA_IMG_PENDING_VERIFY);
  if (updater.onTrial()) Serial.println(F("BOOT: new firmware, on trial until the dash confirms it"));
  {
    namespace sf = can_protocol::meth_start_flag;
    using reset_guard::Cause;
    startReportFlags = static_cast<uint8_t>((pumpWasOn ? sf::PUMP_WAS_ON : 0U) | (armedAgain ? sf::ARMED_AGAIN : 0U) |
                                            (crashHold ? sf::CRASH_HOLD : 0U) | (updater.onTrial() ? sf::ON_TRIAL : 0U) |
                                            (bootCause == Cause::CanRestart ? sf::RESTART_SILENT_CAN : 0U) |
                                            (bootCause == Cause::FaultRestart ? sf::RESTART_ADC : 0U) |
                                            (softStartMs > meth_tune::kSoftStartMs ? sf::GENTLE_START : 0U));
  }

  if (kKnockOn) {
    // On the Nano the threshold offset covers its coarse measurement (8 half counts).
    // This board measures finely, and an offset that size hides knock when the signal is
    // small: replaying the owner's LTspice simulation, 8 missed every knock at low
    // amplitudes that 2 catches. The dash can still set it.
    config.knock.thresholdOffset = 2.0f;
    loadKnockSettings();   // What was set from the dash, if anything.
    loadKnockFrequency();
    knockFinder.begin(knockBoreHz());
    adc_sampler::setKnockExpected(knockBoreHz());
    if (knockLearnedHz > 0.0f) {
      Serial.printf("BOOT: knock filter at %.0f Hz, learned earlier (the bore suggests %u)\n", knockLearnedHz, knockBoreHz());
    }
    beginKnockGain();
  } else {
    config.knock.enabled = false;
    // The potentiometer's control pins held at rest (deselected), so nothing floating
    // can step it.
    pinMode(pins::KNOCK_GAIN_CS, OUTPUT);
    pinMode(pins::KNOCK_GAIN_INC, OUTPUT);
    pinMode(pins::KNOCK_GAIN_UD, OUTPUT);
    digitalWrite(pins::KNOCK_GAIN_CS, HIGH);
    digitalWrite(pins::KNOCK_GAIN_INC, HIGH);
    digitalWrite(pins::KNOCK_GAIN_UD, LOW);
    Serial.println(F("BOOT: knock listening is off in this build (ESP32_KNOCK=0)"));
  }
  adc_sampler::setKnockLimitsMv(KNOCK_ADC_LOW_MV, KNOCK_ADC_HIGH_MV);
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
#if METH_SAFETY_TEST
  serviceSafetyTest();
#endif
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
  // The longest gap between control passes since the last heartbeat (normally 20 ms):
  // anything near the pump keeper's 250 ms is worth knowing about.
  if (lastLoopMs != 0 && now - lastLoopMs > loopGapMaxMs) loopGapMaxMs = now - lastLoopMs;
  lastLoopMs = now;
  endCrashHoldWhenDue(now);
  if (pumpKeeperTrips != keeperTripsReported) {
    keeperTripsReported = pumpKeeperTrips;
    Serial.printf("SAFETY: the main loop went %ld ms without setting the pump; the CAN task switched it off (%lu times)\n",
                  static_cast<long>(kPumpKeeperMs), static_cast<unsigned long>(keeperTripsReported));
  }

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

  if (kKnockOn) serviceKnock(now, readings, rpm);

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
    // Held off after repeated crashes (reset_guard.h): the governor then reports a fault.
    inputs.wantSpray = result.pump.enabled && !crashHold;
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
  if (tuneSavePending && flashWriteAllowed(now)) {
    saveTune();
    tuneSavePending = false;
    tuneUnsaved = false;
  }
  digitalWrite(pins::WARNING_LED, (crashHold || result.failsafe != FailsafeReason::None ||
                                   result.overboostAssistFaultLatched || knockDetected) ? HIGH : LOW);

  serviceCanBus(now, readings, result, knockDetected, knockRaw);

  // The ADC feeds knock, the pressures and the air temperature sensors: if it has gone
  // quiet, start again from reset. Twice at most per ignition cycle (reset_guard.h);
  // after that the board carries on without it: knock reports a sensor fault, the
  // pressures read as missing, and injection uses the ECU's intake temperature.
  if (adcReady && adc_sampler::silentForMs(now) > 3000) {
    if (reset_guard::mayRestartForFault(resetRecord)) {
      Serial.println(F("FAULT: no data from the ADC for 3 s; restarting"));
      setPump(0.0f);
      reset_guard::markFaultRestart(resetRecord);
      ESP.restart();
    }
    Serial.println(F("FAULT: no data from the ADC for 3 s, and restarting has not cured it; carrying on without it "
                     "(no knock, pressures or air temperature sensors)"));
    adcReady = false;
  }

  if (knockSpectrum.valid && elapsed(now, knockSpectrumPrintMs, kKnockSpectrumPrintMs)) {
    knockSpectrumPrintMs = now;
    // What each frequency normally carries, in dB (10 dB = ten times the power): the
    // engine's own spectrum. Peaks that are always there are the block and steady tones.
    Serial.print(F("KSPEC kHz:dB"));
    for (uint8_t bin = 10; bin < knock_spectrum::kBins && bin * knockSpectrum.binHz <= 12000.0f; bin += 2) {
      const float power = 0.5f * (knockSpectrum.floor[bin] + knockSpectrum.floor[bin + 1]);
      Serial.printf(" %.1f:%.0f", (bin + 0.5f) * knockSpectrum.binHz / 1000.0f, 10.0f * log10f(power > 1e-6f ? power : 1e-6f));
    }
    Serial.println();
  }

  if (elapsed(now, lastDebugMs, kSerialHeartbeatMs)) {
    lastDebugMs = now;
    Serial.printf("HB t=%lu map=%.1f%s boostPsi=%.2f baro=%.1f rpm=%u%s duty=%.0f%% hold=%u mode=%u tankLow=%d can=%d "
                  "oilPsi=%.1f%s fuelPsi=%.1f%s pre=%.1f%s knock=%d krest=%umV knockHz=%lu klvl=%.1f kbase=%.1f k2=%.1f kshare=%.2f kfreq=%u kfound=%.0f ktone=%.0f kgain=%.1f%s kpeak=%umV rxOvf=%lu "
                  "loopMax=%lums keeper=%lu crashes=%u%s rxAge=%lums canResets=%lu soft=%lums\n",
                  static_cast<unsigned long>(now), readings.mapKpa, readings.mapValid ? "" : "!", readings.boostPsi,
                  config.map.baroKpa, rpm, engineRpmValid ? "" : "!", pumpDutyPercent, governor.hold(),
                  static_cast<unsigned>(config.mode), readings.tankLow ? 1 : 0, canOnline ? 1 : 0,
                  readings.oilPressurePsi, readings.oilPressureValid ? "" : "!", readings.fuelPressurePsi,
                  readings.fuelPressureValid ? "" : "!", preTempC, preTempValid ? "" : "!", knockDetected ? 1 : 0,
                  static_cast<unsigned>(knockStats.restMv), static_cast<unsigned long>(adc_sampler::knockSamplesPerSecond()),
                  knockDetector.level(), knockDetector.baseline(), knockDetector.second(), knockDetector.share(),
                  knockCenterHz(), knockFinder.estimateHz(), knockToneHz,
                  knockGain.gain(), knockGainAuto ? "" : "(fixed)", static_cast<unsigned>(knockGain.typicalPeak()),
                  static_cast<unsigned long>(canRxOverflows), static_cast<unsigned long>(loopGapMaxMs),
                  static_cast<unsigned long>(pumpKeeperTrips), static_cast<unsigned>(resetRecord.crashes),
                  crashHold ? "(HOLD)" : "", static_cast<unsigned long>(now - lastCanRxMs),
                  static_cast<unsigned long>(canSilentResets), static_cast<unsigned long>(governor.softStartMs()));
    loopGapMaxMs = 0;
  }
}
