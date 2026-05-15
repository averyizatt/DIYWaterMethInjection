#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include <mcp_can.h>
#include <stdlib.h>

#include "actuators.h"
#include "app_config.h"
#include "can_bridge.h"
#include "injection_controller.h"
#include "pins.h"
#include "sensors.h"

namespace {
AppConfig config = defaultConfig();
TankBlend blend = computeTankBlend(1.5f, 0.5f);

MapSensor mapSensor;
FloatSensor floatSensor;
TempSensor engineTempSensor;
TempSensor ambientTempSensor;
PumpDriver pumpDriver;
WarningOutput warningOutput;
InjectionController controller;
CanBridge canBridge;
Preferences preferences;
MCP_CAN can(pins::CAN_CS);

bool canAvailable = false;
uint32_t lastLoopMs = 0;
uint32_t lastDebugMs = 0;
uint32_t lastTempRequestMs = 0;
uint32_t lastCanMs = 0;
bool tempConversionPending = false;
FailsafeReason lastFailsafe = FailsafeReason::None;
FailsafeReason lastReportedCanFault = FailsafeReason::None;
String serialLine;

constexpr char kPrefsNamespace[] = "wmix";
constexpr char kPrefsKeyWater[] = "water_l";
constexpr char kPrefsKeyMeth[] = "meth_l";

// Unsigned subtraction is rollover-safe for millis() timestamps.
inline bool elapsed(uint32_t now, uint32_t start, uint32_t intervalMs) {
  return static_cast<uint32_t>(now - start) >= intervalMs;
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  WATER <liters>   - set water volume");
  Serial.println("  METH <liters>    - set methanol volume");
  Serial.println("  SHOW             - print current config/blend");
  Serial.println("  MODE OFF|BOOST|PRIME");
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
  if (!preferences.begin(kPrefsNamespace, false)) {
    return;
  }

  preferences.putFloat(kPrefsKeyWater, blend.waterLiters);
  preferences.putFloat(kPrefsKeyMeth, blend.methLiters);
  preferences.end();
}

void loadBlend() {
  if (!preferences.begin(kPrefsNamespace, true)) {
    return;
  }

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
  Serial.print(config.map.vMin, 2);
  Serial.print(" to ");
  Serial.print(config.map.vMax, 2);
  Serial.print(" -> (kPa): ");
  Serial.print(config.map.kpaMin, 1);
  Serial.print(" to ");
  Serial.println(config.map.kpaMax, 1);
  Serial.print("Boost start/full (psi): ");
  Serial.print(config.boost.startPsi, 1);
  Serial.print(" / ");
  Serial.println(config.boost.fullPsi, 1);
  Serial.print("K gain (% per psi): ");
  Serial.println(config.gainK, 2);
  Serial.print("Blend water/meth (L): ");
  Serial.print(blend.waterLiters, 2);
  Serial.print(" / ");
  Serial.print(blend.methLiters, 2);
  Serial.print("  Meth%: ");
  Serial.println(blend.methPercent, 1);
  Serial.println("-------------------------------");
}

void sendCanFrames(const SensorReadings &sr, const ControlResult &cr) {
  if (!canAvailable) {
    return;
  }

  // Frame 0x100 — sensor data (all values scaled x10, signed 16-bit big-endian)
  // Bytes 0-1: boost PSI x10  |  2-3: MAP kPa x10
  // Bytes 4-5: engine bay °C x10  |  6-7: ambient °C x10
  const int16_t boostX10 = static_cast<int16_t>(sr.boostPsi * 10.0f);
  const int16_t mapX10 = static_cast<int16_t>(sr.mapKpa * 10.0f);
  const int16_t engTX10 = static_cast<int16_t>(engineTempSensor.celsius() * 10.0f);
  const int16_t ambTX10 = static_cast<int16_t>(ambientTempSensor.celsius() * 10.0f);
  uint8_t sensorFrame[8] = {
      static_cast<uint8_t>(boostX10 >> 8), static_cast<uint8_t>(boostX10 & 0xFF),
      static_cast<uint8_t>(mapX10 >> 8),   static_cast<uint8_t>(mapX10 & 0xFF),
      static_cast<uint8_t>(engTX10 >> 8),  static_cast<uint8_t>(engTX10 & 0xFF),
      static_cast<uint8_t>(ambTX10 >> 8),  static_cast<uint8_t>(ambTX10 & 0xFF)};
  can.sendMsgBuf(0x100, 0, 8, sensorFrame);

  // Frame 0x101 — pump status
  // Byte 0: duty % (0-100)  |  1: pump enabled  |  2: failsafe code  |  3: tank low
  uint8_t pumpFrame[8] = {
      static_cast<uint8_t>(cr.finalDutyPercent),
      cr.pump.enabled ? 1u : 0u,
      static_cast<uint8_t>(cr.failsafe),
      sr.tankLow ? 1u : 0u,
      0, 0, 0, 0};
  can.sendMsgBuf(0x101, 0, 8, pumpFrame);
}

bool parsePositiveFloat(const String &token, float &valueOut) {
  char buffer[32];
  token.toCharArray(buffer, sizeof(buffer));
  char *endPtr = nullptr;
  const float parsed = strtof(buffer, &endPtr);
  if (endPtr == buffer) {
    return false;
  }
  valueOut = parsed;
  return true;
}

void parseCommand(const String &line) {
  String cmd = line;
  cmd.trim();
  if (cmd.length() == 0) {
    return;
  }

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
    } else {
      Serial.println("Invalid WATER value.");
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
    } else {
      Serial.println("Invalid METH value.");
    }
    return;
  }

  if (cmd.startsWith("MODE ") || cmd.startsWith("mode ")) {
    const String value = cmd.substring(5);
    if (value.equalsIgnoreCase("OFF")) {
      config.mode = InjectionMode::Off;
    } else if (value.equalsIgnoreCase("BOOST")) {
      config.mode = InjectionMode::BoostOnly;
    } else if (value.equalsIgnoreCase("PRIME")) {
      config.mode = InjectionMode::Prime;
    } else {
      Serial.println("Invalid MODE. Use OFF|BOOST|PRIME");
      return;
    }
    Serial.print("Mode set to ");
    Serial.println(modeName(config.mode));
    return;
  }

  Serial.println("Unknown command. Type HELP.");
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      parseCommand(serialLine);
      serialLine = "";
      continue;
    }

    if (serialLine.length() < 120) {
      serialLine += c;
    }
  }
}
} // namespace

void setup() {
  Serial.begin(config.serialBaud);
  delay(200);

  loadBlend();

  mapSensor.begin(pins::MAP_SENSOR_ADC, config.map);
  floatSensor.begin(pins::FLOAT_SENSOR_DIGITAL, config.floatActiveLow, config.floatDebounceMs);
  engineTempSensor.begin(pins::TEMP_ENGINE_BAY);
  ambientTempSensor.begin(pins::TEMP_AMBIENT);
  pumpDriver.begin(pins::PUMP_PWM, config.pwmFrequencyHz, config.pwmResolutionBits);
  warningOutput.begin(pins::WARNING_LED, true);

<<<<<<< HEAD
  // MCP2515 CAN bus — 500 kbps, 8 MHz oscillator (change MCP_8MHZ to MCP_16MHZ if your
  // module has a 16 MHz crystal)
  if (can.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    can.setMode(MCP_NORMAL);
    canAvailable = true;
    Serial.println("CAN bus ready (500 kbps)");
  } else {
    Serial.println("WARNING: CAN bus init failed — check wiring/oscillator");
  }

  // Kick off the first temperature conversion
  engineTempSensor.requestConversion();
  ambientTempSensor.requestConversion();
  lastTempRequestMs = millis();
  tempConversionPending = true;

=======
  if (canBridge.begin(pins::CAN_TX, pins::CAN_RX)) {
    Serial.println("CAN: online");
  } else {
    Serial.println("CAN: TWAI init failed — running serial-only");
  }

>>>>>>> 54c27c114127d33f6a78ce395b3c255993478aad
  printSetupSummary();
  printHelp();
}

void loop() {
  handleSerialCommands();

  const uint32_t now = millis();
  if (!elapsed(now, lastLoopMs, config.loopPeriodMs)) {
    return;
  }
  lastLoopMs = now;

<<<<<<< HEAD
  // Temperature: read result 800 ms after conversion request, then request again
  if (tempConversionPending && elapsed(now, lastTempRequestMs, 800)) {
    engineTempSensor.readResult();
    ambientTempSensor.readResult();
    tempConversionPending = false;
  }
  if (!tempConversionPending && elapsed(now, lastTempRequestMs, 1000)) {
    engineTempSensor.requestConversion();
    ambientTempSensor.requestConversion();
    lastTempRequestMs = now;
    tempConversionPending = true;
=======
  // Process incoming CAN frames (ARM/DISARM, manual test commands, config).
  canBridge.poll();

  // Apply ratio update from CCM if the master sent a new value.
  // The ratio tells us what % methanol is physically in the tank.
  if (canBridge.hasRemoteRatio()) {
    const uint8_t pct = canBridge.remoteRatioPercent();
    // Recompute blend keeping the same total-volume scale (100 L nominal).
    blend = computeTankBlend(static_cast<float>(100 - pct), static_cast<float>(pct));
    canBridge.clearRemoteRatio();
  }
  if (canBridge.hasClearFaultsRequest()) {
    lastFailsafe = FailsafeReason::None;
    lastReportedCanFault = FailsafeReason::None;
    canBridge.clearFaultsRequest();
>>>>>>> 54c27c114127d33f6a78ce395b3c255993478aad
  }

  SensorReadings readings = mapSensor.read();
  readings.tankLow = floatSensor.update();

  // If the master has disarmed via CAN, force injection off regardless of local config.
  AppConfig effectiveConfig = config;
  if (!canBridge.isArmed() && canBridge.isOnline()) {
    effectiveConfig.mode = InjectionMode::Off;
  }

  // Manual test overrides normal injection control.
  ControlResult result{};
  if (canBridge.hasPendingManualTest()) {
    result.pump.enabled = true;
    result.pump.dutyPercent = constrain(canBridge.manualTestDuty(), 0, 100);
    result.finalDutyPercent = result.pump.dutyPercent;
  } else {
    result = controller.update(readings, effectiveConfig, blend);
  }

  pumpDriver.apply(result.pump);
  warningOutput.set(result.failsafe != FailsafeReason::None);

<<<<<<< HEAD
  // CAN bus — transmit at 100 ms interval
  if (elapsed(now, lastCanMs, 100)) {
    sendCanFrames(readings, result);
    lastCanMs = now;
=======
  // Report new fault conditions over CAN.
  if (result.failsafe != lastReportedCanFault) {
    if (result.failsafe != FailsafeReason::None) {
      uint8_t code = 0;
      switch (result.failsafe) {
        case FailsafeReason::LowFluid:           code = 0x01; break;
        case FailsafeReason::MapInvalid:         code = 0x05; break;
        case FailsafeReason::InvalidBlend:       code = 0x08; break;
        case FailsafeReason::InvalidBoostConfig: code = 0x08; break;
        default:                                 code = 0x09; break;
      }
      canBridge.sendFault(code, 1 /*WARNING*/, 0, 0);
    }
    lastReportedCanFault = result.failsafe;
>>>>>>> 54c27c114127d33f6a78ce395b3c255993478aad
  }

  if (result.failsafe != lastFailsafe) {
    Serial.print("Failsafe state changed: ");
    Serial.println(failsafeName(result.failsafe));
    lastFailsafe = result.failsafe;
  }

  // Transmit state frame to CCM master.
  canBridge.sendStateIfDue(readings, result, effectiveConfig, now);

  if (!elapsed(now, lastDebugMs, config.debugPeriodMs)) {
    return;
  }
  lastDebugMs = now;

  Serial.print("MAPraw=");
  Serial.print(readings.mapRaw);
  Serial.print(" V=");
  Serial.print(readings.mapVoltage, 3);
  Serial.print(" kPa=");
  Serial.print(readings.mapKpa, 1);
  Serial.print(" boostPsi=");
  Serial.print(readings.boostPsi, 2);
  Serial.print(" low=");
  Serial.print(readings.tankLow ? "1" : "0");
  Serial.print(" armed=");
  Serial.print(canBridge.isArmed() ? "Y" : "N");
  Serial.print(" meth%=");
  Serial.print(blend.methPercent, 1);
  Serial.print(" dutyBase=");
  Serial.print(result.baseDutyPercent, 1);
  Serial.print(" dutyOut=");
  Serial.print(result.finalDutyPercent, 1);
  Serial.print(" pump=");
  Serial.print(result.pump.enabled ? "ON" : "OFF");
  Serial.print(" fs=");
  Serial.print(failsafeName(result.failsafe));
  Serial.print(" engT=");
  if (engineTempSensor.valid()) {
    Serial.print(engineTempSensor.celsius(), 1);
    Serial.print("C");
  } else {
    Serial.print("NC");
  }
  Serial.print(" ambT=");
  if (ambientTempSensor.valid()) {
    Serial.print(ambientTempSensor.celsius(), 1);
    Serial.print("C");
  } else {
    Serial.print("NC");
  }
  Serial.println();
}
