#include <Arduino.h>
#include <Preferences.h>
#include <stdlib.h>

#include "actuators.h"
#include "app_config.h"
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
Preferences preferences;

uint32_t lastLoopMs = 0;
uint32_t lastDebugMs = 0;
FailsafeReason lastFailsafe = FailsafeReason::None;
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
  pumpDriver.begin(pins::PUMP_PWM, config.pwmFrequencyHz, config.pwmResolutionBits);
  warningOutput.begin(pins::WARNING_LED, true);

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

  SensorReadings readings = mapSensor.read();
  readings.tankLow = floatSensor.update();

  ControlResult result = controller.update(readings, config, blend);
  pumpDriver.apply(result.pump);
  warningOutput.set(result.failsafe != FailsafeReason::None);

  if (result.failsafe != lastFailsafe) {
    Serial.print("Failsafe state changed: ");
    Serial.println(failsafeName(result.failsafe));
    lastFailsafe = result.failsafe;
  }

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
  Serial.print(" meth%=");
  Serial.print(blend.methPercent, 1);
  Serial.print(" dutyBase=");
  Serial.print(result.baseDutyPercent, 1);
  Serial.print(" dutyOut=");
  Serial.print(result.finalDutyPercent, 1);
  Serial.print(" pump=");
  Serial.print(result.pump.enabled ? "ON" : "OFF");
  Serial.print(" fs=");
  Serial.println(failsafeName(result.failsafe));
}
