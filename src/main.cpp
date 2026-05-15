#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include <mcp_can.h>
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
TempSensor engineTempSensor;
TempSensor ambientTempSensor;
PumpDriver pumpDriver;
WarningOutput warningOutput;
InjectionController controller;
Preferences preferences;
MCP_CAN can(pins::CAN_CS);

bool canAvailable = false;
uint32_t lastLoopMs = 0;
uint32_t lastDebugMs = 0;
uint32_t lastTempRequestMs = 0;
uint32_t lastCanMs = 0;
bool tempConversionPending = false;
FailsafeReason lastFailsafe = FailsafeReason::None;
String serialLine;

constexpr char kPrefsNamespace[] = "wmix";
constexpr char kPrefsKeyWater[] = "water_l";
constexpr char kPrefsKeyMeth[] = "meth_l";

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
  case InjectionMode::Off:    return "OFF";
  case InjectionMode::BoostOnly: return "BOOST";
  case InjectionMode::Prime:  return "PRIME";
  default:                    return "UNKNOWN";
  }
}

const char *failsafeName(FailsafeReason reason) {
  switch (reason) {
  case FailsafeReason::None:                return "NONE";
  case FailsafeReason::LowFluid:            return "LOW_FLUID";
  case FailsafeReason::MapInvalid:          return "MAP_INVALID";
  case FailsafeReason::InvalidBlend:        return "INVALID_BLEND";
  case FailsafeReason::InvalidBoostConfig:  return "INVALID_BOOST_CONFIG";
  default:                                  return "UNKNOWN";
  }
}

void saveBlend() {
  if (!preferences.begin(kPrefsNamespace, false)) return;
  preferences.putFloat(kPrefsKeyWater, blend.waterLiters);
  preferences.putFloat(kPrefsKeyMeth,  blend.methLiters);
  preferences.end();
}

void loadBlend() {
  if (!preferences.begin(kPrefsNamespace, true)) return;
  const float water = preferences.getFloat(kPrefsKeyWater, blend.waterLiters);
  const float meth  = preferences.getFloat(kPrefsKeyMeth,  blend.methLiters);
  preferences.end();
  blend = computeTankBlend(water, meth);
}

void printSetupSummary() {
  Serial.println("---- Water/Meth Controller ----");
  Serial.print("Mode: ");         Serial.println(modeName(config.mode));
  Serial.print("MAP (V): ");      Serial.print(config.map.vMin, 2);
  Serial.print(" to ");           Serial.print(config.map.vMax, 2);
  Serial.print(" -> (kPa): ");    Serial.print(config.map.kpaMin, 1);
  Serial.print(" to ");           Serial.println(config.map.kpaMax, 1);
  Serial.print("Boost (psi): ");  Serial.print(config.boost.startPsi, 1);
  Serial.print(" / ");            Serial.println(config.boost.fullPsi, 1);
  Serial.print("K gain: ");       Serial.println(config.gainK, 2);
  Serial.print("Blend (L): ");    Serial.print(blend.waterLiters, 2);
  Serial.print(" / ");            Serial.print(blend.methLiters, 2);
  Serial.print("  Meth%: ");      Serial.println(blend.methPercent, 1);
  Serial.println("-------------------------------");
}

void sendCanFrames(const SensorReadings &sr, const ControlResult &cr) {
  if (!canAvailable) return;
  const int16_t boostX10 = static_cast<int16_t>(sr.boostPsi * 10.0f);
  const int16_t mapX10   = static_cast<int16_t>(sr.mapKpa   * 10.0f);
  const int16_t engTX10  = static_cast<int16_t>(engineTempSensor.celsius()  * 10.0f);
  const int16_t ambTX10  = static_cast<int16_t>(ambientTempSensor.celsius() * 10.0f);
  uint8_t sf[8] = {
    static_cast<uint8_t>(boostX10>>8), static_cast<uint8_t>(boostX10&0xFF),
    static_cast<uint8_t>(mapX10>>8),   static_cast<uint8_t>(mapX10&0xFF),
    static_cast<uint8_t>(engTX10>>8),  static_cast<uint8_t>(engTX10&0xFF),
    static_cast<uint8_t>(ambTX10>>8),  static_cast<uint8_t>(ambTX10&0xFF)};
  can.sendMsgBuf(0x100, 0, 8, sf);
  uint8_t pf[8] = {
    static_cast<uint8_t>(cr.finalDutyPercent),
    cr.pump.enabled ? 1u : 0u,
    static_cast<uint8_t>(cr.failsafe),
    sr.tankLow ? 1u : 0u,
    0,0,0,0};
  can.sendMsgBuf(0x101, 0, 8, pf);
}

bool parsePositiveFloat(const String &token, float &out) {
  char buf[32]; token.toCharArray(buf, sizeof(buf));
  char *end = nullptr;
  const float v = strtof(buf, &end);
  if (end == buf) return false;
  out = v; return true;
}

void parseCommand(const String &line) {
  String cmd = line; cmd.trim();
  if (cmd.length() == 0) return;
  if (cmd.equalsIgnoreCase("HELP"))  { printHelp();        return; }
  if (cmd.equalsIgnoreCase("SHOW"))  { printSetupSummary(); return; }
  if (cmd.startsWith("WATER ") || cmd.startsWith("water ")) {
    float v = 0.0f;
    if (parsePositiveFloat(cmd.substring(6), v) && v >= 0.0f) {
      blend = computeTankBlend(v, blend.methLiters); saveBlend();
      Serial.print("Water updated. Meth% = "); Serial.println(blend.methPercent,1);
    } else { Serial.println("Invalid WATER value."); }
    return;
  }
  if (cmd.startsWith("METH ") || cmd.startsWith("meth ")) {
    float v = 0.0f;
    if (parsePositiveFloat(cmd.substring(5), v) && v >= 0.0f) {
      blend = computeTankBlend(blend.waterLiters, v); saveBlend();
      Serial.print("Meth updated. Meth% = "); Serial.println(blend.methPercent,1);
    } else { Serial.println("Invalid METH value."); }
    return;
  }
  if (cmd.startsWith("MODE ") || cmd.startsWith("mode ")) {
    const String val = cmd.substring(5);
    if      (val.equalsIgnoreCase("OFF"))   config.mode = InjectionMode::Off;
    else if (val.equalsIgnoreCase("BOOST")) config.mode = InjectionMode::BoostOnly;
    else if (val.equalsIgnoreCase("PRIME")) config.mode = InjectionMode::Prime;
    else { Serial.println("Invalid MODE. Use OFF|BOOST|PRIME"); return; }
    Serial.print("Mode set to "); Serial.println(modeName(config.mode));
    return;
  }
  Serial.println("Unknown command. Type HELP.");
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') { parseCommand(serialLine); serialLine = ""; continue; }
    if (serialLine.length() < 120) serialLine += c;
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

  if (can.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    can.setMode(MCP_NORMAL);
    pinMode(pins::CAN_INT, INPUT); // MCP2515 pulls LOW when a frame is waiting
    canAvailable = true;
    Serial.println("CAN bus ready (500 kbps)");
  } else {
    Serial.println("CAN bus not present -- running without it");
  }

  engineTempSensor.requestConversion();
  ambientTempSensor.requestConversion();
  lastTempRequestMs = millis();
  tempConversionPending = true;

  printSetupSummary();
  printHelp();
}

void loop() {
  handleSerialCommands();

  const uint32_t now = millis();
  if (!elapsed(now, lastLoopMs, config.loopPeriodMs)) return;
  lastLoopMs = now;

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
  }

  SensorReadings readings = mapSensor.read();
  readings.tankLow = floatSensor.update();

  ControlResult result = controller.update(readings, config, blend);
  pumpDriver.apply(result.pump);
  warningOutput.set(result.failsafe != FailsafeReason::None);

  if (elapsed(now, lastCanMs, 100)) {
    sendCanFrames(readings, result);
    lastCanMs = now;
  }

  // CAN receive -- drain all waiting frames (INT pin pulled LOW by MCP2515 when frame pending).
  if (canAvailable) {
    while (digitalRead(pins::CAN_INT) == LOW) {
      unsigned long rxId;
      uint8_t       rxLen;
      uint8_t  rxBuf[8];
      if (can.readMsgBuf(&rxId, &rxLen, rxBuf) == CAN_OK) {
        Serial.print("CAN RX id=0x");
        Serial.print(rxId, HEX);
        Serial.print(" len=");
        Serial.print(rxLen);
        Serial.print(" data=");
        for (uint8_t i = 0; i < rxLen; i++) {
          if (rxBuf[i] < 0x10) Serial.print('0');
          Serial.print(rxBuf[i], HEX);
          Serial.print(' ');
        }
        Serial.println();
      } else {
        break; // read error, stop draining
      }
    }
  }

  if (result.failsafe != lastFailsafe) {
    Serial.print("Failsafe state changed: ");
    Serial.println(failsafeName(result.failsafe));
    lastFailsafe = result.failsafe;
  }

  if (!elapsed(now, lastDebugMs, config.debugPeriodMs)) return;
  lastDebugMs = now;

  Serial.print("MAPraw=");   Serial.print(readings.mapRaw);
  Serial.print(" V=");       Serial.print(readings.mapVoltage, 3);
  Serial.print(" kPa=");     Serial.print(readings.mapKpa, 1);
  Serial.print(" psi=");     Serial.print(readings.boostPsi, 2);
  Serial.print(" low=");     Serial.print(readings.tankLow ? "1" : "0");
  Serial.print(" meth%=");   Serial.print(blend.methPercent, 1);
  Serial.print(" duty=");    Serial.print(result.finalDutyPercent, 1);
  Serial.print(" pump=");    Serial.print(result.pump.enabled ? "ON" : "OFF");
  Serial.print(" fs=");      Serial.print(failsafeName(result.failsafe));
  Serial.print(" engT=");
  if (engineTempSensor.valid())  { Serial.print(engineTempSensor.celsius(), 1);  Serial.print("C"); }
  else                           { Serial.print("NC"); }
  Serial.print(" ambT=");
  if (ambientTempSensor.valid()) { Serial.print(ambientTempSensor.celsius(), 1); Serial.print("C"); }
  else                           { Serial.print("NC"); }
  Serial.println();
}
