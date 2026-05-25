#include <Arduino.h>
#include <Preferences.h>
#include <stdlib.h>

#include "actuators.h"
#include "app_config.h"
#include "injection_controller.h"
#include "knock_monitor.h"
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
<<<<<<< HEAD
CanBridge canBridge;
KnockMonitor knockMonitor;
=======
>>>>>>> 9f85b5a265fae4c4b6d3167df006c2b6cc9d5f89
Preferences preferences;

uint32_t lastLoopMs = 0;
uint32_t lastDebugMs = 0;
FailsafeReason lastFailsafe = FailsafeReason::None;
String serialLine;

constexpr char kPrefsNamespace[] = "wmix";
constexpr char kPrefsKeyWater[] = "water_l";
constexpr char kPrefsKeyMeth[] = "meth_l";
constexpr uint8_t kFaultSeverityWarning = 1;

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

inline bool elapsed(uint32_t now, uint32_t start, uint32_t intervalMs) {
  return static_cast<uint32_t>(now - start) >= intervalMs;
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  WATER <liters>   - set water volume");
  Serial.println("  METH <liters>    - set methanol volume");
  Serial.println("  SHOW             - print current config/blend");
  Serial.println("  MODE OFF|BOOST|PRIME");
  Serial.println("  KNOCK SHOW");
  Serial.println("  KNOCK ENABLE 0|1");
  Serial.println("  KNOCK BOOSTKPA <kpa>");
  Serial.println("  KNOCK MULT <value>");
  Serial.println("  KNOCK OFFSET <value>");
  Serial.println("  KNOCK MODE LOG|WARN|FORCE|SHUTDOWN");
  Serial.println("  KNOCK RESET");
  Serial.println("  HELP");
}

const char *modeName(InjectionMode mode) {
  switch (mode) {
<<<<<<< HEAD
  case InjectionMode::Off: return "OFF";
  case InjectionMode::BoostOnly: return "BOOST";
  case InjectionMode::Prime: return "PRIME";
  default: return "UNKNOWN";
=======
  case InjectionMode::Off:    return "OFF";
  case InjectionMode::BoostOnly: return "BOOST";
  case InjectionMode::Prime:  return "PRIME";
  default:                    return "UNKNOWN";
>>>>>>> 9f85b5a265fae4c4b6d3167df006c2b6cc9d5f89
  }
}

const char *failsafeName(FailsafeReason reason) {
  switch (reason) {
<<<<<<< HEAD
  case FailsafeReason::None: return "NONE";
  case FailsafeReason::LowFluid: return "LOW_FLUID";
  case FailsafeReason::MapInvalid: return "MAP_INVALID";
  case FailsafeReason::InvalidBlend: return "INVALID_BLEND";
  case FailsafeReason::InvalidBoostConfig: return "INVALID_BOOST_CONFIG";
  default: return "UNKNOWN";
=======
  case FailsafeReason::None:                return "NONE";
  case FailsafeReason::LowFluid:            return "LOW_FLUID";
  case FailsafeReason::MapInvalid:          return "MAP_INVALID";
  case FailsafeReason::InvalidBlend:        return "INVALID_BLEND";
  case FailsafeReason::InvalidBoostConfig:  return "INVALID_BOOST_CONFIG";
  default:                                  return "UNKNOWN";
>>>>>>> 9f85b5a265fae4c4b6d3167df006c2b6cc9d5f89
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
<<<<<<< HEAD
  Serial.print("Mode: "); Serial.println(modeName(config.mode));
  Serial.print("MAP calibration (V): ");
  Serial.print(config.map.vMin, 2); Serial.print(" to "); Serial.print(config.map.vMax, 2);
  Serial.print(" -> (kPa): "); Serial.print(config.map.kpaMin, 1);
  Serial.print(" to "); Serial.println(config.map.kpaMax, 1);
  Serial.print("Boost start/full (psi): ");
  Serial.print(config.boost.startPsi, 1); Serial.print(" / "); Serial.println(config.boost.fullPsi, 1);
  Serial.print("Blend water/meth (L): ");
  Serial.print(blend.waterLiters, 2); Serial.print(" / "); Serial.print(blend.methLiters, 2);
  Serial.print("  Meth%: "); Serial.println(blend.methPercent, 1);
  Serial.print("Knock ADC pin: "); Serial.println(pins::KNOCK_SENSOR_ADC);
  Serial.print("Knock boost gate kPa: "); Serial.println(config.knock.boostEnableKpa, 1);
  Serial.print("Knock threshold (mult/offset): ");
  Serial.print(config.knock.thresholdMultiplier, 2); Serial.print(" / ");
  Serial.println(config.knock.thresholdOffset, 1);
  Serial.println("--------------------------------");
}

const char *knockModeName(KnockResponseMode mode) {
  switch (mode) {
  case KnockResponseMode::LogOnly: return "LOG";
  case KnockResponseMode::WarnOnly: return "WARN";
  case KnockResponseMode::ForceSpray: return "FORCE";
  case KnockResponseMode::SafetyShutdown: return "SHUTDOWN";
  default: return "UNKNOWN";
  }
}

bool parseKnockMode(const String &token, KnockResponseMode &modeOut) {
  if (token.equalsIgnoreCase("LOG")) {
    modeOut = KnockResponseMode::LogOnly;
    return true;
  }
  if (token.equalsIgnoreCase("WARN")) {
    modeOut = KnockResponseMode::WarnOnly;
    return true;
  }
  if (token.equalsIgnoreCase("FORCE")) {
    modeOut = KnockResponseMode::ForceSpray;
    return true;
  }
  if (token.equalsIgnoreCase("SHUTDOWN")) {
    modeOut = KnockResponseMode::SafetyShutdown;
    return true;
  }
  return false;
}

void printKnockSummary() {
  Serial.print("Knock enabled: "); Serial.println(config.knock.enabled ? "YES" : "NO");
  Serial.print("Knock mode: "); Serial.println(knockModeName(config.knock.responseMode));
  Serial.print("Knock boost gate kPa: "); Serial.println(config.knock.boostEnableKpa, 1);
  Serial.print("Knock threshold multiplier: "); Serial.println(config.knock.thresholdMultiplier, 2);
  Serial.print("Knock threshold offset: "); Serial.println(config.knock.thresholdOffset, 2);
  Serial.print("Knock cooldown ms: "); Serial.println(config.knock.eventCooldownMs);
  Serial.print("Knock warn/crit counts: ");
  Serial.print(config.knock.warningThresholdCount); Serial.print("/");
  Serial.println(config.knock.criticalThresholdCount);
}

void configureAnalogSensors() {
  ThermistorConfig t{};
  t.enabled = true;
  t.pullupOhms = 10000.0f;
  t.filterAlpha = 0.2f;

  t.pin = pins::IAT_THERM_PIN;
  iatSensor.begin(t);
  t.pin = pins::ENGINE_BAY_THERM_PIN;
  t.maxValidTempC = 200.0f;
  engineBaySensor.begin(t);
  t.pin = pins::CABIN_THERM_PIN;
  t.maxValidTempC = 120.0f;
  cabinSensor.begin(t);
  t.pin = pins::AMBIENT_THERM_PIN;
  ambientSensor.begin(t);

  PressureConfig p{};
  p.enabled = true;
  p.sensorMinV = 0.5f;
  p.sensorMaxV = 4.5f;
  p.pressureMaxPsi = 100.0f;
  p.maxValidPsi = 250.0f;
  p.dividerTopOhms = 10000.0f;
  p.dividerBottomOhms = 20000.0f;

  p.pin = pins::OIL_PRESSURE_ADC;
  oilPressure.begin(p);
  p.pin = pins::FUEL_PRESSURE_ADC;
  fuelPressure.begin(p);
  p.pin = pins::METH_PRESSURE_ADC;
  methPressure.begin(p);
  p.pin = pins::BOOST_REF_PRESSURE_ADC;
  boostRefPressure.begin(p);
  p.pin = pins::SPARE_PRESSURE_1_ADC;
  sparePressure1.begin(p);
  p.pin = pins::SPARE_PRESSURE_2_ADC;
  sparePressure2.begin(p);
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

  if (r.iatValid) {
    r.iatC = iatSensor.valueC();
  } else if (iatSensor.config().enabled) {
    r.analogFaultFlags |= kFaultIat;
  }

  if (r.engineBayValid) {
    r.engineBayC = engineBaySensor.valueC();
  } else if (engineBaySensor.config().enabled) {
    r.analogFaultFlags |= kFaultEngineBay;
  }

  if (r.cabinValid) {
    r.cabinC = cabinSensor.valueC();
  } else if (cabinSensor.config().enabled) {
    r.analogFaultFlags |= kFaultCabin;
  }

  if (r.ambientValid) {
    r.ambientC = ambientSensor.valueC();
  } else if (ambientSensor.config().enabled) {
    r.analogFaultFlags |= kFaultAmbient;
  }

  if (r.oilPressureValid) {
    r.oilPressurePsi = oilPressure.valuePsi();
  } else if (oilPressure.config().enabled) {
    r.analogFaultFlags |= kFaultOil;
  }

  if (r.fuelPressureValid) {
    r.fuelPressurePsi = fuelPressure.valuePsi();
  } else if (fuelPressure.config().enabled) {
    r.analogFaultFlags |= kFaultFuel;
  }

  if (r.methPressureValid) {
    r.methPressurePsi = methPressure.valuePsi();
  } else if (methPressure.config().enabled) {
    r.analogFaultFlags |= kFaultMeth;
  }

  if (r.boostRefPressureValid) {
    r.boostRefPressurePsi = boostRefPressure.valuePsi();
  } else if (boostRefPressure.config().enabled) {
    r.analogFaultFlags |= kFaultBoostRef;
  }

  if (r.sparePressure1Valid) {
    r.sparePressure1Psi = sparePressure1.valuePsi();
  } else if (sparePressure1.config().enabled) {
    r.analogFaultFlags |= kFaultSpare1;
  }

  if (r.sparePressure2Valid) {
    r.sparePressure2Psi = sparePressure2.valuePsi();
  } else if (sparePressure2.config().enabled) {
    r.analogFaultFlags |= kFaultSpare2;
  }
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

  if (cmd.equalsIgnoreCase("HELP")) { printHelp(); return; }
  if (cmd.equalsIgnoreCase("SHOW")) { printSetupSummary(); return; }

  if (cmd.startsWith("WATER ") || cmd.startsWith("water ")) {
    float value = 0.0f;
    if (parsePositiveFloat(cmd.substring(6), value) && value >= 0.0f) {
      blend = computeTankBlend(value, blend.methLiters);
      saveBlend();
      Serial.print("Water updated. Meth% = "); Serial.println(blend.methPercent, 1);
    } else Serial.println("Invalid WATER value.");
=======
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
>>>>>>> 9f85b5a265fae4c4b6d3167df006c2b6cc9d5f89
    return;
  }
  if (cmd.startsWith("METH ") || cmd.startsWith("meth ")) {
<<<<<<< HEAD
    float value = 0.0f;
    if (parsePositiveFloat(cmd.substring(5), value) && value >= 0.0f) {
      blend = computeTankBlend(blend.waterLiters, value);
      saveBlend();
      Serial.print("Meth updated. Meth% = "); Serial.println(blend.methPercent, 1);
    } else Serial.println("Invalid METH value.");
=======
    float v = 0.0f;
    if (parsePositiveFloat(cmd.substring(5), v) && v >= 0.0f) {
      blend = computeTankBlend(blend.waterLiters, v); saveBlend();
      Serial.print("Meth updated. Meth% = "); Serial.println(blend.methPercent,1);
    } else { Serial.println("Invalid METH value."); }
>>>>>>> 9f85b5a265fae4c4b6d3167df006c2b6cc9d5f89
    return;
  }
  if (cmd.startsWith("MODE ") || cmd.startsWith("mode ")) {
<<<<<<< HEAD
    const String value = cmd.substring(5);
    if (value.equalsIgnoreCase("OFF")) config.mode = InjectionMode::Off;
    else if (value.equalsIgnoreCase("BOOST")) config.mode = InjectionMode::BoostOnly;
    else if (value.equalsIgnoreCase("PRIME")) config.mode = InjectionMode::Prime;
    else {
      Serial.println("Invalid MODE. Use OFF|BOOST|PRIME");
      return;
    }
    Serial.print("Mode set to "); Serial.println(modeName(config.mode));
    return;
  }

  if (cmdUpper == "KNOCK SHOW") {
    printKnockSummary();
    return;
  }

  if (cmdUpper.startsWith("KNOCK ENABLE ")) {
    const String value = cmd.substring(String("KNOCK ENABLE ").length());
    if (value == "1" || value.equalsIgnoreCase("ON") || value.equalsIgnoreCase("TRUE")) {
      config.knock.enabled = true;
    } else if (value == "0" || value.equalsIgnoreCase("OFF") || value.equalsIgnoreCase("FALSE")) {
      config.knock.enabled = false;
    } else {
      Serial.println("Invalid KNOCK ENABLE value. Use 0|1.");
      return;
    }
    Serial.print("Knock enabled: "); Serial.println(config.knock.enabled ? "YES" : "NO");
    return;
  }

  if (cmdUpper.startsWith("KNOCK BOOSTKPA ")) {
    float value = 0.0f;
    if (parsePositiveFloat(cmd.substring(String("KNOCK BOOSTKPA ").length()), value) && value >= 0.0f) {
      config.knock.boostEnableKpa = value;
      Serial.print("Knock boost gate set to "); Serial.println(config.knock.boostEnableKpa, 1);
    } else {
      Serial.println("Invalid KNOCK BOOSTKPA value.");
    }
    return;
  }

  if (cmdUpper.startsWith("KNOCK MULT ")) {
    float value = 0.0f;
    if (parsePositiveFloat(cmd.substring(String("KNOCK MULT ").length()), value) && value > 0.0f) {
      config.knock.thresholdMultiplier = value;
      Serial.print("Knock multiplier set to "); Serial.println(config.knock.thresholdMultiplier, 2);
    } else {
      Serial.println("Invalid KNOCK MULT value.");
    }
    return;
  }

  if (cmdUpper.startsWith("KNOCK OFFSET ")) {
    float value = 0.0f;
    if (parsePositiveFloat(cmd.substring(String("KNOCK OFFSET ").length()), value) && value >= 0.0f) {
      config.knock.thresholdOffset = value;
      Serial.print("Knock offset set to "); Serial.println(config.knock.thresholdOffset, 2);
    } else {
      Serial.println("Invalid KNOCK OFFSET value.");
    }
    return;
  }

  if (cmdUpper.startsWith("KNOCK MODE ")) {
    KnockResponseMode mode = KnockResponseMode::WarnOnly;
    if (!parseKnockMode(cmd.substring(String("KNOCK MODE ").length()), mode)) {
      Serial.println("Invalid KNOCK MODE. Use LOG|WARN|FORCE|SHUTDOWN");
      return;
    }
    config.knock.responseMode = mode;
    Serial.print("Knock mode set to "); Serial.println(knockModeName(config.knock.responseMode));
    return;
  }

  if (cmdUpper == "KNOCK RESET") {
    knockMonitor.clearFaults();
    Serial.println("Knock event counters and faults cleared.");
=======
    const String val = cmd.substring(5);
    if      (val.equalsIgnoreCase("OFF"))   config.mode = InjectionMode::Off;
    else if (val.equalsIgnoreCase("BOOST")) config.mode = InjectionMode::BoostOnly;
    else if (val.equalsIgnoreCase("PRIME")) config.mode = InjectionMode::Prime;
    else { Serial.println("Invalid MODE. Use OFF|BOOST|PRIME"); return; }
    Serial.print("Mode set to "); Serial.println(modeName(config.mode));
>>>>>>> 9f85b5a265fae4c4b6d3167df006c2b6cc9d5f89
    return;
  }
  Serial.println("Unknown command. Type HELP.");
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
<<<<<<< HEAD
    if (c == '\n') {
      parseCommand(serialLine);
      serialLine = "";
      continue;
    }
=======
    if (c == '\n') { parseCommand(serialLine); serialLine = ""; continue; }
>>>>>>> 9f85b5a265fae4c4b6d3167df006c2b6cc9d5f89
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
  configureAnalogSensors();
  knockMonitor.begin(pins::KNOCK_SENSOR_ADC, config.knock);
  pumpDriver.begin(pins::PUMP_PWM, config.pwmFrequencyHz, config.pwmResolutionBits);
  warningOutput.begin(pins::WARNING_LED, true);

<<<<<<< HEAD
  if (canBridge.begin(pins::CAN_TX, pins::CAN_RX)) {
    Serial.println("CAN: online");
  } else {
    Serial.println("CAN: TWAI init failed — running serial-only");
  }

=======
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

>>>>>>> 9f85b5a265fae4c4b6d3167df006c2b6cc9d5f89
  printSetupSummary();
  printHelp();
}

void loop() {
  handleSerialCommands();
  const uint32_t now = millis();
  if (!elapsed(now, lastLoopMs, config.loopPeriodMs)) return;
  lastLoopMs = now;

<<<<<<< HEAD
  canBridge.poll();

  if (canBridge.hasRemoteRatio()) {
    const uint8_t pct = canBridge.remoteRatioPercent();
    blend = computeTankBlend(static_cast<float>(100 - pct), static_cast<float>(pct));
    canBridge.clearRemoteRatio();
  }
  if (canBridge.hasClearFaultsRequest()) {
    lastFailsafe = FailsafeReason::None;
    lastReportedCanFault = FailsafeReason::None;
    canBridge.clearFaultsRequest();
=======
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
>>>>>>> 9f85b5a265fae4c4b6d3167df006c2b6cc9d5f89
  }

  SensorReadings readings = mapSensor.read();
  readings.tankLow = floatSensor.update();
  updateAnalogReadings(readings, now);
  knockMonitor.setConfig(config.knock);
  // Standalone water-meth module currently has no RPM input path, so RPM is reported as 0.
  const KnockStateSnapshot knockState = knockMonitor.update(readings.mapKpa, 0, now);

<<<<<<< HEAD
  AppConfig effectiveConfig = config;
  if (!canBridge.isArmed() && canBridge.isOnline()) effectiveConfig.mode = InjectionMode::Off;

  ControlResult result{};
  if (canBridge.hasPendingManualTest()) {
    result.pump.enabled = true;
    result.pump.dutyPercent = constrain(canBridge.manualTestDuty(), 0, 100);
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
  } else if (knockState.requestForceSpray && result.failsafe == FailsafeReason::None &&
             !result.pump.enabled) {
    result.pump.enabled = true;
    result.pump.dutyPercent = effectiveConfig.dutyMinPercent;
    result.finalDutyPercent = effectiveConfig.dutyMinPercent;
  }

  pumpDriver.apply(result.pump);
  warningOutput.set(result.failsafe != FailsafeReason::None || knockSafetyShutdown ||
                    knockState.warningActive || knockState.criticalActive);

  if (result.failsafe != lastReportedCanFault) {
    if (result.failsafe != FailsafeReason::None) {
      uint8_t code = 0;
      switch (result.failsafe) {
        case FailsafeReason::LowFluid: code = 0x01; break;
        case FailsafeReason::MapInvalid: code = 0x05; break;
        case FailsafeReason::InvalidBlend: code = 0x08; break;
        case FailsafeReason::InvalidBoostConfig: code = 0x08; break;
        default: code = 0x09; break;
      }
      canBridge.sendFault(code, kFaultSeverityWarning, 0, 0);
    }
    lastReportedCanFault = result.failsafe;
  }

  if (knockSafetyShutdown) {
    canBridge.sendFault(can_protocol::meth_fault_code::SAFETY_SHUTDOWN,
                        kFaultSeverityWarning, 0, 0);
  }

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
  canBridge.sendKnockStateIfDue(knockCan, now);

  KnockFaultEvent knockFault{};
  if (knockMonitor.consumeFault(knockFault)) {
    canBridge.sendKnockFault(knockFault.code, knockFault.severity,
                             knockFault.data0, knockFault.data1);
=======
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
>>>>>>> 9f85b5a265fae4c4b6d3167df006c2b6cc9d5f89
  }

  if (result.failsafe != lastFailsafe) {
    Serial.print("Failsafe state changed: ");
    Serial.println(failsafeName(result.failsafe));
    lastFailsafe = result.failsafe;
  }

<<<<<<< HEAD
  canBridge.sendStateIfDue(readings, result, effectiveConfig, now);

  if (!elapsed(now, lastDebugMs, config.debugPeriodMs)) return;
  lastDebugMs = now;

  Serial.print("boostPsi="); Serial.print(readings.boostPsi, 2);
  Serial.print(" iatC="); Serial.print(readings.iatC, 1);
  Serial.print(" bayC="); Serial.print(readings.engineBayC, 1);
  Serial.print(" ambC="); Serial.print(readings.ambientC, 1);
  Serial.print(" oil/fuel/meth=");
  Serial.print(readings.oilPressurePsi, 1); Serial.print("/");
  Serial.print(readings.fuelPressurePsi, 1); Serial.print("/");
  Serial.print(readings.methPressurePsi, 1);
  Serial.print(" dutyOut="); Serial.print(result.finalDutyPercent, 1);
  Serial.print(" fs="); Serial.print(failsafeName(result.failsafe));
  Serial.print(" knockE/T=");
  Serial.print(knockState.energy, 1); Serial.print("/");
  Serial.print(knockState.threshold, 1);
  Serial.print(" knockWC=");
  Serial.print(knockState.warningActive ? "1" : "0");
  Serial.print(knockState.criticalActive ? "1" : "0");
  Serial.print(" analogFault=0x"); Serial.print(readings.analogFaultFlags, HEX);
=======
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
>>>>>>> 9f85b5a265fae4c4b6d3167df006c2b6cc9d5f89
  Serial.println();
}
