#include "knock_ui.h"

#include <stdlib.h>

namespace {
bool parseFloat(const String &token, float &out) {
  char buffer[32];
  token.toCharArray(buffer, sizeof(buffer));
  char *endPtr = nullptr;
  const float parsed = strtof(buffer, &endPtr);
  if (endPtr == buffer) return false;
  out = parsed;
  return true;
}

bool parseUint16(const String &token, uint16_t &out) {
  float temp = 0.0f;
  if (!parseFloat(token, temp) || temp < 0.0f || temp > 65535.0f) return false;
  out = static_cast<uint16_t>(temp);
  return true;
}
} // namespace

void KnockUi::printHelp(Stream &out) {
  out.println("Knock commands:");
  out.println("  KNOCK SHOW");
  out.println("  KNOCK RESET");
  out.println("  KNOCK ENABLE 0|1");
  out.println("  KNOCK THRESH <offset>");
  out.println("  KNOCK MULT <multiplier>");
  out.println("  KNOCK MINRPM <rpm>");
  out.println("  KNOCK MINMAP <kpa>");
  out.println("  KNOCK DEBOUNCE <ms>");
  out.println("  KNOCK GAIN <scale>");
  out.println("  KNOCK FREQ <hz>");
  out.println("  KNOCK BW <hz>");
  out.println("  KNOCK AUTOFREQ 0|1");
}

void KnockUi::printSummary(Stream &out, const KnockConfig &config, const KnockStateSnapshot &state) {
  out.print("Knock enabled: "); out.println(config.enabled ? "YES" : "NO");
  out.print("Knock threshold offset: "); out.println(config.thresholdOffset, 2);
  out.print("Knock adaptive multiplier: "); out.println(config.thresholdMultiplier, 2);
  out.print("Knock min RPM/MAP: ");
  out.print(config.minRpmToArm);
  out.print(" / ");
  out.println(config.minMapKpaToArm, 1);
  out.print("Knock debounce ms: "); out.println(config.eventCooldownMs);
  out.print("Knock gain: "); out.println(config.signalGain, 2);
  out.print("Knock freq/bw Hz: ");
  out.print(config.centerFreqHz, 0);
  out.print(" / ");
  out.println(config.bandwidthHz, 0);

  out.print("Live level RMS: "); out.println(state.knockLevelRms, 2);
  out.print("Adaptive threshold: "); out.println(state.threshold, 2);
  out.print("Event count: "); out.println(state.eventCount);
}

bool KnockUi::handleCommand(const String &line, KnockConfig &config, KnockMonitor &monitor, Stream &out) {
  String cmd = line;
  cmd.trim();
  String upper = cmd;
  upper.toUpperCase();

  if (upper == "KNOCK SHOW") {
    printSummary(out, config, monitor.state());
    return true;
  }
  if (upper == "KNOCK RESET") {
    monitor.clearFaults();
    out.println("Knock faults/events reset.");
    return true;
  }
  if (upper.startsWith("KNOCK ENABLE ")) {
    const String value = cmd.substring(String("KNOCK ENABLE ").length());
    if (value == "1" || value.equalsIgnoreCase("ON") || value.equalsIgnoreCase("TRUE")) config.enabled = true;
    else if (value == "0" || value.equalsIgnoreCase("OFF") || value.equalsIgnoreCase("FALSE")) config.enabled = false;
    else return false;
    out.print("Knock enabled: "); out.println(config.enabled ? "YES" : "NO");
    return true;
  }

  float fValue = 0.0f;
  uint16_t u16Value = 0;

  if (upper.startsWith("KNOCK THRESH ") && parseFloat(cmd.substring(String("KNOCK THRESH ").length()), fValue)) {
    config.thresholdOffset = fValue;
    out.print("Knock threshold offset: "); out.println(config.thresholdOffset, 2);
    return true;
  }
  if (upper.startsWith("KNOCK MULT ") && parseFloat(cmd.substring(String("KNOCK MULT ").length()), fValue)) {
    config.thresholdMultiplier = fValue;
    out.print("Knock multiplier: "); out.println(config.thresholdMultiplier, 2);
    return true;
  }
  if (upper.startsWith("KNOCK MINRPM ") && parseUint16(cmd.substring(String("KNOCK MINRPM ").length()), u16Value)) {
    config.minRpmToArm = u16Value;
    out.print("Knock min RPM: "); out.println(config.minRpmToArm);
    return true;
  }
  if ((upper.startsWith("KNOCK MINMAP ") &&
       parseFloat(cmd.substring(String("KNOCK MINMAP ").length()), fValue)) ||
      (upper.startsWith("KNOCK BOOSTKPA ") &&
       parseFloat(cmd.substring(String("KNOCK BOOSTKPA ").length()), fValue))) {
    config.minMapKpaToArm = fValue;
    config.boostEnableKpa = fValue;
    out.print("Knock min MAP kPa: "); out.println(config.minMapKpaToArm, 1);
    return true;
  }
  if (upper.startsWith("KNOCK DEBOUNCE ") && parseUint16(cmd.substring(String("KNOCK DEBOUNCE ").length()), u16Value)) {
    config.eventCooldownMs = u16Value;
    out.print("Knock debounce ms: "); out.println(config.eventCooldownMs);
    return true;
  }
  if (upper.startsWith("KNOCK GAIN ") && parseFloat(cmd.substring(String("KNOCK GAIN ").length()), fValue)) {
    config.signalGain = fValue;
    out.print("Knock gain: "); out.println(config.signalGain, 2);
    return true;
  }
  if (upper.startsWith("KNOCK FREQ ") && parseFloat(cmd.substring(String("KNOCK FREQ ").length()), fValue)) {
    config.centerFreqHz = fValue;
    config.autoCenterFromBore = false;
    out.print("Knock center Hz: "); out.println(config.centerFreqHz, 0);
    return true;
  }
  if (upper.startsWith("KNOCK BW ") && parseFloat(cmd.substring(String("KNOCK BW ").length()), fValue)) {
    config.bandwidthHz = fValue;
    out.print("Knock bandwidth Hz: "); out.println(config.bandwidthHz, 0);
    return true;
  }
  if (upper.startsWith("KNOCK AUTOFREQ ")) {
    const String value = cmd.substring(String("KNOCK AUTOFREQ ").length());
    if (value == "1") config.autoCenterFromBore = true;
    else if (value == "0") config.autoCenterFromBore = false;
    else return false;
    out.print("Knock auto frequency: "); out.println(config.autoCenterFromBore ? "ON" : "OFF");
    return true;
  }

  return false;
}
