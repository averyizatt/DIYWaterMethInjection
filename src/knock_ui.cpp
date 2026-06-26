#include "knock_ui.h"

#include <stdlib.h>

namespace {
bool parseFloat(const String &token, float &out) {
  char buffer[32];
  token.toCharArray(buffer, sizeof(buffer));
  char *endPtr = nullptr;
  const float parsed = static_cast<float>(strtod(buffer, &endPtr));
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
  out.println("  KNOCK MINLOAD <percent>");
  out.println("  KNOCK DEBOUNCE <ms>");
  out.println("  KNOCK GAIN <scale>");
  out.println("  KNOCK FREQ <hz>");
  out.println("  KNOCK BW <hz>");
  out.println("  KNOCK SPREAD <0.05..0.35>");
  out.println("  KNOCK FFT 0|1");
  out.println("  KNOCK FFTSNR <db>");
  out.println("  KNOCK FFTW <0..1>");
  out.println("  KNOCK FFTSHORT <0..1>");
  out.println("  KNOCK FFTHARM <0..1>");
  out.println("  KNOCK TEMPLATEW <0..1>");
  out.println("  KNOCK MAPRATE <kPa/s>");
  out.println("  KNOCK TRSCALE <1.0..2.5>");
  out.println("  KNOCK TRHOLD <ms>");
  out.println("  KNOCK DETCONF <0..1>");
  out.println("  KNOCK WARNCONF <0..1>");
  out.println("  KNOCK CRITCONF <0..1>");
  out.println("  KNOCK IATSTART <C>");
  out.println("  KNOCK IATPER <scale/C>");
  out.println("  KNOCK BAYSTART <C>");
  out.println("  KNOCK BAYPER <scale/C>");
  out.println("  KNOCK AUTOFREQ 0|1");
}

void KnockUi::printSummary(Stream &out, const KnockConfig &config, const KnockStateSnapshot &state) {
  out.print("Knock enabled: "); out.println(config.enabled ? "YES" : "NO");
  out.print("Knock threshold offset: "); out.println(config.thresholdOffset, 2);
  out.print("Knock adaptive multiplier: "); out.println(config.thresholdMultiplier, 2);
  out.print("Knock min RPM/MAP/LOAD: ");
  out.print(config.minRpmToArm);
  out.print(" / ");
  out.print(config.minMapKpaToArm, 1);
  out.print(" / ");
  out.println(config.minLoadPercentToArm, 1);
  out.print("Knock debounce ms: "); out.println(config.eventCooldownMs);
  out.print("Knock gain: "); out.println(config.signalGain, 2);
  out.print("Knock freq/bw Hz: ");
  out.print(config.centerFreqHz, 0);
  out.print(" / ");
  out.println(config.bandwidthHz, 0);
  out.print("Knock multiband spread: "); out.println(config.multiBandSpread, 3);
  out.print("Knock FFT enabled/minSNR/weight: ");
  out.print(config.fftEnabled ? "ON" : "OFF");
  out.print(" / ");
  out.print(config.fftMinSnrDb, 1);
  out.print(" / ");
  out.println(config.fftWeight, 2);
  out.print("Knock FFT short/harmonic/template weight: ");
  out.print(config.fftShortWeight, 2);
  out.print(" / ");
  out.print(config.fftHarmonicWeight, 2);
  out.print(" / ");
  out.println(config.spectralTemplateWeight, 2);
  out.print("Knock MAP-rate gate/scale/hold: ");
  out.print(config.mapRateGateKpaPerSec, 1);
  out.print(" / ");
  out.print(config.transientThresholdScale, 2);
  out.print(" / ");
  out.println(config.transientHoldMs);
  out.print("Knock detect/warn/critical confidence: ");
  out.print(config.minDetectConfidence, 2);
  out.print(" / ");
  out.print(config.warnConfidence, 2);
  out.print(" / ");
  out.println(config.criticalConfidence, 2);
  out.print("Knock IAT comp start/perC: ");
  out.print(config.iatTempCompStartC, 1);
  out.print(" / ");
  out.println(config.iatTempCompPerC, 3);
  out.print("Knock BAY comp start/perC: ");
  out.print(config.bayTempCompStartC, 1);
  out.print(" / ");
  out.println(config.bayTempCompPerC, 3);

  out.print("Live level RMS: "); out.println(state.knockLevelRms, 2);
  out.print("Band RMS L/M/H: ");
  out.print(state.lowBandRms, 2);
  out.print(" / ");
  out.print(state.midBandRms, 2);
  out.print(" / ");
  out.println(state.highBandRms, 2);
  out.print("Spectral confidence: "); out.println(state.spectralConfidence, 3);
  out.print("FFT SNR dB: "); out.println(state.fftSnrDb, 2);
  out.print("FFT long/short dB: ");
  out.print(state.fftLongSnrDb, 2);
  out.print(" / ");
  out.println(state.fftShortSnrDb, 2);
  out.print("Harmonic score: "); out.println(state.harmonicScore, 2);
  out.print("Template deviation: "); out.println(state.templateDeviation, 3);
  out.print("MAP-rate/transient: ");
  out.print(state.mapRateKpaPerSec, 1);
  out.print(" / ");
  out.println(state.transientScale, 2);
  out.print("Final confidence/risk: ");
  out.print(state.finalConfidence, 2);
  out.print(" / ");
  out.println(state.knockRisk, 2);
  out.print("Health/profile/class/degrade: ");
  out.print(state.healthScore, 1);
  out.print(" / ");
  out.print(state.profile);
  out.print(" / ");
  out.print(state.anomalyClass);
  out.print(" / ");
  out.println(state.degradeMode);
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
  if (upper.startsWith("KNOCK MINLOAD ") && parseFloat(cmd.substring(String("KNOCK MINLOAD ").length()), fValue)) {
    config.minLoadPercentToArm = fValue;
    out.print("Knock min load %: "); out.println(config.minLoadPercentToArm, 1);
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
  if (upper.startsWith("KNOCK SPREAD ") && parseFloat(cmd.substring(String("KNOCK SPREAD ").length()), fValue)) {
    config.multiBandSpread = fValue;
    out.print("Knock multiband spread: "); out.println(config.multiBandSpread, 3);
    return true;
  }
  if (upper.startsWith("KNOCK FFT ")) {
    const String value = cmd.substring(String("KNOCK FFT ").length());
    if (value == "1") config.fftEnabled = true;
    else if (value == "0") config.fftEnabled = false;
    else return false;
    out.print("Knock FFT enabled: "); out.println(config.fftEnabled ? "ON" : "OFF");
    return true;
  }
  if (upper.startsWith("KNOCK FFTSNR ") && parseFloat(cmd.substring(String("KNOCK FFTSNR ").length()), fValue)) {
    config.fftMinSnrDb = fValue;
    out.print("Knock FFT min SNR dB: "); out.println(config.fftMinSnrDb, 1);
    return true;
  }
  if (upper.startsWith("KNOCK FFTW ") && parseFloat(cmd.substring(String("KNOCK FFTW ").length()), fValue)) {
    config.fftWeight = fValue;
    out.print("Knock FFT weight: "); out.println(config.fftWeight, 2);
    return true;
  }
  if (upper.startsWith("KNOCK FFTSHORT ") &&
      parseFloat(cmd.substring(String("KNOCK FFTSHORT ").length()), fValue)) {
    config.fftShortWeight = fValue;
    out.print("Knock FFT short-window weight: "); out.println(config.fftShortWeight, 2);
    return true;
  }
  if (upper.startsWith("KNOCK FFTHARM ") &&
      parseFloat(cmd.substring(String("KNOCK FFTHARM ").length()), fValue)) {
    config.fftHarmonicWeight = fValue;
    out.print("Knock FFT harmonic weight: "); out.println(config.fftHarmonicWeight, 2);
    return true;
  }
  if (upper.startsWith("KNOCK TEMPLATEW ") &&
      parseFloat(cmd.substring(String("KNOCK TEMPLATEW ").length()), fValue)) {
    config.spectralTemplateWeight = fValue;
    out.print("Knock spectral template weight: "); out.println(config.spectralTemplateWeight, 2);
    return true;
  }
  if (upper.startsWith("KNOCK MAPRATE ") &&
      parseFloat(cmd.substring(String("KNOCK MAPRATE ").length()), fValue)) {
    config.mapRateGateKpaPerSec = fValue;
    out.print("Knock MAP-rate gate kPa/s: "); out.println(config.mapRateGateKpaPerSec, 1);
    return true;
  }
  if (upper.startsWith("KNOCK TRSCALE ") &&
      parseFloat(cmd.substring(String("KNOCK TRSCALE ").length()), fValue)) {
    config.transientThresholdScale = fValue;
    out.print("Knock transient threshold scale: "); out.println(config.transientThresholdScale, 2);
    return true;
  }
  if (upper.startsWith("KNOCK TRHOLD ") &&
      parseUint16(cmd.substring(String("KNOCK TRHOLD ").length()), u16Value)) {
    config.transientHoldMs = u16Value;
    out.print("Knock transient hold ms: "); out.println(config.transientHoldMs);
    return true;
  }
  if (upper.startsWith("KNOCK DETCONF ") &&
      parseFloat(cmd.substring(String("KNOCK DETCONF ").length()), fValue)) {
    config.minDetectConfidence = fValue;
    out.print("Knock detect confidence: "); out.println(config.minDetectConfidence, 2);
    return true;
  }
  if (upper.startsWith("KNOCK WARNCONF ") &&
      parseFloat(cmd.substring(String("KNOCK WARNCONF ").length()), fValue)) {
    config.warnConfidence = fValue;
    out.print("Knock warn confidence: "); out.println(config.warnConfidence, 2);
    return true;
  }
  if (upper.startsWith("KNOCK CRITCONF ") &&
      parseFloat(cmd.substring(String("KNOCK CRITCONF ").length()), fValue)) {
    config.criticalConfidence = fValue;
    out.print("Knock critical confidence: "); out.println(config.criticalConfidence, 2);
    return true;
  }
  if (upper.startsWith("KNOCK IATSTART ") &&
      parseFloat(cmd.substring(String("KNOCK IATSTART ").length()), fValue)) {
    config.iatTempCompStartC = fValue;
    out.print("Knock IAT comp start C: "); out.println(config.iatTempCompStartC, 1);
    return true;
  }
  if (upper.startsWith("KNOCK IATPER ") &&
      parseFloat(cmd.substring(String("KNOCK IATPER ").length()), fValue)) {
    config.iatTempCompPerC = fValue;
    out.print("Knock IAT comp per C: "); out.println(config.iatTempCompPerC, 3);
    return true;
  }
  if (upper.startsWith("KNOCK BAYSTART ") &&
      parseFloat(cmd.substring(String("KNOCK BAYSTART ").length()), fValue)) {
    config.bayTempCompStartC = fValue;
    out.print("Knock bay comp start C: "); out.println(config.bayTempCompStartC, 1);
    return true;
  }
  if (upper.startsWith("KNOCK BAYPER ") &&
      parseFloat(cmd.substring(String("KNOCK BAYPER ").length()), fValue)) {
    config.bayTempCompPerC = fValue;
    out.print("Knock bay comp per C: "); out.println(config.bayTempCompPerC, 3);
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
