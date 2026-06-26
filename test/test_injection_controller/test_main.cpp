#include <math.h>
#include <stdio.h>

#include "injection_controller.h"

namespace {

bool approximately(float a, float b, float eps = 0.25f) {
  return fabsf(a - b) <= eps;
}

AppConfig makeConfig() {
  AppConfig cfg{};
  cfg.mode = InjectionMode::BoostOnly;
  cfg.gainK = 30.0f;
  cfg.dutyMinPercent = 18.0f;
  cfg.dutyMaxPercent = 95.0f;
  cfg.overboostWarnDutyPercent = 85.0f;
  cfg.boost.startPsi = 3.5f;
  cfg.boost.fullPsi = 7.5f;
  cfg.boost.overboostWarnPsi = 12.0f;
  cfg.boost.overboostEmergencyPsi = 15.0f;
  return cfg;
}

SensorReadings makeReadings(float boostPsi, bool mapValid = true, bool tankLow = false) {
  SensorReadings r{};
  r.mapValid = mapValid;
  r.tankLow = tankLow;
  r.boostPsi = boostPsi;
  return r;
}

TankBlend makeBlend() {
  TankBlend b{};
  b.totalLiters = 2.0f;
  b.methFraction = 0.25f;
  return b;
}

bool belowSprayThreshold_pumpOff() {
  InjectionController c;
  const AppConfig cfg = makeConfig();
  const TankBlend blend = makeBlend();

  const ControlResult res = c.update(makeReadings(1.0f), cfg, blend);
  return !res.pump.enabled && approximately(res.finalDutyPercent, 0.0f);
}

bool progressiveDuty_ramps() {
  InjectionController c;
  const AppConfig cfg = makeConfig();
  const TankBlend blend = makeBlend();

  c.update(makeReadings(4.2f), cfg, blend);
  const ControlResult res = c.update(makeReadings(5.0f), cfg, blend);

  const float expectedBase = cfg.gainK * (1.0f - blend.methFraction) * (5.0f - cfg.boost.startPsi);
  return res.pump.enabled &&
         res.finalDutyPercent > cfg.dutyMinPercent &&
         res.finalDutyPercent < cfg.dutyMaxPercent &&
         approximately(res.baseDutyPercent, expectedBase, 1.0f);
}

bool aboveFullSpray_usesMaxDuty() {
  InjectionController c;
  const AppConfig cfg = makeConfig();
  const TankBlend blend = makeBlend();

  c.update(makeReadings(4.5f), cfg, blend);
  const ControlResult res = c.update(makeReadings(8.0f), cfg, blend);
  return res.pump.enabled && approximately(res.finalDutyPercent, cfg.dutyMaxPercent);
}

bool overboostWarning_commandsHighDuty() {
  InjectionController c;
  AppConfig cfg = makeConfig();
  cfg.dutyMaxPercent = 90.0f;
  cfg.overboostWarnDutyPercent = 88.0f;
  const TankBlend blend = makeBlend();

  c.update(makeReadings(4.5f), cfg, blend);
  const ControlResult res = c.update(makeReadings(cfg.boost.overboostWarnPsi), cfg, blend);

  return res.overboostAssistActive &&
         !res.overboostEmergencyActive &&
         res.pump.enabled &&
      (res.finalDutyPercent >= cfg.overboostWarnDutyPercent) &&
      (res.finalDutyPercent <= 100.0f);
}

bool emergencyOverboost_commands100() {
  InjectionController c;
  const AppConfig cfg = makeConfig();
  const TankBlend blend = makeBlend();

  c.update(makeReadings(4.5f), cfg, blend);
  const ControlResult res = c.update(makeReadings(cfg.boost.overboostEmergencyPsi), cfg, blend);

  return res.overboostEmergencyActive && res.pump.enabled && approximately(res.finalDutyPercent, 100.0f, 0.1f);
}

bool emergencyFault_latches() {
  InjectionController c;
  const AppConfig cfg = makeConfig();
  const TankBlend blend = makeBlend();

  c.update(makeReadings(4.5f), cfg, blend);
  c.update(makeReadings(cfg.boost.overboostEmergencyPsi), cfg, blend);
  const ControlResult after = c.update(makeReadings(6.0f), cfg, blend);

  return after.overboostAssistFaultLatched;
}

bool pumpStopsAfterBoostDrops_butLatchRemains() {
  InjectionController c;
  const AppConfig cfg = makeConfig();
  const TankBlend blend = makeBlend();

  c.update(makeReadings(4.5f), cfg, blend);
  c.update(makeReadings(cfg.boost.overboostEmergencyPsi), cfg, blend);
  const ControlResult dropped = c.update(makeReadings(0.5f), cfg, blend);

  return !dropped.pump.enabled &&
         approximately(dropped.finalDutyPercent, 0.0f) &&
         dropped.overboostAssistFaultLatched;
}

bool sensorFault_forcesPumpOff_evenDuringOverboost() {
  InjectionController c;
  const AppConfig cfg = makeConfig();
  const TankBlend blend = makeBlend();

  c.update(makeReadings(4.5f), cfg, blend);
  const ControlResult mapFault = c.update(makeReadings(cfg.boost.overboostEmergencyPsi, false, false), cfg, blend);
  const ControlResult lowFluid = c.update(makeReadings(cfg.boost.overboostEmergencyPsi, true, true), cfg, blend);

  return (!mapFault.pump.enabled && mapFault.failsafe == FailsafeReason::MapInvalid) &&
         (!lowFluid.pump.enabled && lowFluid.failsafe == FailsafeReason::LowFluid);
}

} // namespace

int main() {
  int failed = 0;

  auto check = [&failed](const char *name, bool ok) {
    if (!ok) {
      ++failed;
      printf("FAILED: %s\n", name);
    }
  };

  check("belowSprayThreshold_pumpOff", belowSprayThreshold_pumpOff());
  check("progressiveDuty_ramps", progressiveDuty_ramps());
  check("aboveFullSpray_usesMaxDuty", aboveFullSpray_usesMaxDuty());
  check("overboostWarning_commandsHighDuty", overboostWarning_commandsHighDuty());
  check("emergencyOverboost_commands100", emergencyOverboost_commands100());
  check("emergencyFault_latches", emergencyFault_latches());
  check("pumpStopsAfterBoostDrops_butLatchRemains", pumpStopsAfterBoostDrops_butLatchRemains());
  check("sensorFault_forcesPumpOff_evenDuringOverboost", sensorFault_forcesPumpOff_evenDuringOverboost());

  return failed == 0 ? 0 : 1;
}
