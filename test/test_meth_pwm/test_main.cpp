// Host tests for the PWM pump drive: g++ -std=c++11 -I include test/test_meth_pwm/test_main.cpp
#include <math.h>
#include <stdio.h>

#include "meth_tune.h"

using namespace meth_tune;

namespace {
int failures = 0;

void check(bool ok, const char *what) {
  if (!ok) {
    ++failures;
    printf("FAIL: %s\n", what);
  }
}

bool near(float a, float b, float tolerance = 0.6f) { return fabs(a - b) <= tolerance; }

// Full-load inputs: enough airflow that the dose limit does not interfere.
Inputs spraying(float boostPsi, uint16_t rpm = 6000) {
  Inputs in;
  in.wantSpray = true;
  in.boostPsi = boostPsi;
  in.mapKpa = 85.0f + boostPsi * 6.895f;
  in.rpmValid = true;
  in.rpm = rpm;
  return in;
}

Inputs idle(uint8_t reason = kHoldBelowBoost) {
  Inputs in;
  in.idleHold = reason;
  return in;
}

Settings defaults() {
  Settings s;
  loadDefaults(s);
  // These tests were written around a 5 psi start; the shipped start is 0.5 psi
  // (checked in the meth tune tests).
  s.value[kStartPsiX10] = 50;
  return s;
}

// Runs the governor in 20 ms steps and returns the last duty.
float run(PwmGovernor &g, const Settings &s, uint32_t fromMs, uint32_t toMs, const Inputs &in) {
  float duty = 0.0f;
  for (uint32_t now = fromMs; now < toMs; now += 20) duty = g.update(now, s, in);
  return duty;
}

void dutySettingsAreConservativeAndPaired() {
  Settings s = defaults();
  check(s.value[kMinDutyPct] == 25 && s.value[kMaxDutyPct] == 50 && valid(s), "defaults: 25% at start boost, 50% at full");
  uint16_t applied = 0;
  check(set(s, kMinDutyPct, 80, applied) == kSetClamped && applied == 50, "minimum cannot pass the maximum");
  check(set(s, kMaxDutyPct, 15, applied) == kSetClamped && applied == 50, "maximum cannot drop under the minimum");
  check(set(s, kMaxDutyPct, 200, applied) == kSetClamped && applied == 100, "never above 100%");
  check(set(s, kMinDutyPct, 0, applied) == kSetClamped && applied == 10, "never below the 10% floor");
  check(s.value[kPeriodMs] == 4000 && s.value[kMinOnMs] == 1000, "the relay settings are untouched");
  s.value[kMinDutyPct] = 90; s.value[kMaxDutyPct] = 40;
  check(!valid(s), "a saved block with minimum above maximum is rejected");
}

void dutyRisesWithBoostBetweenTheTwoSettings() {
  const Settings s = defaults();   // 5 psi start, 10 psi full
  check(near(targetDuty(s, 5.0f), 25.0f) && near(targetDuty(s, 7.5f), 37.5f) && near(targetDuty(s, 10.0f), 50.0f), "straight line from start to full boost");
  check(near(targetDuty(s, 20.0f), 50.0f) && near(targetDuty(s, 0.0f), 25.0f), "flat outside the range");
  check(near(targetDuty(s, NAN), 25.0f), "a bad boost reading asks for the minimum, never more");
  // Flow follows the square root of duty: the estimate errs on the side of more fluid.
  check(near(flowFraction(25.0f), 0.5f, 0.01f) && near(flowFraction(100.0f), 1.0f, 0.01f) && flowFraction(0.0f) == 0.0f, "flow estimate: sqrt of duty");
  check(flowFraction(50.0f) > 0.5f, "half duty is counted as more than half flow");
}

void steadyInjectionAndTheReasonsItStops() {
  Settings s = defaults();
  s.value[kRampMs] = 0;
  PwmGovernor g;
  check(run(g, s, 0, 1000, idle()) == 0.0f && g.hold() == kHoldBelowBoost, "off below boost");
  check(near(run(g, s, 1000, 2000, spraying(7.5f)), 37.5f) && g.hold() == kHoldNone, "mid boost: mid duty");
  check(near(run(g, s, 2000, 3000, spraying(12.0f)), 50.0f), "full boost: maximum duty");
  check(run(g, s, 3000, 3100, idle(kHoldDisarmed)) == 0.0f && g.hold() == kHoldDisarmed, "disarmed: off at once");
  Inputs noRpm = spraying(8.0f);
  noRpm.rpmValid = false;
  check(run(g, s, 3100, 3200, noRpm) == 0.0f && g.hold() == kHoldRpmMissing, "no engine speed: no injection");
  check(run(g, s, 3200, 3300, spraying(8.0f, 1500)) == 0.0f && g.hold() == kHoldRpmLow, "below the minimum RPM: no injection");
  check(run(g, s, 3300, 3400, idle(kHoldTankLow)) == 0.0f && g.hold() == kHoldTankLow, "tank low is passed through");
  s.value[kMinPreTempC] = 30;
  Inputs cold = spraying(8.0f);
  cold.preValid = true; cold.preC = 10.0f;
  check(run(g, s, 3400, 3500, cold) == 0.0f && g.hold() == kHoldAirCold, "cold air gate");
  cold.preValid = false;
  check(run(g, s, 3500, 3600, cold) == 0.0f && g.hold() == kHoldTempSensor, "the gate needs a working sensor");
}

void rampClimbsFromTheMinimumDuty() {
  Settings s = defaults();
  s.value[kRampMs] = 1000;   // One second from 25% to 50%
  s.value[kRestS] = 0;
  PwmGovernor g;
  run(g, s, 0, 500, idle());
  const float first = g.update(500, s, spraying(12.0f));
  check(near(first, 25.0f, 1.0f), "starts at the minimum duty, not below it and not at full");
  check(near(run(g, s, 520, 1020, spraying(12.0f)), 37.5f, 1.5f), "half way after half the ramp time");
  check(near(run(g, s, 1020, 1700, spraying(12.0f)), 50.0f), "full duty once the ramp time has passed");
  check(near(run(g, s, 1700, 1740, spraying(5.0f)), 25.0f), "falling is immediate");
  run(g, s, 1740, 3000, idle());
  check(near(g.update(3000, s, spraying(12.0f)), 25.0f, 1.0f), "after a pause it starts from the minimum again");
}

void sprayTimeLimitForcesARest() {
  Settings s = defaults();   // 12 s of injection, then 6 s rest
  s.value[kRampMs] = 0;
  PwmGovernor g;
  check(run(g, s, 0, 11900, spraying(12.0f)) > 0.0f, "injecting inside the time limit");
  check(run(g, s, 11900, 12500, spraying(12.0f)) == 0.0f && g.hold() == kHoldResting, "rest once the limit is reached");
  check(run(g, s, 12500, 17500, spraying(12.0f)) == 0.0f, "still resting");
  check(run(g, s, 17500, 18600, spraying(12.0f)) > 0.0f, "injecting again after the rest");
}

void doseLimitCapsDutyAndBelowTheMinimumMeansNothing() {
  Settings s = defaults();   // 60 ml/min nozzle, at most 10% of fuel
  s.value[kRampMs] = 0;
  s.value[kMinRpm] = 0;
  // 6000 RPM at 12 psi: about 885 g/min of fuel, 88 ml/min allowed: no limit at all.
  check(doseLimitDuty(s, 6000, 85.0f + 12.0f * 6.895f) == 100.0f, "plenty of fuel: no cap");
  // 3500 RPM at 6 psi: about 388 g/min, 38.8 ml/min allowed = 65% of the nozzle = 42% duty.
  const float mapKpa = 85.0f + 6.0f * 6.895f;
  check(near(doseLimitDuty(s, 3500, mapKpa), 41.9f, 1.5f), "the cap is the square of the allowed flow fraction");
  s.value[kMaxDutyPct] = 100; s.value[kFullPsiX10] = 60; s.value[kStartPsiX10] = 20;
  PwmGovernor g;
  Inputs in = spraying(6.0f, 3500);
  in.mapKpa = mapKpa;
  check(near(run(g, s, 0, 500, in), 41.9f, 1.5f) && g.hold() == kHoldNone, "duty held at the dose limit");
  // 2000 RPM, barely in boost: under 30 ml/min allowed, less than the 25% minimum duty passes.
  Inputs low = spraying(2.5f, 2000);
  check(run(g, s, 500, 1000, low) == 0.0f && g.hold() == kHoldDoseLimit, "below the minimum duty: nothing, and the reason is named");
  s.value[kMaxDosePct] = 0;
  check(run(g, s, 1000, 1500, low) > 0.0f, "with the limit off it injects");
  Inputs capped = spraying(12.0f);
  capped.capDutyPercent = 30.0f;
  check(near(run(g, s, 1500, 2000, capped), 30.0f), "the master duty cap applies too");
}

void earlyStartAndHotAirAddInsideTheLimits() {
  Settings s = defaults();   // Early start from 3500 RPM
  s.value[kRampMs] = 0;
  PwmGovernor g;
  Inputs cruising = idle();
  cruising.rpmValid = true; cruising.rpm = 5000; cruising.boostPsi = -1.0f; cruising.mapKpa = 80.0f;
  check(near(run(g, s, 0, 500, cruising), 25.0f) && g.earlyStart(), "open throttle at high RPM: the minimum duty before boost");
  // At 4000 RPM the 10% dose limit allows less than the minimum duty passes: it waits.
  cruising.rpm = 4000;
  check(run(g, s, 500, 520, cruising) == 0.0f && g.hold() == kHoldDoseLimit, "the dose limit still rules an early start");
  cruising.rpm = 5000;
  cruising.boostPsi = -8.0f;
  check(run(g, s, 500, 1000, cruising) == 0.0f && !g.earlyStart(), "throttle shut: off");
  s.value[kHotStartC] = 40; s.value[kHotFullC] = 80;
  Inputs hot = spraying(5.0f);
  hot.preValid = true; hot.preC = 60.0f;
  check(near(run(g, s, 1000, 1500, hot), 37.5f) && g.hotAir(), "hot air half way: half way up the duty range");
  hot.preC = 30.0f;
  check(near(run(g, s, 1500, 2000, hot), 25.0f) && !g.hotAir(), "cool air adds nothing");
  Inputs assist = spraying(12.0f);
  assist.assistActive = true; assist.assistDutyPercent = 100.0f;
  check(near(run(g, s, 2000, 2500, assist), 50.0f), "overboost assist is ignored unless enabled");
  s.value[kOverboostAssist] = 1;
  check(near(run(g, s, 2500, 3000, assist), 100.0f), "enabled: assist raises the duty");
}

void pumpTestRunsAtTheDutyAsked() {
  const Settings s = defaults();
  PwmGovernor g;
  check(g.manual(0, s, 0) == 0.0f, "0%: off");
  check(near(g.manual(20, s, 5), 10.0f), "a very low test still turns the pump: 10%");
  check(near(g.manual(40, s, 60), 60.0f) && near(g.manual(60, s, 100), 100.0f), "the duty asked for");
  check(g.hold() == kHoldNone, "a test has no hold reason");
}
} // namespace

int main() {
  dutySettingsAreConservativeAndPaired();
  dutyRisesWithBoostBetweenTheTwoSettings();
  steadyInjectionAndTheReasonsItStops();
  rampClimbsFromTheMinimumDuty();
  sprayTimeLimitForcesARest();
  doseLimitCapsDutyAndBelowTheMinimumMeansNothing();
  earlyStartAndHotAirAddInsideTheLimits();
  pumpTestRunsAtTheDutyAsked();
  if (failures) {
    printf("%d meth PWM test(s) failed\n", failures);
    return 1;
  }
  printf("meth PWM tests passed\n");
  return 0;
}
