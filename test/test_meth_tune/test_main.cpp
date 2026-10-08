// Host tests for the pulse tuning logic: g++ -std=c++11 -I include test/test_meth_tune/test_main.cpp
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

// Full-load inputs: enough airflow that the dose limit does not interfere.
Inputs spraying(float boostPsi, uint16_t rpm = 5500) {
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

struct Run {
  uint32_t onMs = 0;       // Total time the relay was on
  uint32_t risingEdges = 0;
  uint32_t shortestOn = 0xFFFFFFFFUL;   // Shortest completed pump run
  uint32_t shortestOff = 0xFFFFFFFFUL;  // Shortest gap between two runs
  uint32_t shortestCycle = 0xFFFFFFFFUL;  // Between two pump starts
};

// Runs the governor in 20 ms loop steps like the firmware and measures the relay.
template <typename InputAt>
Run simulate(Governor &g, const Settings &s, uint32_t fromMs, uint32_t toMs, InputAt inputAt) {
  Run run;
  bool previous = false, anyRise = false, anyFall = false;
  uint32_t riseAt = 0, fallAt = 0;
  for (uint32_t now = fromMs; now < toMs; now += 20) {
    const bool on = g.update(now, s, inputAt(now));
    if (on) run.onMs += 20;
    if (on && !previous) {
      if (anyRise && now - riseAt < run.shortestCycle) run.shortestCycle = now - riseAt;
      if (anyFall && now - fallAt < run.shortestOff) run.shortestOff = now - fallAt;
      riseAt = now;
      anyRise = true;
      ++run.risingEdges;
    }
    if (!on && previous) {
      if (now - riseAt < run.shortestOn) run.shortestOn = now - riseAt;
      fallAt = now;
      anyFall = true;
    }
    previous = on;
  }
  return run;
}

Settings defaults() {
  Settings s;
  loadDefaults(s);
  return s;
}

void change(Settings &s, uint8_t key, uint16_t value) {
  uint16_t applied = 0;
  set(s, key, value, applied);
}

void defaultsAreConservativeAndValid() {
  const Settings s = defaults();
  check(valid(s), "defaults are valid");
  check(s.value[kPeriodMs] == 4000 && s.value[kMinOnMs] == 1000 && s.value[kMaxOnMs] == 2000,
        "default: the pump runs 1-2 s in every 4 s");
  check(s.value[kMaxOnMs] * 100 / s.value[kPeriodMs] <= 50, "default never exceeds half flow");
  check(s.value[kMinRpm] >= 2000, "default needs the engine well above idle");
  check(s.value[kOverboostAssist] == 0, "overboost assist is off by default");
  check(s.value[kRestS] > 0, "default has a spray time limit");
  check(s.value[kMaxDosePct] > 0 && s.value[kMaxDosePct] <= 10, "default has a dose limit of at most 10% of fuel");
  check(s.value[kMethPercent] == 0 && s.value[kNozzleMlMin] == 60, "default: water only through a 60 ml/min nozzle");
}

void settingsClampAndNeverRaiseOtherValues() {
  Settings s = defaults();
  uint16_t applied = 0;
  check(set(s, kPeriodMs, 500, applied) == kSetClamped && applied == 1000, "cycle cannot go below 1000 ms");
  check(set(s, kPeriodMs, 60000, applied) == kSetClamped && applied == 10000, "cycle is capped at 10 s");
  check(set(s, kPeriodMs, 4000, applied) == kSetOk, "cycle back to 4 s");
  loadDefaults(s);
  check(set(s, kMinOnMs, 100, applied) == kSetClamped && applied == 500, "on-time cannot go below half a second");
  // Raising the minimum cannot push the maximum up.
  check(set(s, kMinOnMs, 3500, applied) == kSetClamped && applied == 2000 && s.value[kMaxOnMs] == 2000, "min on-time stops at max");
  check(set(s, kMaxOnMs, 9000, applied) == kSetClamped && applied == 4000, "max on-time stops at the cycle length");
  check(set(s, kMaxOnMs, 600, applied) == kSetClamped && applied == s.value[kMinOnMs], "max on-time stops at min");
  // Shortening the cycle shortens on-times, never lengthens them.
  loadDefaults(s);
  set(s, kMaxOnMs, 4000, applied);
  set(s, kMinOnMs, 3000, applied);
  set(s, kPeriodMs, 2000, applied);
  check(s.value[kMaxOnMs] == 2000 && s.value[kMinOnMs] == 2000 && valid(s), "on-times shrink with the cycle");
  loadDefaults(s);
  check(set(s, kStartPsiX10, 200, applied) == kSetClamped && applied == 90 && s.value[kFullPsiX10] == 100, "start stays 1 psi under full");
  check(set(s, kFullPsiX10, 20, applied) == kSetClamped && applied == 100, "full stays 1 psi above start");
  check(set(s, 0, 5, applied) == kSetUnsupported && set(s, kCount + 1, 5, applied) == kSetUnsupported, "unknown keys are refused");
  check(set(s, kMinRpm, 3000, applied) == kSetOk && applied == 3000 && valid(s), "a plain change is accepted");
  check(set(s, kMaxDosePct, 90, applied) == kSetClamped && applied == 40, "dose limit is capped at 40% of fuel");
  check(set(s, kMethPercent, 53, applied) == kSetOk && set(s, kNozzleMlMin, 5, applied) == kSetClamped && applied == 20, "blend and nozzle are range checked");
  Settings broken = s;
  broken.value[kPeriodMs] = 200;
  check(!valid(broken), "a sub-1 Hz cycle is never valid saved data");
  broken = s;
  broken.value[kMinOnMs] = 3900;
  check(!valid(broken), "min above max is not valid saved data");
  broken = s;
  broken.value[kMinOnMs] = 100;
  check(!valid(broken), "a blip shorter than half a second is not valid saved data");
}

void onTimeRisesWithBoost() {
  const Settings s = defaults();  // 1000..2000 ms between 5.0 and 10.0 psi
  check(targetOnMs(s, 2.0f) == 1000 && targetOnMs(s, 5.0f) == 1000, "minimum on-time at and below start");
  check(targetOnMs(s, 7.5f) == 1500, "half way at half boost");
  check(targetOnMs(s, 10.0f) == 2000 && targetOnMs(s, 25.0f) == 2000, "maximum on-time at and above full");
  check(targetOnMs(s, NAN) == 1000, "a bad boost reading gives the minimum");
}

void relayRules() {
  check(relaySafeOnMs(300, 4000) == 0, "shorter than half a second is not worth a relay cycle");
  check(relaySafeOnMs(500, 4000) == 500 && relaySafeOnMs(2000, 4000) == 2000, "normal pulses pass");
  check(relaySafeOnMs(3800, 4000) == 3500, "an off gap under half a second is widened by shortening the pulse");
  check(relaySafeOnMs(4000, 4000) == 4000 && relaySafeOnMs(9000, 4000) == 4000, "full cycle is continuous");
  check(relaySafeOnMs(700, 1000) == 500 && relaySafeOnMs(900, 1000) == 500, "1 s cycle: half on, half off at most");
}

void relayIsNeverAbused() {
  // Fastest allowed settings, boost fluttering across the whole range, RPM dropping
  // out and coming back at awkward moments, for ten minutes.
  Settings s = defaults();
  change(s, kPeriodMs, 1000);
  change(s, kMinOnMs, 500);
  change(s, kMaxOnMs, 1000);
  change(s, kRampMs, 0);
  change(s, kRestS, 0);
  change(s, kMaxDosePct, 0);
  Governor g;
  const Run rough = simulate(g, s, 1000, 601000, [](uint32_t now) {
    Inputs in = spraying(5.0f + static_cast<float>((now / 20) % 13) * 0.5f);
    // Long RPM dropouts, plus 60 ms glitches that cut a pulse just before the next cycle.
    in.rpmValid = (now / 730) % 5 != 0 && now % 1700 >= 60;
    return in;
  });
  check(rough.risingEdges > 100, "it does inject");
  check(rough.shortestCycle >= 1000, "never more than one pump start per second");
  check(rough.shortestOff >= 500, "the pump always rests at least half a second before restarting");
  check(rough.risingEdges <= 600, "no more than 600 starts in 600 s");
  // With nothing cutting pulses short, every pump run is at least half a second.
  Governor g2;
  const Run steady = simulate(g2, s, 1000, 601000, [](uint32_t now) {
    return spraying(5.0f + static_cast<float>((now / 20) % 13) * 0.5f);
  });
  check(steady.shortestOn >= 500 && steady.shortestOff >= 500 && steady.shortestCycle >= 1000, "steady running: no short blips");
  // The default 4 s cycle switches four times less often.
  const Settings d = defaults();
  Governor g3;
  const Run slow = simulate(g3, d, 0, 60000, [](uint32_t) { return spraying(8.0f); });
  check(slow.shortestCycle >= 4000 && slow.risingEdges <= 15, "default: one pump start every 4 s at most");
}

void steadyInjectionMatchesTheSettings() {
  Settings s = defaults();
  change(s, kRampMs, 0);
  change(s, kRestS, 0);
  Governor g;
  const Run low = simulate(g, s, 0, 40000, [](uint32_t) { return spraying(5.0f); });
  check(low.risingEdges == 10 && low.onMs == 10 * 1000, "1 s in every 4 s at start boost");
  Governor g2;
  const Run high = simulate(g2, s, 0, 40000, [](uint32_t) { return spraying(12.0f); });
  check(high.risingEdges == 10 && high.onMs == 10 * 2000, "2 s in every 4 s at full boost");
  // Max on-time equal to the cycle means the pump stays on without clicking.
  change(s, kMaxOnMs, 4000);
  Governor g3;
  const Run full = simulate(g3, s, 0, 40000, [](uint32_t) { return spraying(12.0f); });
  check(full.risingEdges == 1 && full.onMs == 40000, "100% is continuous: one start, no relay cycling");
}

void rampStartsAtMinimumAndGrowsPerCycle() {
  Settings s = defaults();  // ramp 500 ms per cycle, 1000..2000 ms
  change(s, kMaxOnMs, 3500);
  change(s, kRestS, 0);
  Governor g;
  uint32_t ons[7] = {0};
  for (int cycle = 0; cycle < 7; ++cycle) {
    ons[cycle] = simulate(g, s, cycle * 4000UL, cycle * 4000UL + 4000UL, [](uint32_t) { return spraying(20.0f); }).onMs;
  }
  check(ons[0] == 1000, "first pulse is the minimum even at full boost");
  check(ons[1] == 1500 && ons[2] == 2000, "each cycle grows by one ramp step");
  check(ons[5] == 3500 && ons[6] == 3500, "it reaches the full on-time and stays there");
  // After a pause it starts from the minimum again.
  simulate(g, s, 28000, 40000, [](uint32_t) { return idle(); });
  check(simulate(g, s, 40000, 44000, [](uint32_t) { return spraying(20.0f); }).onMs == 1000, "ramp restarts after a pause");
}

void rpmGateBlocksAndNamesTheReason() {
  const Settings s = defaults();
  Governor g;
  Inputs in = spraying(12.0f, 1800);
  check(!g.update(0, s, in) && g.hold() == kHoldRpmLow, "below minimum RPM: no injection");
  in.rpmValid = false;
  in.rpm = 6000;
  check(!g.update(20, s, in) && g.hold() == kHoldRpmMissing, "no RPM message: no injection, whatever the last value was");
  in.rpmValid = true;
  check(g.update(40, s, in) && g.hold() == kHoldNone && g.onMs() == 1000, "RPM back: injection starts");
  // Losing RPM in the middle of a pulse cuts the pump at once.
  in.rpmValid = false;
  check(!g.update(60, s, in) && g.onMs() == 0, "pulse is cut the moment RPM is lost");
  in.rpmValid = true;
  check(!g.update(80, s, in) && !g.update(3000, s, in), "and stays off for the rest of that cycle");
  check(g.update(4040, s, in), "next cycle injects again");
  check(!g.update(9000, s, idle(kHoldTankLow)) && g.hold() == kHoldTankLow, "idle reason is passed through");
}

void temperatureGate() {
  Settings s = defaults();
  Governor g;
  Inputs in = spraying(12.0f);
  check(g.update(0, s, in), "gate off by default: no sensor needed");
  change(s, kMinPreTempC, 40);
  Governor g2;
  check(!g2.update(0, s, in) && g2.hold() == kHoldTempSensor, "gate on with no sensor: blocked");
  in.preValid = true;
  in.preC = 25.0f;
  check(!g2.update(20, s, in) && g2.hold() == kHoldAirCold, "air colder than the limit: blocked");
  in.preC = 55.0f;
  check(g2.update(40, s, in) && g2.hold() == kHoldNone, "hot air: injects");
}

void sprayTimeLimitForcesARest() {
  Settings s = defaults();  // 12 s injecting, then 6 s rest
  Governor g;
  const Run first = simulate(g, s, 0, 12000, [](uint32_t) { return spraying(12.0f); });
  check(first.risingEdges == 3, "injects for the allowed 12 s");
  const Run rest = simulate(g, s, 12000, 17900, [](uint32_t) { return spraying(12.0f); });
  check(rest.onMs == 0 && g.hold() == kHoldResting && g.resting(), "then rests even though boost is still there");
  const Run after = simulate(g, s, 17900, 30000, [](uint32_t) { return spraying(12.0f); });
  check(after.risingEdges >= 2, "and resumes after the rest");
  // Brief lifts do not reset the limit: 10 s on boost, 0.5 s off, repeated.
  Governor g2;
  uint32_t total = 0;
  for (int i = 0; i < 6; ++i) {
    total += simulate(g2, s, i * 10500UL, i * 10500UL + 10000UL, [](uint32_t) { return spraying(12.0f); }).onMs;
    simulate(g2, s, i * 10500UL + 10000UL, i * 10500UL + 10500UL, [](uint32_t) { return idle(); });
  }
  check(total < 6 * 3 * 2000 * 8 / 10, "short lifts do not defeat the time limit");
  change(s, kRestS, 0);
  Governor g3;
  const Run endless = simulate(g3, s, 0, 120000, [](uint32_t) { return spraying(12.0f); });
  check(endless.risingEdges == 30, "rest 0 turns the time limit off");
}

void doseLimitFollowsFuelFlow() {
  check(fabsf(fuelGPerMin(5500, 154.0f) - 744.0f) < 3.0f, "about 744 g of fuel a minute at 5500 RPM and 10 psi");
  Settings s = defaults();
  change(s, kMaxOnMs, 4000);  // Ask for the pump on all the time at full boost.
  change(s, kRampMs, 0);
  // Plenty of airflow: 10% of 744 g/min is more than the 60 ml/min nozzle can deliver.
  check(doseLimitOnMs(s, 5500, 154.0f) == 4000, "at full load the nozzle is below the dose limit");
  Governor g;
  g.update(0, s, spraying(12.0f, 5500));
  check(g.onMs() == 4000 && g.hold() == kHoldNone, "full load: continuous");
  // Low airflow: 2500 RPM at 120 kPa burns about 264 g/min; 10% is 26 ml/min = 44% of the nozzle.
  Inputs low = spraying(12.0f, 2500);
  low.mapKpa = 120.0f;
  Governor g2;
  g2.update(0, s, low);
  check(g2.onMs() >= 1700 && g2.onMs() <= 1780, "low airflow: on-time held to 10% of fuel");
  change(s, kMaxDosePct, 20);
  Governor g3;
  g3.update(0, s, low);
  check(g3.onMs() == 3500, "a gap too short for the relay is widened, never closed");
  change(s, kMaxDosePct, 0);
  Governor g4;
  g4.update(0, s, low);
  check(g4.onMs() == 4000, "dose limit off: on-time settings alone");
  // Barely any airflow: the allowed dose is less than the shortest pulse, so nothing.
  change(s, kMaxDosePct, 10);
  change(s, kMinRpm, 0);
  Inputs tiny = spraying(12.0f, 700);
  tiny.mapKpa = 45.0f;
  Governor g5;
  check(!g5.update(0, s, tiny) && g5.hold() == kHoldDoseLimit && g5.onMs() == 0, "too little airflow for even a short pulse: held");
  // A bigger nozzle is limited sooner.
  change(s, kNozzleMlMin, 225);
  check(doseLimitOnMs(s, 5500, 154.0f) < 1400, "a 225 ml/min nozzle is held to about a third at full load");
}

void overboostAssistOnlyWhenEnabledAndCapsApply() {
  Settings s = defaults();
  Inputs in = spraying(16.0f);
  in.assistActive = true;
  in.assistDutyPercent = 100.0f;
  Governor g;
  g.update(0, s, in);
  check(g.onMs() == 1000, "assist off: overboost still follows the ramp and limits");
  change(s, kOverboostAssist, 1);
  Governor g2;
  g2.update(0, s, in);
  check(g2.onMs() == 4000, "assist on: overboost forces full flow at once");
  in.capDutyPercent = 40.0f;
  Governor g3;
  g3.update(0, s, in);
  check(g3.onMs() == 1600, "the master duty cap still wins");
  in.capDutyPercent = 100.0f;
  in.rpm = 2500;
  in.mapKpa = 120.0f;
  Governor g4;
  g4.update(0, s, in);
  check(g4.onMs() < 1800, "and so does the dose limit");
}

void pumpTestUsesTheSameRelayRules() {
  const Settings s = defaults();
  auto run = [&](uint8_t duty, uint32_t &on, uint32_t &edges) {
    Governor g;
    bool previous = false;
    on = edges = 0;
    for (uint32_t now = 0; now < 8000; now += 20) {
      const bool state = g.manual(now, s, duty);
      on += state ? 20 : 0;
      edges += state && !previous;
      previous = state;
    }
  };
  uint32_t on, edges;
  run(25, on, edges);
  check(edges == 2 && on == 2 * 1000, "25% test: 1 s in every 4 s");
  run(5, on, edges);
  check(edges == 2 && on == 2 * 500, "a tiny test duty still runs the pump for half a second");
  run(100, on, edges);
  check(edges == 1 && on == 8000, "100% test is continuous");
  run(0, on, edges);
  check(edges == 0 && on == 0, "0% test keeps the pump off");
}

void thermistorCurve() {
  float c = 0.0f;
  check(thermistorC(599, 2490.0f, c) && fabsf(c - 20.0f) < 1.0f, "3520 ohm reads 20 C");
  check(thermistorC(162, 2490.0f, c) && fabsf(c - 70.0f) < 1.5f, "467 ohm reads 70 C");
  check(thermistorC(809, 2490.0f, c) && fabsf(c - 0.0f) < 1.0f, "9420 ohm reads 0 C");
  check(!thermistorC(1023, 2490.0f, c) && !thermistorC(1015, 2490.0f, c), "unplugged sensor is invalid");
  check(!thermistorC(0, 2490.0f, c) && !thermistorC(5, 2490.0f, c), "shorted sensor is invalid");
}
} // namespace

int main() {
  defaultsAreConservativeAndValid();
  settingsClampAndNeverRaiseOtherValues();
  onTimeRisesWithBoost();
  relayRules();
  relayIsNeverAbused();
  steadyInjectionMatchesTheSettings();
  rampStartsAtMinimumAndGrowsPerCycle();
  rpmGateBlocksAndNamesTheReason();
  temperatureGate();
  sprayTimeLimitForcesARest();
  doseLimitFollowsFuelFlow();
  overboostAssistOnlyWhenEnabledAndCapsApply();
  pumpTestUsesTheSameRelayRules();
  thermistorCurve();
  if (failures) {
    printf("%d meth tune test(s) failed\n", failures);
    return 1;
  }
  printf("meth tune tests passed\n");
  return 0;
}
