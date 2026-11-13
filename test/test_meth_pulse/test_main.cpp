// Host tests for the pulse drive (meth_tune::PulseGovernor): the ESP32 board's pump on its
// MOSFET, full on or full off with slow pulse timing, each start ramped softly.
// g++ -std=c++11 -I include test/test_meth_pulse/test_main.cpp -o meth_pulse_test && ./meth_pulse_test
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

// Boost at this RPM; 6000 RPM is enough airflow that the dose limit does not interfere.
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
  loadDefaults(s);   // 4 s cycle, 1 s at start boost (0.5 psi) to 2 s at full (10 psi), +0.5 s a cycle
  return s;
}

// Runs the governor in 20 ms steps from fromMs up to (not including) toMs; the last duty.
float run(PulseGovernor &g, const Settings &s, uint32_t fromMs, uint32_t toMs, const Inputs &in) {
  float duty = 0.0f;
  for (uint32_t now = fromMs; now < toMs; now += 20) duty = g.update(now, s, in);
  return duty;
}

// How long the pump ran at each duty over a stretch: on (above 0) and at full (100%).
struct Tally {
  uint32_t onMs{0};
  uint32_t fullMs{0};
  bool onlyOffOrFullAfterStart{true};
};
Tally tally(PulseGovernor &g, const Settings &s, uint32_t fromMs, uint32_t toMs, const Inputs &in) {
  Tally t;
  for (uint32_t now = fromMs; now < toMs; now += 20) {
    const float duty = g.update(now, s, in);
    if (duty > 0.0f) t.onMs += 20;
    if (duty >= 100.0f) t.fullMs += 20;
    if (duty > 0.0f && duty < 100.0f && !g.softStarting(now)) t.onlyOffOrFullAfterStart = false;
  }
  return t;
}

void pulsesAreFullOnOrOffWithSlowTiming() {
  const Settings s = defaults();
  PulseGovernor g;
  const Inputs full = spraying(10.0f);
  // First pulse: 1 s (a run starts from the minimum on-time), the first 0.4 s ramping.
  check(near(g.update(0, s, full), kSoftStartFloorPct), "a pulse starts at the soft start's floor");
  check(g.softStarting(0) && g.onMs() == 1000, "first pulse: 1 s, soft starting");
  check(near(run(g, s, 20, 220, full), 5.0f + 95.0f * 200.0f / 400.0f, 1.0f), "halfway through the soft start: about half");
  check(run(g, s, 220, 420, full) == 100.0f && !g.softStarting(400), "full on after 0.4 s");
  check(run(g, s, 420, 1000, full) == 100.0f, "full on to the end of the pulse");
  check(g.update(1000, s, full) == 0.0f, "then fully off");
  check(run(g, s, 1020, 4000, full) == 0.0f, "off for the rest of the 4 s cycle");
  // Each pulse grows by the ramp (0.5 s) up to 2 s at full boost, every one at full pressure.
  Tally second = tally(g, s, 4000, 8000, full);
  check(second.onMs == 1500 && second.fullMs == 1100 && second.onlyOffOrFullAfterStart, "second pulse: 1.5 s, 1.1 s of it at 100%");
  Tally third = tally(g, s, 8000, 12000, full);
  check(third.onMs == 2000 && third.fullMs == 1600 && third.onlyOffOrFullAfterStart, "third pulse: 2 s, the full-boost on-time");
}

void aContinuousRunIsNotRestarted() {
  Settings s = defaults();
  s.value[kMaxOnMs] = 4000;   // On continuously at full boost
  s.value[kRampMs] = 0;
  s.value[kMinOnMs] = 4000;
  s.value[kRestS] = 0;        // No spray time limit, so nothing stops it at 12 s
  PulseGovernor g;
  const Inputs full = spraying(10.0f);
  Tally t = tally(g, s, 0, 12000, full);
  check(t.onMs == 12000 && t.fullMs == 12000 - 400, "on for 12 s, softly started once");
  check(g.update(12000, s, full) == 100.0f && !g.softStarting(12000), "rolling on into the next cycle is not a new start");
}

void theSoftStartCanBeLongerOrOff() {
  const Settings s = defaults();
  PulseGovernor slow;
  slow.setSoftStartMs(800);   // After a reset that came while the pump ran (reset_guard)
  slow.update(0, s, spraying(10.0f));
  check(near(run(slow, s, 20, 420, spraying(10.0f)), 5.0f + 95.0f * 400.0f / 800.0f, 1.0f), "a doubled soft start is half way at 0.4 s");
  check(run(slow, s, 420, 820, spraying(10.0f)) == 100.0f, "and full at 0.8 s");
  PulseGovernor none;
  none.setSoftStartMs(0);
  check(none.update(0, s, spraying(10.0f)) == 100.0f && !none.softStarting(0), "no soft start: full at once");
}

void flowIsTheShareOfTheCycleSoTheDoseLimitIsExact() {
  Settings s = defaults();
  s.value[kMaxDosePct] = 5;
  PulseGovernor g;
  // 2500 RPM at 10 psi burns about 338 g/min; 5% is 16.9 ml/min, 28% of the 60 ml/min
  // nozzle: 1.13 s of every 4 s, though full boost asks for 2 s.
  const Inputs lowAirflow = spraying(10.0f, 2500);
  check(doseLimitOnMs(s, 2500, lowAirflow.mapKpa) > 1100 && doseLimitOnMs(s, 2500, lowAirflow.mapKpa) < 1150, "dose limit: 1.13 s in 4 s");
  run(g, s, 0, 4000, lowAirflow);                 // First pulse: 1 s
  g.update(4000, s, lowAirflow);
  check(g.onMs() > 1100 && g.onMs() < 1150, "the next pulse is held to the dose limit, not grown to 2 s");
  // Less allowed than the shortest pulse: nothing.
  s.value[kMaxDosePct] = 1;
  PulseGovernor held;
  check(run(held, s, 0, 2000, lowAirflow) == 0.0f && held.hold() == kHoldDoseLimit, "under half a second allowed: held, pump off");
}

void theReasonsItStopsStillApply() {
  const Settings s = defaults();
  PulseGovernor g;
  check(run(g, s, 0, 1000, idle()) == 0.0f && g.hold() == kHoldBelowBoost, "below boost: off");
  check(run(g, s, 1000, 2000, spraying(10.0f, 2000)) == 0.0f && g.hold() == kHoldRpmLow, "under the minimum RPM: off");
  check(run(g, s, 2000, 3000, idle(kHoldTankLow)) == 0.0f && g.hold() == kHoldTankLow, "tank low: off");
  // Boost falls away in the middle of a pulse: off at once, and not back on in that cycle.
  PulseGovernor cut;
  run(cut, s, 0, 600, spraying(10.0f));
  check(cut.update(600, s, idle()) == 0.0f, "cut mid-pulse: off at once");
  check(run(cut, s, 620, 1000, spraying(10.0f)) == 0.0f, "back in boost within the cycle: stays off");
  // Twelve seconds of spraying, then the rest.
  PulseGovernor budget;
  run(budget, s, 0, 12100, spraying(10.0f));
  check(budget.hold() == kHoldResting && budget.resting() && budget.update(12100, s, spraying(10.0f)) == 0.0f, "rests after 12 s");
}

void thePumpTestIsTheShareOfTheCycleAtFullPressure() {
  const Settings s = defaults();
  PulseGovernor g;
  check(near(g.manual(0, s, 100), kSoftStartFloorPct), "a test starts softly too");
  float duty = 0.0f;
  for (uint32_t now = 20; now <= 2980; now += 20) duty = g.manual(now, s, 100);
  check(duty == 100.0f && g.onMs() == 4000, "100%: on continuously at full pressure");
  PulseGovernor half;
  uint32_t onMs = 0;
  for (uint32_t now = 0; now < 4000; now += 20) onMs += half.manual(now, s, 50) > 0.0f ? 20 : 0;
  check(onMs == 2000, "50%: 2 s of the 4 s cycle");
  check(half.manual(4000, s, 0) == 0.0f, "0: off");
}
} // namespace

int main() {
  pulsesAreFullOnOrOffWithSlowTiming();
  aContinuousRunIsNotRestarted();
  theSoftStartCanBeLongerOrOff();
  flowIsTheShareOfTheCycleSoTheDoseLimitIsExact();
  theReasonsItStopsStillApply();
  thePumpTestIsTheShareOfTheCycleAtFullPressure();
  if (failures) {
    printf("%d meth pulse test(s) failed\n", failures);
    return 1;
  }
  printf("meth pulse tests passed\n");
  return 0;
}
