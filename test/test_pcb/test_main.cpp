// Host tests for the PCB builds' own logic: the board's inputs (pcb_sensors.h), the pump
// checks (pump_guard.h), the LED (pcb_led.h), the stand-alone settings
// (standalone_settings.h), the trouble codes (diag.h), the watchdogs (watchdogs.h) and the
// flow estimate, valve timing and pump current (flow_estimate.h).
// g++ -std=c++11 -O2 -Wall -I include test/test_pcb/test_main.cpp
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "diag.h"
#include "flow_estimate.h"
#include "watchdogs.h"

#include "pcb_led.h"
#include "pcb_sensors.h"
#include "pump_guard.h"
#include "pwm_drive.h"
#include "standalone_settings.h"

namespace {
int failures = 0;

void check(bool ok, const char *what) {
  if (!ok) {
    ++failures;
    printf("FAIL: %s\n", what);
  }
}

bool near(float a, float b, float tolerance) { return fabsf(a - b) <= tolerance; }

// The pin voltage for a thermistor of `ohms` on the board's circuit, for a sensor supply.
float pinMvForThermistor(float ohms, float railMv) {
  const float bottom = 1.0f / (1.0f / ohms + 1.0f / pcb_sensors::kDividerOhms);
  return railMv * bottom / (bottom + pcb_sensors::kPullupOhms) * pcb_sensors::kInputShare;
}

void inputs() {
  using namespace pcb_sensors;
  check(near(batteryV(1800.0f), 13.8f, 0.01f), "12 V monitor: 1.8 V at the pin is 13.8 V");
  check(near(pumpAmps(950.0f), 9.5f, 0.001f), "0.1 V per amp");
  check(near(inputMv(3214.0f), 4999.0f, 1.0f), "the 10k / 18k divider");
  float c = 0.0f;
  // GM curve points through the board's divider, at two sensor supplies: the supply cancels.
  check(intakeC(pinMvForThermistor(2238.0f, 4700.0f), 4700.0f, c) && near(c, 30.0f, 0.3f), "30 C at 4.7 V");
  check(intakeC(pinMvForThermistor(2238.0f, 4550.0f), 4550.0f, c) && near(c, 30.0f, 0.3f), "...and at 4.55 V");
  check(intakeC(pinMvForThermistor(177.0f, 4700.0f), 4700.0f, c) && near(c, 99.0f, 0.5f), "99 C");
  check(intakeC(pinMvForThermistor(100700.0f, 4700.0f), 4700.0f, c) && near(c, -40.0f, 1.0f), "-40 C");
  const float open = 4700.0f * kDividerOhms / (kDividerOhms + kPullupOhms) * kInputShare;
  check(!intakeC(open, 4700.0f, c), "nothing plugged in: no reading");
  check(!intakeC(5.0f, 4700.0f, c), "shorted: no reading");
  check(!intakeC(pinMvForThermistor(2238.0f, 4700.0f), 3000.0f, c), "sensor supply failed: no reading");
  float psi = 0.0f;
  // 2.5 V of 5 V on a 250 psi sensor is half way: 125 psi. At 4.7 V supply the same share.
  check(linePsi(0.5f * 4700.0f * kInputShare, 4700.0f, 250.0f, psi) && near(psi, 125.0f, 0.2f), "line pressure, ratiometric");
  check(linePsi(0.1f * 4700.0f * kInputShare, 4700.0f, 250.0f, psi) && near(psi, 0.0f, 0.2f), "0.5 V: 0 psi");
  check(!linePsi(0.0f, 4700.0f, 250.0f, psi), "unplugged: no reading");
  check(tankLow(true, false) && !tankLow(false, false), "tank switch open (or a broken wire): low");
  check(!tankLow(true, true) && tankLow(false, true), "a switch that closes when low");
}

pump_guard::Pass running(float duty, float amps, float volts = 13.8f) {
  pump_guard::Pass p;
  p.dutyPct = duty;
  p.ampsValid = true;
  p.amps = amps;
  p.supplyValid = true;
  p.supplyV = volts;
  return p;
}

void guard() {
  using namespace pump_guard;
  {
    Guard g;
    for (uint32_t t = 20; t < 3000; t += 20) g.update(t, running(100.0f, 8.0f));
    check(g.allows() && g.fault() == kNone, "a pump drawing 8 A runs");
    g.update(3020, running(100.0f, 30.0f));
    check(g.fault() == kShort, "30 A: short circuit at once");
    g.update(3040, running(0.0f, 0.0f));
    check(g.fault() == kShort, "it latches");
    g.clear();
    check(g.allows(), "cleared");
  }
  {
    Guard g;
    uint32_t t = 20;
    for (; t < 1000; t += 20) g.update(t, running(100.0f, 8.0f));
    for (uint32_t end = t + 200; t < end; t += 20) g.update(t, running(100.0f, 14.0f));
    check(g.fault() == kNone, "a short peak over the limit is allowed");
    for (uint32_t end = t + 400; t < end; t += 20) g.update(t, running(100.0f, 14.0f));
    check(g.fault() == kOverCurrent, "14 A held: over-current");
  }
  {
    Guard g;
    for (uint32_t t = 20; t < 400; t += 20) g.update(t, running(40.0f, 0.0f));
    check(g.fault() == kNone, "soft start: not judged for an open circuit");
    for (uint32_t t = 400; t < 1200; t += 20) g.update(t, running(100.0f, 0.05f));
    check(g.fault() == kNotConnected, "full drive, no current: not connected");
  }
  {
    Guard g;
    for (uint32_t t = 20; t < 2000; t += 20) g.update(t, running(0.0f, 5.0f));
    check(g.fault() == kStuckOn, "current with the pump off: the switch is stuck on");
  }
  {
    Guard g;
    uint32_t t = 20;
    for (; t < 300; t += 20) g.update(t, running(100.0f, 8.0f));
    for (uint32_t end = t + 300; t < end; t += 20) g.update(t, running(0.0f, 3.0f));
    check(g.fault() == kNone, "current dying away after the pump stops is not a stuck switch");
  }
  {
    Guard g;
    pump_guard::Pass p = running(100.0f, 8.0f);
    p.pressureFitted = p.pressureValid = true;
    p.minPsi = 60.0f;
    p.pressurePsi = 20.0f;
    for (uint32_t t = 20; t < 1400; t += 20) g.update(t, p);
    check(g.fault() == kNone, "pressure has 1.5 s to build");
    for (uint32_t t = 1400; t < 1700; t += 20) g.update(t, p);
    check(g.fault() == kNoPressure, "then no pressure stops it");
    Guard h;
    p.pressurePsi = 300.0f;
    p.maxPsi = 200.0f;
    for (uint32_t t = 20; t < 700; t += 20) h.update(t, p);
    check(h.fault() == kOverPressure, "pressure over the maximum: blocked nozzle");
  }
  {
    Guard g;
    g.update(20, running(0.0f, 0.0f, 9.5f));
    check(!g.allows() && g.hold() == kSupplyLow && g.fault() == kNone, "cranking voltage: held, not a fault");
    g.update(40, running(0.0f, 0.0f, 10.7f));
    check(g.hold() == kSupplyLow, "released only 0.5 V above");
    g.update(60, running(0.0f, 0.0f, 11.2f));
    check(g.allows(), "released");
    for (uint32_t t = 80; t < 1200; t += 20) g.update(t, running(0.0f, 0.0f, 17.0f));
    check(g.hold() == kSupplyHigh, "charging fault: held");
  }
}

void led() {
  using status_light::Show;
  check(pcb_led::lit(Show::Ready, 12345) && pcb_led::lit(Show::LampTest, 10), "ready: steady");
  check(pcb_led::lit(Show::Disarmed, 2050) && !pcb_led::lit(Show::Disarmed, 2500), "disarmed: a blip every 2 s");
  check(pcb_led::lit(Show::Fault, 0) && !pcb_led::lit(Show::Fault, 260), "fault: flashing");
  check(pcb_led::lit(Show::Spraying, 0) != pcb_led::lit(Show::Spraying, 50), "spraying: flicker");
}

void settings() {
  using namespace standalone;
  Settings s;
  loadDefaults(s);
  check(valid(s), "defaults are valid");
  check(s.value[kSource] == kSourceObd11 && bitrateOf(s) == 500000 && !listenOnlyOf(s), "out of the box: OBD-II at 500k");
  uint16_t applied = 0;
  set(s, kSource, kSourceEcu, applied);
  const engine::Profile *haltech = engine::profileNamed("haltech");
  for (uint8_t i = 0; i < engine::kProfileCount; ++i) {
    if (&engine::kProfiles[i] == haltech) set(s, kProfile, i, applied);
  }
  check(bitrateOf(s) == 1000000 && listenOnlyOf(s), "an ECU: its own speed, listening only");
  check(set(s, kBitrateK, 480, applied) == meth_tune::kSetClamped && applied == 500, "a CAN speed snaps to one CAN uses");
  set(s, kMapSensor, 0, applied);
  check(s.value[kMapAt05X10] == 287 && s.value[kMapAt45X10] == 1881, "a MAP sensor preset fills in its points");
  set(s, kMapAt45X10, 2000, applied);
  check(s.value[kMapSensor] == engine::kMapSensorCount, "...changing a point makes it your own");
  check(set(s, kMapAt05X10, 1990, applied) == meth_tune::kSetClamped && applied == 1900, "points kept apart");
  set(s, kSource, kSourceNone, applied);
  check(!hasRpmSource(s) && !hasMapSource(s), "no source and no MAP sensor: nothing to inject on");
  set(s, kMapFrom, 1, applied);
  check(hasMapSource(s) && !hasRpmSource(s), "a MAP sensor alone");
  bool tune = false;
  check(keyNamed("start_psi", tune) == meth_tune::kStartPsiX10 && tune, "tuning found by name");
  check(keyNamed("engine_l", tune) == kEngineDl && !tune, "own settings found by name");
  check(keyNamed("nothing", tune) == 0, "an unknown name");
  for (uint8_t key = 1; key <= kTuneShown; ++key) check(tuneInfoOf(key).name[0] != 0, "every tuning setting has a name");
}

// No engine speed at all: injection on boost alone, without the RPM gate or the dose limit.
void withoutRpm() {
  meth_tune::Settings s;
  meth_tune::loadDefaults(s);
  meth_tune::PulseGovernor g;
  g.setSoftStartMs(0);
  meth_tune::Inputs in;
  in.wantSpray = true;
  in.idleHold = meth_tune::kHoldNone;
  in.boostPsi = 8.0f;
  in.mapKpa = 160.0f;
  in.rpmValid = false;
  g.update(1000, s, in);
  check(g.hold() == meth_tune::kHoldRpmMissing, "normally no RPM means no injection");
  in.rpmRequired = false;
  meth_tune::PulseGovernor h;
  h.setSoftStartMs(0);
  check(h.update(1000, s, in) > 0.0f && h.hold() == meth_tune::kHoldNone, "with no RPM source set up, it sprays on boost");
  // Engine size: a bigger engine gets a proportionally bigger dose allowance.
  check(meth_tune::doseLimitOnMs(s, 2000, 100.0f, 2.0f) >= meth_tune::doseLimitOnMs(s, 2000, 100.0f, 1.0f), "engine size scales the dose");
}

// The PWM drive: duty from the lowest at start boost to the highest at full boost, only
// while the pulse timing allows injection, capped by the dose limit (square-root flow).
void pwmDrive() {
  meth_tune::Settings s;
  meth_tune::loadDefaults(s);
  uint16_t applied = 0;
  meth_tune::set(s, meth_tune::kMaxDutyPct, 90, applied);
  meth_tune::set(s, meth_tune::kMinDutyPct, 30, applied);
  meth_tune::set(s, meth_tune::kMaxDosePct, 0, applied);   // Dose limit off for the ramp
  meth_tune::Inputs in;
  in.rpmValid = true;
  in.rpm = 4000;
  in.mapKpa = 160.0f;
  in.boostPsi = s.value[meth_tune::kStartPsiX10] / 10.0f;
  check(near(pwm_drive::targetDuty(s, in, meth_tune::kHoldNone), 30.0f, 0.01f), "start boost: the lowest duty");
  in.boostPsi = 20.0f;
  check(near(pwm_drive::targetDuty(s, in, meth_tune::kHoldNone), 90.0f, 0.01f), "full boost and above: the highest");
  check(pwm_drive::targetDuty(s, in, meth_tune::kHoldRpmLow) == 0.0f, "a hold of the pulse timing stops it");
  meth_tune::set(s, meth_tune::kMaxDosePct, 10, applied);
  in.rpm = 2500;
  in.mapKpa = 120.0f;
  const float share = static_cast<float>(meth_tune::doseLimitOnMs(s, in.rpm, in.mapKpa)) / s.value[meth_tune::kPeriodMs];
  const float duty = pwm_drive::targetDuty(s, in, meth_tune::kHoldNone);
  check(share < 1.0f && (duty == 0.0f || near(duty, 100.0f * share * share, 0.01f)), "the dose limit caps the duty (square-root flow)");
  pwm_drive::Drive d;
  in.rpm = 6000; in.mapKpa = 200.0f; in.boostPsi = 20.0f;
  meth_tune::set(s, meth_tune::kMaxDosePct, 0, applied);
  check(d.update(1000, s, in, meth_tune::kHoldNone) < 10.0f, "each run soft-starts");
  check(near(d.update(1500, s, in, meth_tune::kHoldNone), 90.0f, 0.01f), "...then the duty asked for");
  check(d.update(1520, s, in, meth_tune::kHoldBelowBoost) == 0.0f, "off when the timing holds");
  pump_guard::Guard g;
  pump_guard::Pass p = running(40.0f, 0.05f);
  p.steadyFromPct = 30.0f;
  for (uint32_t t = 20; t < 1000; t += 20) g.update(t, p);
  check(g.fault() == pump_guard::kNotConnected, "PWM drive: a pump drawing nothing at steady duty is not connected");
}
void currentLimit() {
  using namespace pump_guard;
  Guard g;
  Pass p = running(100.0f, 7.0f);
  p.overAmps = 6.0f;
  uint32_t t = 20;
  for (; t < 1000; t += 20) g.update(t, p);
  check(g.fault() == kOverCurrent, "a lower current limit set for a small pump trips at it");
  Guard h;
  p.overAmps = 20.0f;
  p.amps = 14.0f;
  for (t = 20; t < 1000; t += 20) h.update(t, p);
  check(h.fault() == kOverCurrent, "a limit above the 10 A rating is not taken");
}

void codes() {
  using namespace diag;
  for (uint8_t c = 1; c < kCodeCount; ++c) check(infoOf(c).id[0] != '-' && infoOf(c).text[0] != 0, "every code has an id and a text");
  check(int(kPumpShort) == int(pump_guard::kShort) && int(kLineOverPressure) == int(pump_guard::kOverPressure), "pump codes share the guard's numbers");
  check(crc32("123456789", 9) == 0xCBF43926UL, "CRC-32 check value");
  Store st;
  st.begin(7);
  Freeze f;
  f.rpm = 4200;
  st.report(kSupplyLow, true, 10, f);
  st.report(kSupplyLow, true, 11, f);
  check(st.entry(kSupplyLow).count == 1 && st.entry(kSupplyLow).active, "a condition held is one occurrence");
  st.report(kSupplyLow, false, 12, f);
  f.rpm = 5100;
  st.report(kSupplyLow, true, 20, f);
  const Entry &e = st.entry(kSupplyLow);
  check(e.count == 2 && e.firstS == 10 && e.lastS == 20 && e.firstPowerUp == 7 && e.freeze.rpm == 5100, "counted again, the freeze frame from the last");
  st.event(kPumpKeeper, 30, f);
  st.event(kPumpKeeper, 31, f);
  check(st.entry(kPumpKeeper).count == 2 && !st.entry(kPumpKeeper).active, "events count every time and are not left active");
  check(st.activeOfSeverity(kWarning) == 1 && st.dirty(), "one warning present; changed since written");
  Store::Image img;
  st.save(img);
  Store back;
  check(back.load(img) && back.entry(kSupplyLow).count == 2 && !back.entry(kSupplyLow).active, "stored and read back, nothing active");
  img.entries[kSupplyLow].count = 99;
  Store damaged;
  check(!damaged.load(img) && !damaged.any(), "a damaged image is refused");
  back.begin(8);
  back.report(kSupplyLow, true, 1, f);
  check(back.entry(kSupplyLow).count == 3 && back.entry(kSupplyLow).lastPowerUp == 8, "next power-up: counted on");
  back.clear();
  check(!back.any(), "cleared");
}

void watchdogChecks() {
  using namespace watchdogs;
  PassTimer pt;
  check(!pt.pass(1000) && !pt.pass(1020) && pt.pass(1200) && pt.late() == 1 && pt.worstMs() == 180, "a pass 180 ms late is counted");
  Heartbeat hb;
  check(hb.check(0, 5) && hb.check(250, 5), "a task alive within 300 ms");
  check(!hb.check(320, 5), "its count stopped for 300 ms: stalled");
  check(hb.check(330, 6), "counting again: alive");
  check(RunLimit::limitMs(12, 6) == 20000 && RunLimit::limitMs(12, 0) == kRunNoLimitMs, "the limit follows the longest spray");
  // The defaults: 4 s cycle, longest spray 12 s, rest 6 s.
  RunLimit rl;
  uint32_t t = 0;
  // Full boost for a minute, as the pulse timing runs it: 12 s of injecting with 2 s of a
  // 4 s cycle on, then 6 s of rest, again and again. Never near the budget.
  for (; t < 60000; t += 20) {
    const uint32_t phase = t % 18000;
    rl.update(t, phase < 12000 && phase % 4000 < 2000, 12, 6);
  }
  check(!rl.tripped(), "pulsing as the pulse timing allows, a minute long");
  RunLimit cont;
  for (t = 0; t < 19000; t += 20) cont.update(t, true, 12, 6);
  check(!cont.tripped(), "continuous: allowed up to the longest spray plus margin");
  for (; t < 21000; t += 20) cont.update(t, true, 12, 6);
  check(cont.tripped(), "a pump left on past it trips, latched");
  RunLimit flicker;
  // Boost on and off every 3 s (2 s on, 1 s off) for two minutes: the pulse timing rests
  // when its own budget fills, so the pump is off for that; the off-time drains this one.
  for (t = 0; t < 120000; t += 20) flicker.update(t, t % 3000 < 2000 && (t / 18000) % 2 == 0, 12, 6);
  check(!flicker.tripped(), "boost flicking on and off within the budget");
  RunLimit none;
  for (t = 0; t < 170000; t += 20) none.update(t, t % 4000 < 3000, 12, 0);
  check(!none.tripped(), "no rest set: pulses for minutes are allowed");
  for (; t < 360000; t += 20) none.update(t, true, 12, 0);
  check(none.tripped(), "...but not 3 minutes on without a break");
  OutputCheck oc;
  check(!oc.update(512, 512, true, true), "outputs as set");
  oc.update(512, 0, true, true);
  oc.update(512, 512, true, true);
  oc.update(512, 0, true, true);
  check(!oc.tripped(), "a single difference is not a fault");
  oc.update(0, 0, false, true);
  oc.update(0, 0, false, true);
  check(oc.update(0, 0, false, true), "the valve pin high when it was set low, three passes: latched");
  Persist ps;
  check(!ps.update(0, true, 1000) && !ps.update(900, true, 1000) && ps.update(1000, true, 1000), "a condition held a second");
  check(!ps.update(1020, false, 1000) && !ps.update(1040, true, 1000), "a break starts it again");
  check(plausibleMap(101.0f) && plausibleMap(400.0f) && !plausibleMap(600.0f) && !plausibleMap(2.0f), "manifold pressure plausibility");
  check(plausibleRpm(7000) && !plausibleRpm(20000), "engine speed plausibility");
}

void flowEstimate() {
  using namespace flow;
  check(near(fromPressure(160.0f, 100.0f, 100.0f), 160.0f, 0.01f), "at the pressure it was measured at: the measured flow");
  check(near(fromPressure(160.0f, 100.0f, 25.0f), 80.0f, 0.01f), "a quarter the pressure: half the flow");
  check(near(fromPressure(160.0f, 100.0f, 900.0f), 320.0f, 0.01f), "never over twice the nozzle figure");
  check(fromPressure(160.0f, 0.0f, 100.0f) == 0.0f, "rated pressure unknown: no estimate from pressure");
  Pass p;
  p.valveOpen = true;
  p.dutyPct = 100.0f;
  p.nozzleMlMin = 160.0f;
  bool fromLine = true;
  check(near(estimate(p, fromLine), 160.0f, 0.01f) && !fromLine, "no transducer: the nozzle figure");
  p.ratedPsi = 100.0f;
  p.pressureValid = true;
  p.linePsi = 64.0f;
  check(near(estimate(p, fromLine), 128.0f, 0.01f) && fromLine, "with one: from the pressure");
  p.valveOpen = false;
  check(estimate(p, fromLine) == 0.0f, "valve shut: no flow");
  p.valveOpen = true;
  p.pressureValid = false;
  p.pwm = true;
  p.dutyPct = 25.0f;
  check(near(estimate(p, fromLine), 80.0f, 0.01f), "PWM without pressure: square root of the duty");
  Meter m;
  for (uint32_t t = 0; t <= 60000; t += 20) m.update(t, 120.0f);
  check(near(m.usedMl(), 120.0f, 0.5f) && near(m.driveMl(), 120.0f, 0.5f), "a minute at 120 ml/min is 120 ml");
  m.update(70000, 120.0f);
  check(near(m.usedMl(), 120.0f, 0.5f), "a gap over a second is not counted");
  check(near(m.leftMl(1000.0f), 880.0f, 0.5f) && m.leftMl(0.0f) < 0.0f && m.leftMl(50.0f) == 0.0f, "what is left of the tank");
  m.filled();
  check(m.usedMl() == 0.0f && m.driveMl() > 0.0f, "filled: the count starts again, the drive's stays");
}

void valveTiming() {
  flow::Valve v;
  check(!v.update(0, true, 150) && !v.update(100, true, 150) && v.update(160, true, 150), "first start: the valve opens after the lead");
  v.update(200, false, 150);
  check(v.update(400, true, 150), "after a short stop (pulses): at once, the line still has pressure");
  v.update(500, false, 150);
  check(!v.update(2000, true, 150) && v.update(2160, true, 150), "after a long stop: the lead again");
  flow::Valve w;
  check(w.update(0, true, 0), "no lead: with the pump");
  check(!w.update(20, false, 0), "shut the moment the pump stops");
}

void pumpCurrent() {
  flow::PumpCurrent pc;
  for (uint32_t t = 0; t < 300; t += 20) pc.update(t, 100.0f, 14.0f);
  check(!pc.known(), "the start-up surge is not the typical figure");
  for (uint32_t t = 300; t < 2000; t += 20) pc.update(t, 100.0f, 6.0f);
  check(pc.known() && near(pc.average(), 6.0f, 0.3f) && near(pc.peak(), 6.0f, 0.01f), "typical current at full drive");
  for (uint32_t t = 2000; t < 3000; t += 20) pc.update(t, 40.0f, 3.0f);
  check(near(pc.average(), 6.0f, 0.3f), "part duty is not counted");
}

void settingsAdded() {
  using namespace standalone;
  Settings s;
  loadDefaults(s);
  check(valid(s) && s.value[kPumpAmpsMaxX10] == 100 && s.value[kValveLeadMs] == 0 && s.value[kTankDl] == 0, "new settings' defaults");
  // Values saved by the first PCB firmware (19 settings): the new ones take their defaults.
  Settings old;
  loadDefaults(old);
  uint16_t blob[kCountV1 + 1];
  memcpy(blob, old.value, sizeof(blob));
  blob[kEngineDl] = 50;
  Settings migrated;
  loadDefaults(migrated);
  memcpy(migrated.value, blob, sizeof(blob));
  check(valid(migrated) && migrated.value[kEngineDl] == 50 && migrated.value[kPumpAmpsMaxX10] == 100, "an old store loads with new defaults");
  uint16_t applied = 0;
  check(set(s, kPumpAmpsMaxX10, 200, applied) == meth_tune::kSetClamped && applied == 100, "current limit at most 10 A");
  check(inProfile(kDrive) && inProfile(kNozzleRatedPsi) && !inProfile(kSource) && !inProfile(kArmed) && !inProfile(kLineFullPsi),
        "profiles carry how it sprays, not the wiring or arming");
  bool tune = false;
  check(keyNamed("valve_lead_ms", tune) == kValveLeadMs && !tune, "valve lead found by name");
}
} // namespace

int main() {
  inputs();
  guard();
  led();
  settings();
  withoutRpm();
  pwmDrive();
  currentLimit();
  codes();
  watchdogChecks();
  flowEstimate();
  valveTiming();
  pumpCurrent();
  settingsAdded();
  if (failures) {
    printf("%d failure(s)\n", failures);
    return 1;
  }
  printf("test_pcb: all passed\n");
  return 0;
}
