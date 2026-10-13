// Host tests for the knock amplifier's automatic gain and its potentiometer driver:
// g++ -std=c++11 -I include test/test_knock_agc/test_main.cpp
#include <initializer_list>
#include <math.h>
#include <stdio.h>

#include "knock_agc.h"
#include "x9c.h"

using namespace knock_agc;

namespace {
int failures = 0;
uint32_t seed = 4242;

void check(bool ok, const char *what) {
  if (!ok) {
    ++failures;
    printf("FAIL: %s\n", what);
  }
}

float noise() {
  seed = seed * 1664525u + 1013904223u;
  return ((seed >> 8) & 0xFFFF) / 65535.0f * 2.0f - 1.0f;
}

// ---------------------------------------------------------------------------
// The X9C104 as its datasheet describes it.
// ---------------------------------------------------------------------------
struct Chip {
  int position = 57;        // Wherever it was left: unknown to the driver
  int top = 99;             // The datasheet's part; some modules have fewer steps
  bool selected = false, incHigh = true, upward = false;
  int saves = 0, steps = 0;
  void select(bool on) {
    if (selected && !on && incHigh) ++saves;   // Released with INC high: writes its memory
    selected = on;
  }
  void inc(bool high) {
    if (selected && incHigh && !high) {        // Falling edge while selected: one step
      ++steps;
      position += upward ? 1 : -1;
      if (position < 0) position = 0;          // It stops at the ends; it does not wrap
      if (position > top) position = top;
    }
    incHigh = high;
  }
  void up(bool u) { upward = u; }
  void wait() {}
};

void thePotentiometerIsWalkedNotWritten() {
  Chip chip;
  x9c::Pot pot;
  pot.home(chip);
  check(chip.position == 0 && pot.step() == 0, "homing reaches the bottom from an unknown position");
  pot.moveTo(chip, 40);
  check(chip.position == 40, "up to a position");
  pot.moveTo(chip, 12);
  check(chip.position == 12 && pot.step() == 12, "and down again");
  const int before = chip.steps;
  pot.moveTo(chip, 12);
  check(chip.steps == before, "no pulses when it is already there");
  pot.moveTo(chip, 13);
  check(chip.position == 13, "a single step is exactly one step");
  pot.moveTo(chip, 200);
  check(chip.position == 99 && pot.step() == 99, "never asked beyond the top");
  // A pulse lost to noise leaves the driver's count wrong; a resync puts it right.
  chip.position = 70;
  pot.moveTo(chip, 60);
  check(chip.position != 60, "a miscounted wiper is in the wrong place");
  pot.resync(chip, 60);
  check(chip.position == 60, "resync finds the bottom again and returns to the position");
  check(chip.saves == 0, "the chip's memory is never written (it wears out)");
  check(!chip.selected && chip.incHigh, "the chip is left released and idle");
}

// ---------------------------------------------------------------------------
// The engine and amplifier: a signal of `sensorPeak` counts at gain 1, around the bias.
// ---------------------------------------------------------------------------
const float kBias = 545.0f;   // 1.65 V on the ESP32's ADC

Sample heard(float sensorPeak, float gain, uint16_t rpm, bool knocking = false, bool rpmValid = true) {
  const float swing = sensorPeak * gain * (1.0f + 0.15f * noise());
  float low = kBias - swing, high = kBias + swing;
  Sample s;
  s.low = static_cast<uint16_t>(low < 0.0f ? 0.0f : low);
  s.high = static_cast<uint16_t>(high > 1023.0f ? 1023.0f : high);
  s.rpm = rpm;
  s.rpmValid = rpmValid;
  s.knocking = knocking;
  return s;
}

struct Bench {
  Circuit circuit;
  Controller agc;
  int changes = 0;
  Bench() { agc.begin(circuit, 0); }
  // `passes` control passes of 20 ms with a working amplifier. Returns whether the gain changed.
  bool run(int passes, float sensorPeak, uint16_t rpm, bool knocking = false, bool rpmValid = true) {
    bool any = false;
    for (int n = 0; n < passes; ++n) {
      if (agc.update(heard(sensorPeak, agc.gain(), rpm, knocking, rpmValid))) {
        ++changes;
        any = true;
      }
    }
    return any;
  }
  float typical(float sensorPeak) const { return sensorPeak * agc.gain(); }
};

void theGainModelIsAStraightLine() {
  Circuit c;
  check(gainAt(c, 0) == 1.0f && gainAt(c, 99) == 11.0f, "gain 1 at the bottom, 11 at the top");
  check(fabsf(gainAt(c, 50) - 6.05f) < 0.01f, "and in between in a straight line");
  bool roundTrip = true;
  for (int step = 0; step <= 99; ++step) roundTrip = roundTrip && stepFor(c, gainAt(c, static_cast<uint8_t>(step))) == step;
  check(roundTrip, "every position's gain maps back to that position");
  check(stepFor(c, 0.2f) == 0 && stepFor(c, 50.0f) == 99, "gains outside the range stop at the ends");
}

void quietSignalsAreBroughtUpAndLoudOnesDown() {
  for (float sensorPeak : {12.0f, 25.0f, 60.0f, 120.0f}) {
    Bench bench;
    bench.run(750, sensorPeak, 3000);   // 15 seconds from the lowest gain
    const float typical = bench.typical(sensorPeak);
    check(typical >= kLowPeak * 0.9f && typical <= kHighPeak * 1.1f, "the normal signal ends up inside the wanted band");
    const int settled = bench.changes;
    bench.run(3000, sensorPeak, 3000);  // A minute more of the same
    check(bench.changes - settled <= 2, "once there it stays put: no hunting");
    check(!bench.agc.overloaded() && !bench.agc.silent() && !bench.agc.ineffective(), "and nothing is reported wrong");
  }
  // Louder than the lowest gain can bring into the band, but not clipping: it sits at the bottom.
  Bench loud;
  loud.run(500, 380.0f, 5000);
  check(loud.agc.step() == 0 && !loud.agc.overloaded(), "a strong signal is taken at the lowest gain");
  // Quieter than the highest gain can bring up: it sits at the top, and that is not a fault at idle.
  Bench faint;
  faint.run(1500, 3.0f, 900);
  check(faint.agc.step() == 99 && !faint.agc.silent() && !faint.agc.ineffective(), "a faint signal gets all the gain there is");
}

void clippingIsBackedOffAtOnce() {
  Bench bench;
  bench.run(750, 25.0f, 3000);
  const float before = bench.agc.gain();
  // Suddenly twelve times louder: the ADC is hitting its limits.
  check(bench.agc.update(heard(300.0f, before, 3000)), "the first clipped pass changes the gain");
  check(bench.agc.ratioX256() <= 0.75f * 256 && bench.agc.gain() <= before * 0.75f, "down by at least 3 dB in one go");
  check(bench.agc.settling(), "the next pass is marked as not to be judged");
  check(!bench.agc.update(heard(300.0f, bench.agc.gain(), 3000)) && !bench.agc.settling(), "and is discarded");
  bench.run(100, 300.0f, 3000);
  check(heard(300.0f, bench.agc.gain(), 3000).high < kClipHigh, "it keeps backing off until the signal fits");
  check(!bench.agc.overloaded(), "clipping that gain can fix is not a fault");
  // Clipping with no gain left to take off is one.
  Bench hot;
  hot.run(200, 700.0f, 5000);
  check(hot.agc.step() == 0 && hot.agc.overloaded(), "clipping at the lowest gain is reported");
  hot.run(200, 100.0f, 5000);
  check(!hot.agc.overloaded(), "and clears when the signal fits again");
}

void knockDoesNotSteerTheGain() {
  Bench bench;
  bench.run(750, 25.0f, 3000);
  const uint8_t settled = bench.agc.step();
  // Loud passes the detector has called knock (three times normal, not clipping).
  bench.run(300, 75.0f, 3000, true);
  check(bench.agc.step() == settled, "passes flagged as knock are left out of the average");
  // The same level as the engine's new normal does move it.
  bench.run(300, 75.0f, 3000, false);
  check(bench.agc.step() < settled, "a lasting change in normal noise is followed");
}

void eachEngineSpeedGetsItsGainBack() {
  Bench bench;
  bench.run(750, 20.0f, 2500);
  const uint8_t cruise = bench.agc.step();
  bench.run(750, 90.0f, 5500);
  const uint8_t fast = bench.agc.step();
  check(fast < cruise, "more noise at high RPM, less gain");
  // Back to the first speed: straight to its gain, on the first pass.
  check(bench.agc.update(heard(20.0f, bench.agc.gain(), 2500)) && bench.agc.step() == cruise, "a speed it knows gets its gain at once");
  bench.run(1, 20.0f, 2500);   // The discarded pass
  check(bench.agc.update(heard(90.0f, bench.agc.gain(), 5500)) && bench.agc.step() == fast, "in both directions");
  // The boundary between two ranges has to be passed by 150 RPM before the gain changes.
  Bench edge;
  edge.run(750, 20.0f, 2500);
  const uint8_t low = edge.agc.step();
  edge.run(750, 60.0f, 3500);
  const uint8_t high = edge.agc.step();
  check(low != high, "two neighbouring ranges with different gains");
  check(!edge.run(3, 60.0f, 2900) && edge.agc.step() == high, "just under the boundary: still the upper range");
  check(edge.run(3, 20.0f, 2840) && edge.agc.step() == low, "well under it: the lower range");
  check(!edge.run(3, 20.0f, 3100) && edge.agc.step() == low, "just over the boundary: still the lower range");
  check(edge.run(3, 60.0f, 3160) && edge.agc.step() == high, "well over it: the upper range");
  // An RPM reading that drops out does not send the gain to the idle range.
  const int before = edge.changes;
  edge.run(50, 60.0f, 0, false, false);
  check(edge.changes == before && edge.agc.step() == high, "no RPM: the gain stays where it is");
}

void aDeadSensorIsNoticed() {
  Bench bench;
  bench.run(2000, 0.1f, 3000);
  check(bench.agc.step() == 99 && bench.agc.silent(), "nothing at full gain with the engine at speed: sensor not heard");
  bench.run(200, 30.0f, 3000);
  check(!bench.agc.silent(), "cleared as soon as there is a signal");
  Bench stopped;
  stopped.run(2000, 0.1f, 0);
  check(stopped.agc.step() == 99 && !stopped.agc.silent(), "silence with the engine stopped is just silence");
}

// The owner's module: 30 steps from end to end, whatever the datasheet says.
void aPartWithFewerStepsIsDrivenWithinItsRange() {
  Chip chip;
  chip.top = 30;
  chip.position = 17;
  x9c::Pot pot;
  pot.setTopStep(30);
  pot.home(chip);
  pot.moveTo(chip, 30);
  check(chip.position == 30 && pot.step() == 30, "the top is step 30");
  const int before = chip.steps;
  pot.moveTo(chip, 99);
  check(chip.steps == before && pot.step() == 30, "it is never asked beyond its real top");
  pot.moveTo(chip, 10);
  check(chip.position == 10, "so the count is still right on the way back down");
  // What would happen with the datasheet's count on this part: the count runs ahead of the wiper.
  Chip misjudged;
  misjudged.top = 30;
  x9c::Pot wrong;
  wrong.home(misjudged);
  wrong.moveTo(misjudged, 80);
  wrong.moveTo(misjudged, 40);
  check(misjudged.position == 0 && wrong.step() == 40, "driven as a 99-step part it would lose its place");
  check(chip.saves == 0 && misjudged.saves == 0, "nothing saved either way");

  // The gain control with 30 coarse steps still settles and stays put.
  for (float sensorPeak : {12.0f, 25.0f, 60.0f, 120.0f}) {
    Bench bench;
    bench.circuit.topStep = 30;
    bench.agc.begin(bench.circuit, 0);
    bench.run(750, sensorPeak, 3000);
    const float typical = bench.typical(sensorPeak);
    check(typical >= kLowPeak * 0.9f && typical <= kHighPeak * 1.1f, "30 steps: the signal still ends up inside the band");
    check(bench.agc.step() <= 30, "30 steps: never beyond the top");
    const int settled = bench.changes;
    bench.run(3000, sensorPeak, 3000);
    check(bench.changes - settled <= 2, "30 steps: no hunting");
  }
  Circuit coarse;
  coarse.topStep = 30;
  check(gainAt(coarse, 30) == 11.0f && fabsf(gainAt(coarse, 15) - 6.0f) < 0.01f, "30 steps span the same gain range");
}

void aPotentiometerThatDoesNothingIsNoticed() {
  // A fixed-gain amplifier: the signal is the same whatever is commanded.
  Circuit circuit;
  Controller agc;
  agc.begin(circuit, 0);
  for (int n = 0; n < 1500 && !agc.ineffective(); ++n) agc.update(heard(30.0f, 1.0f, 3000));
  check(agc.ineffective(), "gain asked up by 6 dB with no change in the signal: the potentiometer is not working");
  const uint8_t where = agc.step();
  for (int n = 0; n < 500; ++n) agc.update(heard(30.0f, 1.0f, 3000));
  check(agc.step() == where, "after that the gain is left alone");
  check(gainAt(circuit, where) < 4.0f, "and it was noticed early, not after running to the end");
  // The same the other way: too loud, and commanding less changes nothing.
  agc.begin(circuit, 80);
  for (int n = 0; n < 1500 && !agc.ineffective(); ++n) agc.update(heard(300.0f, 1.0f, 3000));
  check(agc.ineffective(), "gain asked down with no change is noticed too");
  // Engine off: the noise floor does not grow with gain either, and that proves nothing.
  agc.begin(circuit, 0);
  for (int n = 0; n < 3000; ++n) {
    Sample s;
    s.low = 544;
    s.high = 547;
    agc.update(s);
  }
  check(!agc.ineffective() && agc.step() == 99, "a signal too small to judge is not taken as a broken potentiometer");
}

} // namespace

int main() {
  thePotentiometerIsWalkedNotWritten();
  theGainModelIsAStraightLine();
  quietSignalsAreBroughtUpAndLoudOnesDown();
  clippingIsBackedOffAtOnce();
  knockDoesNotSteerTheGain();
  eachEngineSpeedGetsItsGainBack();
  aDeadSensorIsNoticed();
  aPotentiometerThatDoesNothingIsNoticed();
  aPartWithFewerStepsIsDrivenWithinItsRange();
  if (failures == 0) printf("knock gain tests passed\n");
  return failures == 0 ? 0 : 1;
}
