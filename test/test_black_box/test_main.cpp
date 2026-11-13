// Host tests for the black box (include/black_box.h).
// g++ -std=c++11 -I include test/test_black_box/test_main.cpp -o black_box_test && ./black_box_test
#include <stdio.h>
#include <string.h>

#include "black_box.h"

using namespace black_box;

namespace {
int failures = 0;

void check(bool ok, const char *what) {
  if (!ok) {
    ++failures;
    printf("FAIL: %s\n", what);
  }
}

Pass running(float duty, float mapKpa = 130.0f, uint16_t rpm = 4000) {
  Pass p;
  p.dutyPct = duty;
  p.ecuFresh = true;
  p.mapKpa = mapKpa;
  p.rpmValid = true;
  p.rpm = rpm;
  p.armed = true;
  p.canOnline = true;
  p.canRxAgeMs = 3;
  p.loopGapMs = 20;
  p.preValid = true;
  p.preC = 35.0f;
  p.freeHeapBytes = 250000;
  return p;
}

uint8_t out[kSlots][kSampleBytes];

// RTC memory is never cleared: a box starts as noise, as on the car.
void noise(Box &box) { memset(&box, 0xA7, sizeof(box)); }

void powerOnNoiseIsNoRecord() {
  Box box;
  noise(box);
  check(take(box, out) == 0, "after power on the memory holds noise: no record");
}

void aWindowFoldsItsPasses() {
  Box box;
  noise(box);
  Recorder r;
  r.start(box, 1000);
  check(take(box, out) == 0, "a new record has nothing in it yet");
  Pass p = running(0.0f);
  p.hold = 2;   // Below boost
  r.record(box, 1000, p);
  p = running(30.0f);
  r.record(box, 1020, p);
  p = running(60.0f);
  p.loopGapMs = 140;
  p.canRxAgeMs = 400;
  p.softStarting = true;
  r.record(box, 1040, p);
  p = running(45.0f);
  p.freeHeapBytes = 100000;
  r.record(box, 1060, p);
  check(take(box, out) == 1, "the window being filled is in the record");
  const uint8_t *s = out[0];
  check(s[kDutyEnd] == 90 && s[kDutyMax] == 120, "duty at the end of the window and its highest, in 0.5% steps");
  check(s[kMapKpa] == 130 && s[kRpm32] == 125, "manifold pressure and RPM at the end");
  check((s[kEvents] & kPumpStarted) && !(s[kEvents] & kPumpStopped), "the pump started in this window");
  check((s[kEvents] & kEcuFresh) && (s[kEvents] & kCanOnline), "ECU data fresh and CAN up at the end");
  check(s[kLoopGapMs] == 140 && s[kRxAge10] == 40, "the longest control gap and CAN silence in the window");
  check(s[kPreC40] == 75 && s[kHeap2k] == 48, "air before the nozzle; the least free memory");
  check((s[kState] & 0x0F) == 0 && (s[kState] & kArmed) && !(s[kState] & kSoftStarting), "state as at the end of the window");
}

void tenSecondsOldestFirstAndTheRingWraps() {
  Box box;
  noise(box);
  Recorder r;
  r.start(box, 0);
  uint32_t now = 0;
  for (int pass = 0; pass < 750; ++pass, now += 20) {   // 15 s of passes
    Pass p = running(static_cast<float>((pass / 5) % 100), 50.0f + pass / 5);   // 50 .. 199 kPa
    r.record(box, now, p);
  }
  const uint8_t n = take(box, out);
  check(n == kSlots, "ten seconds kept: a full ring");
  // The last window to be filled is pass 749 / 5 = 149; the oldest kept is 149 - 99 = 50.
  check(out[n - 1][kMapKpa] == 199 && out[0][kMapKpa] == 100, "oldest first, newest last");
  bool ordered = true;
  for (uint8_t i = 1; i < n; ++i) ordered = ordered && static_cast<uint8_t>(out[i][kMapKpa] - out[i - 1][kMapKpa]) == 1;
  check(ordered, "one window after another, none missing");
}

void aResetHalfWayThroughAWriteLosesOnlyThatSample() {
  Box box;
  noise(box);
  Recorder r;
  r.start(box, 0);
  for (uint32_t now = 0; now < 1000; now += 20) r.record(box, now, running(25.0f));
  const uint8_t before = take(box, out);
  box.slot[box.next].b[kDutyEnd] ^= 0x55;   // The reset came in the middle of writing it
  check(take(box, out) == before - 1, "a half-written last sample is left out, the rest kept");
  box.check ^= 1;
  check(take(box, out) == 0, "a damaged header: nothing believed");
}

void aStallKeepsToTheGrid() {
  Box box;
  noise(box);
  Recorder r;
  r.start(box, 0);
  for (uint32_t now = 0; now < 500; now += 20) r.record(box, now, running(25.0f));
  Pass late = running(25.0f);
  late.loopGapMs = 900;
  r.record(box, 1400, late);   // The main loop held up for 900 ms
  const uint8_t n = take(box, out);
  check(n == 6 && out[n - 1][kLoopGapMs] == 255, "a stall is one window, its gap shown (at most 255 ms)");
  r.record(box, 1500, running(25.0f));
  check(take(box, out) == 7, "then windows carry on from the grid");
}

void pumpStopsAndCanTroubleAreEvents() {
  Box box;
  noise(box);
  Recorder r;
  r.start(box, 0);
  r.record(box, 0, running(40.0f));
  Pass trouble = running(0.0f);
  trouble.canSetUpAgain = true;
  trouble.canSendFailed = true;
  trouble.keeperTripped = true;
  trouble.canOnline = false;
  trouble.ecuFresh = false;
  r.record(box, 120, trouble);
  r.record(box, 140, running(0.0f));
  take(box, out);
  const uint8_t *s = out[1];
  check((s[kEvents] & kPumpStopped) && (s[kEvents] & kCanSetUpAgain) && (s[kEvents] & kCanSendFailed) && (s[kEvents] & kKeeperTripped),
        "the pump stopping, CAN set up again, a failed send and the keeper are kept for the window");
  check((s[kEvents] & kEcuFresh) && (s[kEvents] & kCanOnline), "states follow the end of the window");
  check(s[kMapKpa] == 130, "and so do the readings");
  forget(box);
  check(take(box, out) == 0, "forgotten once taken");
}

void sumOfTheBytes() {
  uint8_t samples[2][kSampleBytes] = {{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}, {255, 255, 0, 0, 0, 0, 0, 0, 0, 1}};
  check(sumOf(samples, 2) == 55 + 511, "the sum the dash checks the transfer with");
}
} // namespace

int main() {
  powerOnNoiseIsNoRecord();
  aWindowFoldsItsPasses();
  tenSecondsOldestFirstAndTheRingWraps();
  aResetHalfWayThroughAWriteLosesOnlyThatSample();
  aStallKeepsToTheGrid();
  pumpStopsAndCanTroubleAreEvents();
  sumOfTheBytes();
  if (failures == 0) printf("black box tests passed\n");
  return failures == 0 ? 0 : 1;
}
