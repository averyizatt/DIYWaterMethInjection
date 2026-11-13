// Host tests for the crash-loop guard (include/reset_guard.h).
// g++ -std=c++11 -I include test/test_reset_guard/test_main.cpp -o reset_guard_test && ./reset_guard_test
#include <stdio.h>
#include <string.h>

#include "reset_guard.h"

using reset_guard::Cause;
using reset_guard::Record;

namespace {
int failures = 0;

void check(bool ok, const char *what) {
  if (!ok) {
    ++failures;
    printf("FAIL: %s\n", what);
  }
}

Record garbage() {
  Record r;
  memset(&r, 0x5A, sizeof(r));   // What RTC memory holds after the power comes on
  return r;
}

void powerOnStartsClean() {
  Record r = garbage();
  check(reset_guard::boot(r, Cause::PowerOn) == Cause::PowerOn, "power on is power on");
  check(reset_guard::intact(r) && r.crashes == 0 && r.faultRestarts == 0, "power on: nothing counted");
  check(!reset_guard::injectionHeld(r) && reset_guard::mayRestartForFault(r), "power on: free to run");
}

void damagedMemoryIsForgotten() {
  Record r = garbage();
  r.magic = reset_guard::kMagic;   // Right marker, wrong check byte
  r.crashes = 9;
  reset_guard::boot(r, Cause::Crash);
  check(r.crashes == 1, "a damaged record is not believed: this crash is the first");
}

void crashesHoldInjection() {
  Record r = garbage();
  reset_guard::boot(r, Cause::PowerOn);
  reset_guard::boot(r, Cause::Crash);
  check(!reset_guard::injectionHeld(r), "one crash: still injecting");
  reset_guard::boot(r, Cause::Crash);
  check(!reset_guard::injectionHeld(r), "two crashes: still injecting");
  reset_guard::boot(r, Cause::Crash);
  check(reset_guard::injectionHeld(r), "three crashes in one ignition cycle: held off");
  reset_guard::boot(r, Cause::Brownout);
  check(reset_guard::injectionHeld(r), "a brownout does not lift the hold");
  reset_guard::clearCrashes(r);
  check(!reset_guard::injectionHeld(r) && reset_guard::intact(r), "the dash clearing faults lifts it");
  for (int n = 0; n < 3; ++n) reset_guard::boot(r, Cause::Crash);
  reset_guard::boot(r, Cause::PowerOn);
  check(!reset_guard::injectionHeld(r), "so does cycling the ignition");
  for (int n = 0; n < 3; ++n) reset_guard::boot(r, Cause::Crash);
  reset_guard::boot(r, Cause::Requested);
  check(!reset_guard::injectionHeld(r), "and a deliberate restart (firmware update, the dash's restart)");
}

void brownoutsDoNotCount() {
  Record r = garbage();
  reset_guard::boot(r, Cause::PowerOn);
  for (int n = 0; n < 10; ++n) reset_guard::boot(r, Cause::Brownout);
  check(!reset_guard::injectionHeld(r) && r.crashes == 0, "ten brownouts while cranking: nothing held");
  reset_guard::boot(r, Cause::Crash);
  reset_guard::boot(r, Cause::Brownout);
  check(r.crashes == 1, "a brownout keeps the crash count it found");
}

void faultRestartsAreLimited() {
  Record r = garbage();
  reset_guard::boot(r, Cause::PowerOn);
  check(reset_guard::mayRestartForFault(r), "first fault: may restart");
  reset_guard::markFaultRestart(r);
  check(reset_guard::boot(r, Cause::Requested) == Cause::FaultRestart, "a marked software restart is a fault restart");
  check(r.restartMarked == 0, "the mark is used once");
  check(reset_guard::mayRestartForFault(r), "after one: may restart again");
  reset_guard::markFaultRestart(r);
  reset_guard::boot(r, Cause::Requested);
  check(!reset_guard::mayRestartForFault(r), "after two: carry on without restarting");
  check(!reset_guard::injectionHeld(r), "fault restarts do not hold injection off");
  check(reset_guard::boot(r, Cause::Requested) == Cause::Requested, "an unmarked software restart is a requested one");
  check(reset_guard::mayRestartForFault(r), "and it starts the count again");
}

void markSurvivesACrashBeforeTheRestart() {
  // Marked, then a crash before the restart happened: the crash is what is counted.
  Record r = garbage();
  reset_guard::boot(r, Cause::PowerOn);
  reset_guard::markFaultRestart(r);
  check(reset_guard::boot(r, Cause::Crash) == Cause::Crash, "a crash is a crash, marked or not");
  check(r.crashes == 1 && r.faultRestarts == 0 && r.restartMarked == 0, "counted as a crash only");
}

void holdsEndByThemselvesAndGrowLonger() {
  Record r = garbage();
  reset_guard::boot(r, Cause::PowerOn);
  for (int n = 0; n < 3; ++n) reset_guard::boot(r, Cause::Crash);
  check(reset_guard::injectionHeld(r) && reset_guard::holdMs(r) == 30000, "the first hold: 30 s");
  reset_guard::endHold(r);
  check(!reset_guard::injectionHeld(r) && reset_guard::intact(r), "after its rest, injection is tried again");
  reset_guard::boot(r, Cause::Crash);
  check(!reset_guard::injectionHeld(r), "one crash after a hold is a restart again, not a hold");
  for (int n = 0; n < 2; ++n) reset_guard::boot(r, Cause::Crash);
  check(reset_guard::injectionHeld(r) && reset_guard::holdMs(r) == 60000, "three more: held twice as long");
  for (int n = 0; n < 6; ++n) reset_guard::endHold(r);
  check(reset_guard::holdMs(r) == reset_guard::kLongestHoldMs, "never longer than five minutes");
  reset_guard::clearCrashes(r);
  for (int n = 0; n < 3; ++n) reset_guard::boot(r, Cause::Crash);
  check(reset_guard::holdMs(r) == 30000, "after Clear faults a hold starts short again");
}

void armedComesBackAfterResetsTheDriverDidNotAskFor() {
  Record r = garbage();
  reset_guard::boot(r, Cause::PowerOn);
  check(!reset_guard::comesBackArmed(r, Cause::PowerOn), "power on: disarmed");
  reset_guard::noteArmed(r, true);
  check(reset_guard::comesBackArmed(r, reset_guard::boot(r, Cause::Crash)), "after a crash: armed again");
  check(reset_guard::comesBackArmed(r, reset_guard::boot(r, Cause::Brownout)), "after a supply dip: armed again");
  reset_guard::markCanRestart(r);
  check(reset_guard::comesBackArmed(r, reset_guard::boot(r, Cause::Requested)),   // Marked: really a restart over CAN
        "after restarting itself over a silent CAN bus: armed again");
  check(!reset_guard::comesBackArmed(r, reset_guard::boot(r, Cause::Requested)), "a deliberate restart starts disarmed");
  reset_guard::noteArmed(r, true);
  reset_guard::boot(r, Cause::PowerOn);
  check(!reset_guard::comesBackArmed(r, Cause::Crash) && r.armed == 0, "power on forgets it");
  reset_guard::noteArmed(r, false);
  check(!reset_guard::comesBackArmed(r, reset_guard::boot(r, Cause::Crash)), "disarmed stays disarmed");
}

void resetsWhileThePumpRanLengthenTheSoftStart() {
  Record r = garbage();
  reset_guard::boot(r, Cause::PowerOn);
  check(reset_guard::softStartMs(r, 400) == 400, "no trouble: the normal soft start");
  reset_guard::boot(r, Cause::Brownout);
  check(reset_guard::softStartMs(r, 400) == 400, "a supply dip with the pump off (cranking) changes nothing");
  reset_guard::notePump(r, true);
  reset_guard::boot(r, Cause::Brownout);
  check(r.pumpOn == 1, "boot leaves the pump state as it was at the reset, for the firmware to read");
  reset_guard::notePump(r, false);
  check(reset_guard::softStartMs(r, 400) == 800 && r.crashes == 0, "a supply dip with the pump running: twice as gentle");
  reset_guard::notePump(r, true);
  reset_guard::boot(r, Cause::Crash);
  reset_guard::notePump(r, true);
  reset_guard::boot(r, Cause::Crash);
  check(reset_guard::softStartMs(r, 400) == 1600, "at most four times");
  reset_guard::boot(r, Cause::PowerOn);
  check(reset_guard::softStartMs(r, 400) == 400, "a new ignition cycle starts normal again");
}

void canRestartsAreLimitedAndSeparate() {
  Record r = garbage();
  reset_guard::boot(r, Cause::PowerOn);
  for (int n = 0; n < 3; ++n) {
    check(reset_guard::mayRestartForCan(r), "may restart over a silent bus");
    reset_guard::markCanRestart(r);
    check(reset_guard::boot(r, Cause::Requested) == Cause::CanRestart, "a marked restart over CAN is recognised");
  }
  check(!reset_guard::mayRestartForCan(r), "after three: no more restarts over CAN this ignition cycle");
  check(reset_guard::mayRestartForFault(r) && !reset_guard::injectionHeld(r), "they count for nothing else");
}
} // namespace

int main() {
  powerOnStartsClean();
  damagedMemoryIsForgotten();
  crashesHoldInjection();
  brownoutsDoNotCount();
  faultRestartsAreLimited();
  markSurvivesACrashBeforeTheRestart();
  holdsEndByThemselvesAndGrowLonger();
  armedComesBackAfterResetsTheDriverDidNotAskFor();
  resetsWhileThePumpRanLengthenTheSoftStart();
  canRestartsAreLimitedAndSeparate();
  if (failures == 0) printf("reset_guard: all tests passed\n");
  return failures == 0 ? 0 : 1;
}
