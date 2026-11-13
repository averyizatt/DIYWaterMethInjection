#pragma once

// What the last reset says about the firmware, kept in memory that a reset does not
// clear (RTC_NOINIT on the ESP32: it survives a crash, a watchdog and a restart, and
// starts fresh only when the power comes on), so a crash loop is recognised without
// writing to flash. Pure logic with no Arduino calls, tested on a PC (test/test_reset_guard).
//
// Counted from ignition on, and cleared by a deliberate restart (a firmware update, the
// dash's restart, the reset button, flashing over USB):
//   - Crashes (an exception, a task or interrupt watchdog, a CPU lock-up). One is a
//     restart and nothing more: the pump is off within the reset, and the dash arms the
//     controller again. kCrashesToHold of them and injection is held off until the dash
//     clears faults or the ignition is cycled: a pump that starts and stops with every
//     crash is worse than one that stays off and says so.
//   - Restarts the firmware made itself over a fault it found (the ADC going quiet).
//     After kFaultRestartsAllowed, it carries on without that part instead of restarting
//     over and over.
// A brownout (the supply dipping, while cranking for one) is not the firmware's doing:
// it neither counts nor clears anything.
#include <stdint.h>

namespace reset_guard {

enum class Cause : uint8_t {
  PowerOn,       // Power applied: memory is fresh
  Requested,     // A restart asked for, by software or a person
  Crash,         // Exception, task or interrupt watchdog, CPU lock-up
  FaultRestart,  // This firmware restarted itself over a fault (set from a Requested restart)
  Brownout,      // Supply dip or glitch
  Other,
};

constexpr uint8_t kCrashesToHold = 3;
constexpr uint8_t kFaultRestartsAllowed = 2;
constexpr uint32_t kMagic = 0x4D455448UL;   // "METH"

struct Record {
  uint32_t magic;
  uint8_t crashes;
  uint8_t faultRestarts;
  uint8_t restartMarked;   // Set just before the firmware restarts itself over a fault
  uint8_t check;
};

inline uint8_t checkOf(const Record &r) {
  return static_cast<uint8_t>(0xA5U ^ r.crashes ^ static_cast<uint8_t>(r.faultRestarts << 1) ^
                              static_cast<uint8_t>(r.restartMarked << 2) ^ static_cast<uint8_t>(r.magic) ^
                              static_cast<uint8_t>(r.magic >> 24));
}

inline bool intact(const Record &r) { return r.magic == kMagic && r.check == checkOf(r); }

inline void seal(Record &r) {
  r.magic = kMagic;
  r.check = checkOf(r);
}

// At start-up, with the reset as the chip reports it (a software restart as Requested):
// updates the counts and returns what the reset really was.
inline Cause boot(Record &r, Cause reported) {
  if (!intact(r)) {
    r.crashes = r.faultRestarts = r.restartMarked = 0;   // Fresh or damaged memory
  }
  Cause cause = reported;
  if (cause == Cause::Requested && r.restartMarked) cause = Cause::FaultRestart;
  r.restartMarked = 0;
  switch (cause) {
  case Cause::PowerOn:
  case Cause::Requested:
    r.crashes = r.faultRestarts = 0;
    break;
  case Cause::Crash:
    if (r.crashes < 255) ++r.crashes;
    break;
  case Cause::FaultRestart:
    if (r.faultRestarts < 255) ++r.faultRestarts;
    break;
  default:
    break;
  }
  seal(r);
  return cause;
}

inline bool injectionHeld(const Record &r) { return r.crashes >= kCrashesToHold; }
inline bool mayRestartForFault(const Record &r) { return r.faultRestarts < kFaultRestartsAllowed; }

// Just before the firmware restarts itself over a fault.
inline void markFaultRestart(Record &r) {
  r.restartMarked = 1;
  seal(r);
}

// The dash cleared faults: the crashes are forgiven.
inline void clearCrashes(Record &r) {
  r.crashes = 0;
  seal(r);
}

inline const char *name(Cause c) {
  switch (c) {
  case Cause::PowerOn: return "power on";
  case Cause::Requested: return "restart asked for";
  case Cause::Crash: return "CRASH (exception or watchdog)";
  case Cause::FaultRestart: return "restart over a fault this firmware found";
  case Cause::Brownout: return "supply dip (brownout)";
  default: return "other";
  }
}

} // namespace reset_guard
