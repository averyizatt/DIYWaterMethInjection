#pragma once

// What the last reset says about the firmware, kept in memory that a reset does not
// clear (RTC_NOINIT on the ESP32: it survives a crash, a watchdog and a restart, and
// starts fresh only when the power comes on), so a crash loop is recognised without
// writing to flash. Pure logic with no Arduino calls, tested on a PC (test/test_reset_guard).
//
// Counted from ignition on, and cleared by a deliberate restart (a firmware update, the
// dash's restart, the reset button, flashing over USB):
//   - Crashes (an exception, a task or interrupt watchdog, a CPU lock-up). One is a
//     restart and nothing more: the pump is off within the reset, and the board comes
//     back armed if it was. kCrashesToHold of them and injection is held off for a rest
//     (kFirstHoldMs, twice as long each further time, at most kLongestHoldMs), then
//     tried again: a pump that starts and stops with every crash is worse than one that
//     rests and says so, but losing water/meth for the rest of the drive over a passing
//     fault is worse still (owner, November 2026). The dash's Clear faults ends a hold.
//   - Restarts the firmware made itself over a fault it found: the ADC going quiet
//     (kFaultRestartsAllowed, then it carries on without it), or the CAN bus staying
//     silent however often the CAN controller is set up again (kCanRestartsAllowed).
// A brownout (the supply dipping: cranking, or a pump starting) is not the firmware's
// doing: it neither counts as a crash nor clears anything.
//
// Two things the firmware was doing when the reset came are kept too:
//   - armed: the board comes back armed after a crash, a supply dip or a restart over a
//     fault, without waiting for the dash. Power on and a deliberate restart start disarmed.
//   - pumpOn: a reset while the pump was running is counted (pumpResets), and the
//     firmware then starts the pump more gently for the rest of the ignition cycle.
#include <stdint.h>

namespace reset_guard {

enum class Cause : uint8_t {
  PowerOn,       // Power applied: memory is fresh
  Requested,     // A restart asked for, by software or a person
  Crash,         // Exception, task or interrupt watchdog, CPU lock-up
  FaultRestart,  // This firmware restarted itself over the ADC (set from a Requested restart)
  Brownout,      // Supply dip or glitch
  Other,
  CanRestart,    // This firmware restarted itself: the CAN bus stayed silent
};

constexpr uint8_t kCrashesToHold = 3;
constexpr uint32_t kFirstHoldMs = 30000;
constexpr uint32_t kLongestHoldMs = 300000;
constexpr uint8_t kFaultRestartsAllowed = 2;
constexpr uint8_t kCanRestartsAllowed = 3;
constexpr uint8_t kMostSoftStartDoublings = 2;
constexpr uint32_t kMagic = 0x4D455449UL;   // "METI": the second layout (the first was "METH")

constexpr uint8_t kMarkNone = 0;
constexpr uint8_t kMarkAdc = 1;
constexpr uint8_t kMarkCan = 2;

struct Record {
  uint32_t magic;
  uint8_t crashes;         // Since ignition on, or since the last hold ended
  uint8_t faultRestarts;   // Over the ADC
  uint8_t restartMarked;   // kMark*: set just before the firmware restarts itself over a fault
  uint8_t holds;           // Crash holds served this ignition cycle
  uint8_t canRestarts;     // Over a silent CAN bus
  uint8_t pumpResets;      // Resets that came while the pump was running
  uint8_t armed;           // Armed at the moment of the reset
  uint8_t pumpOn;          // Pump running at the moment of the reset
  uint8_t check;
};

inline uint8_t checkOf(const Record &r) {
  return static_cast<uint8_t>(0xA5U ^ r.crashes ^ static_cast<uint8_t>(r.faultRestarts << 1) ^
                              static_cast<uint8_t>(r.restartMarked << 2) ^ static_cast<uint8_t>(r.holds << 3) ^
                              static_cast<uint8_t>(r.canRestarts * 7U) ^ static_cast<uint8_t>(r.pumpResets * 13U) ^
                              static_cast<uint8_t>(r.armed ? 0x5AU : 0U) ^ static_cast<uint8_t>(r.pumpOn ? 0x3CU : 0U) ^
                              static_cast<uint8_t>(r.magic) ^ static_cast<uint8_t>(r.magic >> 24));
}

inline bool intact(const Record &r) { return r.magic == kMagic && r.check == checkOf(r); }

inline void seal(Record &r) {
  r.magic = kMagic;
  r.check = checkOf(r);
}

inline void forgetAll(Record &r) {
  r.crashes = r.faultRestarts = r.restartMarked = r.holds = r.canRestarts = r.pumpResets = 0;
  r.armed = r.pumpOn = 0;
}

// At start-up, with the reset as the chip reports it (a software restart as Requested):
// updates the counts and returns what the reset really was. Afterwards r.armed and
// r.pumpOn still say what was going on at the reset (both 0 after power on or a
// deliberate restart); the firmware reads them, then records the pump as off.
inline Cause boot(Record &r, Cause reported) {
  if (!intact(r)) forgetAll(r);   // Fresh or damaged memory
  Cause cause = reported;
  if (cause == Cause::Requested && r.restartMarked == kMarkAdc) cause = Cause::FaultRestart;
  if (cause == Cause::Requested && r.restartMarked == kMarkCan) cause = Cause::CanRestart;
  r.restartMarked = kMarkNone;
  switch (cause) {
  case Cause::PowerOn:
  case Cause::Requested:
    forgetAll(r);
    break;
  case Cause::Crash:
    if (r.crashes < 255) ++r.crashes;
    if (r.pumpOn && r.pumpResets < 255) ++r.pumpResets;
    break;
  case Cause::Brownout:
    if (r.pumpOn && r.pumpResets < 255) ++r.pumpResets;
    break;
  case Cause::FaultRestart:
    if (r.faultRestarts < 255) ++r.faultRestarts;
    break;
  case Cause::CanRestart:
    if (r.canRestarts < 255) ++r.canRestarts;
    break;
  default:
    break;
  }
  seal(r);
  return cause;
}

// Whether the board should come back armed: it was armed, and the reset was one the
// driver did not ask for. Power on and a deliberate restart forget it (above).
inline bool comesBackArmed(const Record &r, Cause cause) {
  return r.armed && (cause == Cause::Crash || cause == Cause::Brownout || cause == Cause::FaultRestart ||
                     cause == Cause::CanRestart);
}

inline bool injectionHeld(const Record &r) { return r.crashes >= kCrashesToHold; }

// How long this hold lasts, from the start after the crash that began it.
inline uint32_t holdMs(const Record &r) {
  uint32_t ms = kFirstHoldMs;
  for (uint8_t n = 0; n < r.holds && ms < kLongestHoldMs; ++n) ms *= 2;
  return ms > kLongestHoldMs ? kLongestHoldMs : ms;
}

// The hold has run its time: injection is tried again, and the next hold is longer.
inline void endHold(Record &r) {
  r.crashes = 0;
  if (r.holds < 255) ++r.holds;
  seal(r);
}

inline bool mayRestartForFault(const Record &r) { return r.faultRestarts < kFaultRestartsAllowed; }
inline bool mayRestartForCan(const Record &r) { return r.canRestarts < kCanRestartsAllowed; }

// Just before the firmware restarts itself over a fault.
inline void markFaultRestart(Record &r) {
  r.restartMarked = kMarkAdc;
  seal(r);
}
inline void markCanRestart(Record &r) {
  r.restartMarked = kMarkCan;
  seal(r);
}

// The dash cleared faults: the crashes are forgiven, and a later hold starts short again.
inline void clearCrashes(Record &r) {
  r.crashes = 0;
  r.holds = 0;
  seal(r);
}

// Kept current while running, so they are there after a reset. Written only on a change.
inline void noteArmed(Record &r, bool armed) {
  if ((r.armed != 0) == armed) return;
  r.armed = armed ? 1 : 0;
  seal(r);
}
inline void notePump(Record &r, bool on) {
  if ((r.pumpOn != 0) == on) return;
  r.pumpOn = on ? 1 : 0;
  seal(r);
}

// The pump's soft start: twice as long after each reset that came while it ran, up to
// four times. A pump starting draws its largest current and dips the supply most.
inline uint32_t softStartMs(const Record &r, uint32_t baseMs) {
  const uint8_t doublings = r.pumpResets < kMostSoftStartDoublings ? r.pumpResets : kMostSoftStartDoublings;
  return baseMs << doublings;
}

inline const char *name(Cause c) {
  switch (c) {
  case Cause::PowerOn: return "power on";
  case Cause::Requested: return "restart asked for";
  case Cause::Crash: return "CRASH (exception or watchdog)";
  case Cause::FaultRestart: return "restart over a fault this firmware found (the ADC)";
  case Cause::CanRestart: return "restart over a silent CAN bus";
  case Cause::Brownout: return "supply dip (brownout)";
  default: return "other";
  }
}

} // namespace reset_guard
