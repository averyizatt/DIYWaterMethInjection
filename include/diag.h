#pragma once

// Diagnostic trouble codes for the PCB builds, kept the way a car's ECU keeps them: each
// code counts its occurrences and keeps the uptime and power-up of its first and last, a
// freeze frame of the readings at the last one, and whether it is active now. The store is
// written to flash (with a CRC) away from boost, so codes survive power cycles until they
// are cleared from the web page, the serial port or the dash's Clear faults.
//
// Codes only record. What the board does about each (stop the pump, hold, carry on) is
// decided where the condition is found: pump_guard.h, watchdogs.h and main_pcb.cpp.
// Pure logic, tested on a PC (test/test_pcb).
#include <stdint.h>
#include <string.h>

namespace diag {

enum Code : uint8_t {
  kNone = 0,
  // The pump's own checks (pump_guard::Fault, same numbers).
  kPumpShort = 1,
  kPumpOverCurrent = 2,
  kPumpNotConnected = 3,
  kPumpStuckOn = 4,
  kNoLinePressure = 5,
  kLineOverPressure = 6,
  // Supply and sensors.
  kSupplyLow = 7,
  kSupplyHigh = 8,
  kSensorRail = 9,
  kLineSensor = 10,
  kMapSensor = 11,
  kCurrentSenseOffset = 12,
  // Engine data.
  kEngineDataLost = 13,
  kEngineDataImplausible = 14,
  kCanBusOff = 15,
  // Watchdogs (watchdogs.h).
  kLoopOverrun = 16,
  kPumpKeeper = 17,
  kTaskStalled = 18,
  kPumpRunTooLong = 19,
  kOutputMismatch = 20,
  kSettingsCorrupt = 21,
  kLowMemory = 22,
  // Resets.
  kCrashReset = 23,
  kBrownoutReset = 24,
  // The tank.
  kTankLow = 25,
};
constexpr uint8_t kCodeCount = 26;   // Index 0 unused

enum Severity : uint8_t { kInfo = 0, kWarning = 1, kFault = 2 };

struct Info {
  const char *id;      // Short code, as on the web page and serial port
  const char *text;    // What it means
  const char *action;  // What the board did about it
  uint8_t severity;
};

inline Info infoOf(uint8_t code) {
  switch (code) {
  case kPumpShort: return {"P01", "Pump short circuit (over 25 A)", "Pump off at once, latched", kFault};
  case kPumpOverCurrent: return {"P02", "Pump over its current limit", "Pump off, latched", kFault};
  case kPumpNotConnected: return {"P03", "Pump not connected (no current while driven)", "Pump off, latched", kFault};
  case kPumpStuckOn: return {"P04", "Pump switch stuck on (current with the pump off)", "Valve held shut, latched", kFault};
  case kNoLinePressure: return {"P05", "No line pressure at full drive", "Pump off, latched", kFault};
  case kLineOverPressure: return {"P06", "Line pressure too high (blocked nozzle or filter)", "Pump off, latched", kFault};
  case kSupplyLow: return {"S07", "12 V supply low", "Injection held until it recovers", kWarning};
  case kSupplyHigh: return {"S08", "12 V supply too high (charging fault)", "Injection held until it recovers", kWarning};
  case kSensorRail: return {"S09", "Sensor 5 V supply out of range (a shorted sensor wire?)", "Sensor readings ignored", kFault};
  case kLineSensor: return {"S10", "Line pressure transducer open or shorted", "Pressure checks off; flow from the nozzle figure", kWarning};
  case kMapSensor: return {"S11", "MAP sensor open or shorted", "No boost reading: injection held", kFault};
  case kCurrentSenseOffset: return {"S12", "Pump current reads high with the pump off", "Current checks may be wrong", kWarning};
  case kEngineDataLost: return {"E13", "Engine data stopped arriving", "Injection held until it returns", kWarning};
  case kEngineDataImplausible: return {"E14", "Engine data out of range (manifold pressure or engine speed)", "The reading ignored", kWarning};
  case kCanBusOff: return {"E15", "CAN bus-off (wiring, speed or termination)", "CAN restarted", kWarning};
  case kLoopOverrun: return {"W16", "Control pass late (over 100 ms)", "Logged", kInfo};
  case kPumpKeeper: return {"W17", "Control stopped setting the pump", "The keeper switched pump and valve off", kFault};
  case kTaskStalled: return {"W18", "CAN task stopped running", "Injection held; the watchdog restarts the board", kFault};
  case kPumpRunTooLong: return {"W19", "Pump on longer than the longest spray allows", "Pump off, latched", kFault};
  case kOutputMismatch: return {"W20", "Output driver does not match what was asked", "Pump and valve off, latched", kFault};
  case kSettingsCorrupt: return {"W21", "Settings failed their check", "Settings reloaded, disarmed", kFault};
  case kLowMemory: return {"W22", "Memory low", "Wi-Fi page stopped if it gets worse", kWarning};
  case kCrashReset: return {"R23", "Restarted after a crash or watchdog", "Outputs off first; armed again", kWarning};
  case kBrownoutReset: return {"R24", "Restarted after a supply dip (brownout)", "Armed again", kWarning};
  case kTankLow: return {"T25", "Tank low", "Injection stopped (with tank protection on)", kInfo};
  default: return {"-", "", "", kInfo};
  }
}

// The readings when a code was last set.
struct Freeze {
  int16_t boostPsiX10{0};
  uint16_t rpm{0};
  uint16_t ampsX100{0};
  uint16_t linePsiX10{0};
  uint16_t supplyMv{0};
  uint8_t dutyPct{0};
  int8_t airC{0};
};

struct Entry {
  uint16_t count{0};      // Occurrences since cleared (stops at 65535)
  uint16_t firstPowerUp{0};
  uint16_t lastPowerUp{0};
  uint32_t firstS{0};     // Uptime, seconds, in that power-up
  uint32_t lastS{0};
  Freeze freeze{};
  uint8_t active{0};      // Present now
};

// CRC-32 (IEEE, reflected), for the stored copy and the watchdogs' settings check.
inline uint32_t crc32(const void *data, uint32_t len, uint32_t crc = 0) {
  const uint8_t *p = static_cast<const uint8_t *>(data);
  crc = ~crc;
  while (len--) {
    crc ^= *p++;
    for (uint8_t k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xEDB88320UL & (0U - (crc & 1U)));
  }
  return ~crc;
}

class Store {
public:
  static constexpr uint32_t kMagic = 0x44544331UL;   // "DTC1"

  // A new power-up: nothing is active until found again.
  void begin(uint16_t powerUp) {
    powerUp_ = powerUp;
    for (uint8_t c = 0; c < kCodeCount; ++c) entries_[c].active = 0;
  }

  // The condition behind a code, every pass (or whenever it is checked). An occurrence is
  // counted when it becomes present; the freeze frame is taken then.
  void report(uint8_t code, bool present, uint32_t uptimeS, const Freeze &f) {
    if (code == kNone || code >= kCodeCount) return;
    Entry &e = entries_[code];
    if (present && !e.active) {
      if (e.count == 0) { e.firstPowerUp = powerUp_; e.firstS = uptimeS; }
      if (e.count < 0xFFFF) ++e.count;
      e.lastPowerUp = powerUp_;
      e.lastS = uptimeS;
      e.freeze = f;
      dirty_ = true;
      ++events_;
    }
    e.active = present ? 1 : 0;
  }
  // A one-off event (a reset, a keeper trip): counted every time.
  void event(uint8_t code, uint32_t uptimeS, const Freeze &f) {
    if (code == kNone || code >= kCodeCount) return;
    entries_[code].active = 0;
    report(code, true, uptimeS, f);
    entries_[code].active = 0;
  }

  const Entry &entry(uint8_t code) const { return entries_[code < kCodeCount ? code : 0]; }
  bool any() const {
    for (uint8_t c = 1; c < kCodeCount; ++c) if (entries_[c].count) return true;
    return false;
  }
  uint8_t activeOfSeverity(uint8_t severity) const {
    uint8_t n = 0;
    for (uint8_t c = 1; c < kCodeCount; ++c) if (entries_[c].active && infoOf(c).severity == severity) ++n;
    return n;
  }
  // Events since start-up (for the page to notice a new one).
  uint32_t events() const { return events_; }

  // Cleared: counts and freeze frames gone; a condition still present is counted again.
  void clear() {
    for (uint8_t c = 0; c < kCodeCount; ++c) entries_[c] = Entry{};
    dirty_ = true;
  }

  // Changed since last written to flash.
  bool dirty() const { return dirty_; }
  void written() { dirty_ = false; }

  // The stored form: the entries without their active flags, and a CRC over them.
  struct Image {
    uint32_t magic;
    Entry entries[kCodeCount];
    uint32_t crc;
  };
  void save(Image &img) const {
    memset(static_cast<void *>(&img), 0, sizeof(img));
    img.magic = kMagic;
    for (uint8_t c = 0; c < kCodeCount; ++c) { img.entries[c] = entries_[c]; img.entries[c].active = 0; }
    img.crc = crc32(&img, sizeof(img) - sizeof(img.crc));
  }
  // False (and nothing taken) for a damaged or foreign image.
  bool load(const Image &img) {
    if (img.magic != kMagic || img.crc != crc32(&img, sizeof(img) - sizeof(img.crc))) return false;
    for (uint8_t c = 0; c < kCodeCount; ++c) { entries_[c] = img.entries[c]; entries_[c].active = 0; }
    dirty_ = false;
    return true;
  }

private:
  Entry entries_[kCodeCount]{};
  uint16_t powerUp_{0};
  uint32_t events_{0};
  bool dirty_{false};
};

} // namespace diag
