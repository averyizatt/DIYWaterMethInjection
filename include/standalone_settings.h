#pragma once

// The stand-alone PCB build's own settings (docs/PCB.md), beside the pulse tuning it shares
// with every build (meth_tune.h): where engine data comes from, the pressure input, the
// tank switch, Wi-Fi and arming. One table gives each setting its name, label, unit, limits
// and default, for the web page, the serial commands and the store. Pure logic, tested on
// a PC (test/test_pcb).
#include <stdint.h>
#include <string.h>

#include "engine_source.h"
#include "meth_tune.h"

namespace standalone {

enum Source : uint8_t { kSourceNone = 0, kSourceEcu = 1, kSourceObd11 = 2, kSourceObd29 = 3 };

enum Key : uint8_t {
  kSource = 1,         // Engine data on CAN: none, an ECU's broadcast, OBD-II (11-bit or 29-bit IDs)
  kProfile = 2,        // Which ECU (engine::kProfiles)
  kBaseId = 3,         // The ECU's broadcast base ID, for ECUs where it can be moved; 0 = its default
  kBitrateK = 4,       // CAN speed in kbit/s; 0 = the ECU's own (500 for OBD-II)
  kListenOnly = 5,     // 1: never acknowledge frames (a bus with other nodes on it); OBD-II always talks
  kMapFrom = 6,        // 0: manifold pressure from CAN; 1: a MAP sensor on the pressure input
  kMapSensor = 7,      // A preset (engine::kMapSensors), or the last: your own two points
  kMapAt05X10 = 8,     // MAP sensor: kPa at 0.5 V (x10)
  kMapAt45X10 = 9,     // ...and at 4.5 V
  kLineFullPsi = 10,   // Line pressure sensor's full scale (0.5-4.5 V); 0 = none fitted
  kLineMinPsi = 11,    // Spraying, at least this after 1.5 s, or the pump stops (0 = off)
  kLineMaxPsi = 12,    // More than this for 0.5 s stops the pump: a blocked nozzle (0 = off)
  kBaroX10 = 13,       // Barometric pressure when nothing measures it (x10 kPa)
  kEngineDl = 14,      // Engine size, tenths of a litre: the dose limit's fuel estimate
  kFloatClosedWhenLow = 15,  // 1: the tank switch closes when the tank is LOW
  kTankProtection = 16,      // 1: a low tank stops injection
  kWifiAlways = 17,    // 0: Wi-Fi goes off 15 minutes after power-up unless someone is connected
  kArmed = 18,         // Armed: kept through power cycles
  kDrive = 19,         // 0: full-on pulses (meth_tune::PulseGovernor); 1: PWM duty rising with boost (pwm_drive.h)
  kNozzleRatedPsi = 20,  // Line pressure the nozzle flow figure was measured at (flow_estimate.h); 0 = not known
  kTankDl = 21,        // Tank size, tenths of a litre: the fluid count; 0 = no count
  kValveLeadMs = 22,   // After a stop, the valve opens this long after the pump starts (pressure first); 0 = with the pump
  kPumpAmpsMaxX10 = 23,  // Pump current limit (x10 A), at most the 10 A rating
};
constexpr uint8_t kCount = 23;
// Settings saved before keys 20-23 existed: the first 19 are taken, the rest set to defaults.
constexpr uint8_t kCountV1 = 19;

struct Info {
  const char *name;    // For the web page and serial commands
  const char *label;
  const char *unit;
  uint16_t min, max, def;
  uint8_t divisor;     // Shown as value / divisor (10: one decimal)
};

inline Info infoOf(uint8_t key) {
  switch (key) {
  case kSource: return {"source", "Engine data on CAN (0 none, 1 ECU broadcast, 2 OBD-II, 3 OBD-II 29-bit)", "", 0, 3, kSourceObd11, 1};
  case kProfile: return {"ecu", "ECU", "", 0, static_cast<uint16_t>(engine::kProfileCount - 1), 0, 1};
  case kBaseId: return {"base_id", "ECU broadcast base ID (0 = its default)", "", 0, 0x7FF, 0, 1};
  case kBitrateK: return {"bitrate", "CAN speed (0 = the ECU's own)", "kbit/s", 0, 1000, 0, 1};
  case kListenOnly: return {"listen_only", "Only listen to the ECU, never acknowledge", "", 0, 1, 1, 1};
  case kMapFrom: return {"map_from", "Manifold pressure from (0 CAN, 1 MAP sensor on the pressure input)", "", 0, 1, 0, 1};
  case kMapSensor: return {"map_sensor", "MAP sensor", "", 0, engine::kMapSensorCount, 1, 1};
  case kMapAt05X10: return {"map_at_05v", "MAP sensor at 0.5 V", "kPa", 0, 2000, 325, 10};
  case kMapAt45X10: return {"map_at_45v", "MAP sensor at 4.5 V", "kPa", 100, 6000, 2841, 10};
  case kLineFullPsi: return {"line_sensor_psi", "Line pressure transducer range, 0.5-4.5 V (0 = none fitted)", "psi", 0, 500, 0, 1};
  case kLineMinPsi: return {"line_min_psi", "Line pressure at least, spraying (0 = no check)", "psi", 0, 300, 0, 1};
  case kLineMaxPsi: return {"line_max_psi", "Line pressure at most (0 = no check)", "psi", 0, 500, 0, 1};
  case kBaroX10: return {"baro", "Barometric pressure when not measured", "kPa", 700, 1100, 1013, 10};
  case kEngineDl: return {"engine_l", "Engine size", "L", 6, 90, 23, 10};
  case kFloatClosedWhenLow: return {"tank_switch_closed_when_low", "Tank switch closes when the tank is low", "", 0, 1, 0, 1};
  case kTankProtection: return {"tank_protection", "A low tank stops injection", "", 0, 1, 1, 1};
  case kWifiAlways: return {"wifi_always", "Wi-Fi stays on (0: off 15 min after power-up unless connected)", "", 0, 1, 0, 1};
  case kArmed: return {"armed", "Armed", "", 0, 1, 0, 1};
  case kDrive: return {"pump_drive", "Pump drive (0 full-on pulses, 1 PWM duty rising with boost)", "", 0, 1, 0, 1};
  case kNozzleRatedPsi: return {"nozzle_rated_psi", "Line pressure the nozzle flow was measured at (0 = not known)", "psi", 0, 300, 0, 1};
  case kTankDl: return {"tank_l", "Tank size (0 = no fluid count)", "L", 0, 400, 0, 10};
  case kValveLeadMs: return {"valve_lead_ms", "After a stop, the valve opens this long after the pump starts (0 = with the pump)", "ms", 0, 500, 0, 1};
  case kPumpAmpsMaxX10: return {"pump_amps_max", "Pump current limit (10 A at most)", "A", 30, 100, 100, 10};
  default: return {"", "", "", 0, 0, 0, 1};
  }
}

// The pulse tuning (meth_tune.h), named for the web page; 18 and 19 are the PWM drive's.
constexpr uint8_t kTuneShown = 19;
inline Info tuneInfoOf(uint8_t key) {
  const meth_tune::Range r = meth_tune::rangeOf(key);
  switch (key) {
  case meth_tune::kPeriodMs: return {"cycle_ms", "Cycle length", "ms", r.min, r.max, r.def, 1};
  case meth_tune::kMinOnMs: return {"on_start_ms", "Pump on-time at start boost", "ms", r.min, r.max, r.def, 1};
  case meth_tune::kMaxOnMs: return {"on_full_ms", "Pump on-time at full boost (= cycle: continuous)", "ms", r.min, r.max, r.def, 1};
  case meth_tune::kStartPsiX10: return {"start_psi", "Start boost", "psi", r.min, r.max, r.def, 10};
  case meth_tune::kFullPsiX10: return {"full_psi", "Full boost", "psi", r.min, r.max, r.def, 10};
  case meth_tune::kMinRpm: return {"min_rpm", "Minimum engine speed", "rpm", r.min, r.max, r.def, 1};
  case meth_tune::kMaxSprayS: return {"max_spray_s", "Longest continuous spray", "s", r.min, r.max, r.def, 1};
  case meth_tune::kRestS: return {"rest_s", "Rest after that (0 = no limit)", "s", r.min, r.max, r.def, 1};
  case meth_tune::kRampMs: return {"ramp_ms", "On-time growth per cycle (0 = none)", "ms", r.min, r.max, r.def, 1};
  case meth_tune::kMinPreTempC: return {"min_air_c", "Minimum air temperature (0 = off)", "C", r.min, r.max, r.def, 1};
  case meth_tune::kOverboostAssist: return {"overboost_assist", "Overboost assist", "", r.min, r.max, r.def, 1};
  case meth_tune::kMethPercent: return {"meth_pct", "Methanol in the mix, by volume (the rest water)", "%", r.min, r.max, r.def, 1};
  case meth_tune::kNozzleMlMin: return {"nozzle_ml_min", "Nozzle flow, all nozzles together, at full pump pressure", "ml/min", r.min, r.max, r.def, 1};
  case meth_tune::kMaxDosePct: return {"max_dose_pct", "Most fluid, share of fuel (0 = off)", "%", r.min, r.max, r.def, 1};
  case meth_tune::kEarlyRpm: return {"early_rpm", "Early start from (0 = off)", "rpm", r.min, r.max, r.def, 1};
  case meth_tune::kHotStartC: return {"hot_from_c", "Hot air adds on-time from (0 = off)", "C", r.min, r.max, r.def, 1};
  case meth_tune::kHotFullC: return {"hot_full_c", "Hot air reaches full on-time at", "C", r.min, r.max, r.def, 1};
  case meth_tune::kMinDutyPct: return {"pwm_min_pct", "PWM drive: duty at start boost (the lowest that sprays)", "%", r.min, r.max, r.def, 1};
  case meth_tune::kMaxDutyPct: return {"pwm_max_pct", "PWM drive: duty at full boost", "%", r.min, r.max, r.def, 1};
  default: return {"", "", "", 0, 0, 0, 1};
  }
}

struct Settings {
  uint16_t value[kCount + 1];   // Indexed by Key; [0] unused
};

inline void loadDefaults(Settings &s) {
  s.value[0] = 0;
  for (uint8_t key = 1; key <= kCount; ++key) s.value[key] = infoOf(key).def;
}

inline bool valid(const Settings &s) {
  for (uint8_t key = 1; key <= kCount; ++key) {
    const Info i = infoOf(key);
    if (s.value[key] < i.min || s.value[key] > i.max) return false;
  }
  const uint16_t b = s.value[kBitrateK];
  return (b == 0 || b == 125 || b == 250 || b == 500 || b == 1000) && s.value[kMapAt45X10] > s.value[kMapAt05X10];
}

// Changes one setting, clamped to its limits (a CAN speed to the nearest one CAN uses).
// Choosing a MAP sensor preset fills in its two points; changing a point by hand makes it
// "your own". Returns meth_tune::kSetOk, kSetClamped or kSetUnsupported.
inline uint8_t set(Settings &s, uint8_t key, uint16_t requested, uint16_t &applied) {
  if (key < 1 || key > kCount) { applied = 0; return meth_tune::kSetUnsupported; }
  const Info i = infoOf(key);
  uint16_t v = requested < i.min ? i.min : requested > i.max ? i.max : requested;
  if (key == kBitrateK && v != 0) {
    static const uint16_t speeds[4] = {125, 250, 500, 1000};
    uint16_t best = speeds[0];
    for (uint16_t speed : speeds) {
      const int d = static_cast<int>(speed) - v, bd = static_cast<int>(best) - v;
      if ((d < 0 ? -d : d) < (bd < 0 ? -bd : bd)) best = speed;
    }
    v = best;
  }
  if (key == kMapAt05X10 && v + 100 > s.value[kMapAt45X10]) v = s.value[kMapAt45X10] - 100;
  if (key == kMapAt45X10 && v < s.value[kMapAt05X10] + 100) v = s.value[kMapAt05X10] + 100;
  s.value[key] = v;
  if (key == kMapSensor && v < engine::kMapSensorCount) {
    s.value[kMapAt05X10] = static_cast<uint16_t>(engine::kMapSensors[v].kpaAt05 * 10.0f + 0.5f);
    s.value[kMapAt45X10] = static_cast<uint16_t>(engine::kMapSensors[v].kpaAt45 * 10.0f + 0.5f);
  }
  if (key == kMapAt05X10 || key == kMapAt45X10) s.value[kMapSensor] = engine::kMapSensorCount;
  applied = v;
  return v == requested ? meth_tune::kSetOk : meth_tune::kSetClamped;
}

// The CAN speed to run at, in bit/s.
inline uint32_t bitrateOf(const Settings &s) {
  if (s.value[kBitrateK]) return static_cast<uint32_t>(s.value[kBitrateK]) * 1000UL;
  if (s.value[kSource] == kSourceEcu) return engine::kProfiles[s.value[kProfile]].bitrate;
  return 500000UL;
}
// Whether the controller only listens: always for no source at all and for an ECU broadcast
// (unless told to acknowledge); OBD-II has to ask.
inline bool listenOnlyOf(const Settings &s) {
  const uint8_t src = static_cast<uint8_t>(s.value[kSource]);
  if (src == kSourceObd11 || src == kSourceObd29) return false;
  return src == kSourceNone || s.value[kListenOnly] != 0;
}
// Engine speed can come from somewhere.
inline bool hasRpmSource(const Settings &s) { return s.value[kSource] != kSourceNone; }
// Manifold pressure can come from somewhere.
inline bool hasMapSource(const Settings &s) { return s.value[kMapFrom] == 1 || s.value[kSource] != kSourceNone; }

// The settings a tune profile carries (beside all of the pulse tuning): how this setup
// sprays, not how the car is wired. Engine data, sensors, the tank, Wi-Fi and arming stay.
inline bool inProfile(uint8_t key) {
  return key == kDrive || key == kLineMinPsi || key == kLineMaxPsi || key == kNozzleRatedPsi || key == kValveLeadMs;
}

// Looks a setting up by name: in this table (returns key, tune = false) or the pulse
// tuning's (tune = true). 0 when there is none.
inline uint8_t keyNamed(const char *name, bool &tune) {
  for (uint8_t key = 1; key <= kCount; ++key) {
    if (strcmp(infoOf(key).name, name) == 0) { tune = false; return key; }
  }
  for (uint8_t key = 1; key <= kTuneShown; ++key) {
    if (strcmp(tuneInfoOf(key).name, name) == 0) { tune = true; return key; }
  }
  return 0;
}

} // namespace standalone
