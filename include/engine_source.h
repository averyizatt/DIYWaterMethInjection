#pragma once

// Where the stand-alone PCB firmware gets engine speed, manifold pressure, barometric
// pressure and intake air temperature, for any car (docs/PCB.md):
//
//   - An aftermarket ECU's CAN broadcast, from the table in ecu_profiles.h (generated from
//     the Frogdash dash's ECU profiles: MegaSquirt, Haltech, ECUMaster, MaxxECU, rusEFI,
//     Speeduino, AEM, FuelTech, Holley). The board only listens.
//   - OBD-II over CAN (most cars from 2008 on, many earlier): the board asks for engine
//     speed, manifold pressure, barometric pressure and intake air temperature, about ten
//     times a second for the first two.
//   - A MAP sensor on the board's pressure input (0.5-4.5 V or any straight-line sensor
//     on 5 V), with or without one of the above for engine speed.
//
// Everything lands in ecu::Data (ecu_broadcast.h), with the same freshness rules as the
// Frogdash build's MegaSquirt broadcast. Pure logic with no Arduino calls, tested on a PC
// (test/test_ecu_profiles, test/test_engine_source).
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "ecu_broadcast.h"
#include "ecu_profiles.h"

namespace engine {

inline uint32_t be(const uint8_t *d, uint8_t n) {
  uint32_t v = 0;
  for (uint8_t i = 0; i < n; ++i) v = (v << 8) | d[i];
  return v;
}
inline uint32_t le(const uint8_t *d, uint8_t n) {
  uint32_t v = 0;
  for (uint8_t i = n; i > 0; --i) v = (v << 8) | d[i - 1];
  return v;
}
inline uint8_t sizeOf(Layout l) {
  switch (l) {
  case Layout::U8: case Layout::S8: return 1;
  case Layout::U16Be: case Layout::S16Be: case Layout::U16Le: case Layout::S16Le: return 2;
  default: return 4;
  }
}

// One signal's value from a frame, in the field's unit (rpm, kPa, C). False if the frame
// is too short for it or the value is not a number.
inline bool decode(const Signal &s, const uint8_t *d, uint8_t len, float &value) {
  const uint8_t size = sizeOf(s.layout);
  if (s.byte + size > len) return false;
  const uint8_t *p = d + s.byte;
  double raw = 0.0;
  switch (s.layout) {
  case Layout::U8: raw = p[0]; break;
  case Layout::S8: raw = static_cast<int8_t>(p[0]); break;
  case Layout::U16Be: raw = be(p, 2); break;
  case Layout::S16Be: raw = static_cast<int16_t>(be(p, 2)); break;
  case Layout::U16Le: raw = le(p, 2); break;
  case Layout::S16Le: raw = static_cast<int16_t>(le(p, 2)); break;
  case Layout::U32Be: raw = be(p, 4); break;
  case Layout::S32Be: raw = static_cast<int32_t>(be(p, 4)); break;
  case Layout::U32Le: raw = le(p, 4); break;
  case Layout::S32Le: raw = static_cast<int32_t>(le(p, 4)); break;
  case Layout::F32Be:
  case Layout::F32Le: {
    const uint32_t bits = s.layout == Layout::F32Be ? be(p, 4) : le(p, 4);
    float f;
    memcpy(&f, &bits, sizeof(f));
    raw = f;
    break;
  }
  }
  // A scale that is one over a whole number divides, as the dash's decoder does, so both
  // give the same value to the last bit.
  const double scale = s.scale;
  const double inverse = scale != 0.0 && fabs(scale) < 1.0 ? 1.0 / scale : 0.0;
  const double whole = floor(inverse + 0.5);
  double v = inverse != 0.0 && fabs(inverse - whole) < 1e-9 ? raw / whole : raw * scale;
  v += s.offset;
  switch (s.convert) {
  case Convert::None: break;
  case Convert::DegFTenthsToC: v = v / 10.0 * 5.0 / 9.0 - 32.0 * 5.0 / 9.0; break;
  case Convert::DegFToC: v = (v - 32.0) * 5.0 / 9.0; break;
  case Convert::KelvinToC: v = v - 273.15; break;
  case Convert::PsiToKpa: v = v * 6.894757293168361; break;
  case Convert::BarToKpa: v = v * 100.0; break;
  }
  if (!(v == v) || v > 1e9 || v < -1e9) return false;   // NaN or absurd
  value = static_cast<float>(v);
  return true;
}

// The ID a signal is on, for this ECU's base (0: the profile's default).
inline uint32_t idOf(const Profile &p, const Signal &s, uint32_t base) {
  return s.relative ? (base ? base : p.defaultBase) + s.id : s.id;
}

// Returns true when the frame is this ECU's (whatever it carried). Values go into `out`.
inline bool accept(const Profile &p, uint32_t base, uint32_t id, bool extended, const uint8_t *d, uint8_t len,
                   uint32_t now, ecu::Data &out) {
  if (extended != p.extended) return false;
  const uint32_t key = p.idMask ? (id & p.idMask) : id;
  bool mine = false;
  for (uint8_t i = 0; i < p.count; ++i) {
    const Signal &s = p.signals[i];
    if (idOf(p, s, base) != key) continue;
    mine = true;
    float v = 0.0f;
    if (!decode(s, d, len, v)) continue;
    switch (s.field) {
    case Field::Rpm:
      if (v >= 0.0f && v <= ecu::kMaxRpm) ecu::takeRpm(out, static_cast<uint16_t>(v + 0.5f), now);
      break;
    case Field::MapKpa: ecu::takeMap(out, v, now); break;
    case Field::BaroKpa:
      if (v >= ecu::kMinBaroKpa && v <= ecu::kMaxBaroKpa) { out.baroKpa = v; out.baroMs = now; out.baroSeen = true; }
      break;
    case Field::IatC: ecu::takeIntake(out, v, now); break;
    }
  }
  return mine;
}

// The profile with this id, or null.
inline const Profile *profileNamed(const char *id) {
  for (uint8_t i = 0; i < kProfileCount; ++i) {
    if (strcmp(kProfiles[i].id, id) == 0) return &kProfiles[i];
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// OBD-II over CAN (ISO 15765-4): mode 01 requests to the functional address, answers from
// any engine computer. One request at a time; the next goes when the answer is in or after
// kObdAnswerMs. Engine speed and manifold pressure alternate (about ten each a second),
// with barometric pressure and intake air now and then. A PID the car does not answer is
// asked less and less often (it may simply not have it: many cars have no MAP sensor).
// ---------------------------------------------------------------------------
constexpr uint8_t kPidRpm = 0x0C, kPidMap = 0x0B, kPidIat = 0x0F, kPidBaro = 0x33;
constexpr uint32_t kObdAnswerMs = 50;
constexpr uint32_t kObdRequest11 = 0x7DF, kObdRequest29 = 0x18DB33F1UL;

class Obd2 {
public:
  explicit Obd2(bool extended = false) : extended_(extended) {}
  void setExtended(bool extended) { extended_ = extended; }
  bool extended() const { return extended_; }

  // A request due now: fills the ID and the frame (DLC 8). Call every few milliseconds.
  bool due(uint32_t now, uint32_t &id, uint8_t data[8]) {
    if (waiting_ && now - askedMs_ < kObdAnswerMs) return false;
    if (waiting_) miss(asked_);
    const uint8_t pid = next(now);
    id = extended_ ? kObdRequest29 : kObdRequest11;
    memset(data, 0x00, 8);
    data[0] = 0x02;   // Two bytes follow: mode, PID
    data[1] = 0x01;
    data[2] = pid;
    asked_ = pid;
    askedMs_ = now;
    waiting_ = true;
    return true;
  }

  // An answer: true when the frame was one (from any engine computer).
  bool accept(uint32_t id, bool extended, const uint8_t *d, uint8_t len, uint32_t now, ecu::Data &out) {
    if (extended != extended_) return false;
    const bool answer = extended ? (id & 0xFFFFFF00UL) == 0x18DAF100UL : (id >= 0x7E8 && id <= 0x7EF);
    if (!answer) return false;
    if (len < 4 || d[1] != 0x41 || d[0] < 3) return true;
    const uint8_t pid = d[2];
    if (pid == kPidRpm && d[0] >= 4 && len >= 5) {
      ecu::takeRpm(out, static_cast<uint16_t>(((d[3] << 8) | d[4]) / 4), now);
    } else if (pid == kPidMap) {
      ecu::takeMap(out, static_cast<float>(d[3]), now);
    } else if (pid == kPidBaro) {
      const float baro = static_cast<float>(d[3]);
      if (baro >= ecu::kMinBaroKpa && baro <= ecu::kMaxBaroKpa) { out.baroKpa = baro; out.baroMs = now; out.baroSeen = true; }
    } else if (pid == kPidIat) {
      ecu::takeIntake(out, static_cast<float>(d[3]) - 40.0f, now);
    } else {
      return true;
    }
    answered(pid);
    if (pid == asked_) waiting_ = false;
    return true;
  }

  // The car has answered for this PID at least once since start-up.
  bool has(uint8_t pid) const { return slot(pid) >= 0 && answered_[slot(pid)]; }

private:
  static int8_t slot(uint8_t pid) {
    return pid == kPidRpm ? 0 : pid == kPidMap ? 1 : pid == kPidBaro ? 2 : pid == kPidIat ? 3 : -1;
  }
  void answered(uint8_t pid) {
    const int8_t i = slot(pid);
    if (i < 0) return;
    answered_[i] = true;
    misses_[i] = 0;
  }
  void miss(uint8_t pid) {
    const int8_t i = slot(pid);
    if (i >= 0 && misses_[i] < 250) ++misses_[i];
  }
  // How long between asks for a PID: its own pace, longer the more it has gone unanswered
  // (up to about 10 s), so a car without it is not asked ten times a second for nothing.
  uint32_t paceOf(uint8_t i) const {
    static const uint32_t base[4] = {0, 0, 2000, 1000};
    uint32_t pace = base[i];
    if (misses_[i] >= 5) pace = pace < 1000 ? 1000 : pace;
    if (misses_[i] >= 20) pace = 10000;
    return pace;
  }
  uint8_t next(uint32_t now) {
    static const uint8_t pids[4] = {kPidRpm, kPidMap, kPidBaro, kPidIat};
    // Barometer and air temperature when their time has come; otherwise speed and MAP in turn.
    for (uint8_t i = 2; i < 4; ++i) {
      if (now - lastAsk_[i] >= paceOf(i) || !askedOnce_[i]) { mark(i, now); return pids[i]; }
    }
    for (uint8_t n = 0; n < 2; ++n) {
      flip_ = !flip_;
      const uint8_t i = flip_ ? 0 : 1;
      if (now - lastAsk_[i] >= paceOf(i) || !askedOnce_[i]) { mark(i, now); return pids[i]; }
    }
    mark(0, now);
    return kPidRpm;
  }
  void mark(uint8_t i, uint32_t now) { lastAsk_[i] = now; askedOnce_[i] = true; }

  uint32_t askedMs_{0};
  uint32_t lastAsk_[4]{0, 0, 0, 0};
  uint8_t misses_[4]{0, 0, 0, 0};
  bool answered_[4]{false, false, false, false};
  bool askedOnce_[4]{false, false, false, false};
  uint8_t asked_{0};
  bool waiting_{false};
  bool flip_{false};
  bool extended_{false};
};

// ---------------------------------------------------------------------------
// A MAP sensor on the board's pressure input: a straight line through its pressure at
// 0.5 V and at 4.5 V of a 5 V supply, read ratiometrically (as a share of the board's
// measured sensor supply, so the supply's own drift cancels). Presets are the published
// figures, extended along the same straight line, and are only a starting point: with the
// key on and the engine stopped, the reading should match the day's barometer.
// ---------------------------------------------------------------------------
struct MapSensor {
  const char *name;
  float kpaAt05;   // At 0.5 V of 5 V (10 % of the supply)
  float kpaAt45;   // At 4.5 V (90 %)
};
// GM 2- and 3-bar and Freescale MPX4250 as MegaSquirt's TunerStudio lists them (kPa at 0 V
// and 5 V: 8.8/208.0, 1.1/315.5, 10.0/260.0), moved to 0.5 V and 4.5 V.
constexpr MapSensor kMapSensors[] = {
    {"GM 2-bar (16040749)", 28.72f, 188.08f},
    {"GM 3-bar (12223861)", 32.54f, 284.06f},
    {"MPX4250 (2.5 bar)", 35.0f, 235.0f},
};
constexpr uint8_t kMapSensorCount = sizeof(kMapSensors) / sizeof(MapSensor);

// Manifold pressure from the sensor's share of its supply (0..1). False outside 2-98 %:
// unplugged, shorted, or no supply.
inline bool mapFromShare(float share, float kpaAt05, float kpaAt45, float &kpa) {
  if (!(share >= 0.02f && share <= 0.98f)) return false;
  kpa = kpaAt05 + (share - 0.1f) / 0.8f * (kpaAt45 - kpaAt05);
  return true;
}

// ---------------------------------------------------------------------------
// Barometric pressure, to turn manifold pressure into boost: the ECU's own when it sends
// one; otherwise the manifold with the engine stopped (known stopped: RPM 0) or, with no
// engine speed at all, the manifold in the first seconds after power-up (the key turned on
// before cranking); otherwise the setting.
// ---------------------------------------------------------------------------
constexpr uint32_t kBaroAtStartMs = 3000;

class Barometer {
public:
  explicit Barometer(float fallbackKpa = 101.3f) : kpa_(fallbackKpa) {}
  void setFallback(float kpa) { if (!learned_) kpa_ = kpa; }
  float update(uint32_t now, const ecu::Data &d, bool haveRpmSource) {
    if (d.baroKnown()) {
      kpa_ = d.baroKpa;
      learned_ = true;
    } else if (d.mapFresh(now) && d.mapKpa >= ecu::kMinBaroKpa && d.mapKpa <= ecu::kMaxBaroKpa) {
      const bool stopped = d.rpmFresh(now) && d.rpm == 0;
      const bool atStart = !haveRpmSource && now < kBaroAtStartMs;
      if (stopped || atStart) {
        kpa_ = d.mapKpa;
        learned_ = true;
      }
    }
    return kpa_;
  }
  float kpa() const { return kpa_; }
  bool learned() const { return learned_; }

private:
  float kpa_;
  bool learned_{false};
};

} // namespace engine
