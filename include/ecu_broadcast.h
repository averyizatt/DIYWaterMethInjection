#pragma once

// Engine speed, manifold pressure, barometric pressure and intake air temperature read
// straight from the MicroSquirt's CAN broadcast, so the controller needs no MAP sensor
// of its own and does not depend on the dash to decide when to inject.
//
// Both of the ECU's broadcast forms are understood (MS2/Extra, big-endian):
//   Simplified dash broadcast   0x5E8  MAP 0.1 kPa, RPM
//                               0x5E9  intake air 0.1 F (bytes 4-5)
//   Real-time data, base 1520   0x5F0  RPM (bytes 6-7)          (also base 0x700)
//                               0x5F2  baro 0.1 kPa, MAP 0.1 kPa, intake air 0.1 F
// A value is used only while its frames keep arriving: stale means not valid, and the
// controller then does not inject. Pure logic, tested on a PC (test/test_esp32_inputs).
#include <stdint.h>

namespace ecu {

constexpr uint32_t kFreshMs = 500;        // A value older than this is not used.
constexpr float kMinMapKpa = 10.0f;       // Outside these the frame is not believed.
constexpr float kMaxMapKpa = 400.0f;
constexpr float kMinBaroKpa = 60.0f;
constexpr float kMaxBaroKpa = 110.0f;
constexpr uint16_t kMaxRpm = 15000;

struct Data {
  uint16_t rpm{0};
  float mapKpa{0.0f};
  float baroKpa{0.0f};
  float intakeC{0.0f};
  uint32_t rpmMs{0}, mapMs{0}, baroMs{0}, intakeMs{0};
  bool rpmSeen{false}, mapSeen{false}, baroSeen{false}, intakeSeen{false};

  bool rpmFresh(uint32_t now) const { return rpmSeen && static_cast<uint32_t>(now - rpmMs) < kFreshMs; }
  bool mapFresh(uint32_t now) const { return mapSeen && static_cast<uint32_t>(now - mapMs) < kFreshMs; }
  bool intakeFresh(uint32_t now) const { return intakeSeen && static_cast<uint32_t>(now - intakeMs) < 3 * kFreshMs; }
  // Barometric pressure changes slowly: the last good value is kept.
  bool baroKnown() const { return baroSeen; }
};

inline int16_t s16(const uint8_t *d) { return static_cast<int16_t>((static_cast<uint16_t>(d[0]) << 8) | d[1]); }
inline uint16_t u16(const uint8_t *d) { return static_cast<uint16_t>((static_cast<uint16_t>(d[0]) << 8) | d[1]); }
inline float fahrenheitTenthsToC(int16_t tenths) { return (tenths / 10.0f - 32.0f) * 5.0f / 9.0f; }

inline bool isEcuId(uint16_t id) {
  return id == 0x5E8 || id == 0x5E9 || id == 0x5F0 || id == 0x5F2 || id == 0x700 || id == 0x702;
}

inline void takeRpm(Data &out, uint16_t rpm, uint32_t now) {
  if (rpm > kMaxRpm) return;
  out.rpm = rpm; out.rpmMs = now; out.rpmSeen = true;
}
inline void takeMap(Data &out, float kpa, uint32_t now) {
  if (!(kpa >= kMinMapKpa && kpa <= kMaxMapKpa)) return;
  out.mapKpa = kpa; out.mapMs = now; out.mapSeen = true;
}
inline void takeIntake(Data &out, float c, uint32_t now) {
  if (!(c > -45.0f && c < 200.0f)) return;
  out.intakeC = c; out.intakeMs = now; out.intakeSeen = true;
}

// Returns true when the frame was one of the ECU's. Frames of the wrong length are ignored.
inline bool accept(uint16_t id, const uint8_t *d, uint8_t len, uint32_t now, Data &out) {
  if (!isEcuId(id)) return false;
  if (len != 8) return true;
  if (id == 0x5E8) {
    takeMap(out, s16(d) / 10.0f, now);
    takeRpm(out, u16(d + 2), now);
  } else if (id == 0x5E9) {
    takeIntake(out, fahrenheitTenthsToC(s16(d + 4)), now);
  } else if (id == 0x5F0 || id == 0x700) {
    takeRpm(out, u16(d + 6), now);
  } else {  // 0x5F2 / 0x702
    const float baro = s16(d) / 10.0f;
    if (baro >= kMinBaroKpa && baro <= kMaxBaroKpa) { out.baroKpa = baro; out.baroMs = now; out.baroSeen = true; }
    takeMap(out, s16(d + 2) / 10.0f, now);
    takeIntake(out, fahrenheitTenthsToC(s16(d + 4)), now);
  }
  return true;
}

} // namespace ecu
