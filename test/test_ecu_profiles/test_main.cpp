// Host tests for the stand-alone PCB firmware's engine data (engine_source.h): every ECU
// signal against the Frogdash dash's own decoder (vectors.h, from tools/ecu_profiles.py),
// then the OBD-II poller, a MAP sensor on the pressure input and the barometer.
// g++ -std=c++11 -O2 -Wall -I include test/test_ecu_profiles/test_main.cpp
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "engine_source.h"
#include "vectors.h"

namespace {
int failures = 0;

void check(bool ok, const char *what) {
  if (!ok) {
    ++failures;
    printf("FAIL: %s\n", what);
  }
}

bool near(double a, double b, double tolerance) { return fabs(a - b) <= tolerance; }

engine::Field fieldOf(const char *key) {
  if (strcmp(key, "ecu.rpm") == 0) return engine::Field::Rpm;
  if (strcmp(key, "ecu.map_kpa") == 0) return engine::Field::MapKpa;
  if (strcmp(key, "ecu.baro_kpa") == 0) return engine::Field::BaroKpa;
  return engine::Field::IatC;
}

// Each vector: the signal for its key on its ID decodes to what the dash decodes.
void sameAsTheDash() {
  unsigned checked = 0;
  for (unsigned n = 0; n < kVectorCount; ++n) {
    const Vector &v = kVectors[n];
    const engine::Profile &p = engine::kProfiles[v.profile];
    const uint32_t key = p.idMask ? (v.id & p.idMask) : v.id;
    bool found = false;
    for (uint8_t i = 0; i < p.count; ++i) {
      const engine::Signal &s = p.signals[i];
      if (engine::idOf(p, s, 0) != key || s.field != fieldOf(v.key)) continue;
      found = true;
      float value = 0.0f;
      const bool ok = engine::decode(s, v.data, 8, value);
      const double tolerance = 1e-6 * (fabs(v.value) > 1.0 ? fabs(v.value) : 1.0) + 1e-5;
      if (!ok || !near(value, v.value, tolerance)) {
        ++failures;
        printf("FAIL: %s 0x%lX %s: %.6f, the dash %.6f\n", p.id, static_cast<unsigned long>(v.id), v.key, value, v.value);
      }
      ++checked;
    }
    if (!found) {
      ++failures;
      printf("FAIL: %s has no %s on 0x%lX\n", p.id, v.key, static_cast<unsigned long>(v.id));
    }
  }
  check(checked == kVectorCount, "every vector checked");
  printf("%u frames decoded as the dash decodes them\n", checked);
}

void put16(uint8_t *d, int value) { d[0] = static_cast<uint8_t>(value >> 8); d[1] = static_cast<uint8_t>(value); }

void megasquirtLikeTheFrogdashBuild() {
  // The MegaSquirt profile must read the same as ecu_broadcast.h, which the car's build uses.
  const engine::Profile *ms = engine::profileNamed("megasquirt");
  check(ms != nullptr, "MegaSquirt is in the table");
  if (!ms) return;
  uint8_t g2[8] = {};
  put16(g2, 852);
  put16(g2 + 2, 1543);
  put16(g2 + 4, 1220);
  ecu::Data a, b;
  check(engine::accept(*ms, 0, 0x5F2, false, g2, 8, 100, a), "0x5F2 is MegaSquirt's");
  ecu::accept(0x5F2, g2, 8, 100, b);
  check(near(a.mapKpa, b.mapKpa, 1e-4) && near(a.baroKpa, b.baroKpa, 1e-4) && near(a.intakeC, b.intakeC, 1e-3),
        "same values as the car's own decoder");
  check(!engine::accept(*ms, 0, 0x5F2, true, g2, 8, 100, a), "a 29-bit frame is not the 11-bit ECU's");
  check(!engine::accept(*ms, 0, 0x360, false, g2, 8, 100, a), "another ECU's ID is not taken");
}

void haltechAndTheRanges() {
  const engine::Profile *h = engine::profileNamed("haltech");
  check(h != nullptr && h->bitrate == 1000000, "Haltech at 1 Mbit/s");
  if (!h) return;
  uint8_t f[8] = {};
  put16(f, 4321);        // RPM
  put16(f + 2, 2034);    // MAP 203.4 kPa
  ecu::Data d;
  check(engine::accept(*h, 0, 0x360, false, f, 8, 10, d) && d.rpm == 4321 && near(d.mapKpa, 203.4, 0.01), "Haltech 0x360");
  put16(f + 2, 9000);    // 900 kPa: not believed
  engine::accept(*h, 0, 0x360, false, f, 8, 20, d);
  check(near(d.mapKpa, 203.4, 0.01), "a manifold pressure out of range is ignored");
  ecu::Data e;
  engine::accept(*h, 0, 0x360, false, f, 3, 30, e);
  check(!e.rpmSeen || e.rpm == 4321, "a frame too short for a signal leaves it alone");
}

void movableBase() {
  const engine::Profile *emu = engine::profileNamed("ecumaster");
  check(emu != nullptr && emu->defaultBase == 0x600, "ECUMaster's stream at 0x600 by default");
  if (!emu) return;
  uint8_t f[8] = {0xB8, 0x0B, 0, 25, 0x96, 0x00, 0, 0};   // 3000 rpm, IAT 25 C, MAP 150 kPa
  ecu::Data d;
  check(engine::accept(*emu, 0, 0x600, false, f, 8, 5, d) && d.rpm == 3000 && near(d.mapKpa, 150, 0.01) && near(d.intakeC, 25, 0.01),
        "ECUMaster at its default base");
  ecu::Data moved;
  check(!engine::accept(*emu, 0x400, 0x600, false, f, 8, 5, moved), "with the base moved, 0x600 is not its");
  check(engine::accept(*emu, 0x400, 0x400, false, f, 8, 5, moved) && moved.rpm == 3000, "...0x400 is");
}

void holleySerialBits() {
  const engine::Profile *holley = engine::profileNamed("holley");
  check(holley != nullptr && holley->extended && holley->idMask != 0, "Holley: 29-bit, serial bits masked");
  if (!holley) return;
  float rpm = 2500.0f;
  uint32_t bits;
  memcpy(&bits, &rpm, 4);
  uint8_t f[8] = {static_cast<uint8_t>(bits >> 24), static_cast<uint8_t>(bits >> 16), static_cast<uint8_t>(bits >> 8),
                  static_cast<uint8_t>(bits), 0, 0, 0, 0};
  ecu::Data d;
  check(engine::accept(*holley, 0, 0x1E005000UL | 0x123, true, f, 8, 7, d) && d.rpm == 2500, "Holley's float RPM, any serial");
}

void obd2Poller() {
  engine::Obd2 obd(false);
  ecu::Data d;
  uint32_t id = 0;
  uint8_t req[8];
  // The first requests: barometer, air temperature, then speed and MAP in turn.
  bool sawRpm = false, sawMap = false;
  uint32_t now = 0;
  for (int n = 0; n < 8; ++n) {
    now += 60;
    if (!obd.due(now, id, req)) continue;
    check(id == 0x7DF && req[0] == 2 && req[1] == 1, "a mode 01 request to the functional address");
    uint8_t answer[8] = {0x03, 0x41, req[2], 0, 0, 0, 0, 0};
    if (req[2] == engine::kPidRpm) { answer[0] = 4; answer[3] = 0x2E; answer[4] = 0xE0; sawRpm = true; }   // 3000 rpm
    if (req[2] == engine::kPidMap) { answer[3] = 180; sawMap = true; }
    if (req[2] == engine::kPidBaro) answer[3] = 86;
    if (req[2] == engine::kPidIat) answer[3] = 70;   // 30 C
    check(obd.accept(0x7E8, false, answer, 8, now + 5, d), "the engine's answer is taken");
  }
  check(sawRpm && sawMap, "speed and MAP asked for");
  check(d.rpm == 3000 && near(d.mapKpa, 180, 0.01) && near(d.baroKpa, 86, 0.01) && near(d.intakeC, 30, 0.01), "OBD-II values");
  check(obd.has(engine::kPidRpm) && obd.has(engine::kPidMap), "answered PIDs remembered");
  // Asked often: in one second, well over ten requests (one at a time, 50 ms each at most).
  int asks = 0;
  for (uint32_t t = now + 10; t < now + 1010; t += 5) {
    if (obd.due(t, id, req)) {
      ++asks;
      uint8_t answer[8] = {0x03, 0x41, req[2], 100, 0, 0, 0, 0};
      if (req[2] == engine::kPidRpm) answer[0] = 4;
      obd.accept(0x7E9, false, answer, 8, t + 2, d);
    }
  }
  check(asks >= 15, "answered quickly, it asks again quickly");
  // A car that never answers the barometer is asked for it less and less.
  engine::Obd2 quiet(true);
  int baroAsks = 0;
  for (uint32_t t = 0; t < 60000; t += 5) {
    if (!quiet.due(t, id, req)) continue;
    check(id == engine::kObdRequest29, "29-bit requests on 0x18DB33F1");
    if (req[2] == engine::kPidBaro) { ++baroAsks; continue; }   // Never answered
    uint8_t answer[8] = {0x04, 0x41, req[2], 10, 0, 0, 0, 0};
    quiet.accept(0x18DAF110UL, true, answer, 8, t + 1, d);
  }
  check(baroAsks > 5 && baroAsks < 40, "an unanswered PID is asked less often");
  check(!obd.accept(0x18DAF110UL, true, req, 8, 1, d), "a 29-bit answer is not the 11-bit poller's");
}

void mapSensor() {
  float kpa = 0.0f;
  check(engine::mapFromShare(0.1f, 28.72f, 188.08f, kpa) && near(kpa, 28.72, 0.01), "0.5 V of 5 V: the first point");
  check(engine::mapFromShare(0.9f, 28.72f, 188.08f, kpa) && near(kpa, 188.08, 0.01), "4.5 V: the second");
  check(engine::mapFromShare(0.5f, 32.54f, 284.06f, kpa) && near(kpa, 158.3, 0.05), "GM 3-bar half way");
  check(!engine::mapFromShare(0.0f, 28.72f, 188.08f, kpa) && !engine::mapFromShare(0.99f, 28.72f, 188.08f, kpa),
        "unplugged or shorted is no reading");
  // The presets agree with MegaSquirt's published 0 V / 5 V figures.
  check(near(engine::kMapSensors[1].kpaAt05, 1.1 + 0.1 * (315.5 - 1.1), 0.01), "GM 3-bar preset");
}

void barometer() {
  engine::Barometer baro(101.3f);
  ecu::Data d;
  ecu::takeMap(d, 85.0f, 100);
  check(near(baro.update(100, d, false), 85.0, 0.01), "no engine speed at all: the manifold at power-up");
  ecu::takeMap(d, 140.0f, 5000);
  check(near(baro.update(5000, d, false), 85.0, 0.01), "...not later");
  engine::Barometer withRpm(101.3f);
  ecu::Data e;
  ecu::takeMap(e, 86.0f, 100);
  ecu::takeRpm(e, 900, 100);
  check(near(withRpm.update(100, e, true), 101.3, 0.01), "engine running: the setting");
  ecu::takeRpm(e, 0, 200);
  ecu::takeMap(e, 86.0f, 200);
  check(near(withRpm.update(200, e, true), 86.0, 0.01), "engine stopped: the manifold");
  e.baroKpa = 84.0f; e.baroSeen = true;
  check(near(withRpm.update(300, e, true), 84.0, 0.01), "the ECU's barometer first");
}
} // namespace

int main() {
  sameAsTheDash();
  megasquirtLikeTheFrogdashBuild();
  haltechAndTheRanges();
  movableBase();
  holleySerialBits();
  obd2Poller();
  mapSensor();
  barometer();
  if (failures) {
    printf("%d failure(s)\n", failures);
    return 1;
  }
  printf("test_ecu_profiles: all passed\n");
  return 0;
}
