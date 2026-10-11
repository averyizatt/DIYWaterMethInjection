// Host tests for the ESP32 board's inputs: the ECU's CAN broadcast and the analog conversions.
// g++ -std=c++11 -I include test/test_esp32_inputs/test_main.cpp
#include <math.h>
#include <stdio.h>

#include "analog_inputs.h"
#include "ecu_broadcast.h"

namespace {
int failures = 0;

void check(bool ok, const char *what) {
  if (!ok) {
    ++failures;
    printf("FAIL: %s\n", what);
  }
}

bool near(float a, float b, float tolerance) { return fabs(a - b) <= tolerance; }

void put16(uint8_t *d, int value) { d[0] = static_cast<uint8_t>(value >> 8); d[1] = static_cast<uint8_t>(value); }

void ecuRealtimeBroadcast() {
  ecu::Data data;
  uint8_t g0[8] = {}, g2[8] = {};
  put16(g0 + 6, 3450);                       // RPM
  put16(g2, 852);                            // Baro 85.2 kPa (Grand Junction)
  put16(g2 + 2, 1543);                       // MAP 154.3 kPa
  put16(g2 + 4, 1220);                       // Intake air 122.0 F = 50 C
  check(ecu::accept(0x5F0, g0, 8, 1000, data) && ecu::accept(0x5F2, g2, 8, 1000, data), "both frames are the ECU's");
  check(data.rpm == 3450 && near(data.mapKpa, 154.3f, 0.05f) && near(data.baroKpa, 85.2f, 0.05f) && near(data.intakeC, 50.0f, 0.1f), "values decoded");
  check(data.rpmFresh(1400) && data.mapFresh(1400) && data.intakeFresh(1400) && data.baroKnown(), "fresh while frames arrive");
  check(!data.rpmFresh(1500) && !data.mapFresh(1600), "stale after half a second: not valid");
  check(data.intakeFresh(2400) && !data.intakeFresh(2600), "air temperature is allowed a little longer");
  check(data.baroKnown(), "barometric pressure is kept");
  // The alternative base address carries the same groups.
  ecu::Data alt;
  check(ecu::accept(0x700, g0, 8, 5, alt) && ecu::accept(0x702, g2, 8, 5, alt) && alt.rpm == 3450 && near(alt.mapKpa, 154.3f, 0.05f), "base 0x700 too");
}

void ecuSimplifiedBroadcast() {
  ecu::Data data;
  uint8_t g0[8] = {}, g1[8] = {};
  put16(g0, 987);                            // MAP 98.7 kPa
  put16(g0 + 2, 2100);                       // RPM
  put16(g1 + 4, 860);                        // Intake air 86.0 F = 30 C
  check(ecu::accept(0x5E8, g0, 8, 10, data) && ecu::accept(0x5E9, g1, 8, 10, data), "simplified frames accepted");
  check(data.rpm == 2100 && near(data.mapKpa, 98.7f, 0.05f) && near(data.intakeC, 30.0f, 0.1f), "values decoded");
  check(!data.baroKnown(), "this form carries no barometric pressure");
}

void ecuFramesThatAreNotBelieved() {
  ecu::Data data;
  uint8_t frame[8] = {};
  check(!ecu::accept(0x301, frame, 8, 0, data) && !ecu::accept(0x5F1, frame, 8, 0, data), "other IDs are not the ECU's values");
  put16(frame + 6, 3000);
  check(ecu::accept(0x5F0, frame, 7, 0, data) && !data.rpmSeen, "a short frame is ignored");
  put16(frame + 6, 40000);
  ecu::accept(0x5F0, frame, 8, 0, data);
  check(!data.rpmSeen, "an impossible RPM is ignored");
  uint8_t g2[8] = {};
  put16(g2, 0); put16(g2 + 2, 0); put16(g2 + 4, 0);
  ecu::accept(0x5F2, g2, 8, 0, data);
  check(!data.mapSeen && !data.baroSeen, "zero pressure (ECU just powered) is not a reading");
  put16(g2 + 2, 5000);
  ecu::accept(0x5F2, g2, 8, 0, data);
  check(!data.mapSeen, "500 kPa is not a reading");
  // Millisecond counter wrapping does not make an old value look fresh for long.
  put16(g2, 850); put16(g2 + 2, 1000);
  ecu::accept(0x5F2, g2, 8, 0xFFFFFF00u, data);
  check(data.mapFresh(0x00000050u) && !data.mapFresh(0x00000200u), "freshness works across the counter wrap");
}

void pressureTransducer() {
  analog_inputs::Pressure oil;
  // 0.5 V at the sensor is 333 mV at the pin: 0 psi. 4.5 V is 3000 mV: 100 psi.
  oil.update(333.3f);
  check(oil.valid && near(oil.psi, 0.0f, 0.1f), "0.5 V: 0 psi");
  for (int i = 0; i < 60; ++i) oil.update(1666.7f);
  check(near(oil.psi, 50.0f, 0.2f) && near(oil.sensorV, 2.5f, 0.01f), "2.5 V: 50 psi, after the filter settles");
  for (int i = 0; i < 60; ++i) oil.update(3000.0f);
  check(near(oil.psi, 100.0f, 0.2f), "4.5 V: 100 psi");
  oil.update(20.0f);
  check(!oil.valid, "unplugged: not valid");
  oil.update(1666.7f);
  check(oil.valid && near(oil.psi, 50.0f, 0.1f), "plugged back in: the first reading is taken as it is");
  oil.update(3290.0f);
  check(!oil.valid, "shorted to 5 V: not valid");
  analog_inputs::Pressure low;
  low.update(300.0f);
  check(low.valid && low.psi == 0.0f, "slightly under 0.5 V reads 0, not negative");
}

void intakeThermistor() {
  float c = 0.0f;
  // 2238 ohm at 30 C on a 2490 ohm pull-up to 3300 mV: 3300 * 2238 / 4728 = 1562 mV.
  check(analog_inputs::thermistorC(1562.0f, 3300.0f, 2490.0f, c) && near(c, 30.0f, 1.0f), "30 C");
  // 177 ohm at 99 C: 3300 * 177 / 2667 = 219 mV.
  check(analog_inputs::thermistorC(219.0f, 3300.0f, 2490.0f, c) && near(c, 99.0f, 2.0f), "99 C");
  check(!analog_inputs::thermistorC(3295.0f, 3300.0f, 2490.0f, c), "open sensor");
  check(!analog_inputs::thermistorC(5.0f, 3300.0f, 2490.0f, c), "shorted sensor");
  check(!analog_inputs::thermistorC(1500.0f, 0.0f, 2490.0f, c), "no supply reading");
}

void tankFloatSwitch() {
  analog_inputs::FloatSwitch tank;
  tank.begin(true, 0, 100, 2000);
  check(tank.isLow(), "low until proven otherwise");
  check(tank.update(false, 50) && !tank.update(false, 160), "fluid seen for the debounce time: not low");
  check(!tank.update(true, 1000) && !tank.update(true, 1050), "a splash is not low");
  check(!tank.update(false, 1100) && !tank.update(false, 1300), "back to fluid");
  check(!tank.update(true, 2000) && !tank.update(true, 2200) && !tank.update(true, 4000), "low, but not yet for long enough");
  check(tank.update(true, 4200), "low for two seconds: low");
  check(!tank.update(false, 4400) || !tank.update(false, 4600), "refilled: clears after the debounce");
}
} // namespace

int main() {
  ecuRealtimeBroadcast();
  ecuSimplifiedBroadcast();
  ecuFramesThatAreNotBelieved();
  pressureTransducer();
  intakeThermistor();
  tankFloatSwitch();
  if (failures) {
    printf("%d ESP32 input test(s) failed\n", failures);
    return 1;
  }
  printf("ESP32 input tests passed\n");
  return 0;
}
