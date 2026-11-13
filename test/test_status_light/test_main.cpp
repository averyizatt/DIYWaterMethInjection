// Host tests for the board's RGB LED (include/status_light.h).
// g++ -std=c++11 -I include test/test_status_light/test_main.cpp -o status_light_test && ./status_light_test
#include <stdio.h>

#include "status_light.h"

using namespace status_light;

namespace {
int failures = 0;

void check(bool ok, const char *what) {
  if (!ok) {
    ++failures;
    printf("FAIL: %s\n", what);
  }
}

bool is(Color c, uint8_t r, uint8_t g, uint8_t b) { return c.r == r && c.g == g && c.b == b; }

constexpr uint32_t kAfterLampTest = 5000;

Color shown(const Facts &f, uint32_t since = kAfterLampTest) { return colorOf(classify(since, f), since, f.dutyPct); }

Facts armedAndReady() {
  Facts f;
  f.armed = true;
  return f;
}

void testLampTestComesFirst() {
  Facts f = armedAndReady();
  f.fault = true;
  check(is(shown(f, 0), kBright, 0, 0) && is(shown(f, 299), kBright, 0, 0), "red first");
  check(is(shown(f, 300), 0, kBright, 0) && is(shown(f, 599), 0, kBright, 0), "then green");
  check(is(shown(f, 600), 0, 0, kBright) && is(shown(f, 899), 0, 0, kBright), "then blue");
  check(classify(900, f) == Show::Fault, "then what is going on");
}

void testGoodIsGreenAndNotArmedIsDimWhite() {
  check(is(shown(armedAndReady()), 0, kBright, 0), "armed and ready: green");
  Facts off;
  check(is(shown(off), kDim, kDim, kDim), "not armed: dim white");
}

void testSprayingIsBlueAndBrighterWithDuty() {
  Facts f = armedAndReady();
  f.dutyPct = 25.0f;
  const Color quarter = shown(f);
  f.dutyPct = 100.0f;
  const Color full = shown(f);
  check(quarter.r == 0 && quarter.g == 0 && full.r == 0 && full.g == 0, "spraying: blue only");
  check(full.b == kSprayHigh && quarter.b > kSprayLow && quarter.b < full.b, "brighter with more duty");
  f.dutyPct = 0.4f;
  check(shown(f).b == kSprayLow, "the least duty is still visibly blue");
  f.dutyPct = 26.0f;
  check(same(shown(f), quarter), "small duty changes do not change the colour");
}

void testPumpTestIsPurple() {
  Facts f;
  f.pumpTest = true;
  f.dutyPct = 30.0f;
  check(is(shown(f), kBright, 0, kBright), "pump test: purple");
  f.dutyPct = 0.0f;
  check(classify(kAfterLampTest, f) == Show::Disarmed, "a pump test that is not running the pump shows what is underneath");
}

void testFaultFlashesRedAndOutranksWaiting() {
  Facts f = armedAndReady();
  f.fault = true;
  f.waiting = true;
  check(is(shown(f, 5000), kBright, 0, 0), "fault: red...");
  check(is(shown(f, 5250), 0, 0, 0), "...flashing");
  check(is(shown(f, 5500), kBright, 0, 0), "...on again");
  f.dutyPct = 40.0f;
  check(classify(kAfterLampTest, f) == Show::Spraying, "a spray still shows as a spray");
}

void testWaitingIsYellow() {
  Facts f = armedAndReady();
  f.waiting = true;
  const Color c = shown(f);
  check(c.r == kBright && c.g > 0 && c.g < c.r && c.b == 0, "waiting: yellow (red with some green)");
  check(same(shown(f, 5000), shown(f, 5250)), "steady, not flashing");
}

void testUpdatingFlashesCyanOverEverything() {
  Facts f = armedAndReady();
  f.updating = true;
  f.fault = true;
  check(is(shown(f, 5000), 0, kBright, kBright), "updating: cyan...");
  check(is(shown(f, 5250), 0, 0, 0), "...flashing");
}

void testWrittenOnChangeAndOnceASecond() {
  Light light;
  const Color green{0, kBright, 0};
  const Color red{kBright, 0, 0};
  check(light.due(0, green), "the first colour is written");
  check(!light.due(20, green), "unchanged: not written again...");
  check(!light.due(999, green), "...within the second");
  check(light.due(1000, green), "written again after a second");
  check(light.due(1020, red), "a change is written at once");
  check(!light.due(1040, red), "and not again");
  check(light.due(0xFFFFFFF0UL, red), "a long gap (or the millisecond count wrapping) rewrites it");
  check(!light.due(0x00000010UL, red), "across the wrap, within the second: not rewritten");
}
} // namespace

int main() {
  testLampTestComesFirst();
  testGoodIsGreenAndNotArmedIsDimWhite();
  testSprayingIsBlueAndBrighterWithDuty();
  testPumpTestIsPurple();
  testFaultFlashesRedAndOutranksWaiting();
  testWaitingIsYellow();
  testUpdatingFlashesCyanOverEverything();
  testWrittenOnChangeAndOnceASecond();
  if (failures) {
    printf("%d status light test(s) failed\n", failures);
    return 1;
  }
  printf("status light tests passed\n");
  return 0;
}
