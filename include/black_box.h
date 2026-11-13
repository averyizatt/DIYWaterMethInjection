#pragma once

// The black box: what the board was doing in the last ten seconds, kept in memory a
// reset does not clear (RTC slow memory on the ESP32: it survives a crash, a watchdog, a
// supply dip and a restart, and holds noise only after power on). After a reset nobody
// asked for, the firmware takes the record that was left, starts a new one, and sends the
// old one to the dash (CAN contract extension 9), which writes it to its event log.
// Owner's request, November 2026: no extra wiring, just the last ten seconds.
//
// One sample per kWindowMs, kSlots of them. Each control pass (20 ms) folds into the
// sample being filled, which is written to its slot on every pass, so a reset loses at
// most the pass it came in. Pure logic with no Arduino calls, tested on a PC
// (test/test_black_box).
#include <stdint.h>
#include <string.h>

namespace black_box {

constexpr uint8_t kSlots = 100;            // 10 s of 100 ms
constexpr uint32_t kWindowMs = 100;
constexpr uint8_t kSampleBytes = 10;
constexpr uint32_t kMagic = 0x424C4B31UL;  // "BLK1"

// One sample, byte by byte (the same bytes go to the dash):
enum Byte : uint8_t {
  kDutyEnd = 0,   // Pump duty at the end of the window, 0.5% steps
  kDutyMax = 1,   // Highest pump duty in the window, 0.5% steps
  kMapKpa = 2,    // Manifold pressure from the ECU at the end, kPa absolute; 0 = not fresh
  kRpm32 = 3,     // Engine RPM / 32 at the end; 0 = not known
  kState = 4,     // Low four bits: why it is holding (meth_tune::Hold); then the bits below
  kEvents = 5,    // The bits below
  kRxAge10 = 6,   // Longest time with nothing received on CAN, 10 ms steps (255 = 2.55 s or more)
  kLoopGapMs = 7, // Longest gap between control passes, ms (255 = 255 ms or more)
  kPreC40 = 8,    // Air before the nozzle, C + 40 at the end; 0 = no reading
  kHeap2k = 9,    // Least free memory in the window, 2 KB steps (255 = 510 KB or more)
};
// kState, as at the end of the window:
constexpr uint8_t kArmed = 1 << 4;
constexpr uint8_t kSoftStarting = 1 << 5;
constexpr uint8_t kManualTest = 1 << 6;
constexpr uint8_t kCrashHold = 1 << 7;
// kEvents: the first two as at the end of the window, the rest if they happened in it.
constexpr uint8_t kEcuFresh = 1 << 0;
constexpr uint8_t kCanOnline = 1 << 1;
constexpr uint8_t kPumpStarted = 1 << 2;
constexpr uint8_t kPumpStopped = 1 << 3;
constexpr uint8_t kCanSetUpAgain = 1 << 4;
constexpr uint8_t kCanSendFailed = 1 << 5;
constexpr uint8_t kKeeperTripped = 1 << 6;
constexpr uint8_t kTankLow = 1 << 7;

struct Sample {
  uint8_t b[kSampleBytes];
  uint8_t check;
};

// Lives in RTC memory. slot[next] is the window being filled; the `count` before it are
// finished, oldest first (at most kSlots - 1: the ring holds the open window too).
struct Box {
  uint32_t magic;
  uint8_t next;
  uint8_t count;
  uint8_t spare;
  uint8_t check;
  Sample slot[kSlots];
};

inline uint8_t checkOf(const uint8_t *bytes, uint8_t n) {
  uint8_t c = 0x5A;
  for (uint8_t i = 0; i < n; ++i) c = static_cast<uint8_t>(((c << 1) | (c >> 7)) ^ bytes[i]);
  return c;
}

inline uint8_t headerCheck(const Box &box) {
  const uint8_t bytes[6] = {static_cast<uint8_t>(box.magic), static_cast<uint8_t>(box.magic >> 8),
                            static_cast<uint8_t>(box.magic >> 16), static_cast<uint8_t>(box.magic >> 24), box.next, box.count};
  return checkOf(bytes, 6);
}

inline void sealHeader(Box &box) {
  box.magic = kMagic;
  box.check = headerCheck(box);
}

// What one control pass saw.
struct Pass {
  float dutyPct{0.0f};
  bool ecuFresh{false};
  float mapKpa{0.0f};
  bool rpmValid{false};
  uint16_t rpm{0};
  uint8_t hold{0};
  bool armed{false};
  bool softStarting{false};
  bool manualTest{false};
  bool crashHold{false};
  bool tankLow{false};
  bool canOnline{false};
  bool canSetUpAgain{false};   // The CAN controller was set up (again) since the last pass
  bool canSendFailed{false};   // A frame did not get out since the last pass
  bool keeperTripped{false};   // The CAN task had to switch the pump off since the last pass
  uint32_t canRxAgeMs{0};
  uint32_t loopGapMs{0};
  bool preValid{false};
  float preC{0.0f};
  uint32_t freeHeapBytes{0};
};

inline uint8_t saturate(uint32_t value) { return value > 255U ? 255U : static_cast<uint8_t>(value); }

inline uint8_t halfPercent(float duty) {
  if (!(duty > 0.0f)) return 0;
  return duty >= 100.0f ? 200 : static_cast<uint8_t>(duty * 2.0f + 0.5f);
}

class Recorder {
public:
  // A new record, from now. Call after taking the old one (take()).
  void start(Box &box, uint32_t now) {
    box.next = 0;
    box.count = 0;
    box.slot[0].check = static_cast<uint8_t>(~checkOf(box.slot[0].b, kSampleBytes));   // Nothing in it yet
    sealHeader(box);
    windowStart_ = now;
    open_ = false;
    started_ = true;
  }

  // Every control pass.
  void record(Box &box, uint32_t now, const Pass &p) {
    if (!started_) start(box, now);
    if (open_ && now - windowStart_ >= kWindowMs) {
      advance(box);
      // Late passes do not stretch the windows: they keep to the 100 ms grid, and a long
      // stall shows as a long gap in the sample it ended in.
      const uint32_t behind = now - windowStart_;
      windowStart_ += behind - behind % kWindowMs;
    }
    fold(p);
    Sample &s = box.slot[box.next];
    memcpy(s.b, window_, kSampleBytes);
    s.check = checkOf(s.b, kSampleBytes);
  }

private:
  void advance(Box &box) {
    box.next = static_cast<uint8_t>((box.next + 1) % kSlots);
    if (box.count < kSlots - 1) ++box.count;
    // The slot now open still holds the oldest sample: marked empty before the header
    // says it is the newest, so a reset in between cannot pass it off as the last window.
    box.slot[box.next].check = static_cast<uint8_t>(~checkOf(box.slot[box.next].b, kSampleBytes));
    sealHeader(box);
    open_ = false;
  }

  void fold(const Pass &p) {
    const uint8_t duty = halfPercent(p.dutyPct);
    const uint8_t heap = saturate(p.freeHeapBytes / 2048U);
    const uint8_t rxAge = saturate(p.canRxAgeMs / 10U);
    const uint8_t gap = saturate(p.loopGapMs);
    uint8_t events = 0;
    if (duty > 0 && !lastPumpOn_) events |= kPumpStarted;
    if (duty == 0 && lastPumpOn_) events |= kPumpStopped;
    if (p.canSetUpAgain) events |= kCanSetUpAgain;
    if (p.canSendFailed) events |= kCanSendFailed;
    if (p.keeperTripped) events |= kKeeperTripped;
    if (p.tankLow) events |= kTankLow;
    lastPumpOn_ = duty > 0;
    if (!open_) {
      memset(window_, 0, sizeof(window_));
      window_[kHeap2k] = heap;
      open_ = true;
    }
    window_[kDutyEnd] = duty;
    if (duty > window_[kDutyMax]) window_[kDutyMax] = duty;
    window_[kMapKpa] = p.ecuFresh ? saturate(static_cast<uint32_t>(p.mapKpa > 0.0f ? p.mapKpa + 0.5f : 0.0f)) : 0;
    window_[kRpm32] = p.rpmValid ? saturate((p.rpm + 16U) / 32U) : 0;
    window_[kState] = static_cast<uint8_t>((p.hold & 0x0FU) | (p.armed ? kArmed : 0U) | (p.softStarting ? kSoftStarting : 0U) |
                                           (p.manualTest ? kManualTest : 0U) | (p.crashHold ? kCrashHold : 0U));
    // The two states as at the end; the events gathered over the window.
    window_[kEvents] = static_cast<uint8_t>((window_[kEvents] & ~(kEcuFresh | kCanOnline)) | events |
                                            (p.ecuFresh ? kEcuFresh : 0U) | (p.canOnline ? kCanOnline : 0U));
    if (rxAge > window_[kRxAge10]) window_[kRxAge10] = rxAge;
    if (gap > window_[kLoopGapMs]) window_[kLoopGapMs] = gap;
    const float pre = p.preC + 40.0f;
    window_[kPreC40] = p.preValid && pre >= 1.0f ? saturate(static_cast<uint32_t>(pre + 0.5f)) : 0;
    if (heap < window_[kHeap2k]) window_[kHeap2k] = heap;
  }

  uint32_t windowStart_{0};
  uint8_t window_[kSampleBytes]{};
  bool open_{false};
  bool started_{false};
  bool lastPumpOn_{false};
};

// The record a reset left behind, oldest first, into out (kSlots x kSampleBytes). Returns
// how many samples; 0 when there is none (after power on the memory holds noise) or the
// record is damaged. A sample whose own check fails (half written when the reset came) is
// left out; only the last one can be.
inline uint8_t take(const Box &box, uint8_t out[][kSampleBytes]) {
  if (box.magic != kMagic || box.check != headerCheck(box) || box.next >= kSlots || box.count >= kSlots) return 0;
  uint8_t n = 0;
  for (uint8_t i = 0; i <= box.count; ++i) {
    const uint8_t slot = static_cast<uint8_t>((box.next + kSlots - box.count + i) % kSlots);
    const Sample &s = box.slot[slot];
    if (checkOf(s.b, kSampleBytes) != s.check) continue;
    memcpy(out[n++], s.b, kSampleBytes);
  }
  return n;
}

inline void forget(Box &box) { box.magic = 0; }

// The record as one run of bytes, for the dash: samples one after another.
inline uint16_t sumOf(const uint8_t samples[][kSampleBytes], uint8_t count) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < count; ++i)
    for (uint8_t j = 0; j < kSampleBytes; ++j) sum = static_cast<uint16_t>(sum + samples[i][j]);
  return sum;
}

} // namespace black_box
