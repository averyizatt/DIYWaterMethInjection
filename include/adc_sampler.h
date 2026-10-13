#pragma once

// All analog inputs of the ESP32 board, sampled by the ADC's own hardware timer.
//
// One continuous conversion pattern does everything: knock, oil, knock, fuel, knock,
// air before, knock, air after, at 76.9 kHz in total. So the knock input is read every
// other conversion, 38.5 thousand times a second, and each slow input 9,600 times.
//
// A background task takes the results as they arrive. Every knock sample goes through
// the knock filters there and then (knock_dsp.h); nothing is skipped. The main loop
// collects the half-millisecond energy frames those produce, and the averages of the
// slow inputs.
//
// The filters are designed for the sample rate that is measured, not the one asked
// for: if the ADC turns out to run at another rate, they are redesigned for it.
#include <stddef.h>
#include <stdint.h>

#include "knock_dsp.h"

namespace adc_sampler {

constexpr uint32_t kKnockRateHz = 38462;   // Asked of the ADC (half of 76,924 conversions a second)
constexpr float kRailLow = 30.0f;          // A knock sample at or beyond these (10-bit counts)
constexpr float kRailHigh = 985.0f;        // is at the limit of the amplifier or the ADC

enum Slow : uint8_t { kOil = 0, kFuel = 1, kPre = 2, kPost = 3, kSlowCount = 4 };

// The raw knock signal since the last call: for the gain control and the sensor checks.
struct KnockStats {
  uint16_t low{0};       // Lowest and highest sample
  uint16_t high{0};
  uint32_t railed{0};    // Samples at a limit
  uint32_t samples{0};
  float bias{0.0f};      // Resting level
};

// Starts the ADC and the collecting task. False when the ADC could not be set up.
bool begin(int knockPin, int oilPin, int fuelPin, int prePin, int postPin);

// The knock frequency and band width to listen at. Cheap to call every pass: the
// filters are redesigned only when a value changes.
void setKnockBand(float centreHz, float bandwidthHz);

// The energy frames completed since the last call, oldest first. Returns how many.
size_t takeKnockFrames(knock_dsp::Frame *out, size_t capacity);

// False when no knock sample arrived since the last call.
bool takeKnockStats(KnockStats &out);

// Whether the sample rate is high enough for the second-resonance filter.
bool knockSecondActive();

// Average pin voltage in millivolts since the last call for this input.
// Returns false when no sample arrived in that time.
bool averageMv(Slow input, float &millivolts);

// Milliseconds since data last arrived from the ADC (for the self-check).
uint32_t silentForMs(uint32_t nowMs);

// Knock samples per second as actually delivered, and the rate the filters are
// designed for (the same, once measured).
uint32_t knockSamplesPerSecond();
uint32_t knockFilterRateHz();

} // namespace adc_sampler
