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
#include "knock_spectrum.h"

namespace adc_sampler {

constexpr uint32_t kKnockRateHz = 38462;   // Asked of the ADC (half of 76,924 conversions a second)

enum Slow : uint8_t { kOil = 0, kFuel = 1, kPre = 2, kPost = 3, kSlowCount = 4 };

// The raw knock signal since the last call: for the gain control and the sensor checks.
struct KnockStats {
  uint16_t low{0};       // Lowest and highest sample, 10-bit counts
  uint16_t high{0};
  uint32_t railed{0};    // Samples at a limit of the amplifier (setKnockLimitsMv)
  uint32_t samples{0};
  float bias{0.0f};      // Resting level, counts
  float lowMv{0.0f};     // The same three in millivolts at the pin, through the ADC's
  float highMv{0.0f};    // factory calibration
  float restMv{0.0f};
};

// The lowest and highest voltage the knock amplifier can put on the pin. Call before
// begin(); without it the whole ADC range is assumed.
void setKnockLimitsMv(float lowMv, float highMv);

// Starts the ADC and the collecting task. False when the ADC could not be set up.
bool begin(int knockPin, int oilPin, int fuelPin, int prePin, int postPin);

// The knock frequency and band width to listen at. Cheap to call every pass: the
// filters are redesigned only when a value changes.
void setKnockBand(float centreHz, float bandwidthHz);

// The frequency the knock is expected at (from the bore): the spectrum's search range
// is set around it. Cheap to call every pass.
void setKnockExpected(float expectedHz);

// The spectrum of the block that stood furthest above the noise floor since the last
// call (knock_spectrum.h). False when no block was completed in that time.
bool takeKnockSpectrum(knock_spectrum::Snapshot &out);

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
