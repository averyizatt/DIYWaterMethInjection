#pragma once

// All analog inputs of the ESP32 board, sampled by the ADC's own hardware timer.
//
// The knock amplifier needs evenly spaced fast samples (64 at 19.2 kHz, as
// knock_band.h expects); the pressure and temperature inputs only need averages.
// One continuous conversion pattern does both: knock, oil, knock, fuel, knock, air
// before, knock, air after, at 38.46 kHz in total, so the knock input is read every
// other conversion (19.23 kHz) and each slow input 4,800 times a second.
// A background task collects the results; the main loop asks for the newest.
#include <stdint.h>

#include "knock_band.h"

namespace adc_sampler {

enum Slow : uint8_t { kOil = 0, kFuel = 1, kPre = 2, kPost = 3, kSlowCount = 4 };

// Starts the ADC and the collecting task. False when the ADC could not be set up.
bool begin(int knockPin, int oilPin, int fuelPin, int prePin, int postPin);

// The newest complete knock burst, scaled to 10 bits as knock_band::analyze expects.
// Returns false when no new burst has arrived since the last call.
bool latestBurst(uint16_t out[knock_band::kSamples]);

// The lowest and highest knock sample (10 bits) since the last call: every sample, not
// only the burst that was analysed, so clipping anywhere in between is seen.
// Returns false when no knock sample arrived in that time.
bool knockRange(uint16_t &low, uint16_t &high);

// Average pin voltage in millivolts since the last call for this input.
// Returns false when no sample arrived in that time.
bool averageMv(Slow input, float &millivolts);

// Milliseconds since data last arrived from the ADC (for the self-check).
uint32_t silentForMs(uint32_t nowMs);

// Knock samples per second as actually delivered, measured over the last second.
// It should read close to knock_band::kSampleRateHz; the knock frequency depends on it.
uint32_t knockSamplesPerSecond();

} // namespace adc_sampler
