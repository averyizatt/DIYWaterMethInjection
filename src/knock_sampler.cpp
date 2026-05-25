#include "knock_sampler.h"

#include <Arduino.h>

void KnockSampler::begin(int adcPin, uint16_t sampleRateHz) {
  adcPin_ = adcPin;
  sampleRateHz_ = sampleRateHz > 0 ? sampleRateHz : 8000;

  if (adcPin_ >= 0) {
    pinMode(adcPin_, INPUT);
    analogReadResolution(12);
#if defined(ADC_11db)
    analogSetPinAttenuation(adcPin_, ADC_11db);
#endif
  }
}

void KnockSampler::setSampleRate(uint16_t sampleRateHz) {
  if (sampleRateHz > 0) sampleRateHz_ = sampleRateHz;
}

uint8_t KnockSampler::captureBlock(uint16_t *outSamples, uint8_t maxSamples,
                                   uint16_t clipLowAdc, uint16_t clipHighAdc,
                                   KnockSamplerStats &stats) const {
  stats = KnockSamplerStats{};
  if (adcPin_ < 0 || outSamples == nullptr || maxSamples == 0) return 0;

  const uint32_t periodUs = sampleRateHz_ > 0 ? (1000000UL / sampleRateHz_) : 125;
  uint32_t nextTick = micros();

  for (uint8_t i = 0; i < maxSamples; ++i) {
    while (static_cast<int32_t>(micros() - nextTick) < 0) {
    }
    nextTick += periodUs;

    const uint16_t raw = static_cast<uint16_t>(analogRead(adcPin_));
    outSamples[i] = raw;
    if (raw < stats.minAdc) stats.minAdc = raw;
    if (raw > stats.maxAdc) stats.maxAdc = raw;
    if (raw <= clipLowAdc) ++stats.clipLowCount;
    if (raw >= clipHighAdc) ++stats.clipHighCount;
    ++stats.sampleCount;
  }

  return maxSamples;
}
