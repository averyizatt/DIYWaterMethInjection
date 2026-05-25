#pragma once

#include <stdint.h>

struct KnockSamplerStats {
  uint16_t minAdc{4095};
  uint16_t maxAdc{0};
  uint16_t clipLowCount{0};
  uint16_t clipHighCount{0};
  uint16_t sampleCount{0};
};

class KnockSampler {
public:
  void begin(int adcPin, uint16_t sampleRateHz);
  void setSampleRate(uint16_t sampleRateHz);

  // Captures up to maxSamples ADC samples into outSamples.
  // Returns captured sample count.
  uint8_t captureBlock(uint16_t *outSamples, uint8_t maxSamples,
                       uint16_t clipLowAdc, uint16_t clipHighAdc,
                       KnockSamplerStats &stats) const;

  int pin() const { return adcPin_; }
  uint16_t sampleRateHz() const { return sampleRateHz_; }

private:
  int adcPin_{-1};
  uint16_t sampleRateHz_{8000};
};
