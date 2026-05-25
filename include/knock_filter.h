#pragma once

#include <stdint.h>

struct KnockFilterConfig {
  float sampleRateHz{8000.0f};
  float centerFreqHz{6500.0f};
  float bandwidthHz{1800.0f};
  float biasAlpha{0.002f};
  float envelopeAlpha{0.20f};
  float rmsAlpha{0.12f};
  float signalGain{1.0f};
};

class KnockFilterPipeline {
public:
  void configure(const KnockFilterConfig &config);
  void reset();

  // Process a raw ADC sample and return the current RMS-like knock level.
  float processSample(float rawAdcSample);

  float rawSample() const { return rawSample_; }
  float bias() const { return bias_; }
  float centered() const { return centered_; }
  float bandpassed() const { return bandpassed_; }
  float envelope() const { return envelope_; }
  float rms() const { return rms_; }

private:
  void updateCoefficients();

  KnockFilterConfig config_{};

  // Biquad state
  float b0_{0.0f};
  float b1_{0.0f};
  float b2_{0.0f};
  float a1_{0.0f};
  float a2_{0.0f};
  float z1_{0.0f};
  float z2_{0.0f};

  float rawSample_{0.0f};
  float bias_{2048.0f};
  float centered_{0.0f};
  float bandpassed_{0.0f};
  float envelope_{0.0f};
  float rmsSq_{0.0f};
  float rms_{0.0f};
};
