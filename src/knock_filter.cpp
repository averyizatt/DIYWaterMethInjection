#include "knock_filter.h"
#include "app_config.h"

#include <math.h>

namespace {
constexpr float kPi = 3.14159265358979323846f;

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}
} // namespace

void KnockFilterPipeline::configure(const KnockFilterConfig &config) {
  config_ = config;
  config_.sampleRateHz = clampFloat(config_.sampleRateHz,
                                    static_cast<float>(knock_sensor_specs::kMinSampleRateHz),
                                    static_cast<float>(knock_sensor_specs::kMaxSampleRateHz));
  const float maxCenterHz = (config_.sampleRateHz * knock_sensor_specs::kNyquistSafetyFactor) <
                                    knock_sensor_specs::kFreqMaxHz
                                ? (config_.sampleRateHz * knock_sensor_specs::kNyquistSafetyFactor)
                                : knock_sensor_specs::kFreqMaxHz;
  config_.centerFreqHz =
      clampFloat(config_.centerFreqHz, knock_sensor_specs::kFreqMinHz, maxCenterHz);
  config_.bandwidthHz = clampFloat(config_.bandwidthHz, 300.0f, config_.sampleRateHz * 0.35f);
  config_.biasAlpha = clampFloat(config_.biasAlpha, 0.0001f, 0.05f);
  config_.envelopeAlpha = clampFloat(config_.envelopeAlpha, 0.01f, 0.8f);
  config_.rmsAlpha = clampFloat(config_.rmsAlpha, 0.005f, 0.6f);
  config_.signalGain = clampFloat(config_.signalGain, 0.05f, 20.0f);

  updateCoefficients();
}

void KnockFilterPipeline::reset() {
  z1_ = 0.0f;
  z2_ = 0.0f;
  rawSample_ = 0.0f;
  centered_ = 0.0f;
  bandpassed_ = 0.0f;
  envelope_ = 0.0f;
  rmsSq_ = 0.0f;
  rms_ = 0.0f;
}

void KnockFilterPipeline::updateCoefficients() {
  const float q = clampFloat(config_.centerFreqHz / config_.bandwidthHz, 0.3f, 12.0f);
  const float w0 = 2.0f * kPi * (config_.centerFreqHz / config_.sampleRateHz);
  const float sw = sinf(w0);
  const float cw = cosf(w0);
  const float alpha = sw / (2.0f * q);

  const float a0 = 1.0f + alpha;
  b0_ = alpha / a0;
  b1_ = 0.0f;
  b2_ = -alpha / a0;
  a1_ = (-2.0f * cw) / a0;
  a2_ = (1.0f - alpha) / a0;
}

float KnockFilterPipeline::processSample(float rawAdcSample) {
  rawSample_ = rawAdcSample;

  // Track DC center slowly to remove bias around ~1.65 V midpoint.
  bias_ += config_.biasAlpha * (rawSample_ - bias_);
  centered_ = (rawSample_ - bias_) * config_.signalGain;

  // Direct Form II transposed biquad bandpass.
  const float y = (b0_ * centered_) + z1_;
  z1_ = (b1_ * centered_) - (a1_ * y) + z2_;
  z2_ = (b2_ * centered_) - (a2_ * y);
  bandpassed_ = y;

  const float rectified = fabsf(bandpassed_);
  envelope_ += config_.envelopeAlpha * (rectified - envelope_);

  const float energy = bandpassed_ * bandpassed_;
  rmsSq_ += config_.rmsAlpha * (energy - rmsSq_);
  if (rmsSq_ < 0.0f) rmsSq_ = 0.0f;
  rms_ = sqrtf(rmsSq_);

  return rms_;
}
