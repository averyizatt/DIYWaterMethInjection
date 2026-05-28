#include <stdint.h>

#include <cmath>
#include <random>

#include "knock_detector_logic.h"

namespace {
constexpr float kPi = 3.14159265358979323846f;

void fillConstant(uint16_t *buffer, uint8_t count, uint16_t value) {
  for (uint8_t i = 0; i < count; ++i) {
    buffer[i] = value;
  }
}

void fillSine(uint16_t *buffer,
              uint8_t count,
              float sampleRateHz,
              float freqHz,
              float amplitude,
              float phase,
              float dc = 2048.0f) {
  for (uint8_t i = 0; i < count; ++i) {
    const float t = static_cast<float>(i) / sampleRateHz;
    const float sample = dc + (amplitude * sinf((2.0f * kPi * freqHz * t) + phase));
    float clipped = sample;
    if (clipped < 0.0f) clipped = 0.0f;
    if (clipped > 4095.0f) clipped = 4095.0f;
    buffer[i] = static_cast<uint16_t>(clipped);
  }
}

KnockDetectorConfig testConfig() {
  KnockDetectorConfig cfg{};
  cfg.enabled = true;
  cfg.minRpmToArm = 2500;
  cfg.minMapKpaToArm = 140.0f;
  cfg.minLoadPercentToArm = 35.0f;
  cfg.eventCooldownMs = 100;
  cfg.autoCenterFromBore = false;
  cfg.centerFreqHz = 3200.0f;
  cfg.bandwidthHz = 1200.0f;
  cfg.multiBandSpread = 0.14f;
  cfg.fftEnabled = false;
  cfg.thresholdOffset = 4.0f;
  cfg.thresholdMultiplier = 2.5f;
  cfg.baselineLearningEnabled = true;
  cfg.baselineLearnAlpha = 0.03f;
  cfg.signalGain = 1.0f;
  cfg.biasAlpha = 0.002f;
  cfg.envelopeAlpha = 0.2f;
  cfg.rmsAlpha = 0.12f;
  cfg.sampleRateHz = 8000.0f;
  cfg.clipLowAdc = 5;
  cfg.clipHighAdc = 4090;
  cfg.clipPercentForFault = 40;
  cfg.stuckAdcDelta = 2;
  cfg.faultHoldMs = 900;
  cfg.missingSignalRms = 0.3f;
  cfg.warningThresholdCount = 2;
  cfg.criticalThresholdCount = 4;
  return cfg;
}

bool noSignal_noEvents() {
  KnockDetectorLogic logic;
  auto cfg = testConfig();
  logic.configure(cfg);
  logic.reset();

  uint16_t samples[64];
  fillConstant(samples, 64, 2048);

  uint32_t now = 0;
  uint8_t totalEvents = 0;
  for (int i = 0; i < 60; ++i) {
    now += 20;
    const auto frame = logic.processBlock(samples, 64, 70.0f, 160.0f, 0.0f, 35.0f, 45.0f,
                        now, 2048, 2048, 0, 0);
    totalEvents = frame.eventCount;
    if (frame.detected) return false;
  }
  return totalEvents == 0;
}

bool normalVibrationBelowThreshold_noEvents() {
  KnockDetectorLogic logic;
  auto cfg = testConfig();
  logic.configure(cfg);
  logic.reset();

  uint16_t samples[64];
  uint32_t now = 0;

  for (int i = 0; i < 30; ++i) {
    now += 20;
    fillSine(samples, 64, cfg.sampleRateHz, 3200.0f, 8.0f, static_cast<float>(i));
    logic.processBlock(samples, 64, 20.0f, 110.0f, 0.0f, 35.0f, 45.0f,
               now, 2030, 2066, 0, 0);
  }

  for (int i = 0; i < 30; ++i) {
    now += 20;
    fillSine(samples, 64, cfg.sampleRateHz, 3200.0f, 10.0f, static_cast<float>(i));
    const auto frame = logic.processBlock(samples, 64, 65.0f, 170.0f, 0.0f, 35.0f, 45.0f,
                        now, 2028, 2068, 0, 0);
    if (frame.detected) return false;
  }

  return true;
}

bool randomNoise_noEvents() {
  KnockDetectorLogic logic;
  auto cfg = testConfig();
  logic.configure(cfg);
  logic.reset();

  std::mt19937 rng(42U);
  std::uniform_int_distribution<int> noise(-10, 10);

  uint16_t samples[64];
  uint32_t now = 0;

  for (int i = 0; i < 80; ++i) {
    for (uint8_t s = 0; s < 64; ++s) {
      const int value = 2048 + noise(rng);
      samples[s] = static_cast<uint16_t>(value < 0 ? 0 : (value > 4095 ? 4095 : value));
    }
    now += 20;
    const auto frame = logic.processBlock(samples, 64, 75.0f, 180.0f, 0.0f, 35.0f, 45.0f,
                        now, 2030, 2066, 0, 0);
    if (frame.detected) return false;
  }

  return true;
}

bool targetFreqBurst_triggersWhenArmed() {
  KnockDetectorLogic logic;
  auto cfg = testConfig();
  cfg.thresholdMultiplier = 2.0f;
  cfg.thresholdOffset = 3.0f;
  logic.configure(cfg);
  logic.reset();

  uint16_t samples[64];
  uint32_t now = 0;

  for (int i = 0; i < 35; ++i) {
    now += 20;
    fillSine(samples, 64, cfg.sampleRateHz, 3200.0f, 8.0f, static_cast<float>(i));
    logic.processBlock(samples, 64, 20.0f, 110.0f, 0.0f, 35.0f, 45.0f,
               now, 2030, 2066, 0, 0);
  }

  bool detected = false;
  for (int i = 0; i < 8; ++i) {
    now += 20;
    fillSine(samples, 64, cfg.sampleRateHz, 3200.0f, 80.0f, static_cast<float>(i));
    const auto frame = logic.processBlock(samples, 64, 78.0f, 180.0f, 0.0f, 35.0f, 45.0f,
                        now, 1960, 2140, 0, 0);
    if (frame.detected) {
      detected = true;
      break;
    }
  }

  return detected;
}

bool wrongFreqBurst_isRejected() {
  KnockDetectorLogic logic;
  auto cfg = testConfig();
  cfg.thresholdMultiplier = 2.8f;
  logic.configure(cfg);
  logic.reset();

  uint16_t samples[64];
  uint32_t now = 0;

  for (int i = 0; i < 35; ++i) {
    now += 20;
    fillSine(samples, 64, cfg.sampleRateHz, 3200.0f, 8.0f, static_cast<float>(i));
    logic.processBlock(samples, 64, 20.0f, 110.0f, 0.0f, 35.0f, 45.0f,
               now, 2030, 2066, 0, 0);
  }

  for (int i = 0; i < 12; ++i) {
    now += 20;
    fillSine(samples, 64, cfg.sampleRateHz, 900.0f, 90.0f, static_cast<float>(i));
    const auto frame = logic.processBlock(samples, 64, 78.0f, 180.0f, 0.0f, 35.0f, 45.0f,
                        now, 1950, 2150, 0, 0);
    if (frame.detected) return false;
  }

  return true;
}

bool changingBias_noFalseEvents() {
  KnockDetectorLogic logic;
  auto cfg = testConfig();
  logic.configure(cfg);
  logic.reset();

  uint16_t samples[64];
  uint32_t now = 0;

  for (int i = 0; i < 70; ++i) {
    const float dc = 1900.0f + (static_cast<float>(i) * 4.0f);
    fillSine(samples, 64, cfg.sampleRateHz, 3200.0f, 8.0f, static_cast<float>(i), dc);
    now += 20;
    const uint16_t minAdc = static_cast<uint16_t>(dc - 20.0f);
    const uint16_t maxAdc = static_cast<uint16_t>(dc + 20.0f);
    const auto frame = logic.processBlock(samples, 64, 70.0f, 180.0f, 0.0f, 35.0f, 45.0f,
                        now, minAdc, maxAdc, 0, 0);
    if (frame.detected) return false;
  }

  return true;
}

bool knockBelowArmThreshold_doesNotTrigger() {
  KnockDetectorLogic logic;
  auto cfg = testConfig();
  cfg.thresholdMultiplier = 1.8f;
  cfg.thresholdOffset = 2.0f;
  logic.configure(cfg);
  logic.reset();

  uint16_t samples[64];
  uint32_t now = 0;

  for (int i = 0; i < 40; ++i) {
    now += 20;
    fillSine(samples, 64, cfg.sampleRateHz, 3200.0f, 70.0f, static_cast<float>(i));
    const auto frame = logic.processBlock(samples, 64, 22.0f, 120.0f, 0.0f, 35.0f, 45.0f,
                        now, 1970, 2130, 0, 0);
    if (frame.detected || frame.armed) return false;
  }

  return true;
}

bool knockWhileArmed_triggers() {
  KnockDetectorLogic logic;
  auto cfg = testConfig();
  cfg.thresholdMultiplier = 1.8f;
  cfg.thresholdOffset = 2.0f;
  logic.configure(cfg);
  logic.reset();

  uint16_t samples[64];
  uint32_t now = 0;

  for (int i = 0; i < 35; ++i) {
    now += 20;
    fillSine(samples, 64, cfg.sampleRateHz, 3200.0f, 8.0f, static_cast<float>(i));
    logic.processBlock(samples, 64, 18.0f, 110.0f, 0.0f, 35.0f, 45.0f,
               now, 2030, 2066, 0, 0);
  }

  for (int i = 0; i < 12; ++i) {
    now += 20;
    fillSine(samples, 64, cfg.sampleRateHz, 3200.0f, 80.0f, static_cast<float>(i));
    const auto frame = logic.processBlock(samples, 64, 85.0f, 190.0f, 0.0f, 35.0f, 45.0f,
                        now, 1960, 2140, 0, 0);
    if (frame.detected && frame.armed) return true;
  }

  return false;
}
} // namespace

int main() {
  int failed = 0;

  failed += noSignal_noEvents() ? 0 : 1;
  failed += normalVibrationBelowThreshold_noEvents() ? 0 : 1;
  failed += randomNoise_noEvents() ? 0 : 1;
  failed += targetFreqBurst_triggersWhenArmed() ? 0 : 1;
  failed += wrongFreqBurst_isRejected() ? 0 : 1;
  failed += changingBias_noFalseEvents() ? 0 : 1;
  failed += knockBelowArmThreshold_doesNotTrigger() ? 0 : 1;
  failed += knockWhileArmed_triggers() ? 0 : 1;

  return failed == 0 ? 0 : 1;
}
