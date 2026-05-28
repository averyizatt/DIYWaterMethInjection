#include "knock_detector_logic.h"

#include <math.h>

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr uint32_t kBaselineLearnMinSamples = 400;
constexpr uint8_t kLoadBins = 8;

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

uint8_t loadBin(float loadPercent) {
  const float clamped = clampFloat(loadPercent, 0.0f, 100.0f);
  const uint8_t bin = static_cast<uint8_t>(clamped / (100.0f / kLoadBins));
  return bin >= kLoadBins ? kLoadBins - 1 : bin;
}

void computeFftSNR(const uint16_t *samples,
                   uint8_t sampleCount,
                   uint8_t startIndex,
                   uint8_t windowCount,
                   float sampleRateHz,
                   float targetHz,
                   float bandwidthHz,
                   float harmonicWeight,
                   float &targetMagOut,
                   float &noiseMagOut,
                   float &harmonicScoreOut,
                   float &snrDbOut) {
  targetMagOut = 0.0f;
  noiseMagOut = 0.0f;
  harmonicScoreOut = 0.0f;
  snrDbOut = -40.0f;
  if (samples == nullptr || sampleCount < 16 || windowCount < 16 || sampleRateHz < 2000.0f) return;

  float mean = 0.0f;
  for (uint8_t n = 0; n < windowCount; ++n) {
    const uint8_t idx = static_cast<uint8_t>(startIndex + n);
    mean += static_cast<float>(samples[idx]);
  }
  mean /= static_cast<float>(windowCount);

  uint16_t targetBins = 0;
  uint16_t harmonicBins = 0;
  uint16_t noiseBins = 0;
  const float bwHalf = clampFloat(bandwidthHz * 0.5f, 200.0f, sampleRateHz * 0.25f);
  const float harmonicHz = targetHz * 2.0f;
  const float minNoiseHz = 1000.0f;
  const float maxNoiseHz = sampleRateHz * 0.45f;
  float harmonicMag = 0.0f;

  for (uint8_t k = 1; k < (windowCount / 2); ++k) {
    const float freqHz = (sampleRateHz * static_cast<float>(k)) / static_cast<float>(windowCount);
    float real = 0.0f;
    float imag = 0.0f;

    for (uint8_t n = 0; n < windowCount; ++n) {
      const uint8_t idx = static_cast<uint8_t>(startIndex + n);
      const float phase = (2.0f * kPi * static_cast<float>(k) * static_cast<float>(n)) /
                          static_cast<float>(windowCount);
      const float window = 0.5f - (0.5f * cosf((2.0f * kPi * static_cast<float>(n)) /
                                               static_cast<float>(windowCount - 1)));
      const float x = (static_cast<float>(samples[idx]) - mean) * window;
      real += x * cosf(phase);
      imag -= x * sinf(phase);
    }

    const float mag = sqrtf((real * real) + (imag * imag)) / static_cast<float>(sampleCount);
    if (fabsf(freqHz - targetHz) <= bwHalf) {
      targetMagOut += mag;
      ++targetBins;
      continue;
    }

    if (harmonicHz <= maxNoiseHz && fabsf(freqHz - harmonicHz) <= bwHalf) {
      harmonicMag += mag;
      ++harmonicBins;
      continue;
    }

    if (freqHz >= minNoiseHz && freqHz <= maxNoiseHz) {
      noiseMagOut += mag;
      ++noiseBins;
    }
  }

  if (targetBins > 0) targetMagOut /= static_cast<float>(targetBins);
  if (harmonicBins > 0) harmonicMag /= static_cast<float>(harmonicBins);
  if (noiseBins > 0) noiseMagOut /= static_cast<float>(noiseBins);
  harmonicScoreOut = clampFloat(harmonicMag / (targetMagOut + 0.001f), 0.0f, 1.5f);

  const float combinedTarget = targetMagOut + (clampFloat(harmonicWeight, 0.0f, 1.0f) * harmonicMag);
  const float denom = noiseMagOut + 0.001f;
  const float ratio = combinedTarget / denom;
  snrDbOut = 20.0f * log10f(ratio + 0.001f);
}

float confidenceFromSnr(float snrDb) {
  return clampFloat((snrDb + 6.0f) / 18.0f, 0.0f, 1.0f);
}

KnockOperatingProfile detectProfile(float mapKpa,
                                    float loadPercent,
                                    float mapRateKpaPerSec,
                                    float iatC,
                                    float bayC,
                                    const KnockDetectorConfig &cfg) {
  if (iatC > (cfg.iatTempCompStartC + 30.0f) || bayC > (cfg.bayTempCompStartC + 35.0f)) {
    return KnockOperatingProfile::HeatSoak;
  }
  if (loadPercent < 15.0f && mapKpa < (cfg.minMapKpaToArm * 0.85f)) {
    return KnockOperatingProfile::Idle;
  }
  if (mapRateKpaPerSec > (cfg.mapRateGateKpaPerSec * 0.5f) && loadPercent > 20.0f) {
    return KnockOperatingProfile::Spool;
  }
  if (mapRateKpaPerSec < (-cfg.mapRateGateKpaPerSec * 0.5f)) {
    return KnockOperatingProfile::Lift;
  }
  return KnockOperatingProfile::SteadyLoad;
}

float profileScaleFor(KnockOperatingProfile profile, const KnockDetectorConfig &cfg) {
  switch (profile) {
  case KnockOperatingProfile::Idle:
    return cfg.profileScaleIdle;
  case KnockOperatingProfile::Spool:
    return cfg.profileScaleSpool;
  case KnockOperatingProfile::Lift:
    return cfg.profileScaleLift;
  case KnockOperatingProfile::HeatSoak:
    return cfg.profileScaleHeatSoak;
  case KnockOperatingProfile::SteadyLoad:
  default:
    return cfg.profileScaleSteady;
  }
}

float temperatureScaleFor(float iatC, float bayC, const KnockDetectorConfig &cfg) {
  const float iatDelta = iatC - cfg.iatTempCompStartC;
  const float bayDelta = bayC - cfg.bayTempCompStartC;
  const float iatBoost = iatDelta > 0.0f ? iatDelta * cfg.iatTempCompPerC : 0.0f;
  const float bayBoost = bayDelta > 0.0f ? bayDelta * cfg.bayTempCompPerC : 0.0f;
  return clampFloat(1.0f + iatBoost + bayBoost, 1.0f, cfg.maxTempCompScale);
}
} // namespace

void KnockDetectorLogic::configure(const KnockDetectorConfig &config) {
  config_ = config;

  const float baseCenter = config_.autoCenterFromBore ? estimateKnockCenterHz(config_.boreMm)
                                                       : config_.centerFreqHz;
  const float spread = clampFloat(config_.multiBandSpread, 0.05f, 0.35f);
  centerMidHz_ = baseCenter;
  centerLowHz_ = baseCenter * (1.0f - spread);
  centerHighHz_ = baseCenter * (1.0f + spread);

  KnockFilterConfig filterCfg{};
  filterCfg.sampleRateHz = config_.sampleRateHz;
  filterCfg.bandwidthHz = config_.bandwidthHz;
  filterCfg.biasAlpha = config_.biasAlpha;
  filterCfg.envelopeAlpha = config_.envelopeAlpha;
  filterCfg.rmsAlpha = config_.rmsAlpha;
  filterCfg.signalGain = config_.signalGain;

  filterCfg.centerFreqHz = centerLowHz_;
  filterLow_.configure(filterCfg);
  filterCfg.centerFreqHz = centerMidHz_;
  filterMid_.configure(filterCfg);
  filterCfg.centerFreqHz = centerHighHz_;
  filterHigh_.configure(filterCfg);
}

void KnockDetectorLogic::reset() {
  for (uint8_t i = 0; i < kLoadBins; ++i) {
    baselineMap_[i] = 0.0f;
    longBaselineMap_[i] = 0.0f;
    baselineMapSamples_[i] = 0;
    spectralTemplateMap_[i] = 0.0f;
    spectralTemplateSamples_[i] = 0;
    centerBiasMap_[i] = 0.0f;
    adaptiveThresholdMultMap_[i] = config_.thresholdMultiplier;
  }
  fallbackBaseline_ = 0.0f;
  healthScore_ = 100.0f;
  degradeMode_ = KnockDegradeMode::Full;
  transientHoldUntilMs_ = 0;
  lastEventMs_ = 0;
  lowSignalSinceMs_ = 0;
  stuckSinceMs_ = 0;
  totalBaselineSamples_ = 0;
  eventWindow_ = 0;
  eventCount_ = 0;
  filterLow_.reset();
  filterMid_.reset();
  filterHigh_.reset();
}

KnockDetectorFrame KnockDetectorLogic::processBlock(const uint16_t *samples,
                                                    uint8_t sampleCount,
                                                    float loadPercent,
                                                    float mapKpa,
                                                    float mapRateKpaPerSec,
                                                    float iatC,
                                                    float bayC,
                                                    uint32_t nowMs,
                                                    uint16_t minAdc,
                                                    uint16_t maxAdc,
                                                    uint16_t clipLowCount,
                                                    uint16_t clipHighCount) {
  KnockDetectorFrame frame{};
  if (samples == nullptr || sampleCount == 0) {
    frame.sensorFault = true;
    return frame;
  }

  frame.loadPercent = clampFloat(loadPercent, 0.0f, 100.0f);
  frame.mapRateKpaPerSec = mapRateKpaPerSec;
  frame.iatC = iatC;
  frame.bayC = bayC;
  frame.profile = detectProfile(mapKpa, frame.loadPercent, mapRateKpaPerSec, iatC, bayC, config_);
  frame.profileScale = clampFloat(profileScaleFor(frame.profile, config_), 0.8f, 1.8f);
  frame.tempScale = temperatureScaleFor(iatC, bayC, config_);

  for (uint8_t i = 0; i < sampleCount; ++i) {
    const float sample = static_cast<float>(samples[i]);
    filterLow_.processSample(sample);
    filterMid_.processSample(sample);
    filterHigh_.processSample(sample);
  }

  frame.rawAdc = filterMid_.rawSample();
  frame.biasAdc = filterMid_.bias();
  frame.centered = filterMid_.centered();
  frame.filtered = filterMid_.bandpassed();
  frame.envelope = filterMid_.envelope();

  frame.lowBandRms = filterLow_.rms();
  frame.midBandRms = filterMid_.rms();
  frame.highBandRms = filterHigh_.rms();

  float selectedRms = frame.midBandRms;
  frame.selectedCenterHz = centerMidHz_;
  if (frame.lowBandRms > selectedRms) {
    selectedRms = frame.lowBandRms;
    frame.selectedCenterHz = centerLowHz_;
  }
  if (frame.highBandRms > selectedRms) {
    selectedRms = frame.highBandRms;
    frame.selectedCenterHz = centerHighHz_;
  }

  const uint8_t bin = loadBin(frame.loadPercent);
  const float bias = clampFloat(centerBiasMap_[bin], -1200.0f, 1200.0f);
  frame.expectedCenterHz = clampFloat(centerMidHz_ + bias, centerLowHz_ * 0.90f, centerHighHz_ * 1.10f);
  frame.rms = selectedRms;
  const float bandSum = frame.lowBandRms + frame.midBandRms + frame.highBandRms + 0.0001f;
  const float rawBandConfidence = clampFloat(selectedRms / bandSum, 0.0f, 1.0f);
  frame.spectralConfidence = rawBandConfidence;

  if (config_.fftEnabled) {
    const uint8_t longCount = sampleCount;
    uint8_t shortCount = sampleCount / 2;
    if (shortCount < 16) shortCount = sampleCount;
    const uint8_t shortStart = static_cast<uint8_t>(sampleCount - shortCount);

    float targetLong = 0.0f;
    float noiseLong = 0.0f;
    float harmonicLong = 0.0f;
    computeFftSNR(samples,
                  sampleCount,
                  0,
                  longCount,
                  config_.sampleRateHz,
                  frame.expectedCenterHz,
                  config_.bandwidthHz,
                  config_.fftHarmonicWeight,
                  targetLong,
                  noiseLong,
                  harmonicLong,
                  frame.fftLongSnrDb);

    float targetShort = 0.0f;
    float noiseShort = 0.0f;
    float harmonicShort = 0.0f;
    computeFftSNR(samples,
                  sampleCount,
                  shortStart,
                  shortCount,
                  config_.sampleRateHz,
                  frame.expectedCenterHz,
                  config_.bandwidthHz,
                  config_.fftHarmonicWeight,
                  targetShort,
                  noiseShort,
                  harmonicShort,
                  frame.fftShortSnrDb);

    frame.fftTargetMag = (targetLong + targetShort) * 0.5f;
    frame.fftNoiseMag = (noiseLong + noiseShort) * 0.5f;
    frame.harmonicScore = clampFloat((harmonicLong + harmonicShort) * 0.5f, 0.0f, 1.5f);

    const float longConf = confidenceFromSnr(frame.fftLongSnrDb);
    const float shortConf = confidenceFromSnr(frame.fftShortSnrDb);
    const float shortW = clampFloat(config_.fftShortWeight, 0.0f, 1.0f);
    const float fftConfidence = ((1.0f - shortW) * longConf) + (shortW * shortConf);
    frame.fftSnrDb = ((1.0f - shortW) * frame.fftLongSnrDb) + (shortW * frame.fftShortSnrDb);

    frame.spectralConfidence = clampFloat(
        ((1.0f - config_.fftWeight) * frame.spectralConfidence) + (config_.fftWeight * fftConfidence),
        0.0f, 1.0f);
  }

  const float clippedPercent = static_cast<float>(clipLowCount + clipHighCount) * 100.0f /
                               static_cast<float>(sampleCount);
  frame.clippingDetected = clippedPercent >= static_cast<float>(config_.clipPercentForFault);

  const bool stuckNow = static_cast<uint16_t>(maxAdc - minAdc) <= config_.stuckAdcDelta;
  if (stuckNow) {
    if (stuckSinceMs_ == 0) stuckSinceMs_ = nowMs;
  } else {
    stuckSinceMs_ = 0;
  }

  const bool missingNow = frame.rms < config_.missingSignalRms;
  if (missingNow) {
    if (lowSignalSinceMs_ == 0) lowSignalSinceMs_ = nowMs;
  } else {
    lowSignalSinceMs_ = 0;
  }

  const bool stuckFault = (stuckSinceMs_ != 0) && ((nowMs - stuckSinceMs_) >= config_.faultHoldMs);
  const bool missingFault = (lowSignalSinceMs_ != 0) && ((nowMs - lowSignalSinceMs_) >= config_.faultHoldMs);
  frame.sensorFault = frame.clippingDetected || stuckFault || missingFault;

  frame.armed = config_.enabled && !frame.sensorFault &&
                (mapKpa >= config_.minMapKpaToArm) &&
                (frame.loadPercent >= config_.minLoadPercentToArm);

  if (spectralTemplateMap_[bin] <= 0.0f) {
    spectralTemplateMap_[bin] = rawBandConfidence;
  }
  frame.templateDeviation = fabsf(rawBandConfidence - spectralTemplateMap_[bin]);
  const float templatePenalty = clampFloat(frame.templateDeviation / 0.35f, 0.0f, 1.0f);
  const float templateW = clampFloat(config_.spectralTemplateWeight, 0.0f, 1.0f);
  frame.spectralConfidence = clampFloat(frame.spectralConfidence * (1.0f - (templateW * templatePenalty)),
                                        0.0f, 1.0f);

  if (fallbackBaseline_ <= 0.0f) fallbackBaseline_ = frame.rms;
  if (baselineMap_[bin] <= 0.0f) baselineMap_[bin] = fallbackBaseline_;
  if (longBaselineMap_[bin] <= 0.0f) longBaselineMap_[bin] = baselineMap_[bin];

  frame.shortBaseline = baselineMap_[bin];
  frame.longBaseline = longBaselineMap_[bin];

  const bool canLearn = config_.baselineLearningEnabled && !frame.sensorFault &&
                        (frame.spectralConfidence < 0.65f || frame.rms < (baselineMap_[bin] * 1.5f));
  if (canLearn) {
    baselineMap_[bin] += config_.baselineLearnAlpha * (frame.rms - baselineMap_[bin]);
    longBaselineMap_[bin] += config_.longTermBaselineAlpha * (frame.rms - longBaselineMap_[bin]);
    spectralTemplateMap_[bin] += config_.baselineLearnAlpha * (rawBandConfidence - spectralTemplateMap_[bin]);
    fallbackBaseline_ += config_.baselineLearnAlpha * 0.25f * (frame.rms - fallbackBaseline_);
    centerBiasMap_[bin] += config_.baselineLearnAlpha * 0.15f * (frame.selectedCenterHz - frame.expectedCenterHz);
    centerBiasMap_[bin] = clampFloat(centerBiasMap_[bin], -1200.0f, 1200.0f);
    if (baselineMapSamples_[bin] < 65535) ++baselineMapSamples_[bin];
    if (spectralTemplateSamples_[bin] < 65535) ++spectralTemplateSamples_[bin];
    ++totalBaselineSamples_;
  }

  frame.baselineLearned = totalBaselineSamples_ >= kBaselineLearnMinSamples;
  const bool binLearned = baselineMapSamples_[bin] >= 8;
  frame.shortBaseline = binLearned ? baselineMap_[bin] : fallbackBaseline_;
  frame.longBaseline = longBaselineMap_[bin] > 0.0f ? longBaselineMap_[bin] : frame.shortBaseline;

  const float driftDen = frame.longBaseline + 0.001f;
  frame.driftPercent = fabsf(frame.shortBaseline - frame.longBaseline) * 100.0f / driftDen;

  const float driftRatio = frame.driftPercent / 100.0f;
  adaptiveThresholdMultMap_[bin] += config_.adaptiveMultLearnAlpha * (driftRatio * 1.2f - 0.25f);
  adaptiveThresholdMultMap_[bin] = clampFloat(adaptiveThresholdMultMap_[bin],
                                              config_.adaptiveMultMin,
                                              config_.adaptiveMultMax);
  frame.adaptiveMultiplier = adaptiveThresholdMultMap_[bin];

  if (fabsf(mapRateKpaPerSec) >= config_.mapRateGateKpaPerSec) {
    transientHoldUntilMs_ = nowMs + config_.transientHoldMs;
  }
  frame.transientScale = (nowMs <= transientHoldUntilMs_) ?
                         clampFloat(config_.transientThresholdScale, 1.0f, 2.5f) : 1.0f;

  const float healthPenalty =
      (frame.sensorFault ? 32.0f : 0.0f) +
      (frame.clippingDetected ? 12.0f : 0.0f) +
      clampFloat((frame.templateDeviation - 0.22f) * 55.0f, 0.0f, 18.0f) +
      clampFloat((frame.driftPercent - config_.driftWarnPercent) * 0.35f, 0.0f, 20.0f);
  const float healthTarget = clampFloat(100.0f - healthPenalty, 8.0f, 100.0f);
  healthScore_ += 0.05f * (healthTarget - healthScore_);
  healthScore_ = clampFloat(healthScore_, 0.0f, 100.0f);

  if (healthScore_ <= config_.failsafeHealthThreshold) {
    degradeMode_ = KnockDegradeMode::Failsafe;
  } else if (healthScore_ <= config_.conservativeHealthThreshold) {
    degradeMode_ = KnockDegradeMode::Conservative;
  } else {
    degradeMode_ = KnockDegradeMode::Full;
  }
  frame.degradeMode = degradeMode_;
  frame.healthScore = healthScore_;

  float degradeScale = 1.0f;
  if (degradeMode_ == KnockDegradeMode::Conservative) degradeScale = 1.20f;
  if (degradeMode_ == KnockDegradeMode::Failsafe) degradeScale = 1.45f;

  frame.baseline = frame.shortBaseline;
  frame.threshold = ((frame.baseline * frame.adaptiveMultiplier) + config_.thresholdOffset) *
                    frame.transientScale * frame.tempScale * frame.profileScale * degradeScale;
  frame.threshold = clampFloat(frame.threshold, config_.thresholdOffset, 5000.0f);

  if (eventWindow_ > 0 && (nowMs - lastEventMs_) > 1000U) {
    --eventWindow_;
  }

  const bool fftGate = !config_.fftEnabled || (frame.fftSnrDb >= config_.fftMinSnrDb);
  const float fftConfidence = config_.fftEnabled ? confidenceFromSnr(frame.fftSnrDb) : frame.spectralConfidence;
  const float harmonicConfidence = clampFloat(frame.harmonicScore / 1.5f, 0.0f, 1.0f);
  const float templateConfidence = 1.0f - clampFloat(frame.templateDeviation / 0.5f, 0.0f, 1.0f);
  const float weightSum = config_.confidenceWeightSpectral +
                          config_.confidenceWeightFft +
                          config_.confidenceWeightHarmonic +
                          config_.confidenceWeightTemplate;
  const float invWeight = weightSum > 0.001f ? 1.0f / weightSum : 1.0f;
  frame.finalConfidence = clampFloat(
      invWeight *
          ((config_.confidenceWeightSpectral * frame.spectralConfidence) +
           (config_.confidenceWeightFft * fftConfidence) +
           (config_.confidenceWeightHarmonic * harmonicConfidence) +
           (config_.confidenceWeightTemplate * templateConfidence)),
      0.0f, 1.0f);

  const float riskMap = clampFloat(fabsf(frame.mapRateKpaPerSec) / (config_.mapRateGateKpaPerSec + 0.001f),
                                   0.0f, 1.0f);
  const float riskEvents = clampFloat(static_cast<float>(eventWindow_) /
                                          static_cast<float>(config_.criticalThresholdCount + 1),
                                      0.0f, 1.0f);
  frame.knockRisk = clampFloat((config_.riskMapRateWeight * riskMap) +
                                   (config_.riskConfidenceWeight * frame.finalConfidence) +
                                   (config_.riskEventWeight * riskEvents),
                               0.0f, 1.0f);

  const bool overThreshold = (frame.rms > frame.threshold) &&
                             (frame.finalConfidence >= config_.minDetectConfidence) &&
                             fftGate &&
                             (degradeMode_ != KnockDegradeMode::Failsafe);
  const bool debounceElapsed = (lastEventMs_ == 0) || ((nowMs - lastEventMs_) >= config_.eventCooldownMs);

  frame.reasonFlags = 0;
  if (!frame.armed) frame.reasonFlags |= REASON_ARM_BLOCKED;
  if (frame.sensorFault) frame.reasonFlags |= REASON_SENSOR_FAULT;
  if (frame.transientScale > 1.01f) frame.reasonFlags |= REASON_TRANSIENT_GATE;
  if (frame.finalConfidence < config_.minDetectConfidence) frame.reasonFlags |= REASON_LOW_CONFIDENCE;
  if (!fftGate) frame.reasonFlags |= REASON_FFT_GATE;
  if (degradeMode_ == KnockDegradeMode::Conservative) frame.reasonFlags |= REASON_DEGRADE_CONSERVATIVE;
  if (degradeMode_ == KnockDegradeMode::Failsafe) frame.reasonFlags |= REASON_DEGRADE_FAILSAFE;
  if (frame.profile == KnockOperatingProfile::HeatSoak) frame.reasonFlags |= REASON_PROFILE_HEATSOAK;

  if (frame.armed && overThreshold && debounceElapsed) {
    frame.detected = true;
    lastEventMs_ = nowMs;
    ++eventCount_;
    if (eventWindow_ < 255 && frame.finalConfidence >= config_.warnConfidence) ++eventWindow_;
  }

  frame.warningActive = (eventWindow_ >= config_.warningThresholdCount) ||
                        (frame.detected && frame.finalConfidence >= config_.warnConfidence);
  frame.criticalActive = ((eventWindow_ >= config_.criticalThresholdCount) ||
                         (frame.detected && frame.finalConfidence >= config_.criticalConfidence)) &&
                         (degradeMode_ != KnockDegradeMode::Failsafe);

  frame.anomalyClass = KnockAnomalyClass::None;
  if (frame.sensorFault) {
    frame.anomalyClass = KnockAnomalyClass::SensorIssue;
  } else if (frame.detected && frame.finalConfidence >= config_.warnConfidence) {
    if (frame.transientScale > 1.01f && frame.finalConfidence < config_.criticalConfidence) {
      frame.anomalyClass = KnockAnomalyClass::MechanicalTransient;
    } else {
      frame.anomalyClass = KnockAnomalyClass::LikelyKnock;
    }
  } else if (frame.rms > (frame.threshold * 1.1f) && frame.finalConfidence < config_.minDetectConfidence) {
    frame.anomalyClass = KnockAnomalyClass::UnknownEnergy;
  }

  frame.eventCount = eventCount_;

  return frame;
}

float KnockDetectorLogic::estimateKnockCenterHz(float boreMm) {
  // Practical estimate: 2.2x of the classic combustion chamber radial mode
  // puts common 4-cylinder bores in the 5.5-8 kHz range.
  const float boreM = clampFloat(boreMm, 60.0f, 120.0f) / 1000.0f;
  const float fundamentalHz = 900.0f / (kPi * boreM);
  return clampFloat(fundamentalHz * 2.2f, 3500.0f, 9000.0f);
}
