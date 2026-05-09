#pragma once

#include <stdint.h>

enum class InjectionMode : uint8_t {
  Off = 0,
  BoostOnly,
  Prime
};

struct MixRatio {
  uint8_t waterPercent{50};
  uint8_t methPercent{50};
};

inline bool isValidRatio(const MixRatio &ratio) {
  return static_cast<uint16_t>(ratio.waterPercent) + static_cast<uint16_t>(ratio.methPercent) == 100;
}

inline float ratioTrim(const MixRatio &ratio) {
  if (!isValidRatio(ratio)) {
    return 1.0f;
  }

  constexpr float kBaselineWaterPercent = 50.0f;
  constexpr float kTrimSlopeDivisor = 500.0f;

  // Placeholder trim hook so ratio modes can influence commanded output.
  // Final trim strategy should be based on nozzle sizing and calibration data.
  return 1.0f + (static_cast<float>(ratio.waterPercent) - kBaselineWaterPercent) / kTrimSlopeDivisor;
}
