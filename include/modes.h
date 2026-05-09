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

  constexpr float kDefaultWaterPercent = 50.0f;
  // 500 -> ±10% trim across extreme 0-100 water split (placeholder only).
  constexpr float kTrimScalingFactor = 500.0f;

  // Placeholder trim hook so ratio modes can influence commanded output.
  // Final trim strategy should be based on nozzle sizing and calibration data.
  return 1.0f + (static_cast<float>(ratio.waterPercent) - kDefaultWaterPercent) / kTrimScalingFactor;
}
