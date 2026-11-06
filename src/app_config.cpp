#include "app_config.h"

TankBlend computeTankBlend(float waterLiters, float methLiters) {
  TankBlend blend{};
  blend.waterLiters = waterLiters >= 0.0f ? waterLiters : 0.0f;
  blend.methLiters = methLiters >= 0.0f ? methLiters : 0.0f;
  blend.totalLiters = blend.waterLiters + blend.methLiters;

  if (blend.totalLiters <= 0.0f) {
    blend.methPercent = 0.0f;
    blend.methFraction = 0.0f;
    return blend;
  }

  // Core requested equation: Meth% = Vm / (Vm + Vw) * 100.
  blend.methPercent = (blend.methLiters / blend.totalLiters) * 100.0f;
  blend.methFraction = blend.methPercent / 100.0f;
  return blend;
}

AppConfig defaultConfig() { return AppConfig{}; }
