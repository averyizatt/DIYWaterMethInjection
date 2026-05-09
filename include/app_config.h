#pragma once

#include "modes.h"

struct BoostThresholds {
  float startKpa{130.0f};
  float fullKpa{180.0f};
};

struct AppConfig {
  InjectionMode mode{InjectionMode::BoostOnly};
  MixRatio ratio{50, 50};
  BoostThresholds boost{};
  uint8_t pwmMin{50};
  uint8_t pwmMax{255};
  uint32_t loopPeriodMs{20};
};

AppConfig defaultConfig();
