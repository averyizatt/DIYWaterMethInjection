#pragma once

#include <stdint.h>

enum class InjectionMode : uint8_t {
  Off = 0,
  BoostOnly,
  Prime
};
