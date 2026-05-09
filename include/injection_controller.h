#pragma once

#include "actuators.h"
#include "app_config.h"
#include "sensors.h"

class InjectionController {
public:
  PumpCommand update(const SensorReadings &readings, const AppConfig &config) const;
};
