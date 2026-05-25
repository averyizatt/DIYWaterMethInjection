#pragma once

#include <Arduino.h>

#include "app_config.h"
#include "knock_monitor.h"

class KnockUi {
public:
  static void printHelp(Stream &out);
  static void printSummary(Stream &out, const KnockConfig &config, const KnockStateSnapshot &state);
  static bool handleCommand(const String &line, KnockConfig &config, KnockMonitor &monitor, Stream &out);
};
