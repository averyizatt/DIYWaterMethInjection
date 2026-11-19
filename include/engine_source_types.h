#pragma once

// The ECU table's types (ecu_profiles.h is generated from the Frogdash dash's profiles by
// tools/ecu_profiles.py). Pure data, no Arduino calls.
#include <stdint.h>

namespace engine {

enum class Field : uint8_t { Rpm, MapKpa, BaroKpa, IatC };
// Where the bits are: one byte, or big-endian (Motorola) / little-endian (Intel) words, or a float.
enum class Layout : uint8_t { U8, S8, U16Be, S16Be, U16Le, S16Le, U32Be, S32Be, U32Le, S32Le, F32Be, F32Le };
// What the value is in after scale and offset, when it is not the field's own unit.
enum class Convert : uint8_t { None, DegFTenthsToC, DegFToC, KelvinToC, PsiToKpa, BarToKpa };

struct Signal {
  uint32_t id;       // The frame's ID, or its offset from the ECU's base ID when relative
  bool relative;
  Field field;
  Layout layout;
  uint8_t byte;      // First byte, from 0
  double scale;     // double, so a scale such as 1/30 divides exactly as the dash's decoder does
  double offset;
  Convert convert;
};

struct Profile {
  const char *id;          // The dash's profile name (megasquirt, haltech...)
  const char *label;
  uint32_t bitrate;
  bool extended;           // 29-bit IDs
  uint32_t idMask;         // Non-zero: IDs are matched after AND with this (Holley's serial bits)
  uint32_t defaultBase;    // For relative IDs: the ECU's default base ID (set in its software)
  const char *status;      // How far the dash's profile has been checked: car, simulated, documented
  const Signal *signals;
  uint8_t count;
};

} // namespace engine
