#pragma once

#include <stdint.h>

namespace can_protocol {

struct CanFrame {
  uint16_t id{0};
  uint8_t dlc{0};
  uint8_t data[8]{};
};

constexpr uint16_t ID_BLOCK_MASTER_BASE = 0x100;
constexpr uint16_t ID_BLOCK_ENGINE_METH_BASE = 0x300;

constexpr uint16_t ID_ENGINE_METH_STATE = 0x300;
constexpr uint16_t ID_ENGINE_METH_COMMAND = 0x301;
constexpr uint16_t ID_METH_CONFIG_BROADCAST = 0x302;
constexpr uint16_t ID_ENGINE_SENSOR_EXT = 0x303;
constexpr uint16_t ID_METH_CONFIG_ACK = 0x304;
constexpr uint16_t ID_ENGINE_METH_FAULT = 0x305;
constexpr uint16_t ID_ENGINE_KNOCK_STATE = 0x307;
constexpr uint16_t ID_ENGINE_KNOCK_FAULT = 0x308;

inline uint8_t clampU8(int value) {
  if (value < 0) return 0;
  if (value > 255) return 255;
  return static_cast<uint8_t>(value);
}

inline uint8_t tempToOffset40(int tempC) {
  return clampU8(tempC + 40);
}

enum class FaultSeverity : uint8_t {
  WARNING = 1,
  CRITICAL = 2,
};

namespace meth_command {
constexpr uint8_t ARM = 0x01;
constexpr uint8_t MANUAL_TEST_DUTY = 0x02;
constexpr uint8_t STOP_MANUAL_TEST = 0x03;
constexpr uint8_t CLEAR_FAULTS = 0x04;
} // namespace meth_command

namespace meth_fault_code {
constexpr uint8_t SAFETY_SHUTDOWN = 0x0A;
} // namespace meth_fault_code

namespace knock_fault_code {
constexpr uint8_t ADC_FAULT = 0x01;
constexpr uint8_t SENSOR_DISCONNECTED = 0x02;
constexpr uint8_t KNOCK_WARNING = 0x03;
constexpr uint8_t KNOCK_CRITICAL = 0x04;
} // namespace knock_fault_code

struct MethConfigBroadcast {
  uint8_t version{0};
  uint8_t ratio_percent{0};
  uint8_t desired_armed{0};
};

inline bool unpackMethConfigBroadcast(const CanFrame &frame, MethConfigBroadcast &out) {
  if (frame.dlc < 3) return false;
  out.version = frame.data[0];
  out.ratio_percent = frame.data[1];
  out.desired_armed = frame.data[2];
  return true;
}

struct EngineSensorExt {
  uint8_t oil_pressure_psi{0};
  uint8_t fuel_pressure_psi{0};
  uint8_t meth_pressure_psi{0};
  uint8_t boost_ref_pressure_psi{0};
  int8_t ambient_temp_c{0};
  int8_t cabin_temp_c{0};
  uint16_t analog_fault_flags{0};
};

inline CanFrame packEngineSensorExt(const EngineSensorExt &ext) {
  CanFrame frame{};
  frame.id = ID_ENGINE_SENSOR_EXT;
  frame.dlc = 8;
  frame.data[0] = ext.oil_pressure_psi;
  frame.data[1] = ext.fuel_pressure_psi;
  frame.data[2] = ext.meth_pressure_psi;
  frame.data[3] = ext.boost_ref_pressure_psi;
  frame.data[4] = static_cast<uint8_t>(ext.ambient_temp_c);
  frame.data[5] = static_cast<uint8_t>(ext.cabin_temp_c);
  frame.data[6] = static_cast<uint8_t>(ext.analog_fault_flags & 0xFFU);
  frame.data[7] = static_cast<uint8_t>((ext.analog_fault_flags >> 8) & 0xFFU);
  return frame;
}

struct EngineKnockState {
  uint8_t status_flags{0};
  uint8_t energy{0};
  uint8_t baseline{0};
  uint8_t threshold{0};
  uint8_t event_count{0};
  uint8_t last_event_rpm_div100{0};
  uint8_t last_event_boost_kpa{0};
};

inline CanFrame packEngineKnockState(const EngineKnockState &knock) {
  CanFrame frame{};
  frame.id = ID_ENGINE_KNOCK_STATE;
  frame.dlc = 8;
  frame.data[0] = knock.status_flags;
  frame.data[1] = knock.energy;
  frame.data[2] = knock.baseline;
  frame.data[3] = knock.threshold;
  frame.data[4] = knock.event_count;
  frame.data[5] = knock.last_event_rpm_div100;
  frame.data[6] = knock.last_event_boost_kpa;
  frame.data[7] = 0;
  return frame;
}

inline CanFrame packEngineKnockFault(uint8_t code, uint8_t severity, uint8_t data0, uint8_t data1) {
  CanFrame frame{};
  frame.id = ID_ENGINE_KNOCK_FAULT;
  frame.dlc = 4;
  frame.data[0] = code;
  frame.data[1] = severity;
  frame.data[2] = data0;
  frame.data[3] = data1;
  return frame;
}

} // namespace can_protocol
