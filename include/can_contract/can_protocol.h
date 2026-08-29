#pragma once

#include <stdint.h>

namespace can_protocol {

constexpr uint32_t CAN_BITRATE = 500000;
constexpr uint16_t CAN_PROTOCOL_SCHEMA_VERSION = 1;

constexpr uint16_t ID_MASTER_HEARTBEAT = 0x200;
constexpr uint16_t ID_ENGINE_METH_STATE = 0x300;
constexpr uint16_t ID_ENGINE_METH_COMMAND = 0x301;
constexpr uint16_t ID_ENGINE_METH_FAULT = 0x302;
constexpr uint16_t ID_ENGINE_SENSOR_EXT = 0x303;
constexpr uint16_t ID_METH_CONFIG_BROADCAST = 0x304;
constexpr uint16_t ID_METH_CONFIG_REQUEST = 0x305;
constexpr uint16_t ID_METH_CONFIG_ACK = 0x306;
constexpr uint16_t ID_ENGINE_KNOCK_STATE = 0x307;
constexpr uint16_t ID_ENGINE_KNOCK_FAULT = 0x308;
constexpr uint16_t ID_ENGINE_RUNTIME = 0x309;
constexpr uint16_t ID_KNOCK_LIVE_HOOK = 0x30B;
constexpr uint16_t ID_KNOCK_CONFIG_PAGE_1 = 0x30C;
constexpr uint16_t ID_KNOCK_CONFIG_PAGE_2 = 0x30D;

enum class MethState : uint8_t { OFF = 0, ARMED = 1, SPRAYING = 2, FAULT = 3, TEST = 4 };
enum class FlowStatus : uint8_t { UNKNOWN = 0, OK = 1, LOW_FLOW = 2, NO_FLOW = 3 };
enum class FaultSeverity : uint8_t { INFO = 0, WARNING = 1, CRITICAL = 2 };

namespace meth_command {
constexpr uint8_t ARM = 0x01;
constexpr uint8_t MANUAL_TEST_DUTY = 0x02;
constexpr uint8_t STOP_MANUAL_TEST = 0x03;
constexpr uint8_t SET_BOOST_TRIGGER = 0x04;
constexpr uint8_t SET_IAT_THRESHOLD = 0x05;
constexpr uint8_t CLEAR_FAULTS = 0x06;
constexpr uint8_t KNOCK_SET_ENABLE = 0x40;
constexpr uint8_t KNOCK_SET_THRESHOLD_OFFSET = 0x41;
constexpr uint8_t KNOCK_SET_ADAPTIVE_MULTIPLIER_X10 = 0x42;
constexpr uint8_t KNOCK_SET_MIN_RPM_DIV100 = 0x43;
constexpr uint8_t KNOCK_SET_MIN_MAP_KPA = 0x44;
constexpr uint8_t KNOCK_SET_DEBOUNCE_MS_DIV10 = 0x45;
constexpr uint8_t KNOCK_SET_GAIN_X10 = 0x46;
constexpr uint8_t KNOCK_SET_CENTER_FREQ_DIV100 = 0x47;
constexpr uint8_t KNOCK_SET_BANDWIDTH_DIV100 = 0x48;
constexpr uint8_t KNOCK_SET_AUTO_FREQ_FROM_BORE = 0x49;
constexpr uint8_t KNOCK_CLEAR_EVENTS = 0x4A;
} // namespace meth_command

namespace config_ack_status {
constexpr uint8_t OK = 0x00;
constexpr uint8_t UNSUPPORTED_COMMAND = 0x01;
constexpr uint8_t INVALID_LENGTH = 0x02;
constexpr uint8_t VALUE_CLAMPED = 0x03;
} // namespace config_ack_status

namespace meth_fault_code {
constexpr uint8_t LOW_TANK = 0x01;
constexpr uint8_t NO_FLOW = 0x02;
constexpr uint8_t LOW_FLOW = 0x03;
constexpr uint8_t PUMP_OVERCURRENT = 0x04;
constexpr uint8_t SENSOR_FAIL = 0x05;
constexpr uint8_t OVER_TEMP = 0x06;
constexpr uint8_t CAN_TIMEOUT = 0x07;
constexpr uint8_t CONFIG_INVALID = 0x08;
constexpr uint8_t SAFETY_SHUTDOWN = 0x09;
} // namespace meth_fault_code

namespace knock_fault_code {
constexpr uint8_t KNOCK_WARNING = 0x01;
constexpr uint8_t KNOCK_CRITICAL = 0x02;
constexpr uint8_t SENSOR_DISCONNECTED = 0x03;
constexpr uint8_t SIGNAL_CLIPPING = 0x04;
constexpr uint8_t BASELINE_NOT_LEARNED = 0x05;
constexpr uint8_t ADC_FAULT = 0x06;
} // namespace knock_fault_code

struct CanFrame {
  uint16_t id = 0;
  uint8_t dlc = 0;
  uint8_t data[8]{};
};

inline uint8_t clampU8(int value) {
  return static_cast<uint8_t>(value < 0 ? 0 : (value > 255 ? 255 : value));
}

inline uint8_t tempToOffset40(int celsius) {
  return clampU8(celsius + 40);
}

inline int8_t offset40ToTemp(uint8_t encoded) {
  return static_cast<int8_t>(static_cast<int16_t>(encoded) - 40);
}

struct EngineMethState {
  // 0x300, DLC 8
  // B0 meth state, B1 pump duty, B2 tank %, B3 flow status,
  // B4 gauge boost kPa, B5 IAT+40, B6 engine bay+40, B7 fault flags.
  uint8_t meth_state = 0;
  uint8_t pump_duty = 0;
  uint8_t tank_level = 0;
  uint8_t flow_status = 0;
  uint8_t boost_kpa = 0;
  int8_t iat_c = 0;
  int8_t engine_bay_c = 0;
  uint8_t fault_flags = 0;
};

struct EngineSensorExt {
  // 0x303, DLC 8
  // Pressure bytes are psi * 2, so 1 count = 0.5 psi.
  uint8_t oil_pressure_psi_x2 = 0;
  uint8_t fuel_pressure_psi_x2 = 0;
  uint8_t meth_pressure_psi_x2 = 0;
  uint8_t boost_ref_pressure_psi_x2 = 0;
  int8_t ambient_temp_c = 0;
  int8_t cabin_temp_c = 0;
  uint16_t analog_fault_flags = 0;
};

struct EngineKnockState {
  // 0x307, DLC 8
  // status_flags bits: 0 enabled, 1 signal valid, 2 warning, 3 critical,
  // 4 baseline learned, 5 sensor fault, 6 clipping.
  uint8_t status_flags = 0;
  uint8_t energy = 0;
  uint8_t baseline = 0;
  uint8_t threshold = 0;
  uint8_t event_count = 0;
  uint8_t last_event_rpm_div100 = 0;
  // Gauge boost kPa at the last knock event.
  uint8_t last_event_boost_kpa = 0;
  uint8_t reserved = 0;
};

struct KnockLiveHook {
  // 0x30B, DLC 8
  // flags bits: 0 enabled, 1 armed, 2 detected, 3 sensor fault,
  // 4 clipping, 5 warning, 6 critical, 7 baseline learned.
  uint8_t flags = 0;
  uint8_t live_knock_rms = 0;
  uint8_t adaptive_threshold = 0;
  uint8_t adaptive_baseline = 0;
  uint8_t event_count = 0;
  uint8_t bias_adc_div16 = 0;
  uint8_t raw_adc_div16 = 0;
  uint8_t envelope_level = 0;
};

struct KnockConfigPage1 {
  // 0x30C, DLC 8
  // config_flags bits: 0 enabled, 1 autoCenterFromBore.
  uint8_t config_flags = 0;
  uint8_t threshold_offset = 0;
  uint8_t adaptive_multiplier_x10 = 0;
  uint8_t min_rpm_div100 = 0;
  uint8_t min_map_kpa = 0;
  uint8_t debounce_ms_div10 = 0;
  uint8_t gain_x10 = 0;
  uint8_t center_frequency_div100 = 0;
};

struct KnockConfigPage2 {
  // 0x30D, DLC 8
  uint8_t bandwidth_div100 = 0;
  uint8_t sample_rate_div100 = 0;
  uint8_t samples_per_update = 0;
  uint8_t bias_alpha_x1000 = 0;
  uint8_t rms_alpha_x100 = 0;
  uint8_t envelope_alpha_x100 = 0;
  uint8_t bore_mm = 0;
  uint8_t reserved = 0;
};

inline CanFrame packEngineMethState(const EngineMethState &state) {
  CanFrame frame{};
  frame.id = ID_ENGINE_METH_STATE;
  frame.dlc = 8;
  frame.data[0] = state.meth_state;
  frame.data[1] = state.pump_duty;
  frame.data[2] = state.tank_level;
  frame.data[3] = state.flow_status;
  frame.data[4] = state.boost_kpa;
  frame.data[5] = tempToOffset40(state.iat_c);
  frame.data[6] = tempToOffset40(state.engine_bay_c);
  frame.data[7] = state.fault_flags;
  return frame;
}

inline CanFrame packEngineSensorExt(const EngineSensorExt &ext) {
  CanFrame frame{};
  frame.id = ID_ENGINE_SENSOR_EXT;
  frame.dlc = 8;
  frame.data[0] = ext.oil_pressure_psi_x2;
  frame.data[1] = ext.fuel_pressure_psi_x2;
  frame.data[2] = ext.meth_pressure_psi_x2;
  frame.data[3] = ext.boost_ref_pressure_psi_x2;
  frame.data[4] = tempToOffset40(ext.ambient_temp_c);
  frame.data[5] = tempToOffset40(ext.cabin_temp_c);
  frame.data[6] = static_cast<uint8_t>(ext.analog_fault_flags & 0xFFU);
  frame.data[7] = static_cast<uint8_t>((ext.analog_fault_flags >> 8U) & 0xFFU);
  return frame;
}

inline CanFrame packEngineKnockState(const EngineKnockState &state) {
  CanFrame frame{};
  frame.id = ID_ENGINE_KNOCK_STATE;
  frame.dlc = 8;
  frame.data[0] = state.status_flags;
  frame.data[1] = state.energy;
  frame.data[2] = state.baseline;
  frame.data[3] = state.threshold;
  frame.data[4] = state.event_count;
  frame.data[5] = state.last_event_rpm_div100;
  frame.data[6] = state.last_event_boost_kpa;
  frame.data[7] = state.reserved;
  return frame;
}

inline CanFrame packKnockLiveHook(const KnockLiveHook &hook) {
  CanFrame frame{};
  frame.id = ID_KNOCK_LIVE_HOOK;
  frame.dlc = 8;
  frame.data[0] = hook.flags;
  frame.data[1] = hook.live_knock_rms;
  frame.data[2] = hook.adaptive_threshold;
  frame.data[3] = hook.adaptive_baseline;
  frame.data[4] = hook.event_count;
  frame.data[5] = hook.bias_adc_div16;
  frame.data[6] = hook.raw_adc_div16;
  frame.data[7] = hook.envelope_level;
  return frame;
}

inline CanFrame packKnockConfigPage1(const KnockConfigPage1 &page) {
  CanFrame frame{};
  frame.id = ID_KNOCK_CONFIG_PAGE_1;
  frame.dlc = 8;
  frame.data[0] = page.config_flags;
  frame.data[1] = page.threshold_offset;
  frame.data[2] = page.adaptive_multiplier_x10;
  frame.data[3] = page.min_rpm_div100;
  frame.data[4] = page.min_map_kpa;
  frame.data[5] = page.debounce_ms_div10;
  frame.data[6] = page.gain_x10;
  frame.data[7] = page.center_frequency_div100;
  return frame;
}

inline CanFrame packKnockConfigPage2(const KnockConfigPage2 &page) {
  CanFrame frame{};
  frame.id = ID_KNOCK_CONFIG_PAGE_2;
  frame.dlc = 8;
  frame.data[0] = page.bandwidth_div100;
  frame.data[1] = page.sample_rate_div100;
  frame.data[2] = page.samples_per_update;
  frame.data[3] = page.bias_alpha_x1000;
  frame.data[4] = page.rms_alpha_x100;
  frame.data[5] = page.envelope_alpha_x100;
  frame.data[6] = page.bore_mm;
  frame.data[7] = page.reserved;
  return frame;
}

inline CanFrame packEngineMethFault(uint8_t code, uint8_t severity, uint8_t data0, uint8_t data1) {
  CanFrame frame{};
  frame.id = ID_ENGINE_METH_FAULT;
  frame.dlc = 4;
  frame.data[0] = code;
  frame.data[1] = severity;
  frame.data[2] = data0;
  frame.data[3] = data1;
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

inline CanFrame packConfigAck(uint8_t command, uint8_t status, uint8_t value, uint8_t schemaVersion) {
  CanFrame frame{};
  frame.id = ID_METH_CONFIG_ACK;
  frame.dlc = 4;
  frame.data[0] = command;
  frame.data[1] = status;
  frame.data[2] = value;
  frame.data[3] = schemaVersion;
  return frame;
}

} // namespace can_protocol
