// CAN on the PCB through the ESP32-S3's own controller (TWAI). See pcb_can.h.
#include "pcb_can.h"

#include <driver/twai.h>
#include <string.h>

#include "pcb_board.h"

namespace pcb_can {
namespace {
bool installed = false;
bool listening = false;

bool timingFor(uint32_t bitrate, twai_timing_config_t &t) {
  switch (bitrate) {
  case 125000: { twai_timing_config_t c = TWAI_TIMING_CONFIG_125KBITS(); t = c; return true; }
  case 250000: { twai_timing_config_t c = TWAI_TIMING_CONFIG_250KBITS(); t = c; return true; }
  case 500000: { twai_timing_config_t c = TWAI_TIMING_CONFIG_500KBITS(); t = c; return true; }
  case 1000000: { twai_timing_config_t c = TWAI_TIMING_CONFIG_1MBITS(); t = c; return true; }
  default: return false;
  }
}
} // namespace

bool begin(uint32_t bitrate, bool listenOnly) {
  end();
  twai_timing_config_t timing;
  if (!timingFor(bitrate, timing)) return false;
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(static_cast<gpio_num_t>(pcb::CAN_TX),
                                                              static_cast<gpio_num_t>(pcb::CAN_RX),
                                                              listenOnly ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL);
  general.rx_queue_len = 64;
  general.tx_queue_len = 16;
  general.alerts_enabled = TWAI_ALERT_NONE;
  const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&general, &timing, &filter) != ESP_OK) return false;
  installed = true;
  listening = listenOnly;
  if (twai_start() != ESP_OK) {
    end();
    return false;
  }
  return true;
}

void end() {
  if (!installed) return;
  twai_stop();
  twai_driver_uninstall();
  installed = false;
}

bool running() {
  if (!installed) return false;
  twai_status_info_t s;
  return twai_get_status_info(&s) == ESP_OK && s.state == TWAI_STATE_RUNNING;
}

bool receive(Frame &frame, uint32_t timeoutMs) {
  if (!installed) return false;
  twai_message_t m;
  if (twai_receive(&m, pdMS_TO_TICKS(timeoutMs)) != ESP_OK) return false;
  if (m.rtr) return false;
  frame.id = m.identifier;
  frame.extended = m.extd;
  frame.len = m.data_length_code > 8 ? 8 : m.data_length_code;
  memcpy(frame.data, m.data, 8);
  return true;
}

bool send(const Frame &frame, uint32_t timeoutMs) {
  if (!installed || listening) return false;
  twai_message_t m = {};
  m.identifier = frame.id;
  m.extd = frame.extended ? 1 : 0;
  m.data_length_code = frame.len > 8 ? 8 : frame.len;
  memcpy(m.data, frame.data, m.data_length_code);
  return twai_transmit(&m, pdMS_TO_TICKS(timeoutMs)) == ESP_OK;
}

Health health() {
  Health h;
  if (!installed) return h;
  twai_status_info_t s;
  if (twai_get_status_info(&s) != ESP_OK) return h;
  h.running = s.state == TWAI_STATE_RUNNING;
  h.busOff = s.state == TWAI_STATE_BUS_OFF;
  h.txErrors = s.tx_error_counter;
  h.rxErrors = s.rx_error_counter;
  h.rxMissed = s.rx_missed_count + s.rx_overrun_count;
  h.txFailed = s.tx_failed_count;
  h.busErrors = s.bus_error_count;
  return h;
}

bool service() {
  if (!installed) return false;
  twai_status_info_t s;
  if (twai_get_status_info(&s) != ESP_OK) return false;
  if (s.state == TWAI_STATE_BUS_OFF) {
    twai_initiate_recovery();
    return false;
  }
  if (s.state == TWAI_STATE_STOPPED) {
    twai_start();   // Recovered from bus-off: running again
    return false;
  }
  return s.state == TWAI_STATE_RUNNING;
}

} // namespace pcb_can
