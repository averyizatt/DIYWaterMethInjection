#pragma once

// CAN on the PCB: the ESP32-S3's own CAN controller (TWAI) and the board's TCAN1042V
// transceiver (pcb_board.h: TX GPIO8, RX GPIO9). Both PCB builds use it; src/pcb_can.cpp.
#include <stdint.h>

namespace pcb_can {

struct Frame {
  uint32_t id;
  bool extended;
  uint8_t len;
  uint8_t data[8];
};

// Starts the controller at 125k, 250k, 500k or 1M. listenOnly: never transmits, never
// acknowledges (for a bus that already has its own nodes, such as an ECU and its dash).
bool begin(uint32_t bitrate, bool listenOnly);
void end();
bool running();
// Waits up to timeoutMs for a frame.
bool receive(Frame &frame, uint32_t timeoutMs);
// Queues a frame; false if it could not be queued within timeoutMs (or listen only).
bool send(const Frame &frame, uint32_t timeoutMs);

struct Health {
  bool running{false};
  bool busOff{false};
  uint32_t txErrors{0};     // The controller's error counters
  uint32_t rxErrors{0};
  uint32_t rxMissed{0};     // Frames lost because the receive queue was full
  uint32_t txFailed{0};
  uint32_t busErrors{0};
};
Health health();
// Called every pass: brings a controller that went bus-off back (it waits 128 x 11
// recessive bits first, as CAN requires). Returns false while it is not running.
bool service();

} // namespace pcb_can
