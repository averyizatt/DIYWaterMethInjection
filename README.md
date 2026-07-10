# DIY Water/Meth Injection Controller

Standalone Arduino Nano firmware for a water/meth controller with:

- MCP2515 (SPI) CAN bus only
- MAP/float/temperature/pressure analog sensing
- Pump control and failsafe handling
- Knock DSP detection with CAN telemetry hooks for a remote UI/main controller

## Hardware and firmware model

This device does not require a local display. Knock and system observability are exposed over CAN so a separate main controller can render UI and issue tuning commands.

## Sensors and control

Implemented analog channels:

- MAP/boost sensor
- IAT, engine bay, cabin, ambient thermistors
- Oil/fuel/meth/boost-ref pressure transducers
- Optional spare pressure channels

The firmware performs conversion, smoothing, and fault detection on all supported channels.

## CAN bus interface

Existing controller frames:

- `0x300` `ID_ENGINE_METH_STATE`
- `0x303` `ID_ENGINE_SENSOR_EXT`
- `0x307` `ID_ENGINE_KNOCK_STATE`
- `0x308` `ID_ENGINE_KNOCK_FAULT`

Engine context input:

- `0x309` `ID_ENGINE_RUNTIME`, received by this module for knock arming context

`0x309` engine runtime payload:

- `B0` RPM low byte
- `B1` RPM high byte
- `B2` optional absolute MAP kPa from the sender
- `B3` optional validity flags: bit0 RPM valid, bit1 MAP valid
- Remaining bytes reserved

For `0x300` and knock event payloads, boost fields are gauge boost in kPa, clamped to `0` below local barometric pressure. Absolute MAP is used internally for sensor validation and knock arming, but is not sent as the CAN boost byte.

`0x307` knock state payload, sent every 50 ms:

- `B0` status flags: bit0 enabled, bit1 signal valid, bit2 warning active, bit3 critical active, bit4 baseline learned, bit5 sensor fault, bit6 clipping detected
- `B1` knock energy, 0-255
- `B2` baseline, 0-255
- `B3` threshold, 0-255
- `B4` event count, wraps 0-255
- `B5` last event RPM / 100
- `B6` last event boost kPa
- `B7` reserved, sent as 0

`0x308` knock fault payload:

- `B0` fault code
- `B1` severity
- `B2` data0
- `B3` data1

On the Nano firmware, knock processing is local: raw ADC bias removal, tunable gain, rectified envelope tracking, learned noise baseline, and adaptive thresholding. CAN tuning commands update the live processor for enable, threshold offset, adaptive multiplier, debounce, gain, and related config broadcasts. RPM is received over CAN on `0x309`; local MAP plus CAN RPM are used as knock arming gates, and `0x307` byte 5 reports last-event RPM / 100.

Added knock UI hook frames for remote display/debug:

- `0x30B` knock live hook frame
- `0x30C` knock config page 1
- `0x30D` knock config page 2

`0x30B` payload:

- `B0` flags: bit0 enabled, bit1 armed, bit2 detected, bit3 sensorFault, bit4 clipping, bit5 warning, bit6 critical, bit7 baselineLearned
- `B1` live knock RMS
- `B2` adaptive threshold
- `B3` adaptive baseline
- `B4` event count
- `B5` bias ADC (scaled /16)
- `B6` raw ADC (scaled /16)
- `B7` envelope level

`0x30C` payload:

- `B0` config flags: bit0 enabled, bit1 autoCenterFromBore
- `B1` threshold offset
- `B2` adaptive multiplier x10
- `B3` min RPM /100
- `B4` min MAP kPa
- `B5` debounce ms /10
- `B6` gain x10
- `B7` center frequency /100 Hz

`0x30D` payload:

- `B0` bandwidth /100 Hz
- `B1` sample rate /100 Hz
- `B2` samples per update
- `B3` bias alpha x1000
- `B4` RMS alpha x100
- `B5` envelope alpha x100
- `B6` bore mm
- `B7` reserved

Knock tuning commands are accepted on `ID_ENGINE_METH_COMMAND` (`0x301`) with these command bytes:

- `0x40` set enable (`B1`: 0/1)
- `0x41` set threshold offset (`B1`)
- `0x42` set adaptive multiplier x10 (`B1`)
- `0x43` set minimum RPM /100 (`B1`)
- `0x44` set minimum MAP/boost kPa (`B1`)
- `0x45` set debounce ms /10 (`B1`)
- `0x46` set gain x10 (`B1`)
- `0x47` set center frequency /100 Hz (`B1`)
- `0x48` set bandwidth /100 Hz (`B1`)
- `0x49` set auto frequency from bore (`B1`: 0/1)
- `0x4A` clear knock events/fault latch

Config requests sent on `0x305` trigger the module to rebroadcast `0x30C` and `0x30D`. Runtime command/config responses are sent on `0x306` as:

- `B0` command byte
- `B1` status (`0` OK, `1` unsupported, `2` invalid length, `3` value clamped)
- `B2` applied value
- `B3` schema version

## Knock DSP pipeline

The knock subsystem is modular and runs as:

- high-rate ADC block capture
- bias removal
- biquad bandpass filtering
- rectified envelope plus RMS-like energy
- adaptive threshold (`threshold = baseline * multiplier + offset`)
- RPM/MAP arming gates
- debounce/event window logic
- clipping/missing/stuck sensor fault detection

## Serial (optional)

Serial knock commands are still available for bench bring-up, but the intended production path is CAN-based hooks and commands.

## Runtime hardening for automotive conditions

The firmware includes additional resilience measures for noisy and transient environments:

- Boost hysteresis and latch logic to prevent rapid on/off pump chatter near threshold.
- CAN command timeout fail-safe: if the master controller stops sending updates, the module disarms and exits manual test mode.
- MCP2515 recovery loop: repeated CAN TX errors mark CAN offline and trigger periodic re-init attempts.
- Knock config input sanitation for both serial and CAN tuning commands (range-constrained values).
- Analog critical fault shutdown (IAT, engine bay temp, meth pressure channels): pump output is forced off until faults clear.

## Unit tests

Host-side unit tests were added for knock detector logic in [test/test_knock_detector/test_main.cpp](test/test_knock_detector/test_main.cpp), covering:

- no signal
- normal vibration below threshold
- random noise
- true knock burst at target frequency
- wrong-frequency burst rejection
- changing bias
- knock below RPM/MAP arm threshold
- knock while armed

Run tests:

```bash
platformio test -e native_knock_tests
```

## Knock tuning workflow

Use this sequence when calibrating a new engine/setup:

1. Log normal engine noise with no knock.
2. Find the baseline envelope/RMS level in your operating range.
3. Set threshold above baseline.
4. Test under different RPM and load ranges.
5. Adjust center frequency and bandwidth until true knock is captured while false positives are minimized.

The current knock path enforces knock-sensor datasheet constraints in firmware:

- knock frequency window constrained to 3-25 kHz
- runtime center frequency constrained to sensor range and ADC/Nyquist headroom
- temperature compensation start points constrained to -40 to 150°C
- default temperature compensation slope derived from 0.04 mV/g°C @ 26 mV/g sensitivity

## Build and upload

Nano environments:

- `arduino_nano`: modern bootloader upload speed (115200)
- `arduino_nano_old_bootloader`: clone/old bootloader upload speed (57600)

Build/upload commands:

```bash
platformio run -e arduino_nano
platformio run -e arduino_nano -t upload
```

If upload fails on a clone Nano, use:

```bash
platformio run -e arduino_nano_old_bootloader -t upload
```

Nano wiring used by the firmware:

- MAP analog input: `A4`
- Knock analog input: `A5`
- Float switch (active low): `D3`
- Pump relay output: `D2`
- Warning LED output: `D7`
- MCP2515 CAN CS: `D10`
- MCP2515 CAN INT: `D8`
- MCP2515 CAN MOSI/SI: `D11`
- MCP2515 CAN MISO/SO: `D12`
- MCP2515 CAN SCK: `D13`
- Oil pressure analog input: `A6`
- Fuel pressure analog input: `A7`

The firmware initializes the MCP2515 at `500 kbps` with the common `8 MHz` CAN module crystal setting. It prints received CAN frames to serial and transmits protocol-matched `0x300` meth state, `0x307` knock state, and `0x30B` knock hook frames every 50 ms, `0x303` extended sensor frames every 250 ms, and `0x30C`/`0x30D` knock config pages every 1 s.

Pressure transducers default to 0.5-4.5 V, 0-100 PSI linear sensors through a 10k/20k divider, so 5 V sensor output becomes about 3.33 V at the Nano ADC.

On a one-node bench setup, MCP2515 transmit can report failure because no other CAN node ACKs the frame. The firmware logs counted TX failures without reinitializing the controller, so it can keep servicing RX and will recover once a terminated, powered second node is present.

The Nano profile builds a reduced firmware (`main_nano.cpp` + `sensors_nano.cpp`) so it fits ATmega328P flash/RAM.
