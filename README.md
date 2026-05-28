# DIY Water/Meth Injection Controller

Standalone Nano ESP32 firmware for a water/meth controller with:

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

## Build and upload

```bash
platformio run -e arduino_nano_esp32
platformio run -e arduino_nano_esp32 -t upload
```
