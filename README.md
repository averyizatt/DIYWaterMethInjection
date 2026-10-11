# DIY Water/Meth Injection Controller

Two boards run this controller, with the same CAN messages:

- **Arduino Nano** (`src/main_nano.cpp`): pump on a mechanical relay, its own MAP sensor.
- **ESP32-S3** (`src/main_esp32.cpp`, the replacement): pump on a MOSFET with PWM,
  manifold pressure and RPM from the MicroSquirt over CAN, firmware updates from the
  dash over CAN. See [ESP32-S3 board](#esp32-s3-board-pump-on-a-mosfet-with-pwm) and
  [docs/ESP32_HARDWARE.md](docs/ESP32_HARDWARE.md).

Standalone Arduino Nano firmware for a water/meth controller with:

- MCP2515 (SPI) CAN bus only
- MAP/float/temperature/pressure analog sensing
- Pump control and failsafe handling
- Knock DSP detection with CAN telemetry hooks for a remote UI/main controller

## Hardware and firmware model

This device does not require a local display. Knock and system observability are exposed over CAN so a separate main controller can render UI and issue tuning commands.

## Pulse tuning (pump switched by a mechanical relay)

The pump is switched directly by a mechanical relay, so flow is set by how long the
pump runs in each slow cycle, not by PWM. The logic is in
[include/meth_tune.h](include/meth_tune.h) and is adjustable over CAN (shared contract
extension 4): commands `0x10` (set one setting) and `0x11` (save / revert / defaults /
report) on `0x301`, with acknowledgements, settings, status and the two intake
temperatures reported on `0x30F`.

Fixed rules, not adjustable:

- The pump runs for at least 500 ms or not at all, rests for at least 500 ms or stays
  on continuously, and starts at most once per cycle. The cycle cannot be set below
  1000 ms. A pulse that is blocked part-way stays off until the next cycle.
- No injection without a fresh engine RPM message (`0x309`) at or above the minimum.
- Dose limit: fluid is held to a set percentage of estimated fuel flow, from RPM and
  manifold pressure (`kFuelGPerMinPerRpmKpa` in meth_tune.h is for a 2.3 L engine).
- Changing one setting never raises another; values outside the limits are clamped
  and the applied value is acknowledged.

Two optional triggers for cooling, both inside every rule above:

- Early start: from a set RPM, injection begins before boost while the throttle is
  open (manifold pressure within 1.5 psi of atmospheric), at the start-boost on-time.
- Hot air: air before the nozzle above a set temperature lengthens the on-time,
  reaching the full on-time at a second temperature. It only adds to injection that
  is already active.

Until the thermistor before the nozzle is fitted, the temperature features use the
ECU's intake air temperature, which the dash forwards about four times a second
(`0x301` command `0x12`, not acknowledged, stale after 1.5 s).

| Setting | Range | Default |
| --- | --- | --- |
| Cycle length | 1000-10000 ms | 4000 |
| Pump on-time at start boost | 500-10000 ms | 1000 |
| Pump on-time at full boost | 500-10000 ms (= cycle: continuous) | 2000 |
| Start boost | 1.0-30.0 psi | 5.0 |
| Full boost | 2.0-35.0 psi | 10.0 |
| Minimum engine RPM | 0-8000 | 2500 |
| Longest continuous spray | 1-120 s | 12 |
| Rest after that (0 = no limit) | 0-60 s | 6 |
| On-time growth per cycle (0 = none) | 0-5000 ms | 500 |
| Minimum air temperature before the nozzle (0 = off) | 0-120 C | 0 |
| Overboost assist (the older 85% / 100% forcing) | 0-1 | 0 |
| Methanol in the tank | 0-100 % | 0 |
| Nozzle flow | 20-1000 ml/min | 60 |
| Most fluid as a share of fuel flow (0 = off) | 0-40 % | 10 |
| Early start RPM (0 = off) | 0-8000 | 3500 |
| Hot air adds on-time from (0 = off) | 0-145 C | 0 |
| Hot air reaches full on-time at | 20-150 C | 70 |

Settings apply at once and are kept in RAM until a SAVE action writes them to EEPROM.
A blank or corrupt EEPROM loads the defaults above. The older boost-start command
(`0x301/0x04`) and the `0x304` configuration frame set the same start-boost value.

Two intake air thermistors (GM-style open-element IAT) report the air temperature
before (`A1`) and after (`A2`) the nozzle. Wire each as 5 V -> 2.49 kohm -> pin ->
sensor -> GND.

## ESP32-S3 board: pump on a MOSFET with PWM

Wiring, parts and first start-up checks: [docs/ESP32_HARDWARE.md](docs/ESP32_HARDWARE.md).

What the board does differently, and only this:

- **Pump speed instead of pulses.** The pump runs continuously while injecting, at a
  duty that follows boost: the lowest duty at start boost, a straight line up to the
  highest duty at full boost (`PwmGovernor` in [include/meth_tune.h](include/meth_tune.h)).
  Two settings were added for it (shared contract extension 5); the Nano stores them
  and ignores them, and the ESP32 ignores the three pulse settings.

  | Setting | Range | Default |
  | --- | --- | --- |
  | Lowest pump duty (18) | 10-100 % | 25 |
  | Highest pump duty (19) | 10-100 % | 50 |

- **Flow is not proportional to duty.** Pressure at the nozzle rises roughly with
  duty and flow with the square root of pressure, so flow is taken as
  `nozzle flow x sqrt(duty)`: 25 % duty is about half the nozzle's rating. The dose
  limit uses the same rule, and when it would need less than the lowest duty the
  pump stays off (`hold` = dose limit) rather than dribbling.
- Every other rule is the same: minimum RPM, air-temperature gate, longest spray and
  rest, early start and hot air (they ask for duty, not on-time), overboost assist,
  dose limit and the cap from the dash. "On-time growth" becomes the time the duty may
  take to climb from lowest to highest.
- The status frame on `0x30F` reports the pump as on-time out of a period of 1000:
  the duty in tenths of a percent.
- **Manifold pressure, RPM, barometric pressure and intake temperature come straight
  from the MicroSquirt's CAN broadcast** ([include/ecu_broadcast.h](include/ecu_broadcast.h):
  0x5E8/0x5E9, or 0x5F0/0x5F2). A value older than half a second is not used, and
  without fresh manifold pressure and RPM the controller does not inject. The dash's
  `0x309` RPM is the backup for RPM only.
- Knock, pressures and temperatures are sampled by the ADC's own timer
  ([src/adc_sampler.cpp](src/adc_sampler.cpp)): the knock input at 19.2 kHz, the same
  rate the Nano used, so the knock settings mean the same thing.
- Settings are saved in the ESP32's flash instead of EEPROM.
- **Firmware updates over CAN** from the dash (`0x680`/`0x681`, module 3), refused
  while the pump runs or boost is near, with the pump held off during the transfer.
- If the main loop stops, the pump is switched off within 0.25 s; a watchdog then
  restarts the board. An optional master relay output opens the pump's 12 V feed
  whenever no spray is under way, against a MOSFET that has failed switched on.

## Sensors and control

### Controller UI pump test and tank protection

Update both the ESP32 controller and Nano firmware for these controls. On the
METH page, disarm injection and hold **HOLD: TEST 5s** to run the relay-driven
pump at 100% for at most five seconds. **STOP PUMP** cancels the test and disarms
normal injection. A three-second cooldown prevents immediate repeated tests.
The Nano also enforces its own timeout if the UI or CAN connection fails.
Route the nozzle into a suitable container before testing: this operates the
real pump and sprays fluid.

Hold **TANK GUARD** while stopped/disarmed to bypass only the float protection;
the button then reads **BYPASS!**. Hold again to restore protection. The choice
is not saved and defaults to protection enabled after restart. With bypass on,
the pump can run with an empty tank; verify fluid yourself. MAP and other
reported faults still block operation. Wait for fresh Nano status after changing
the guard before arming/testing. Older Nano firmware ignores the bypass flag.

The two-wire float is on **D3 to GND**, with an internal pull-up. The documented
active-low setting means **closed/LOW = low fluid**, **open/HIGH = fluid OK**.
This is a binary switch, not a percentage-level sender; the UI now displays
**LOW / OK**, or **--** while offline. CAN still carries 0/100 for compatibility.
If LOW never changes, inspect D3 and the switch continuity in both float positions.
The Nano serial heartbeat at 115200 baud now prints `floatD3` and `lowWhen` beside
`tankLow` to distinguish the electrical input from the debounced result. A switch
that closes when full has the opposite polarity to this configuration; do not
reverse the firmware setting without verifying both float positions.

The previous controller Arm gate required IAT and meth-pressure validity even
though both channels are disabled in the supplied Nano pin map. Arm now follows
the Nano's boost-mode MAP/fault status and tank protection instead. Rejections
are displayed as offline, low tank, or module fault.

Periodic disarmed configuration broadcasts previously cancelled manual tests.
They now preserve an active test; explicit ARM/DISARM/STOP and the local timeout
still cancel it, and repeated test commands cannot restart its timer.

Implemented analog channels:

- MAP/boost sensor
- IAT, engine bay, cabin, ambient thermistors
- Oil/fuel/meth/boost-ref pressure transducers
- Optional spare pressure channels

The firmware performs conversion, smoothing, and fault detection on all supported channels.

## CAN bus interface

Schema 2 uses the canonical shared header. Command/knock ACKs are now on
`0x30A`; `0x306` is reserved for configuration broadcast ACKs. Update the main
controller and Nano together. See [CAN audit](../../docs/CAN_COMPATIBILITY_AUDIT.md).

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

Config requests sent on `0x305` trigger the module to rebroadcast `0x30C` and `0x30D`. Runtime command responses are sent on `0x30A` as:

- `B0` command byte
- `B1` status (`0` OK, `1` unsupported, `2` invalid length, `3` value clamped)
- `B2` applied value
- `B3` schema version

## Knock listening on the Nano

The Nano build uses [include/knock_band.h](include/knock_band.h). (The larger
`knock_detector_logic` / `knock_monitor` sources in this repo are not part of the Nano
build; see `build_src_filter` in platformio.ini.)

Each 20 ms loop pass takes one burst from the knock amplifier on `A5`: 64 samples at
19.2 kHz (ADC free-running, /64 clock), 3.3 ms. The burst is measured with integer
Goertzel filters:

- **level**: amplitude in the knock band, five 300 Hz bins around the centre frequency
- **share**: the band's share of the burst's total energy (ringing is narrow, engine
  noise is broadband)
- **peak** and **bias**: largest swing and resting level, for checking the amplifier

The centre frequency comes from the bore, `f = 1800 / (pi * bore_mm)` kHz: about
6.0 kHz for the 96 mm bore of the Ford 2.3 L Lima this build defaults to. It can be set
over CAN in 300 Hz steps from 2.4 to 7.8 kHz.

The detector learns the normal level per 1000 RPM and calls a hit only when the level
exceeds `learned * multiplier + offset` and the share is high enough. Two hits within
0.6 s raise a warning; a strong, clearly narrow hit is critical at once. Knock-like
bursts are never learned as normal. Readings are judged only while armed (enabled,
RPM and MAP above their minimums, sensor not clipping and resting near mid-scale).

Limits: the bursts are not timed to combustion (the Nano has no crank or ignition
signal), so single knock events are caught about one time in four and sustained knock
within a fraction of a second. Nothing here retards timing.

`0x307` carries level, learned level, threshold and, in its spare byte, the share.
`0x30B` carries the resting level and the largest swing.

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
platformio test -c platformio-tests.ini -e native_knock_tests
```

The ESP32 board's logic has host tests too: the PWM pump rules
([test/test_meth_pwm/test_main.cpp](test/test_meth_pwm/test_main.cpp)) and the ECU
broadcast decoding and sensor scaling
([test/test_esp32_inputs/test_main.cpp](test/test_esp32_inputs/test_main.cpp)).

```bash
g++ -std=c++11 -I include test/test_meth_pwm/test_main.cpp -o meth_pwm_test && ./meth_pwm_test
g++ -std=c++11 -I include test/test_esp32_inputs/test_main.cpp -o esp32_inputs_test && ./esp32_inputs_test
```

The knock listening logic has a host test
([test/test_knock_band/test_main.cpp](test/test_knock_band/test_main.cpp)) with
synthetic engine noise and knock: frequency selectivity, integer overflow, learning
per RPM, false alarms over twenty simulated minutes and catch rate.

```bash
platformio test -c platformio-tests.ini -e native_knock_band_tests
# or directly:
g++ -std=c++11 -I include test/test_knock_band/test_main.cpp -o knock_band_test && ./knock_band_test
```

The pulse tuning logic has its own host test
([test/test_meth_tune/test_main.cpp](test/test_meth_tune/test_main.cpp)): relay rate
rules, RPM and temperature gates, dose limit, spray time limit, ramp, early start,
hot-air addition, setting limits and the thermistor curve.

```bash
platformio test -c platformio-tests.ini -e native_meth_tune_tests
# or directly:
g++ -std=c++11 -I include test/test_meth_tune/test_main.cpp -o meth_tune_test && ./meth_tune_test
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

The default environment is `arduino_nano` (115200 baud). The ESP32-S3 board is the
environment `esp32s3_water_meth`: pick it in the PlatformIO toolbar, or

```bash
platformio run -e esp32s3_water_meth -t upload
```

Both need DIYComfortControlModule checked out next to this repository (the shared CAN
contract is read from it).

```bash
platformio run
platformio run -t upload
```

For a clone with the old bootloader, change `upload_speed` in `platformio.ini`
to `57600`. Host test environments live separately in `platformio-tests.ini`.

Nano wiring used by the firmware:

| Signal | Nano pin | Wiring notes |
| --- | --- | --- |
| MAP analog input | `A0` | GM 3-bar MAP through 10k/20k divider. Avoid `A4` because it is SDA and can be biased by I2C pull-ups. |
| Knock analog input | `A5` | Knock sensor through an amplifier that rests near 2.5 V and swings both ways (AC signal, not an envelope). |
| Air temperature before the nozzle | `A1` | GM-style IAT thermistor to GND, 2.49 kohm pull-up to 5 V. |
| Air temperature after the nozzle | `A2` | GM-style IAT thermistor to GND, 2.49 kohm pull-up to 5 V. |
| Oil pressure analog input | `A6` | 0.5-4.5 V, 0-100 psi sensor through 10k/20k divider. Analog-input only pin. |
| Fuel pressure analog input | `A7` | 0.5-4.5 V, 0-100 psi sensor through 10k/20k divider. Analog-input only pin. |
| Float switch | `D3` | Active low; wire switch to ground, firmware uses internal pull-up. |
| Pump relay output | `D2` | Drives relay/control input. Use a relay/transistor module appropriate for the pump current. |
| Warning LED output | `D7` | Active high warning output. |
| Bench-test button | `D9` | Active low with internal pull-up; currently reserved for bench bring-up. |
| MCP2515 CAN CS | `D10` | SPI chip select. |
| MCP2515 CAN INT | `D8` | MCP2515 interrupt pin. Firmware also polls receive status. |
| MCP2515 CAN MOSI/SI | `D11` | Nano hardware SPI MOSI to MCP2515 SI. |
| MCP2515 CAN MISO/SO | `D12` | Nano hardware SPI MISO from MCP2515 SO. |
| MCP2515 CAN SCK | `D13` | Nano hardware SPI clock. |
| MCP2515 VCC/GND | `5V` / `GND` | Match your MCP2515 module requirements and share ground with the display/comfort module. |
| CANH/CANL | MCP2515 transceiver | Wire CANH to CANH and CANL to CANL on the other module; use proper bus termination. |

Disabled/unwired in the current Nano firmware:

- Meth pressure input: disabled (`-1`)
- Boost reference pressure input: disabled (`-1`)
- Spare pressure inputs: disabled (`-1`)
- IAT, engine bay, cabin, ambient, and DHT inputs: disabled (`-1`)

The firmware initializes the MCP2515 at `500 kbps` with the common `8 MHz` CAN module crystal setting. It prints received CAN frames to serial and transmits protocol-matched `0x300` meth state, `0x307` knock state, and `0x30B` knock hook frames every 50 ms, `0x303` extended sensor frames every 250 ms, and `0x30C`/`0x30D` knock config pages every 1 s. The `0x303` pressure bytes use psi x2 encoding, so one count is 0.5 psi.

Pressure transducers default to 0.5-4.5 V, 0-100 PSI linear sensors through a 10k/20k divider, so 5 V sensor output becomes about 3.33 V at the Nano ADC.

The GM 3-bar MAP input also defaults to a 10k/20k divider. Firmware reconstructs the sensor-side 0.5-4.5 V signal before converting to absolute kPa.

On a one-node bench setup, MCP2515 transmit can report failure because no other CAN node ACKs the frame. The firmware logs counted TX failures without reinitializing the controller, so it can keep servicing RX and will recover once a terminated, powered second node is present.

For CAN debugging, the Nano firmware logs every RX frame, every TX attempt with result code, init bitrate/clock/mode results, and periodic MCP2515 health (`CANSTAT`, `INTF`, read `STAT`, `EFLG`, `REC`, `TEC`, INT pin level). A received `0x305` should immediately be followed by TX attempts for `0x30C` and `0x30D`.

The Nano profile builds a reduced firmware (`main_nano.cpp` + `sensors_nano.cpp`) so it fits ATmega328P flash/RAM.
