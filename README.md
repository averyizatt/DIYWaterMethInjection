# DIY Water/Meth Injection Controller

Firmware for the car's water/meth and knock controller: an ESP32-S3 DevKitC-1 with an
MCP2515 CAN module (`src/main_esp32.cpp`, PlatformIO environment `esp32s3_water_meth`).
Wiring, parts, bench tests and first start-up checks:
[docs/ESP32_HARDWARE.md](docs/ESP32_HARDWARE.md).

- **Pump on a MOSFET, full on or full off** in timed pulses, each run started with a
  short soft ramp of the duty ([include/meth_tune.h](include/meth_tune.h)).
- **Manifold pressure, RPM, barometer and intake temperature from the MicroSquirt's CAN
  broadcast** ([include/ecu_broadcast.h](include/ecu_broadcast.h): 0x5E8/0x5E9, or
  0x5F0/0x5F2). A value older than half a second is not used, and without fresh
  manifold pressure and RPM the controller does not inject. The dash's `0x309` RPM is the
  backup for RPM only.
- **Its own sensors:** intake air before the nozzle (on either of two inputs; the firmware
  finds which), oil and fuel pressure, tank float, knock.
- **Firmware updates over CAN** from the dash (`0x680`/`0x681`, module 3), refused while
  the pump runs or boost is near, with the pump held off during the transfer. The
  workflow publishes each build as the `watermeth-latest` release, which the dash
  downloads.
- **Knock listening is off for now** (November 2026, the owner's choice, until the knock
  amplifier's input works on the bench): `ESP32_KNOCK` in `src/main_esp32.cpp`, 0 by
  default; add `-DESP32_KNOCK=1` to the build flags to bring it back. The knock state
  (`0x307`) goes out marked not enabled and knock commands are refused; injection never
  depended on knock.

Every CAN message, command and setting is in the shared contract, read from
DIYComfortControlModule (`shared/can_contract`) checked out next to this repository.

## How the pump is run

The pump is an ordinary diaphragm pump far bigger than the nozzle, so the pressure it
holds follows its motor voltage: at part duty the nozzle gets part pressure and sprays
coarsely. So it runs **full on or full off**, and flow is set by how long it runs in each
cycle (`PulseTiming` and `PulseGovernor` in [include/meth_tune.h](include/meth_tune.h)).
The settings are changed from the dash (shared contract extension 4: commands `0x10`
and `0x11` on `0x301`, with acknowledgements, settings, status and the intake
temperatures reported on `0x30F`; every setting report says the drive is
`meth_drive::PULSE`, extension 10).

Fixed rules, not adjustable:

- The pump runs for at least 500 ms or not at all, rests for at least 500 ms or stays
  on continuously, and starts at most once per cycle. The cycle cannot be set below
  1000 ms. A pulse that is blocked part-way stays off until the next cycle.
- Every run starts with a soft start: the duty climbs from 5% to full over 0.4 s
  (longer after a reset that came while the pump ran). A run that carries straight on
  into the next cycle is not restarted.
- No injection without fresh engine RPM at or above the minimum.
- Dose limit: fluid is held to a set percentage of estimated fuel flow, from RPM and
  manifold pressure (`kFuelGPerMinPerRpmKpa` in meth_tune.h is for a 2.3 L engine). The
  flow is the nozzle's rating for the share of the cycle the pump runs.
- Changing one setting never raises another; values outside the limits are clamped
  and the applied value is acknowledged.

Two optional triggers for cooling, both inside every rule above:

- Early start: from a set RPM, injection begins before boost while the throttle is
  open (manifold pressure within 1.5 psi of atmospheric), at the start-boost on-time.
- Hot air: air before the nozzle above a set temperature lengthens the on-time,
  reaching the full on-time at a second temperature. It only adds to injection that
  is already active.

Without a reading from the sensor before the nozzle, the temperature features use the
MicroSquirt's intake temperature.

| Setting | Range | Default |
| --- | --- | --- |
| Cycle length | 1000-10000 ms | 4000 |
| Pump on-time at start boost | 500-10000 ms | 1000 |
| Pump on-time at full boost | 500-10000 ms (= cycle: continuous) | 2000 |
| Start boost | 0.5-30.0 psi | 0.5 |
| Full boost | 2.0-35.0 psi | 10.0 |
| Minimum engine RPM | 0-8000 | 2500 |
| Longest continuous spray | 1-120 s | 12 |
| Rest after that (0 = no limit) | 0-60 s | 6 |
| On-time growth per cycle (0 = none) | 0-5000 ms | 500 |
| Minimum air temperature before the nozzle (0 = off) | 0-120 C | 0 |
| Overboost assist (the older 85% / 100% forcing) | 0-1 | 0 |
| Methanol in the tank | 0-100 % | 0 |
| Nozzle flow | 20-1000 ml/min | 160 (measured: the label says 60) |
| Most fluid as a share of fuel flow (0 = off) | 0-40 % | 10 |
| Early start RPM (0 = off) | 0-8000 | 3500 |
| Hot air adds on-time from (0 = off) | 0-145 C | 0 |
| Hot air reaches full on-time at | 20-150 C | 70 |
| Lowest / highest pump duty (settings 18, 19) | 10-100 % | 25 / 50 |

Settings 18 and 19 belong to the drive the board had until November 2026 (the pump
running continuously at a duty that followed boost). They are kept because the CAN
contract has them, and ignored.

Settings apply at once and are kept in RAM until a SAVE action writes them to flash
(only away from boost). A blank or damaged store loads the defaults above. Injection
starts at the start boost and stops 0.6 psi below it. The older boost-start command
(`0x301/0x04`) and the `0x304` configuration frame set the same start-boost value.

## Safety

- If the main loop stops setting the pump, the CAN task switches it off within 0.25 s;
  a watchdog then restarts the board. A crash switches the pump off before anything
  else. Repeated crashes hold injection off for a while, longer each time.
- Silence on CAN: the controller is set up again after 1.5 s, the board restarted after
  15 s (three times per ignition at most).
- After a crash, a supply dip or its own recovery restart the board comes back armed if
  it was, sends a start report (`0x302` code `0x0A`) and its black box of the last 10 s
  (`0x310`).
- Tank float protection, the pump test's own timeout, and the dash's arm and test gates.
- The board's own RGB LED shows the state: green ready, blue spraying, red flashing for
  something to fix ([docs/ESP32_HARDWARE.md](docs/ESP32_HARDWARE.md#the-led-on-the-board)).

## CAN bus interface

Classical CAN, 500 kbit/s, standard IDs, schema 2 of the shared header. Command and
knock acknowledgements are on `0x30A`; `0x306` is for configuration broadcast
acknowledgements.

Sent by this controller:

- `0x300` meth state, `0x303` extended sensors, `0x307` knock state, `0x308` knock
  faults, `0x30B` knock live hook, `0x30C`/`0x30D` knock configuration pages, `0x30E`
  knock detail, `0x30F` pulse tuning, `0x302` start report, `0x310` black box, `0x681`
  firmware update replies.

Received:

- MicroSquirt broadcast (above), `0x309` engine runtime from the dash (RPM backup),
  `0x301` commands, `0x304` configuration, `0x305` configuration requests, `0x680`
  firmware update.

For `0x300` and knock event payloads, boost fields are gauge boost in kPa, clamped to
`0` below the barometer.

`0x307` knock state payload, sent every 50 ms:

- `B0` status flags: bit0 enabled, bit1 signal valid, bit2 warning active, bit3 critical active, bit4 baseline learned, bit5 sensor fault, bit6 clipping detected
- `B1` knock energy, 0-255
- `B2` baseline, 0-255
- `B3` threshold, 0-255
- `B4` event count, wraps 0-255
- `B5` last event RPM / 100
- `B6` last event boost kPa
- `B7` the share of the sound in the knock band

`0x308` knock fault payload: `B0` fault code, `B1` severity, `B2` data0, `B3` data1.

`0x30B` payload: `B0` flags (bit0 enabled, bit1 armed, bit2 detected, bit3 sensorFault,
bit4 clipping, bit5 warning, bit6 critical, bit7 baselineLearned), `B1` live knock
level, `B2` adaptive threshold, `B3` adaptive baseline, `B4` event count, `B5` resting
level (counts of a 10-bit ADC on 5 V, /16), `B6` raw sample /16, `B7` largest swing.

`0x30C` payload: `B0` flags (bit0 enabled, bit1 autoCenterFromBore), `B1` threshold
offset, `B2` adaptive multiplier x10, `B3` min RPM /100, `B4` min MAP kPa, `B5` debounce
ms /10, `B6` gain x10, `B7` centre frequency /100 Hz.

`0x30D` payload: `B0` bandwidth /100 Hz, `B1` sample rate /100 Hz, `B2` samples per
update, `B3` bias alpha x1000, `B4` RMS alpha x100, `B5` envelope alpha x100, `B6` bore
mm, `B7` reserved.

Knock tuning commands on `0x301`: `0x40` enable, `0x41` threshold offset, `0x42`
adaptive multiplier x10, `0x43` minimum RPM /100, `0x44` minimum MAP kPa, `0x45`
debounce ms /10, `0x46` gain x10, `0x47` centre frequency /100 Hz, `0x48` bandwidth
/100 Hz, `0x49` frequency from the bore or found from the engine, `0x4A` clear knock
events; `0x4B`-`0x4D` amplifier gain, bore and frequency search (extension 6).
Replies on `0x30A`: `B0` command, `B1` status (`0` OK, `1` unsupported, `2` invalid
length, `3` value clamped), `B2` applied value, `B3` schema version.

## Knock listening (when built in)

Every sample, 38 thousand a second, goes through band-pass filters on the cylinder's
two resonances ([include/knock_dsp.h](include/knock_dsp.h)); the real knock frequency is
found from the spectrum ([include/knock_spectrum.h](include/knock_spectrum.h)) and the
amplifier's gain is set automatically through an X9C104 digital potentiometer
([include/knock_agc.h](include/knock_agc.h), [include/x9c.h](include/x9c.h)). The
circuit, its bench tests and what the owner's LTspice simulation shows when replayed
through this code ([tools/ltspice](tools/ltspice)) are in
[docs/ESP32_HARDWARE.md](docs/ESP32_HARDWARE.md).

## Tests

The logic that decides what the pump does is pure C++ with no Arduino calls, tested on
a PC. The workflow runs every one of these on each change:

```bash
for name in test_meth_tune test_meth_pulse test_esp32_inputs test_reset_guard test_black_box \
            test_status_light test_knock_agc test_knock_dsp test_knock_spectrum; do
  g++ -std=c++11 -O2 -Wall -I include test/$name/test_main.cpp -o /tmp/$name && /tmp/$name
done
g++ -std=c++11 -O2 -Wall -I include test/test_injection_controller/test_main.cpp \
    src/injection_controller.cpp src/app_config.cpp -o /tmp/injection && /tmp/injection
```

- `test_meth_tune`: the pulse timing, RPM and temperature gates, dose limit, spray time
  limit, ramp, early start, hot air, setting limits and the thermistor curve.
- `test_meth_pulse`: full on or off, the soft start at each start and not between
  cycles of one run, the dose limit by time, the pump test.
- `test_esp32_inputs`: the ECU broadcast, the pressure and temperature conversions, the
  intake sensor found on either input, the float switch.
- `test_reset_guard`, `test_black_box`, `test_status_light`: recovery after resets, the
  black box, the LED.
- `test_knock_*`: the knock listening, frequency finder and automatic gain.
- `test_injection_controller`: arming, tank and sensor failsafes, overboost.

`platformio-tests.ini` has the same as PlatformIO native environments.

## Build and upload

```bash
platformio run -t upload            # esp32s3_water_meth, the default
platformio run -e esp32s3_safety_test -t upload   # bench: make it fail on purpose
```

The bench test environments (`esp32s3_pump_test`, `esp32s3_inputs_test`,
`esp32s3_knock_test`, `esp32s3_knock_gain_test`, `esp32s3_safety_test`) are described
in [docs/ESP32_HARDWARE.md](docs/ESP32_HARDWARE.md). Flash `esp32s3_water_meth` again
afterwards. In the car, new builds go in from the dash over CAN.
