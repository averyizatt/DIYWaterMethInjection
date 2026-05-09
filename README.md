# DIY Water/Meth Injection (ESP32-S3)

Starter repository for an ESP32-S3 based water/methanol injection controller using:
- GM MAP sensor (boost reference)
- Float level sensor (tank low safety)
- 72W diaphragm pump + nozzle
- Custom PCB + housing

This repo now includes a **working conservative firmware baseline** you can build on.

## Goals

- Read manifold pressure (MAP) and tank level
- Trigger injection when in boost
- Support tunable operating modes and configurable water:meth ratio settings
- Keep the project structured for safe expansion (failsafes, logging, calibration, tuning UI)

## Safety Notes

- This is experimental DIY control code and **not automotive-certified**.
- Add hard safety interlocks in hardware where possible.
- Bench test with fused power, pressure-safe plumbing, and leak checks before road testing.
- Use conservative default thresholds and verify sensor calibration.

## Project Structure

```text
include/
  actuators.h             # Pump output abstraction
  app_config.h            # Tunable config + defaults
  injection_controller.h  # Core state/control interface
  modes.h                 # Mode and ratio helpers
  pins.h                  # Hardware pin map
  sensors.h               # MAP/float sensor interfaces
src/
  actuators.cpp
  app_config.cpp
  injection_controller.cpp
  main.cpp                # Application lifecycle
  sensors.cpp
platformio.ini
```

## Current Firmware Behavior

- Reads MAP sensor ADC and converts voltage to kPa using configurable linear calibration.
- Converts manifold absolute pressure to **boost PSI** using configurable barometric reference.
- Reads float switch with debounce.
- Disables injection on low fluid and asserts warning output.
- Accepts user blend entry over serial:
  - `WATER <liters>`
  - `METH <liters>`
- Computes methanol percent using:
  - `Meth% = Vm / (Vm + Vw) * 100`
- Stores blend values in ESP32 NVS (Preferences).
- Progressive boost-based duty using:
  - `Duty = K(1 - M)(Pboost - Pstart)` (clamped)
  - forced full duty at configured full-boost threshold.
- Outputs periodic serial diagnostics.

## Conservative Default Control Targets

- No injection below ~3.5 psi (default `startPsi`)
- Progressive injection through mid boost
- Full injection near ~7.5 psi (default `fullPsi`)
- Duty gain reduced as methanol fraction increases
- Intended for supplemental cooling/knock margin, not meth-dependent tuning

## Adjustable Constants

Most important constants are in:
- `include/app_config.h`
- `src/app_config.cpp`

Example defaults include:
- GM 3-bar style MAP calibration **approximate example** (`0.5 V to 4.5 V`, `20 kPa to 312 kPa`) — verify against your exact sensor datasheet
- Pump PWM frequency `100 Hz` (within recommended 50-200 Hz range)
- `startPsi = 3.5`, `fullPsi = 7.5`
- `gainK = 30.0` (% duty per psi)

## Build

This scaffold uses PlatformIO + Arduino framework:

```bash
pio run
```

Upload (when hardware is connected):

```bash
pio run -t upload
```

## First Setup

1. Set real board pin assignments in:
   - `include/pins.h`
2. Verify MAP calibration values match your specific GM sensor.
3. Confirm float switch polarity (`floatActiveLow`) and wiring.
4. Verify pump driver hardware is rated for startup current (8-12A possible).

## Serial Commands

- `HELP`
- `SHOW`
- `MODE OFF`
- `MODE BOOST`
- `MODE PRIME`
- `WATER 1.5`
- `METH 0.5`
