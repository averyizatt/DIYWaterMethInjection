# DIY Water/Meth Injection (ESP32-S3)

Starter repository for an ESP32-S3 based water/methanol injection controller using:
- GM MAP sensor (boost reference)
- Float level sensor (tank low safety)
- 72W diaphragm pump + nozzle
- Custom PCB + housing

This repo is intentionally a **framework starting point** (architecture + stubs), not a finished tuned controller.

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

## Current Firmware Behavior (Scaffold)

- Reads MAP + float sensor through placeholder/stub drivers.
- Runs a basic control loop.
- Enables pump only when:
  - mode allows injection
  - MAP is above boost threshold
  - tank is not low
- Uses placeholder PWM command logic as a framework.

## Planned Next Steps

1. Implement actual GM MAP calibration curve and validation.
2. Implement real float sensor debounce/filtering.
3. Add real pump PWM output + frequency/current checks.
4. Add config persistence (NVS) and runtime tuning interface (Serial/BLE/Wi-Fi).
5. Add failsafe strategy (sensor fault, dry tank, overrun timeout, limp output).
6. Add data logging for tuning (MAP, duty, mode, fault state).

## Build

This scaffold uses PlatformIO + Arduino framework:

```bash
pio run
```

Upload (when hardware is connected):

```bash
pio run -t upload
```
