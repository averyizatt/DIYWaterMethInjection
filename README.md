# Water-Meth Controller Module

This module runs the standalone water-meth controller firmware and publishes state over the shared CAN contract.

## Analog sensor support in-module

The analog thermistor and pressure support is implemented in this module (not in CCM runtime logic).

### Thermistor channels (NTC, passive 2-wire)

Implemented channels:

- IAT
- Engine bay temperature
- Cabin temperature
- Ambient temperature

Wiring per channel:

```text
3.3V -> 10k pullup -> ADC node -> NTC sensor -> GND
```

The module performs:

- oversampled ADC reads
- voltage/resistance conversion
- Steinhart-Hart temperature conversion
- smoothing filter
- open/short/out-of-range fault detection

### Pressure channels (3-wire transducers)

Implemented channels:

- Oil pressure
- Fuel pressure
- Meth pressure
- Boost reference pressure
- Spare pressure 1
- Spare pressure 2

Sensor output scaling:

```text
Sensor OUT -> 220R protection -> 10k -> ADC node -> 20k -> GND
```

The module performs:

- ADC node voltage read + reconstruction of sensor output voltage
- configurable 0.5V–4.5V transfer conversion to PSI
- optional calibration scale/offset
- smoothing and fault detection

## CAN telemetry

- `0x300` (`ID_ENGINE_METH_STATE`) now publishes real IAT and engine-bay temperature values.
- `0x303` (`ID_ENGINE_SENSOR_EXT`) publishes pressure channels plus ambient/cabin temperatures and analog fault flags.
- `0x307` (`ID_ENGINE_KNOCK_STATE`) publishes knock status, energy/baseline/threshold, and rolling event counters.
- `0x308` (`ID_ENGINE_KNOCK_FAULT`) publishes knock warning/critical and sensor health faults.

## Nano ESP32 pinout used by this firmware

Source of truth: `modules/water-meth/include/pins.h`

| Function | Pin |
| --- | --- |
| MAP / boost sensor ADC | A0 |
| Knock sensor ADC | A1 |
| Float switch digital | D3 |
| Pump PWM output | D2 |
| Warning LED output | GPIO17 |
| Native TWAI TX | D5 |
| Native TWAI RX | D6 |
| IAT thermistor | D4 |
| DHT11 input | D6 |
| Engine bay thermistor | GPIO8 |
| Cabin thermistor | GPIO9 |
| Ambient thermistor | GPIO10 |
| Oil pressure ADC | A4 |
| Fuel pressure ADC | A5 |
| Meth / BoostRef / Spare1 / Spare2 pressure ADC | GPIO13 / GPIO14 / GPIO15 / GPIO16 |

### MCP2515 SPI wiring note

If using an SPI CAN module instead of native TWAI, use the Nano ESP32 hardware SPI bus (this is a mutually-exclusive wiring profile with the default GPIO8 sensor usage):

- MOSI = D11
- MISO = D12
- SCK = D13
- CS / INT default to D10 and D7 in this module
- RST can be assigned to any free GPIO if used

The default pin map uses CS=D10, INT=D7, level switch=D3, pump PWM=D2, knock=A1, and IAT=D4.
Native TWAI RX (D6) and DHT11 (D6) share a pin in this mapping, so they are mutually exclusive unless you remap one.

## Knock subsystem in-module

Knock detection now runs entirely in this firmware:

- ADC sampling + centered absolute energy computation
- adaptive baseline + dynamic threshold (`multiplier * baseline + offset`)
- warning/critical event windows with cooldown
- sensor health checks (low activity/disconnect + clipping)
- response modes:
  - `LOG` (telemetry/fault reporting only)
  - `WARN` (currently same runtime behavior as LOG, reserved for stricter warning-only policy)
  - `FORCE` (force minimum spray on critical knock)
  - `SHUTDOWN` (force pump off on critical knock)

### Serial commands for knock tuning

- `KNOCK SHOW`
- `KNOCK ENABLE 0|1`
- `KNOCK BOOSTKPA <kpa>`
- `KNOCK MULT <value>`
- `KNOCK OFFSET <value>`
- `KNOCK MODE LOG|WARN|FORCE|SHUTDOWN`
- `KNOCK RESET`

### Optional/unsupported sensors

- If a pressure channel is physically unwired, leave it disabled in firmware config to avoid persistent fault bits.
- MAP/boost control uses the dedicated MAP ADC (`GPIO1 / A0`); the boost-reference pressure channel remains optional telemetry.

## Build

Build this module independently with PlatformIO from this directory:

```bash
cd modules/water-meth
pio run -e arduino_nano_esp32
```
