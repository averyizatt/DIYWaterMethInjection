# ESP32-S3 water/meth and knock controller: hardware

This board replaces the Arduino Nano. Firmware: `src/main_esp32.cpp`, PlatformIO
environment `esp32s3_water_meth`.

## What changed from the Nano board

| | Nano board | ESP32-S3 board |
|---|---|---|
| Pump | mechanical relay, long pulses | MOSFET with PWM: the pump runs slower or faster |
| Manifold pressure | MAP sensor on the board | from the MicroSquirt over CAN; no sensor here |
| Engine speed | forwarded by the dash | from the MicroSquirt over CAN (the dash's value is the backup) |
| Air temperature for the temperature features | sensor before the nozzle, else the dash's value | sensor before the nozzle, else the MicroSquirt's own |
| Knock, oil pressure, fuel pressure, tank float | here | here, unchanged |
| New firmware | USB cable | from the dash over CAN, or USB |

Every CAN message, command and setting is the same as before. The dash does not need
to know which board is fitted; it reads the two new settings (lowest and highest pump
duty) if the controller reports them.

**No fresh data from the MicroSquirt means no injection.** The controller needs the
ECU's manifold pressure and RPM, each less than half a second old. A pump test still
works without the ECU.

## Parts

- ESP32-S3-DevKitC-1 (the 8 MB flash version the board definition expects)
- MCP2515 CAN module with an **8 MHz** crystal (the common blue module)
- Logic-level N-channel MOSFET module rated 15 A or more that switches fully from a
  3.3 V signal, or a bare MOSFET that is specified fully on at a 2.5-3.3 V gate
- Flyback diode across the pump, rated at least the pump's current (for example a 10 A
  Schottky)
- Automotive 12 V to 5 V converter for the DevKit
- Resistors: two 10 k + two 20 k (pressure dividers), one or two 2.49 k (thermistor
  pull-ups), three 10 k (pull-downs), 100 ohm (gate), 1 k + 2 k (CAN module MISO)
- Fuse for the pump, the size the pump's maker gives

## Pin map

| Signal | GPIO | Wiring |
|---|---|---|
| CAN module CS | 10 | |
| CAN module SI (MOSI) | 11 | |
| CAN module SCK | 12 | |
| CAN module SO (MISO) | 13 | through the 1 k / 2 k divider below if the module runs on 5 V |
| Knock amplifier | 1 | 0-3.1 V, resting near 1.65 V |
| Oil pressure | 2 | 0.5-4.5 V sensor through 10 k / 20 k |
| Fuel pressure | 4 | 0.5-4.5 V sensor through 10 k / 20 k |
| Air temperature before the nozzle | 5 | 3.3 V - 2.49 k - pin - sensor - ground |
| Air temperature after the nozzle (spare) | 6 | same; leave unconnected if not fitted |
| Tank float switch | 7 | switch to ground; closed while there is fluid |
| Pump MOSFET gate | 15 | 100 ohm in series, 10 k from gate to ground |
| Warning LED | 16 | high = warning; LED and resistor to ground |
| Master pump relay (optional) | 17 | high = pump allowed; 10 k to ground at the driver |

Not used on purpose: GPIO 0, 3, 45, 46 (start-up pins), 19 and 20 (USB), 26-37 (flash).

**Nothing above 3.3 V may reach an ESP32 pin.** That is the main difference from the
Nano when moving the sensors over.

## Power

Ignition-switched 12 V through a fuse to the 5 V converter; its output to the DevKit's
`5V` pin and to the 5 V sensors. One ground point shared by the DevKit, the CAN
module, the sensors and the MOSFET's source. The pump's ground current must not flow
through the sensor grounds: run the MOSFET source to the chassis with its own wire.

## Pump

```
+12 V -- fuse -- [master relay contact, optional] --+-- pump (+)
                                                    |
                                   diode: stripe ---+
                                   diode: other end -+-- pump (-)
                                                     |
                                              MOSFET drain
GPIO15 -- 100 ohm --+-- MOSFET gate
                    |
                  10 k
                    |
ground -------------+-- MOSFET source
```

- The 10 k from gate to ground is not optional. While the ESP32 is starting, being
  flashed or has reset, its pins float, and this resistor is what keeps the pump off.
- The diode goes across the pump with its stripe to the +12 V side. Without it the
  MOSFET will be damaged by the motor's voltage spike at every switch-off.
- PWM is 200 Hz. The firmware sets the pump no lower than the "lowest pump duty"
  setting, so it never sits stalled.

### Master relay (recommended)

A MOSFET that fails usually fails **switched on**, and the pump would then run for as
long as the ignition is on. Putting the relay you already have in the pump's 12 V feed
removes that: GPIO17 closes it when a spray starts and opens it three seconds after
the spray ends. It clicks once per spray, never with the PWM. Drive the relay coil the
way the Nano's output drove it (transistor or relay module), with a 10 k from the
driver's input to ground. If you do not fit it, leave GPIO17 unconnected.

The firmware also switches the pump and the relay off by itself, within a quarter of
a second, if its main loop ever stops running, and the gate pull-down covers a reset.

## CAN module

Same kind of MCP2515 module as the sensor gateway: 8 MHz crystal, 500 kbit/s. The
module's INT pin is not connected. Fit the module's termination jumper only if this
controller is at an end of the bus.

If the module is powered from 5 V (the usual TJA1050 version needs that), its SO
output is a 5 V signal: put 1 k in series and 2 k to ground at GPIO13.

In TunerStudio the MicroSquirt must be broadcasting on CAN: either the dash broadcast
("simplified", IDs 0x5E8/0x5E9) or real-time data at base 1520 (0x5F0/0x5F2) - the
dash already needs one of them. The controller understands both.

## Sensors

**Oil and fuel pressure** (0.5-4.5 V, 0-100 psi, 5 V supply): sensor signal - 10 k -
pin, and 20 k from the pin to ground. The pin sees two thirds of the sensor voltage,
3.0 V at 100 psi. These are the dividers the Nano board used; they move over as they
are. A sensor that is unplugged or shorted reads as a fault, not as zero pressure.

**Air temperature** (GM-style thermistor): 3.3 V - 2.49 k - pin - sensor - ground.
On the Nano the 2.49 k went to 5 V; here it must go to the DevKit's `3V3` pin.

**Knock**: the amplifier's output must rest near 1.65 V (two equal resistors from
3.3 V to ground give that) and swing between 0 and 3.1 V. On the Nano it rested at
2.5 V on a 5 V supply. The dash's knock page shows the resting level and the largest
swing, and the controller reports a sensor fault when the resting level is far off.

**Tank float**: between GPIO7 and ground, no resistor needed. Closed means fluid;
an open switch or a broken wire reads as an empty tank.

## First start-up checks

Open the serial monitor (USB, 115200). Once a second the controller prints a line:

```
HB t=12000 map=98.4 boostPsi=-0.3 baro=100.5 rpm=850 duty=0% hold=1 mode=0 tankLow=0 can=1 oilPsi=42.0 fuelPsi=39.5 pre=24.1 knock=0 bias=512 knockHz=19231 rxOvf=0
```

- `can=1`: the CAN module answered.
- `map` and `rpm` without a `!` after them: the MicroSquirt's broadcast is arriving.
  A `!` means stale or missing, and the controller will not inject.
- `oilPsi`, `fuelPsi`, `pre` without `!`: those sensors are reading.
- `bias` near 512: the knock amplifier is resting at mid-scale.
- `knockHz` within a few percent of 19231: the knock frequency setting means what it
  says. If it reads something else, the knock frequency is off by the same ratio -
  report the number before relying on knock detection.

Then, engine off, run a pump test from the dash and check the pump runs and stops.

## Firmware updates over CAN

The controller can take new firmware over CAN like the gateway and the taillight
controller (module number 3). It refuses to start an update, and drops one that is
running, when the pump is on or boost is within 1 psi of the start threshold. During
an update the pump is held off. A new firmware runs on trial: if the dash does not
confirm it within 90 seconds, the controller goes back to the previous one.

The first flash is by USB. After that the dash does it: every change to this
repository is built on GitHub and published as the `watermeth-latest` release, the
dash downloads it during **Update now**, and **Dash management > Support** offers
"Install water/meth firmware" when the build on the Pi differs from the one the
controller reports. A build flashed from VS Code reports build 0, so the dash will
offer the published build over it.

While the Nano board is fitted the dash shows "Water/meth firmware: no answer (normal
with the Nano board)". The Nano cannot take firmware over CAN.

## The Nano board in the meantime

Nothing was taken away from the Nano. Its firmware (`src/main_nano.cpp`, environment
`arduino_nano`, still the default) builds as before and behaves as before; the dash
works with either board and tells them apart by itself. The Nano firmware now also
stores the two pump-duty settings, which it does not use, and a tune saved by the
earlier Nano firmware is kept when the new one is flashed.
