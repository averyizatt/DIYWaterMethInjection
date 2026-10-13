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
- X9C104 digital potentiometer module (100 k, 100 positions) for the knock amplifier's
  gain, and a rail-to-rail op amp that runs on 3.3 V (MCP6002 or similar)
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
| Knock gain potentiometer CS | 14 | to the X9C104 module's CS |
| Knock gain potentiometer INC | 18 | to its INC |
| Knock gain potentiometer U/D | 21 | to its U/D |

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

### Testing the driver by itself

Before anything else is wired, the environment `esp32s3_pump_test` drives only the
pump output: off for the first 10 seconds after start-up, then 10 % duty, 20 %, and so
on to 100 %, two seconds each, then off until the board's reset button is pressed. It
uses the same pin and PWM frequency as the real firmware, and prints each step on the
serial monitor (115200). Have fluid in the pump: it should not run dry. Flash
`esp32s3_water_meth` again afterwards.

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
The amplifier's gain is set by the firmware: see the next section.

**Tank float**: between GPIO7 and ground, no resistor needed. Closed means fluid;
an open switch or a broken wire reads as an empty tank.

## Knock amplifier with automatic gain

Engine noise at the knock sensor is many times larger at high RPM than at idle. With a
fixed gain the signal is either a few ADC counts at idle or clipping at speed. The
firmware sets the gain itself through an X9C104 digital potentiometer, keeping normal
engine noise in the lower part of the ADC's range with room above it for knock
([include/knock_agc.h](../include/knock_agc.h)).

### Circuit

- **Bias point:** two 100 k resistors from 3.3 V to ground, with 10 uF across the lower
  one. Everything rests at this 1.65 V.
- **Input:** the sensor through a capacitor to the op amp's + input, which also goes to
  the bias point through a resistor. Together with the sensor's own capacitance this
  is a high-pass. The simulated values, 1.5 nF and 47 k with a 1.35 nF sensor and
  cable, put its corner at 4.8 kHz: it removes rumble well, but it sits just under the
  6 kHz knock frequency, costs about 2 dB there, and moves with the sensor's
  capacitance, which is not known for this part. 10 nF and 100 k put the corner near
  1.3 kHz, clear of the knock band.
- **Gain:** the potentiometer's VW and VL terminals are the feedback resistor, from the
  op amp's output to its - input (leave VH unconnected). 10 k from the - input to the
  **bias point, not to ground**. Gain is then 1 at the bottom of the potentiometer and
  11 at the top, and the output rests at 1.65 V at every gain. If that 10 k goes to
  ground, the op amp amplifies the bias itself and the output sits on the rail.
- **Op amp and output, as simulated in LTspice:** an LM358 on 5 V. Its output goes from
  about 0.05 to 3.6 V, which is more than the ESP32's pin may see, so it reaches GPIO1
  through a divider: 10 k in series, 10 k to ground, 2.2 nF to ground (a 14 kHz
  low-pass). The pin then rests at 825 mV and can go from 25 to 1800 mV. Those two
  limits are `KNOCK_ADC_LOW_MV` and `KNOCK_ADC_HIGH_MV` in `include/pins_esp32.h`; the
  gain control needs them to recognise clipping, because 1800 mV is nowhere near the
  top of the ADC's own range.
- **A better divider for the same op amp:** 3.3 k in series, 15 k to ground, 3.9 nF to
  ground. The pin rests at 1.35 V and can go from 40 to 2950 mV: 64 % more signal for
  the ADC to work with, and still safe. Set the two limits to 40 and 2950.
- **Or a rail-to-rail op amp on 3.3 V** straight to the pin through 1 k, with 10 nF to
  ground: limits 20 and 3100.
- **X9C104 module:** VCC to **5 V** (the chip does not run on 3.3 V), GND to ground.
  CS, INC and U/D come straight from GPIO14, 18 and 21: it accepts 3.3 V signals and
  sends nothing back.

If you use a different resistor than 10 k, change `KNOCK_GAIN_AT_BOTTOM` and
`KNOCK_GAIN_AT_TOP` in `include/pins_esp32.h`: gain at the top is 1 + 100 k / that
resistor.

**How many steps the potentiometer has is a setting**, `KNOCK_GAIN_TOP_STEP` in
`include/pins_esp32.h`, set to 30: the X9C104's datasheet says 100 positions, but this
module measures 30 steps from end to end. It matters, because the chip is stepped up
and down by pulses and cannot be asked where it is: the firmware counts. With the wrong
number it would lose its place. The bench test below shows whether the number is right.

The firmware walks the wiper to the bottom at start-up and again whenever the engine is
stopped. It also never lets the chip save its position: its memory is good for 100,000
saves, and the gain changes many times a minute.

### What the firmware does with it

- Every half second it looks at the typical size of the signal. Too small: gain up
  1.5 dB. Too large: down 1.5 dB. In between it does nothing.
- The moment the signal touches the ADC's limits it takes 3 dB off at once. It watches
  every sample for that, not only the ones analysed for knock.
- It remembers the gain that suited each 1000 RPM, and goes straight back to it when
  the engine returns to that speed.
- Passes the knock detector calls loud are left out, so knock does not turn the gain
  down on itself.
- When the gain changes, the detector's learned "normal" levels are rescaled by the
  same amount, so knock right after a change is still caught. The one pass (20 ms)
  that spans the change is not judged.

Faults it reports through the usual knock sensor fault:

- clipping at the lowest gain (the amplifier is too hot for this engine: use a larger
  resistor than 10 k);
- nothing heard at the highest gain above 2000 RPM (sensor or wiring).

If changing the gain shows no effect on the signal (potentiometer not fitted, or a
control wire off), it says so on the serial monitor and leaves the gain alone until the
next start. For an amplifier with no potentiometer, build with `-DKNOCK_GAIN_POT=0`.

The numbers on the dash's knock page (level, normal level, threshold) are measured
after the amplifier, so all three move together when the gain changes. The gain itself
is on the serial line (`kgain`), not on the dash yet.

### Testing it by itself

The environment `esp32s3_knock_gain_test` steps the potentiometer from bottom to top in
eleven stops, three seconds each, and prints at each one the position, the gain it
should give, the resistance to expect across VW and VL, and what the ADC hears
(resting level, swing, the level in each knock filter, the real sample rate, clipping).

1. Module on its own, meter across VW and VL: the resistance should follow the printed
   value, 0, 10, 20 ... 100 k. That proves the three control wires. **If the last stop
   reads about 30 k instead of 100 k**, the part has the datasheet's 99 steps after
   all: set `KNOCK_GAIN_TOP_STEP` to 99.
2. In the circuit, with a steady signal (tap near the sensor, or feed a tone): the
   swing should grow with the gain. A tone swept through 6 kHz and 9.9 kHz should peak
   in "knock" and "second".
3. "rest" should match the circuit (about 825 mV with the 10 k / 10 k divider) and stay
   put at every gain.
4. "rate" should read about 38460.

Flash `esp32s3_water_meth` again afterwards.

## How knock is listened for

The Nano looks at one 3.3 ms burst of the knock signal in every 20 ms, so a single
knock is caught about one time in six. The ESP32 filters **every sample**, 38 thousand
a second, all the time ([include/knock_dsp.h](../include/knock_dsp.h)):

- Three filters run on each sample: a band-pass on the cylinder's knock frequency
  (about 6 kHz for the 96 mm bore), a band-pass on its next resonance 1.66 times
  higher (about 9.9 kHz), and a wide one from 1 to 17 kHz to compare against.
- Every 20 ms a window the length of one knock is slid over the result and the loudest
  position taken. The window is about 30 degrees of crank rotation: 2.5 ms at 2000 RPM,
  1 ms at 5000. It reaches across the joins between passes, so no knock falls in a gap.
- A hit is: louder than the level learned for this RPM (times the multiplier, plus the
  offset: the same two settings as before), narrow-band, **and** standing out from the
  rest of its own 20 ms. Knock is a burst; a noise floor that has simply come up, or a
  steady whine at the knock frequency, is not.
- Two hits within 0.6 s raise the warning, as before. A single strong hit raises it at
  once when it is unmistakable: clearly narrow-band, or the second resonance is ringing
  above its own learned level too.
- The filters are designed for the sample rate the ADC is measured to deliver, and are
  redesigned if that turns out to differ from what was asked. The knock frequency
  setting means what it says either way.

In the PC simulation (synthetic engine noise with valve clicks): 300 of 300 single
knocks at random moments heard, 30 of 30 episodes of sustained knock caught in about
40 ms, 30 of 30 single hard knocks caught at once, and no false alarm in ten minutes of
normal running at three engine speeds. That is a simulation, not the engine: the
multiplier and offset still have to be set on the car.

What reaches the dash is unchanged in meaning: level, learned level and threshold in
the same units, and the share of the sound that is in the knock band.

## What the LTspice simulation shows

`Knock_Bosch_0261231183_Sweep` (five feedback resistors, six source amplitudes, 3000
RPM with three knocks) was replayed through the firmware's own code: the simulated ADC
voltage sampled as the ESP32 samples it, through the knock filters, the detector and
the gain control with the 30-step potentiometer (`tools/ltspice`, below).

With a **fixed gain**, each resistor only works over a narrow range of sensor signal:
too little gain and the knocks are lost in the measurement, too much and the amplifier
clips. Knocks caught of the three (W = warning raised, k = the amplifier clips at the
knocks, c = it clips all the time):

| Sensor signal | gain 1.47 | gain 2.0 | gain 3.2 | gain 5.7 | gain 11 |
|---|---|---|---|---|---|
| x0.25 | 0 | 0 | 0 | 1 | 3 W |
| x0.5 | 0 | 0 | 1 | 3 W | 3 W |
| x1 | 1 | 3 W | 3 W | 3 W | 3 W k |
| x2 | 3 W | 3 W | 3 W | 3 W k | 3 W k |
| x5 | 3 W | 3 W k | 3 W k | 0 k | 0 c |
| x10 | 3 W k | 3 W k | 0 k | 0 c | 0 c |

With the **potentiometer under automatic control** all three knocks are caught at every
amplitude, a 40 to 1 range, without clipping except slightly at the largest:

| Sensor signal | Gain it settles at | Normal swing at the pin | Knocks caught |
|---|---|---|---|
| x0.25 | 11 (the top) | 51 mV | 3 of 3 |
| x0.5 | 11 (the top) | 103 mV | 3 of 3 |
| x1 | 8.0 | 150 mV | 3 of 3 |
| x2 | 4.0 | 150 mV | 3 of 3 |
| x5 | 1.7 | 156 mV | 3 of 3 |
| x10 | 1.0 (the bottom) | 187 mV | 3 of 3, clipping at the knocks |

What follows from it:

- **Gain 1 to 11 (RG = 10 k) suits a sensor signal between about the simulation's x1 and
  x10.** Below x1 the gain control runs out of gain; above x10 it runs out of
  attenuation. Which of those the real engine is, only the engine can say: read
  `kgain` and `kpeak` on the serial line at idle and at high RPM. If `kgain` sits at
  11, use a smaller RG (4.7 k gives 1 to 22). If it sits at 1 and still clips, the
  signal needs attenuating before the amplifier.
- **The detector's threshold offset mattered more than the gain.** With the Nano's
  offset of 8 no knock was caught at x0.25 and one of three at x0.5; with 2 all are.
  The ESP32 firmware now starts at 2.
- The weakest of the three knocks is only 1.1 to 1.2 times the threshold with the
  default multiplier of 2.5. In the simulation the knocks are about three times the
  normal ringing at each firing; a real engine's ratio is what the multiplier has to
  be set for.

What the simulation is not: a model of this exact sensor. Its capacitance (1.15 nF) and
sensitivity are borrowed from a related Bosch part, as the simulation's own notes say,
and its signal sizes and knock sizes are invented. Two measurements would make it real:
the sensor's capacitance with a meter, and `kpeak` / `kgain` from the running engine.

To replay a simulation again after changing it (the LTspice `.raw` file must have been
saved with `V(adc)`, `V(vout)` and `V(sensor)`):

```bash
python tools/ltspice/ltraw.py path/to/Knock_Bosch_0261231183_Sweep.raw waves
g++ -std=c++17 -O2 -I include tools/ltspice/ltreplay.cpp -o ltreplay
./ltreplay waves            # as built: 10k/10k divider, offset 2, gain 1 to 11
./ltreplay waves 0.82 2 22  # 3.3k/15k divider, offset 2, gain 1 to 22
```

## First start-up checks

Open the serial monitor (USB, 115200). Once a second the controller prints a line:

```
HB t=12000 map=98.4 boostPsi=-0.3 baro=100.5 rpm=850 duty=0% hold=1 mode=0 tankLow=0 can=1 oilPsi=42.0 fuelPsi=39.5 pre=24.1 knock=0 krest=825mV knockHz=38460 klvl=14.2 kbase=13.1 k2=6.0 kshare=0.17 kgain=6.3 kpeak=150mV rxOvf=0
```

- `can=1`: the CAN module answered.
- `map` and `rpm` without a `!` after them: the MicroSquirt's broadcast is arriving.
  A `!` means stale or missing, and the controller will not inject.
- `oilPsi`, `fuelPsi`, `pre` without `!`: those sensors are reading.
- `krest` is where the knock amplifier rests at the pin: 825 mV for the simulated
  circuit.
- `klvl` is the knock-band level of the last 20 ms and `kbase` the level learned as
  normal for this RPM; `k2` is the second resonance and `kshare` how much of the sound
  is in the knock band (ordinary noise: under about 0.25; knock: well over 0.35).
- `kgain` is the amplifier gain the firmware has chosen and `kpeak` the typical swing
  it sees either side of rest; with the engine running `kpeak` should settle between
  about 130 and 320 mV for the simulated circuit (16 to 40 % of the room between rest
  and the nearer limit). `kgain=1.0(fixed)` means gain control is off. **`kpeak`
  divided by `kgain` is the real size of the sensor's signal**, the number the
  simulation had to guess.
- `knockHz` is the knock samples a second the ADC really delivers, about 38460. The
  filters follow it by themselves; if it reads far from that, say so, because it also
  tells how fine the listening is.

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
