# The water/meth controller PCB

Firmware for the water/meth controller PCB, rev D: a Seeed XIAO ESP32-S3 with a pump driver
and current sense, a solenoid valve output, two intake air inputs, a 0.5–4.5 V pressure input,
a tank level switch, 12 V and sensor-supply monitors and CAN, all on one 14-way AMPSEAL
connector. The board itself (schematic, layout, parts, simulation) is its KiCad project,
WaterMethController, and its README.

This is a different board from the **WaterMethPrototypeV2** board in the car (an ESP32-S3
DevKitC-1 with an MCP2515 module, [ESP32_HARDWARE.md](ESP32_HARDWARE.md)), and each has its
own firmware. Neither runs on the other's board.

## The builds (PlatformIO environments)

| Environment | Board | What it is for |
| --- | --- | --- |
| `WaterMethPrototypeV2` | DevKitC-1 prototype (the car) | The default. Published as `watermeth-latest`; the dash installs it over CAN |
| `WaterMethPrototypeV2-PumpTest`, `-InputsTest`, `-KnockTest`, `-KnockGainTest`, `-SafetyTest` | DevKitC-1 prototype | Bench tests ([ESP32_HARDWARE.md](ESP32_HARDWARE.md)) |
| `WaterMethPCBStandalone` | PCB | **Any car, no dash.** Engine data from the car's CAN, set up from a phone over the board's own Wi-Fi |
| `WaterMethPCBFrogDash` | PCB | **With a Frogdash dash.** The dash's CAN protocol, exactly as the prototype speaks it |

In VS Code, the PlatformIO sidebar lists each environment with its tasks (Build, Upload,
Monitor). From a terminal:

```bash
pio run -e WaterMethPCBStandalone -t upload     # or WaterMethPCBFrogDash
pio device monitor
```

The first time, flash by USB-C on the XIAO. If it is not found, hold its **B** (boot) button
while plugging it in. After that:

- **Stand-alone:** use **Firmware** on the web page.
- **Frogdash:** new firmware comes from the dash over CAN, once the dash knows about the PCB
  (see below).

Each build carries a mark (`pcb_board.h`) and refuses firmware without it. An image for the
prototype board, or for the other PCB build, is never installed, whichever way it arrives. CI
publishes both PCB images to the `watermeth-pcb-latest` release, never to `watermeth-latest`.

## Wiring (J1, AMPSEAL 14-way)

| Pin | | Pin | |
| --- | --- | --- | --- |
| 1 | Sensor +5 V | 8 | Tank level switch (to ground) |
| 2 | Pump − (switched) | 9 | +12 V in |
| 3 | Pump + and solenoid + (fused 12 V out) | 10 | Intake air 1 |
| 4 | +12 V in | 11 | Intake air 2 |
| 5 | Ground | 12 | Pressure signal (0.5–4.5 V) |
| 6 | CAN H | 13 | Solenoid − (switched) |
| 7 | CAN L | 14 | Sensor ground |

- **12 V:** feed pins 4 and 9 from a fused supply that is switched with the ignition.
- **Intake air:** GM-style thermistors (the usual two-wire intake air sensor). Put one before
  the nozzle, on either input; the board finds which. A second one after the nozzle shows the
  cooling.
- **Pressure input:** either a line pressure transducer (0.5–4.5 V, 100–250 psi) between the pump
  and the nozzle, or (stand-alone only) a MAP sensor. Not both. See
  [The pressure input](#the-pressure-input-line-pressure-and-flow).
- **Solenoid valve:** a normally closed 12 V valve, up to 2 A, at the nozzle end of the line.
  It opens with the pump and shuts the moment the pump stops, so the line does not dribble
  after a pulse.
- **Tank switch:** a float that closes to ground while there is fluid. A broken wire then
  reads as a low tank, which is the safe way round. A float that works the other way is a
  setting.
- **CAN termination (jumper JP4):** fit it only if the board is at an end of the bus. Never on
  a car's OBD-II bus, which is already terminated.

## Stand-alone: any car

The board gets engine data from the car's CAN and decides everything itself.

1. **Connect CAN H and CAN L:**
   - For **OBD-II**: OBD port pin 6 (CAN H) and pin 14 (CAN L). Most cars from 2008 on, and
     many earlier ones, have CAN there.
   - For an **aftermarket ECU**: its CAN bus.
2. **Join the board's Wi-Fi.** Turn the ignition on. On a phone, join **WaterMeth-xxxx** with
   password `watermeth`, then open `http://192.168.4.1`. Set your own Wi-Fi password first (in
   the Wi-Fi section). The Wi-Fi turns off 15 minutes after power-up if nobody is connected,
   unless **Wi-Fi stays on** is set. It comes back at the next power-up.
3. **Engine data on CAN:**
   - **OBD-II** (the default): the board asks the car for engine speed and manifold pressure
     about ten times a second each. It also asks for barometric pressure and intake air
     temperature now and then.
   - **An ECU's CAN broadcast:** pick the ECU. The board only listens and never transmits on
     that bus. The list comes from the Frogdash dash's ECU profiles (each from the maker's
     published CAN document):
     - MegaSquirt
     - Haltech
     - ECUMaster EMU
     - MaxxECU
     - rusEFI
     - Speeduino
     - AEM Infinity
     - FuelTech
     - Holley (Dominator/Terminator X, and Sniper)

     Those not yet proven on a car are marked so. For ECUMaster and rusEFI, give the
     broadcast's base ID if it was moved. The CAN speed is the ECU's own unless set.

   The page says whether frames are arriving, and from when.
4. **Check boost.** With the engine stopped, **Manifold** should read the day's barometer (for
   example, about 101 kPa at sea level and 85 kPa at 5,000 ft), and **Boost** about 0.
5. **No MAP from the car?** Many MAF-only engines give none over OBD-II, and the page says so.
   - Fit a MAP sensor on the pressure input.
   - Set **Manifold pressure from** to the sensor.
   - Pick the sensor (GM 2- or 3-bar, MPX4250), or give its pressure at 0.5 V and at 4.5 V.

   Engine speed still comes from CAN. A MAP sensor with nothing on CAN works too, but then
   there is no engine-speed gate and no dose limit (both need engine speed). Note that OBD-II
   reports manifold pressure only up to 255 kPa (about 22 psi of boost at sea level); past
   that, use a MAP sensor.
6. **Nozzle and mix:**
   - Pick the nozzle's rating and how many nozzles, or better, measure it. Disarmed, engine
     off, catch a **Test spray** in a cup, weigh it, and enter the grams; the page works out
     the flow. (The prototype's "60 ml/min" nozzle measured 160 at its pump's pressure.)
   - Set the methanol share of the mix.
   - Set the **engine size**; the dose limit's fuel estimate scales with it.
7. **Pump drive:**
   - **Full-on pulses** (the default): the pump runs at full pressure for part of each cycle,
     the best spray from an ordinary diaphragm pump.
   - **PWM:** a duty that rises from the start-boost duty to the full-boost duty, for pumps
     that are made for it.
8. **Arm it.** The armed state is kept through power cycles. **Save settings** keeps the
   rest; it is written once the engine is away from boost.

### The web page

Made for a phone first. Try it without a board in the
[live preview](https://averyizatt.github.io/DIYWaterMethInjection/).

| Tab | What is on it |
| --- | --- |
| **Live** | State and why (waiting for boost, tank low, a fault…), **Arm/Disarm**, gauges for boost, line pressure, flow and the pump (duty, amps, valve), engine speed, manifold, barometer, air, supply, tank, test spray, clear faults, save |
| **Tuning** | The pump drive, when to spray, how much (pulse on-times or PWM duties), hot air, limits |
| **Nozzle & fluid** | Nozzle flow from its rating or measured, the pressure it was measured at, the mix, the fluid count and tank |
| **Hardware** | The pressure input (line pressure transducer, MAP sensor or nothing), the solenoid valve, the pump's current, the tank switch |
| **Engine** | Where engine data comes from, and whether it is arriving |
| **Diagnostics** | Trouble codes with their freeze frames; the watchdogs |
| **System** | Tune profiles, settings backup and restore, Wi-Fi, firmware |

Every setting has its limits from the board; a value outside them is clamped, and the page
shows what was applied.

**Tune profiles.** Four named slots (Street, Track, Race, Winter by default; rename them). Each
keeps how the system sprays: all of the tuning, the pump drive, the pressure the nozzle was
measured at, the valve lead and the line pressure limits. It does not keep the wiring, engine
data, the tank or arming, so a profile from one car's setup is safe to load on the same board
later. Loading is refused while spraying, and a loaded profile is saved.

**Backup.** **Download settings** saves every setting as a JSON file; **Restore** puts one
back (on this board or another), setting by setting, through the same limits.

### USB serial

The same settings at 115200 baud: `help`, `show`, `set start_psi 3.5`, `arm`, `disarm`,
`test 3`, `stop`, `clear`, `codes`, `clear_codes`, `refilled`, `save`, `revert`, `defaults`,
`profile save 1 Track`, `profile load 1`, `wifi <password>`.

## The pressure input: line pressure and flow

The pressure input takes one 0.5–4.5 V ratiometric sensor (read as a share of the measured
sensor supply, so the supply's drift cancels). Choose it under **Hardware**:

- **Line pressure transducer** (the usual choice, 100–250 psi) between the pump and the
  nozzle. Set its range. It gives:
  - **No line pressure** and **line pressure too high** checks (below).
  - **Flow from pressure.** A nozzle's flow goes with the square root of the pressure across
    it, so once the nozzle has been measured at a known pressure:

    flow = measured flow × √(line pressure ÷ pressure it was measured at)

    The **measure it** test on the Nozzle tab records the mean line pressure during the test
    spray (after its first second), and **Use the measurement** stores both the flow and that
    pressure. No flow meter is needed. Without a known pressure the flow is taken as the
    nozzle figure whenever the valve is open (for the PWM drive, × √duty).
  - The **fluid count**: the flow added up while the valve is open, since the tank was last
    filled. Set the tank size and press **Tank filled** after filling. It is kept through
    power cycles (written every few minutes away from boost).
  - A transducer that reads open or shorted for a second sets code S10; the pressure checks
    are then off and the flow comes from the nozzle figure.
- **MAP sensor** (stand-alone only), when the car gives no manifold pressure on CAN.
- **Nothing:** no pressure checks; flow from the nozzle figure.

The Frogdash build takes a 250 psi transducer, as the board's README specifies.

## The solenoid valve

A normally closed 12 V valve at the nozzle end of the line, on its own MOSFET.

- It opens only while the pump runs and shuts the moment it stops, so the line holds its
  pressure between pulses and does not dribble after one.
- It is pulled shut by hardware whenever the ESP32 is unpowered, in reset or booting, and
  switched off from registers by the crash hook.
- A pump switch that fails shorted (code P04) still cannot spray: the valve stays shut.
- **Valve lead** (stand-alone, 0–500 ms, default 0). After the pump has been off for over a
  second (the line's pressure has bled away), the valve stays shut for the lead while the
  pump builds pressure, so the first of the spray is atomised. Between pulses no lead is
  applied. A diaphragm pump's own pressure switch or bypass takes the dead-headed moment;
  keep the lead short.
- The valve pin is read back every pass (below).

## Pump current

The pump's current is measured on every 20 ms pass (5 mΩ shunt, INA240, 0.1 V per amp).

- The pump guard's checks use it (below). The over-current limit is a setting (3–10 A) for a
  smaller pump; it is never above the 10 A the board and pump are rated for.
- **Typical at full** is this pump's own current at full drive, averaged past each run's
  start-up surge. Watch it over time: a clogging filter or nozzle draws more, a worn pump
  less. **Peak** is the highest since power-up.
- **At power-up** the current sense is checked with the pump off: over 0.3 A (but under the
  stuck-on 1 A) sets code S12, a current sense reading off.

## With a Frogdash dash

`WaterMethPCBFrogDash` speaks the dash's CAN protocol, exactly as the prototype board does:

- **From the dash:** arming, tuning, the pump test, firmware updates and the start report all
  work as before. Manifold pressure and RPM come from the MegaSquirt broadcast.
- **0x303:** line pressure (up to 127.5 psi, the contract's range).
- **0x300:** the flow status (OK, low flow, no flow).
- **0x302:** the pump's own faults are sent with the contract's existing codes, so the dash's
  alerts and event log show them:
  - PUMP_OVERCURRENT
  - NO_FLOW
  - LOW_FLOW
  - SAFETY_SHUTDOWN
- **Knock:** this board has no knock input. Knock is reported as not enabled.

Connect it to the dash's bus at 500 kbit/s.

**Before the car uses it:** the dash's firmware updates still send the `watermeth-latest`
(prototype) image. The PCB refuses that image: the update then fails with "check failed" and
the running firmware is kept. The dash needs to learn to send `WaterMethPCBFrogDash.bin` to a
PCB.

## What both builds check

### The pump guard (pump_guard.h)

| Check | When | Then |
| --- | --- | --- |
| Short circuit | One reading over 25 A with the pump on | Off at once, latched |
| Over-current | Over the limit (10 A, or lower as set) for 0.3 s while running steadily | Off, latched |
| Pump not connected | Under 0.3 A after 0.6 s running steadily | Off, latched |
| Pump switch stuck on | Current with the pump off | Latched; the valve stays shut |
| No line pressure | Under the set minimum after 1.5 s at full drive (with a line sensor) | Off, latched |
| Line pressure too high | Over the set maximum for 0.5 s: a blocked nozzle | Off, latched |
| 12 V supply | Under 10.5 V, or over 16.5 V for 1 s | Held, not latched; released 0.5 V back inside |

Latched faults stay until **Clear faults** (on the web page or the dash) or a power cycle.

### The watchdogs (watchdogs.h)

Each watches one thing, independently of the logic meant to get it right, with one action:

| Watchdog | Watches | Then |
| --- | --- | --- |
| Task watchdog | Both cores' tasks, 2 s | Restart; the crash hook switches pump and valve off first, from registers |
| Pump keeper | Control setting the pump every pass, from the CAN task on the other core | Pump and valve off after 0.25 s (code W17) |
| CAN task heartbeat | The CAN task (which carries the keeper) still counting | Injection held after 0.3 s (W18); the task watchdog restarts the board at 2 s |
| Output read-back | The pump PWM's duty register and the valve pin, against what was set | A difference for 3 passes: off, latched (W20) |
| Spray budget | The pump's own on-time, filled while on and drained while off at the pulse timing's rate (longest spray per rest): never over the longest spray + 8 s. With no rest set, 3 minutes on without a break | Off, latched (W19). Counted from the output, not from the pulse timing that should have stopped it; that timing's own budget always runs ahead of this one, so a working system never trips it |
| Pass timer | Control passes later than 100 ms | Logged (W16) |
| Settings check | The settings in memory every second (every value in range); the stored copies by CRC at power-up | Reloaded from flash (or defaults), disarmed (W21) |
| Engine data | Manifold pressure outside 10–450 kPa, engine speed over 12,000 rpm | That reading not used (E14) |
| Memory | Free memory under 24 KB (W22) | Under 12 KB the Wi-Fi page is stopped: control first |
| Brownout | The ESP32's own brownout detector | Restart (R24), armed again |

### Trouble codes (diag.h)

Every condition above, and the pump guard's, is kept as a trouble code the way a car's ECU
keeps them: how many times, the first and last (power-up number and seconds since), whether
it is present now, and a **freeze frame** of the last one (boost, engine speed, pump duty and
amps, line pressure, supply, air temperature). Codes are written to flash with a CRC, away
from boost, at most once a minute. They are on the Diagnostics tab and the serial port
(`codes`); `clear_codes` or **Clear codes** empties the list. **Clear faults** (and the
dash's) only releases latched faults; the codes stay.

| Code | Meaning |
| --- | --- |
| P01–P06 | The pump guard's faults (short, over-current, not connected, stuck on, no pressure, over-pressure) |
| S07, S08 | 12 V supply low, high |
| S09 | Sensor 5 V supply out of range |
| S10 | Line pressure transducer open or shorted |
| S11 | MAP sensor open or shorted |
| S12 | Current sense reads high with the pump off |
| E13 | Engine data stopped arriving (armed) |
| E14 | Engine data out of range |
| E15 | CAN bus-off |
| W16–W22 | The watchdogs, above |
| R23, R24 | Restarted after a crash or watchdog; after a brownout |
| T25 | Tank low |

On the Frogdash build a watchdog's latched fault goes to the dash as 0x302 SAFETY_SHUTDOWN
with the code number in byte 2; the pump guard's go with their own codes, as before.

### The prototype's rules

These all come on top of the prototype's rules:

- The pump keeper on the other core.
- The watchdogs.
- The crash hold.
- Armed again after an unasked-for reset.
- Tank protection.
- The pulse timing's limits.
- The five-second pump test, only when disarmed. On the stand-alone build, not with the
  engine running.

**The LED** (the XIAO's orange one) shows:

| Pattern | Meaning |
| --- | --- |
| Steady | Armed and ready |
| Fast flicker | Spraying |
| Flashing | A fault |
| Slow blink | Waiting for engine data |
| A short blip every 2 s | Not armed |

## First power-up on the bench

Nothing of this firmware has run on a real PCB yet. Before the car:

1. **USB only.**
   - At power-up a `SELFTEST:` line reports the pump current at rest, the sensor supply, the
     12 V and that both outputs read back off.
   - The serial heartbeat (`HB …`) should show the sensor supply at about 4.7 V. The supply
     reading is near 0, so a pump test is held for low supply: that is correct.
   - The intake air reading should be about room temperature with a sensor fitted, and `!`
     (no reading) without.
   - Opening and closing the tank switch should change `tankLow`.
2. **12 V, no pump, no valve.** A test spray should trip **pump not connected** after 0.6 s.
   That shows the current sense is working.
3. **12 V with the pump and valve, the nozzle in a cup.**
   - The test spray should run, the valve should click open and shut with it, and **Pump**
     should show the current.
   - Measure the nozzle while you are at it: with the transducer fitted, the page records the
     test's line pressure too.
   - Watch the Diagnostics tab: no W20 (output read-back) code. If one appears, the PWM's
     duty read-back does not behave as expected on this hardware; report it before driving.
4. **CAN.** Stand-alone with OBD-II: the page shows frames arriving and the barometer at key-on.
   Frogdash: the dash sees the board as the water/meth controller.

## Code

- **`src/main_pcb.cpp`:** both builds (`PCB_FROGDASH` 1 or 0).
- **`src/pcb_can.cpp`:** the ESP32's own CAN controller.
- **Pure logic, tested on a PC:**
  - `include/pcb_board.h`: pins.
  - `pcb_sensors.h`: the inputs.
  - `pump_guard.h`: the checks.
  - `pwm_drive.h`: the PWM drive.
  - `pcb_led.h`: the LED.
  - `flow_estimate.h`: flow from line pressure, the fluid count, the valve lead, the pump's typical current.
  - `diag.h`: the trouble codes.
  - `watchdogs.h`: the software watchdogs.
  - `engine_source.h`: ECU broadcasts, OBD-II, a MAP sensor, the barometer.
  - `standalone_settings.h`: the settings.
  - `web_ui.h`: the page.
- **Tests:**
  - `test/test_pcb`.
  - `test/test_ecu_profiles`: checks every ECU signal against the dash's own decoder.
- **`tools/web_ui_preview.py`** builds `preview/index.html`: the web page with a simulated
  controller behind it, published as the [live preview](https://averyizatt.github.io/DIYWaterMethInjection/).
  Run it after changing the page or the settings; CI runs it with `--check`.
- **`include/ecu_profiles.h`** is generated from the dash's profiles. Regenerate it when they
  change: `python tools/ecu_profiles.py <path to frogdash>`.
