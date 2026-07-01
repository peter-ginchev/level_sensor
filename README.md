# level_sensor

Firmware for a **Particle Photon 2** that automatically controls a water pump for a
pressurised water system (e.g. a well/borehole pump feeding a membrane pressure
tank). It decides when to run the pump from three sensors — **water level**, **line
pressure**, and **flow** — shows live status on an e-paper display, and publishes
telemetry to the Particle Cloud.

The design goals, in priority order:

1. **Always have water under pressure** — never withhold the pump when there is real demand.
2. **Protect the equipment** — no dry running, and a hard cap on over-pressure.
3. **Avoid wear** — keep pump starts low (target ≤ ~20 per hour) without hurting goals 1–2.

## Hardware

| Function | Device / Pin |
|---|---|
| MCU | Particle Photon 2 (platform `p2`), external Wi-Fi antenna |
| Analog front-end | NCD PR33-8 with TI **ADS1115** 16-bit ADC (I²C), gain ×2 |
| Water level | Hydrostatic (submersible) 4–20 mA sensor on ADC ch **0** (~0–10 m) |
| Line pressure | 0–6 bar 4–20 mA sensor on ADC ch **1** |
| Flow | Reed-switch pulse water meter on pin **D10** (10 pulses/L) |
| Pump | Relay on pin **S4** (drives pump power) |
| Display | Pervasive Displays 4.17" e-paper (`eScreen_EPD_417_KS_0D`) over SPI |

## How it works

The main loop runs roughly every **500 ms** (`DELAY_MS`). On each pass it reads every
sensor, asks each one for a vote, arbitrates them into a single pump command, applies
run-time / anti-short-cycle rules, drives the relay, and refreshes the display.

### Sensor votes

Every sensor returns one of three states — `STOP`, `OK_TO_STOP`, or `START`:

| Sensor | `START` (wants pump on) | `STOP` (must turn off) | Otherwise |
|---|---|---|---|
| **Water level** | — | level < **1.5 m** (dry-run protection) | `OK_TO_STOP` |
| **Pressure** | running & p < **4.0 bar** (cut-out not yet reached), or stopped & p < **2.5 bar** (cut-in) | p > **5.5 bar** (emergency over-pressure) | `OK_TO_STOP` |
| **Flow** | flow > **12 L/min** (real demand) | — | `OK_TO_STOP` |

### Arbitration

The votes are combined with a strict priority each cycle:

1. **Any `STOP` → force the pump off immediately** (safety wins; bypasses all timing).
2. Else **any `START` → run the pump**.
3. Else (**all `OK_TO_STOP`**) → the pump may stop.

So the pump runs while pressure is below cut-out **or** there is meaningful flow, and
the "natural" stop happens only once pressure has recovered to cut-out and flow is low
— i.e. the system is satisfied.

### Minimum run time

Once started, the pump is held on for at least **45 s** (`MIN_ON_TIME`) before a
"natural" off is honoured, so a brief dip can't produce an ultra-short run.

### Anti-short-cycle hysteresis (graduated, recency-weighted)

Rapid on/off cycling wears the motor. To damp it *without* ever withholding water, the
controller can **bridge** brief gaps by keeping an already-running pump on a little past
the point it would otherwise stop — but only as much as recent cycling justifies:

- It keeps a rolling **one-hour history of starts** and scores it with a **recency
  weight** (a start "now" counts ≈ 1, one an hour ago ≈ 0).
- The allowed bridge time is `clamp((weightedStarts − FLOOR) × MS_PER_START, 0, 2 min)`.
- An **isolated long run** is a single start → score ≈ 0 → **no bridge**, it just stops.
  Genuine short-cycling → higher score → longer bridge, up to the 2-minute cap.
- The bridge only ever **extends a running pump**; real restarts still come from a
  `START` vote. The 5.5 bar emergency stop remains the hard backstop.

This means a long, high-demand run stops promptly once demand ends, while a burst of
small starts earns a graduated extension to consolidate them into fewer, longer runs.

## E-paper display

The screen shows, top to bottom: **Level**, **Pressure**, **Flow**, and **Pump** state
(with the reason it's on: `time` = min-run, `hist` = hysteresis bridge). Status lines
show reed-sensor debug, the hourly **ON counter** and last **run time**, and a
timestamped clock (local time is **UTC+2** with EU/Bulgaria DST handling).

## Cloud telemetry

Once per minute the firmware publishes 60-second averages to the Particle Cloud:

| Event | Meaning | Unit |
|---|---|---|
| `depth_mm` | Water level | millimetres |
| `pressure_mbar` | Line pressure | millibar |
| `flow_lpm` | Flow rate | litres/min |

View them live at [console.particle.io](https://console.particle.io).

## Configuration & tuning

The main tunables live as constants in `src/level_sensor.cpp`:

| Constant | Default | Purpose |
|---|---|---|
| `PUMP_ON_PRESSURE` | 2500 mbar | Cut-in pressure (start when stopped) |
| `PUMP_OFF_PRESSURE` | 4000 mbar | Cut-out pressure (stop target while running) |
| `PUMP_STOP_PRESSURE` | 5500 mbar | Emergency high-pressure stop |
| Level `STOP` threshold | 1500 mm | Dry-run protection cut-off |
| Flow `START` threshold | 12 L/min | Demand detection |
| `MIN_ON_TIME` | 45 s | Minimum run once started |
| `HYST_MAX_BRIDGE_MS` | 120 s | Max hysteresis extension |
| `HYST_BRIDGE_FLOOR` | 3.0 | Weighted recent starts below this → no bridging |
| `HYST_MS_PER_START` | 20 s | Extension earned per weighted start above the floor |

Sensor calibration is set where the sensors are constructed in `setup()`: the level
channel uses a multiplier (`117` → matches the displayed reading), and the pressure
channel a `-130` mbar (−0.13 bar) offset.

## Building & flashing

Built for the **`p2`** platform (Photon 2). Using Particle Workbench (VS Code) or the
CLI:

```
particle compile p2
particle flash <device>            # or "Particle: Cloud Flash" in Workbench
```

Monitor logs and cloud events with:

```
particle serial monitor --follow
```

### Libraries and CI

Third-party libraries are vendored under `lib/` as git submodules (see `.gitmodules`).
After cloning:

```
git submodule update --init --recursive
```

The Adafruit submodules map their whole upstream repo into `lib/<name>/src/`, which also
brings their `examples/` folders. Those examples aren't part of this firmware and don't
all compile, so they're excluded from the build:

- **Locally**: `particle.ignore` filters `lib/*/src/examples/**` and `lib/*/examples/**`.
- **In CI**: `.github/workflows/main.yaml` strips `examples/`, `test/`, `tests/`
  directories under `lib/` before the cloud buildpack runs (which does not honour
  `particle.ignore`).

GitHub Actions compiles the firmware on every push to `main` and uploads the binary as a
build artifact.

## Project structure

```
src/level_sensor.cpp        # all application logic (sensors, control, display, telemetry)
lib/                        # vendored libraries (submodules): ADS1X15, BusIO, PDLS e-paper
project.properties          # Particle project metadata
particle.ignore             # local-build source exclusions
.github/workflows/main.yaml # CI: compile for p2
```

## Author

Peter Ginchev — project firmware version `1` (`PRODUCT_VERSION(1)`).
