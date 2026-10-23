# Architecture

```
          +------------------------------ Application/app.c ---------------------------------+
          |  cooperative scheduler: poll sensors, zero procedure, 50 Hz output, 10 Hz temp, 1 Hz status |
          +----+----------------+-----------------+--------------------+--------------------+--+
               |                |                 |                    |                    |
        Drivers/BMP581   Drivers/MS5611     Drivers/RSC          Services/SensorManager   Drivers/CAN
        (static A)       (static B)         (differential)        health, fusion, zeroing  DroneCAN encoder
               \               |                /                 uses Services/Filters         |
                \              |               /                  Services/AirData (IAS,CAS,TAS,altitude)
                 +----- Drivers/Common/adb_bus.h (HAL boundary, mockable) -----+                |
                                          |                                                      |
                              Platform/stm32h7  (SPI + CS GPIO, 1 MHz timer, FDCAN)  <-----------+
```
Rule: everything above `adb_bus.h` is portable C11 with no STM32 includes and is exercised on a PC.

## Module responsibilities
| Module | Responsibility | Key design decision |
|---|---|---|
| `Drivers/MS5611` | reset, PROM + CRC4, D1/D2 conversion state machine, int64 compensation incl. 2nd order | never reads before the *maximum* conversion time (datasheet: early read returns 0 and corrupts the conversion) |
| `Drivers/BMP581` | startup checks, config, burst read, Q16/Q6 decode | no calibration needed (on-chip compensation); drdy via clear-on-read INT_STATUS prevents duplicate samples |
| `Drivers/RSC` | EEPROM read + parse, ADC init, T/P conversion cycle, polynomial compensation, zero correction | EEPROM (mode 0) and ADC (mode 1) are two `adb_bus_t` objects; double-precision maths |
| `Services/AirData` | IAS, CAS, TAS, ISA pressure altitude | explicit invalid/negative/over-range handling; limits documented in the header |
| `Services/Filters` | time-step-aware low-pass, stuck detector, spike gate with step confirmation | no hidden state, no allocation |
| `Services/SensorManager` | per-channel health, staleness, dual static fusion, zeroing statistics, node health | see "Static-pressure strategy" |
| `Drivers/CAN` | single-frame DroneCAN encoders, float16 | spec-verified IDs/layout; not a full stack (see below) |
| `Application` | scheduling, diagnostics events, publish rules | invalid data is withheld, never published |
| `Platform/stm32h7` | HAL glue | all board specifics isolated in `board_config.h` |

## Periodic tasks (`app_step`, called from the main loop at >= 1 kHz)
| Task | Period | Work | Failure behaviour |
|---|---|---|---|
| sensor state machines | every call | 0-1 SPI transaction per sensor per call | bus error -> counted, machine restarts its cycle, SensorManager marks health |
| output | 20 ms | fuse, IAS, altitude, publish StaticPressure + IndicatedAirspeed | invalid -> message withheld, NodeStatus health changes |
| temperature | 100 ms | StaticTemperature (sensor temperature, **not** outside air) | withheld if no valid sensor |
| status | 1 s | NodeStatus (health, mode, fault flags in vendor code) | -- |
| re-init | 1 s per failed sensor | blocking init, ~3 ms (MS5611), ~8 ms (RSC EEPROM read + delays) | stays failed, retried |

Execution time and jitter have **not been measured** (no target). Estimate: a 4-byte SPI transaction at 5 MHz is ~10 us on the wire;
RSC EEPROM read (454 B) ~0.8 ms; so the main loop should iterate far faster than the 20 ms output period.
Memory (measured on host, 64-bit): `app_t` 1464 B, platform SPI buffers 960 B static. Target (32-bit) sizes will be smaller.

## Static-pressure strategy (engineering decision, thresholds provisional)
1. A sensor is *usable* if its last result is fresh (< 200 ms), passed spike + stuck checks, and is not FAILED.
2. Both usable and |A-B| <= 150 Pa: inverse-variance blend (BMP581 0.41 Pa, MS5611 1.2 Pa 1-sigma -> BMP581 weight ~8x).
3. Both usable but disagree: hold the previous output for 3 cycles (glitch rejection), then trust the sensor *closer to the last fused value*
   (continuity), inflate the published variance, latch a DISAGREE flag until the sensors agree within 75 Pa for 20 cycles.
   No history to arbitrate with -> output INVALID (never a coin toss).
4. One usable -> that sensor alone + SINGLE flag. None -> INVALID.
Why not plain averaging: one biased or frozen sensor would corrupt altitude by its full error divided by two, undetected.

## Airspeed
`V = sqrt(2 q / rho0)` is **indicated** airspeed at ISA sea-level density, valid for incompressible flow (error vs the compressible
CAS formula: +0.1 % at 31 m/s, +1.0 % at 99 m/s). TAS and CAS functions exist but are not published (TAS needs outside-air temperature,
which the box does not measure). Negative dp below -10 Pa is flagged as reversed tubes; |dp| < 3 Pa reports 0 m/s.
Zero offset: RSC total error band is +/-0.75 %FSS = +/-90 Pa uncorrected, +/-0.25 %FSS = +/-30 Pa after auto-zero (datasheet Table 6),
equal to ~12 m/s vs ~7 m/s apparent speed at standstill. Hence the boot-time zero procedure (rejected if |offset| > 150 Pa or
the standard deviation > 3 Pa, i.e. wind on the probe).

## DroneCAN scope
Implemented: NodeStatus (341), StaticPressure (1028), StaticTemperature (1029), IndicatedAirspeed (1021) as single-frame broadcasts.
Not implemented (use libcanard + dronecan_dsdlc for these): reception, services (GetNodeInfo, parameters, firmware update), dynamic
node-ID allocation, multi-frame messages such as RawAirData. Consequence: the node is visible and streams data, but cannot be configured
over the bus and may not be listed with a name/version in tools that require GetNodeInfo.
