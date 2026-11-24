# First bench test - DroneCAN GUI Tool + USB-CAN adapter

**Purpose:** a short, ordered procedure for the first power-up of the air data box on a bench, with the expected result and a pass/fail
criterion at each step. Stop at the first failing step and send the results (section 8) to the firmware developer.

**What is verified and what is not.** The GUI Tool steps (serial port + bitrate dialog, local node ID, Bus Monitor) follow its public
documentation (ArduPilot wiki, ARK Electronics guide). The expected values come from the firmware design and the host tests. **Nothing in
this procedure has been run on hardware yet.** Items that depend on information still pending from the board owner are written as
`<BITRATE>` etc.

## 0. Things to know before starting
| Topic | What to expect |
|---|---|
| Node ID | The firmware currently uses the fixed node ID **42** (provisional). Give the GUI Tool a *different* local node ID (e.g. 100). |
| GetNodeInfo | **Not implemented yet** (requested as a later task). The node appears in the node list but with **no name/version**, and "Fetch All" parameters will not work. This is expected, not a failure. |
| Bitrate | Must equal the firmware's FDCAN bit timing. Value pending from the board owner: `<BITRATE>`. A mismatch shows up as no node and no frames. |
| First seconds | After a **power-up** the firmware measures the differential-pressure zero for about 2-4 s (NodeStatus mode MAINTENANCE). During that time no IndicatedAirspeed / RawAirData is sent. **Keep both RSC ports open to still ambient air** (no blowing, no wind, no tubes attached). |
| Cold boot | The zero is measured only after a **power-on** reset. A debugger or NRST reset counts as a warm boot and skips it. **Remove and re-apply power** before the test (do not just press reset). |
| Fault flags | The NodeStatus *vendor-specific status code* carries the firmware fault flags (table in section 6). No other log is available yet. |

## 1. Equipment
- The board, powered from its intended supply (target rail 3.3 V for the sensors).
- USB-CAN adapter supported by the GUI Tool: an SLCAN-compatible adapter (appears as a serial port) or SocketCAN on Linux. Specific adapter models were not verified here.
- Twisted pair for CAN_H / CAN_L plus a ground reference; **120 ohm termination at both ends** of the bus (many adapters have a switch).
- ST-Link (or equivalent) to flash the Cortex-M7 image.
- PC with the DroneCAN GUI Tool. Windows: installer from the DroneCAN project. Linux: `python3 -m pip install dronecan_gui_tool dronecan`.
- Pressure source for the dp test: a syringe or just a gentle breath through a short silicone tube; for the static check, any barometer (a phone barometer is enough for a rough comparison).

## 2. Safety checks with power OFF
1. Supply polarity and voltage: the **RSC is not reverse-polarity protected** and the 3.3 V variant is specified for 3.0-3.6 V (Honeywell Table 5, note 3). Measure the rail before connecting.
2. No short between 3.3 V, GND, CAN_H, CAN_L (multimeter).
3. CAN: resistance between CAN_H and CAN_L should be about 60 ohm with both terminations present (about 120 ohm with one).
4. Pressure ports of the RSC are open to ambient; no tube attached for steps 1-6.

## 3. Procedure
| # | Action | Expected result | Pass if |
|---|---|---|---|
| 1 | Apply power. Measure the sensor rail. | 3.3 V +/- 5 % | within range |
| 2 | Flash the firmware to the Cortex-M7 (ST-Link), then **remove and re-apply power**. | board boots, no overheating | no abnormal heat |
| 3 | Connect the adapter (CAN_H, CAN_L, GND), termination on. Start the GUI Tool, select the adapter's serial port, set the CAN bitrate to `<BITRATE>`, click OK, then tick/press "Set" for the local node ID (e.g. 100). | tool connects without error | connected |
| 4 | Look at the node list. | a node with **ID 42** appears; name field empty (no GetNodeInfo); uptime counts up once per second; health OK | node visible, uptime increases |
| 5 | Watch NodeStatus mode for the first ~5 s. | INITIALIZATION (under 1 s) -> MAINTENANCE (zeroing, 2-4 s) -> **OPERATIONAL** | reaches OPERATIONAL |
| 6 | Tools -> Bus Monitor -> press the camera icon to capture for 10 s, then pause. | messages: NodeStatus (341), StaticPressure (1028), StaticTemperature (1029), IndicatedAirspeed (1021), RawAirData (1027) | all five types present, decoded without errors |
| 7 | Count messages in the 10 s capture (RawAirData: count complete transfers, not frames). | about 10 / 500 / 100 / 500 / 200 | within +/- 10 % |
| 8 | Read StaticPressure. Compare with a reference barometer. | local ambient pressure (about 95-102 kPa near sea level; lower at altitude) | within a few hundred Pa of the reference (MS5611 datasheet: +/-150 Pa at 25 C, 750 mbar; a phone barometer is itself only accurate to a few hundred Pa) |
| 9 | Read the NodeStatus vendor code. | **0x0000** (no fault flags) | 0 means both static sensors are in use and nothing is flagged |
| 10 | Read RawAirData `differential_pressure` and IndicatedAirspeed with the ports open. | dp within +/-5 Pa of 0 after the zero; IAS 0.0 m/s | |dp| < 5 Pa |
| 11 | Sign check: blow gently into **port 1** (the pressure port, datasheet "Port 1 - Port 2"). | dp becomes **positive** (tens to a few hundred Pa), IAS rises; release -> returns to about 0 | positive for port 1; if negative, the tubes are reversed (flag DP_NEGATIVE appears below -10 Pa) |
| 12 | Static sensor failure: disconnect the MS5611 I2C (SCL or SDA), wait 2 s. Reconnect, wait 3 s. | flags B_FAILED + STATIC_SINGLE, health WARNING, StaticPressure continues without a gap; after reconnect health returns to OK within about 3 s | continuity of StaticPressure, recovery without reset |
| 13 | Differential sensor failure: disconnect the RSC (SPI2 CS or SCK), wait 2 s. Reconnect, wait 5 s. | IndicatedAirspeed and RawAirData **stop**, flag DIFF_FAILED, health ERROR; after reconnect they resume (the zero from boot is kept) | messages stop and resume |
| 14 | Other static sensor: repeat step 12 for the BMP581 (SPI4). | flag A_FAILED + STATIC_SINGLE, StaticPressure continues | as step 12 |
| 15 | Reset behaviour: press NRST (warm boot) and watch. | node reappears; **no zeroing** (mode goes straight to OPERATIONAL); IAS/RawAirData immediately, with the uncorrected sensor offset (up to +/-90 Pa, about 12 m/s apparent) | documents the warm-boot safeguard |
| 16 | **Optional, only with the board owner's agreement:** bus-off test - short CAN_H to CAN_L for about 3 s, then release. | node silent while shorted, then resumes within about 1-2 s without a reset | node resumes |
| 17 | **Optional soak:** leave running for 80 minutes. | NodeStatus uptime never goes backwards (the old firmware wrapped at 71.6 min) | monotonic |

## 4. Expected message set
| Message | Data type ID | Rate | Content |
|---|---|---|---|
| NodeStatus | 341 | 1 Hz | uptime, health (0 OK .. 3 CRITICAL), mode, vendor code = fault flags |
| StaticPressure | 1028 | 50 Hz | float32 Pa (fused), float16 variance (about 0.15 Pa^2 when both sensors agree) |
| StaticTemperature | 1029 | 10 Hz | float16 K - **sensor temperature, not outside air** |
| IndicatedAirspeed | 1021 | 50 Hz | float16 m/s (sea-level-density formula), float16 variance |
| RawAirData | 1027 | 20 Hz | 3 frames per transfer: static + differential pressure (Pa), sensor temperatures (K), unused temperature = NaN |

Estimated bus load: 171 frames/s, about 2 % at 1 Mbit/s and about 4-5 % at 500 kbit/s.

## 5. NodeStatus health and mode
| Health | Meaning in this firmware |
|---|---|
| 0 OK | both static sensors in use, differential sensor valid, no flags |
| 1 WARNING | degraded: single static sensor, static disagreement, failed sensor, negative dp, dp range error, or a rejected zero |
| 2 ERROR | a required quantity is missing (no static pressure, or differential sensor invalid) |
| 3 CRITICAL | no valid air data at all |

Mode: 1 INITIALIZATION until the first valid static pressure; 2 MAINTENANCE while zeroing; 0 OPERATIONAL otherwise.

## 6. Vendor-specific status code (fault flags)
Read the 16-bit value in hex and test the bits:

| Bit | Mask | Flag | Meaning |
|---|---|---|---|
| 0 | 0x0001 | A_FAILED | BMP581 declared failed |
| 1 | 0x0002 | B_FAILED | MS5611 declared failed |
| 2 | 0x0004 | DIFF_FAILED | RSC declared failed |
| 3 | 0x0008 | A_STALE | BMP581: no fresh sample for 200 ms |
| 4 | 0x0010 | B_STALE | MS5611 stale |
| 5 | 0x0020 | DIFF_STALE | RSC stale |
| 6 | 0x0040 | A_STUCK | BMP581 returning identical values |
| 7 | 0x0080 | B_STUCK | MS5611 returning identical values |
| 8 | 0x0100 | STATIC_DISAGREE | the two static sensors differ by more than 150 Pa |
| 9 | 0x0200 | STATIC_SINGLE | only one static sensor in use |
| 10 | 0x0400 | STATIC_NONE | no valid static pressure |
| 11 | 0x0800 | DP_NEGATIVE | dp below -10 Pa (reversed tubes or large offset) |
| 12 | 0x1000 | ZERO_FAILED | zero rejected (airflow or offset above 100 Pa, or noisy) |
| 13 | 0x2000 | ZEROING | zero measurement in progress |
| 14 | 0x4000 | DP_RANGE | dp out of range |

Examples: `0x0000` all good. `0x0202` = MS5611 failed, running on the BMP581 alone. `0x1000` = the boot zero was rejected.

## 7. Troubleshooting
| Symptom | Likely cause | What to check |
|---|---|---|
| Nothing at all on the bus, not even NodeStatus, with a healthy CAN link | `adb_app_init()` refused to start because TIM2 is not a 1 MHz time base (wrong prescaler for the clock tree) | TIM2 prescaler = timer clock / 1 MHz - 1 (docs/CUBEIDE_SETUP.md section 4); a debugger shows `adb_app_init()` returning `ADB_ERR_CONFIG` |
| No node, no frames at all | bitrate mismatch, missing termination, transceiver unpowered, FDCAN not started, CAN_H/CAN_L swapped | `<BITRATE>`; 60 ohm across the bus; scope CAN_H/CAN_L |
| Frames seen but not decoded | wrong bitrate on one side, or a 5-bit transfer-ID problem | bus monitor error counters; bitrate |
| Node appears for a few seconds, then vanishes | bus-off or stuck TX FIFO (no other node acknowledging) | adapter acknowledges frames? termination? the firmware restarts the controller after about 100 ms-1 s |
| Only NodeStatus, no pressure data | sensors failed to initialise | vendor code: B_FAILED / A_FAILED / DIFF_FAILED; sensor supply; bus wiring |
| StaticPressure present, no IAS / RawAirData | RSC invalid, or still zeroing, or dp negative beyond tolerance | vendor code bits 2, 12, 13, 11 |
| dp not near 0 at rest, IAS jumping | boot zero was skipped (warm boot) or rejected (wind/offset) | power-cycle in still air; bit 12 |
| Uptime restarts from 0 | board resets (watchdog/brown-out) | supply stability (all rails) |

## 8. What to send back
1. The node-list screenshot (steps 4-5) and the Bus Monitor capture (step 6).
2. The hex vendor code at steps 9, 12, 13, 14.
3. Measured values: rail voltage, StaticPressure vs reference, dp at rest, dp when blowing into port 1.
4. Any step that failed, with the bitrate and adapter used.
