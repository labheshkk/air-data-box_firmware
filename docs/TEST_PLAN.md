# Test and Acceptance Plan

Legend: **[EXECUTED]** = run in the authoring sandbox (host, no hardware). **[PROPOSED]** = to be run on hardware; no result exists.

## A. Executed (host)
`./scripts/check.sh` -> gcc `-fanalyzer` + strict warnings on all modules, then 8 suites under AddressSanitizer + UBSan:

| Suite | Assertions | Covers |
|---|---|---|
| test_ms5611 | 55 | datasheet worked example (2007 / 100009), 2nd-order cold cases, range/invalid, CRC fail, early-read protection, bus errors |
| test_bmp581 | 46 | SPI dummy read, reset/ID/NVM checks, config registers, Q-format decode, drdy/oor flags |
| test_airdata | 45 | IAS/CAS/TAS/altitude vs Python reference, negative/NaN/over-range, bad config |
| test_rsc | 78 | Table 3-3 temperature examples, CRC check values, EEPROM parse/CRC policy, polynomial vs Python, zero correction, faults |
| test_filters | 26 | LPF step response, stuck detector, spike gate with step confirmation |
| test_sensor_manager | 172 | fusion, failure/recovery, stale, arbitration by continuity, unresolved, stuck, zeroing outcomes, node health |
| test_dronecan | 4645 | float16 vectors + exhaustive round-trip, CAN ID/tail byte/payload vectors, transfer-ID wrap |
| test_app | 2358 | end-to-end with simulated sensors: autozero, 50/10/1 Hz rates, sensor failure + recovery, disagreement, missing sensor at boot, wind during zero, TX overrun |

`./scripts/mutation_check.sh`: 19 deliberately injected bugs (sign errors, wrong coefficients, removed checks...) -> **19 killed, 0 survived**.
Two real defects were found by these tests during development (zero-retry timeout; event flood) and fixed.

## B. Proposed hardware acceptance tests (in order; stop at the first failure)
| # | Test | Method | Pass criterion |
|---|---|---|---|
| 1 | Power + levels | 3.3 V rail, CS idle high, MISO not floating | rails within datasheet range |
| 2 | Bus signature | logic analyser on each CS | SPI modes: MS5611/BMP581 mode 0, RSC EEPROM mode 0 / ADC mode 1; SCLK <= 5 MHz |
| 3 | Identity | read BMP581 CHIP_ID, MS5611 PROM+CRC, RSC EEPROM catalog string | ID 0x50 (derived!), CRC ok, catalog = RSCDJJI060MDSE3 |
| 4 | Static pressure vs reference | compare to a calibrated barometer, both sensors | difference within datasheet accuracy (MS5611 +/-1.5 mbar, see datasheet) |
| 5 | RSC zero | both ports open to ambient | `ZERO_OK`; residual < 5 Pa; **record RSC `crc_verified` and whether Traw interpretation yields ~0 Pa** (open items U1, U2) |
| 6 | RSC span | apply known dp (e.g. 500 Pa via manometer/calibrator) | within +/-0.75 %FSS (90 Pa) uncorrected, +/-30 Pa after zero |
| 7 | Sign | blow into port 1 | dp positive, IAS rises (otherwise tubes reversed) |
| 8 | CAN | DroneCAN GUI Tool / CAN analyser at agreed bitrate | node appears, NodeStatus 1 Hz, StaticPressure/IAS 50 Hz, values decode, no bus errors |
| 9 | Fault injection | disconnect each sensor in turn (hot) | health/flags per ARCHITECTURE.md; recovery after reconnect without reset |
| 10 | Disagreement | apply a pressure step to one static port only (tube kink) | DISAGREE flag, output follows continuous sensor |
| 11 | Timing | toggle a GPIO in `app_step`; scope | loop period << 20 ms; output jitter < 1 ms (**unmeasured**) |
| 12 | Soak | 24 h run | zero watchdog resets, bounded error counters |
