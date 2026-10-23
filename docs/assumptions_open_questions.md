# Assumptions, Open Questions, Known Limitations

Every item below is a **provisional engineering decision**, not a customer requirement. Each lives behind a config struct or one
platform file so it can change without touching driver logic.

## A. Assumptions made (no board / specification was provided)
| # | Assumption | Where it lives | Risk if wrong |
|---|---|---|---|
| A1 | Drivers are bus-agnostic through `adb_bus_t` | `Drivers/Common/adb_bus.h` | none |
| A2 | Reference wiring: all three sensors on one SPI bus, separate chip selects (RSC needs CS_ADC and CS_EE) | `Platform/stm32h7/board_config.h` | pin map from schematic |
| A3 | Sensors at 3.3 V | hardware | level shifting |
| A4 | BMP581: OSR_P x4, OSR_T x1, 50 Hz, normal mode | `app_default_cfg()` | tune on bench |
| A5 | MS5611: OSR 4096, temperature every 8th sample | `app_default_cfg()` | lower rate (~50 Hz) |
| A6 | RSC: 90 SPS normal mode, temperature every 10th sample, CRC accepted-but-flagged | `app_default_cfg()` | rate/noise trade-off |
| A7 | DroneCAN node ID 42, priorities 16/24, rates 50/10/1 Hz, classic CAN; bitrate set in CubeMX (**customer value needed**) | `app_cfg_t`, `dronecan.h` | message mapping/rates |
| A8 | Airspeed: rho0 = 1.225 kg/m3, IAS only published | `airdata_default_cfg()` | systematic error vs true airspeed |
| A9 | Static fusion thresholds: 150 Pa disagreement, 3-cycle confirm, 200 ms stale, 5 errors to fail | `sm_default_cfg()` | nuisance flags / missed faults |
| A10 | IAS variance from dp sigma 30 Pa (RSC 0.25 %FSS after zero), floor 5 m/s | `app_cfg_t.dp_sigma_pa` | consumer filtering weights |
| A11 | Boot-time autozero within 10 s; rejected when |offset| > 150 Pa or std > 3 Pa | `sm_cfg_t`, `app_cfg_t` | no zero if wind at boot |
| A12 | BMP581 CHIP_ID = 0x50 (derived from the register reset value) | `bmp581.c` | init fails with ADB_ERR_ID on first power-up |

## B. Questions for the company (send as one email)
1. Hardware: dev board or custom PCB? Schematic / pin list (sensor bus, chip selects, CAN transceiver)?
2. DroneCAN: required messages and fields, node ID (static or dynamic), CAN bitrate, publish rates? Is GetNodeInfo / parameter service required?
3. Is +/-60 mbar (about 99 m/s IAS) sufficient for the platform?
4. Required latency/rate for airspeed and altitude? Is pressure altitude required on the bus (the standard air-data set has no altitude message)?
5. Acceptance test: instruments and tools the reviewer will use?
6. Toolchain: CubeIDE project or CMake? Which core runs the firmware?

## C. Known limitations (be upfront about these in a review)
1. **Nothing has run on hardware.** All evidence is host simulation. Simulators were written by the same author from the same datasheets, so a shared misreading would not be caught.
2. Not compiled with the real STM32 HAL or arm-none-eabi-gcc. `Platform/stm32h7` was syntax-checked against a hand-written stub only.
3. RSC open items U1-U3 (CRC init/range, meaning of Traw, zero-target generalisation) are unresolved; they are surfaced at runtime but only a real chip closes them.
4. MS5611 CRC-4 follows application note AN520 (not provided); the test is self-consistency only.
5. DroneCAN: minimal single-frame encoder, not libcanard. No reception, services, dynamic node-ID, or RawAirData.
6. No timing measurements, no watchdog test, no stack/WCET analysis.
7. TAS/CAS exist but are not published; temperature published is the sensor temperature, not outside air.
8. Altitude is ISA pressure altitude (reference 101325 Pa), computed and available in `app_t` but not on the bus (no standard message).
9. No claim of MISRA, ISO 26262, DO-178C or any safety-standard compliance.
10. Platform re-initialises the SPI peripheral when switching between RSC ADC (mode 1) and other devices (mode 0); a dedicated SPI instance for the RSC would remove this overhead.
