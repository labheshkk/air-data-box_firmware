# Datasheet Verification Report

Status legend: **VERIFIED** = read directly from the uploaded manufacturer datasheet (page/section given).
**DERIVED** = follows from a datasheet field but is not stated as a sentence. **UNCONFIRMED** = not in the
provided documents; must be checked on hardware or in another document.

Documents used: TE `MS5611-01BA03` datasheet 06/2017 (2917208.pdf) · Bosch `BST-BMP581-DS004-13`
(bst-bmp581-ds004.pdf) · Honeywell `TruStability RSC Series` 32321348 Issue G (06/24) ·
ST `STM32H755xI` datasheet (uploaded; **not yet reviewed**, FDCAN pin/clock work pending).

## 1. Honeywell RSCDJJI060MDSE3 — decoded from Figure 3 (p.7)

| Field | Value | Status |
|---|---|---|
| Series / package | RSC, **D** = DIP | VERIFIED p.7 |
| Ports | **JJ** = dual radial barbless ports, same side | VERIFIED p.7 |
| Temp. range | **I** = Industrial, compensated −20…85 °C | VERIFIED p.7 |
| Range | **060MD** = **±60 mbar differential** (= ±6000 Pa) | VERIFIED p.7, Table 6 |
| Output | **S** = SPI | VERIFIED p.7 |
| Transfer function | **E** = *external math* (host applies polynomial from EEPROM) | VERIFIED p.7 |
| Supply | **3** = 3.3 Vdc (3.0–3.6 V for ranges ≥ 60 mbar) | VERIFIED p.7, Table 5 |
| Over-pressure / burst | 850 / 1000 mbar | VERIFIED Table 6 |
| Common-mode pressure | 10 000 mbar | VERIFIED Table 6 |
| Total error band / after auto-zero | ±0.75 %FSS / ±0.25 %FSS (FSS = 120 mbar → ±0.9 mbar / ±0.3 mbar) | VERIFIED Table 6 |
| ENOB @ 20 SPS | 15 bit | VERIFIED Table 6 |
| Max SPI clock | 6.66 MHz (min SCLK period 150 ns) | VERIFIED Table 1, Fig. 2 |
| Startup | ≤ 2 ms to accept commands | VERIFIED Table 5 |
| ADC SPI mode | **mode 1 (CPOL 0, CPHA 1)**; EEPROM uses mode 0 or 3 — *different modes on the same pins* | VERIFIED p.24 |
| CS_EE and CS_ADC | must never both be low | VERIFIED p.23–24 |
| Commands | RESET `0x06`; WREG `0x40|RR<<2|NN`; start conversion `0x08`; EEPROM read `0000 X011` + addr | VERIFIED p.23–26 |
| Config bytes | EEPROM addr 61, 63, 65, 67 → ADC regs 0–3 via WREG `0x43` | VERIFIED p.26 |
| Config reg 1 | DR[2:0], MODE[1:0], bit2=1, TS (bit1; 1=temperature), bit0=0 | VERIFIED Table 3-1 |
| Data rates | Normal mode 20/45/90/175/330/600/1000 SPS; Fast 40/90/180/350/660/1200/2000 | VERIFIED Table 3-1/3-2 |
| Temperature word | first 14 bits of 24, two's complement, 0.03125 °C/LSB; Table 3-3 examples | VERIFIED p.26–27 |
| Pressure word | 24-bit two's complement (counts) | VERIFIED p.27 |
| EEPROM layout | range @27 (float), Pmin @31 (float), unit @35, offset coeffs @130, span @210, shape @290 (4 floats each, little-endian), checksum @450–451 | VERIFIED Tables 2-1…2-6 |
| Compensation | Pint1 = Praw − (Σ Off_k·Traw^k); Pint2 = Pint1 / (Σ Span_k·Traw^k); Pfs = Σ Shape_k·Pint2^k; P = Pfs·Prange + Pmin | VERIFIED p.20 |
| CRC | CRC-16-CCITT, polynomial 0x1021 | VERIFIED p.29 |
| CRC **initial value** and which bytes it covers (U1) | not stated in the datasheet (inside Honeywell's sample `crc.c`, not provided). Driver tries init 0x0000 and 0xFFFF over bytes 0..449, reports `crc_verified`; default policy accepts-but-flags | **UNCONFIRMED** |
| Meaning of `Traw` in the polynomial (U2) | not stated. Driver uses the signed 14-bit code. Hardware test #5 (zero residual ~0 Pa) discriminates | **UNCONFIRMED** |
| Zero procedure for bidirectional range (U3) | datasheet 5.0 formula maps zero pressure to the *bottom* of the range (right for 0..Pmax parts); Table 4-4 describes autozero as 50 % full scale. Driver generalises to target `(0-Pmin)/Prange`; identical to 5.0 when Pmin = 0 | **UNCONFIRMED** (engineering decision) |
| Reset-to-command delay, NOP byte during reads | not stated; driver waits 1 ms after reset and clocks 0x00 while reading | **UNCONFIRMED** |
| CS_ADC across conversion | datasheet figures show CS_ADC low during the whole sequence; platform releases CS per transaction | **UNCONFIRMED** (hardware test) |
| Units string stored in EEPROM (mbar vs Pa) | read at runtime from EEPROM bytes 35–39 | runtime |

Engineering consequence: ±6000 Pa full scale ⇒ max indicated airspeed ≈ 99 m/s at ρ = 1.225 kg/m³ (≈ 357 km/h).
Open requirement question: is that range enough for the vehicle?

RSC driver: **written** (`Drivers/RSC`), with every UNCONFIRMED item above made visible at runtime (`crc_verified`, `catalog_match`, zero-residual check) rather than hidden. Honeywell's sample code (`Pressure_Comp.c`, `crc.c`, referenced p.28) was not available; downloading it would close U1-U3.

## 2. TE MS5611-01BA03 (06/2017)

| Parameter | Value | Where | Status |
|---|---|---|---|
| Interface | SPI **or** I2C, selected by pin PS (low = SPI, high = I2C) | p.6 | VERIFIED |
| SPI | modes 0 and 3, ≤ 20 MHz, CSB active low | p.5–6 | VERIFIED |
| I2C address | `111011Cx`, C = complement of CSB (0x77 / 0x76 7-bit) | p.12 | VERIFIED (7-bit form DERIVED) |
| Supply | 1.8–3.6 V; 100 nF at VDD | p.3, 17 | VERIFIED |
| Commands | Reset `0x1E`; D1 `0x40/42/44/46/48`; D2 `0x50/52/54/56/58`; ADC read `0x00`; PROM `0xA0…0xAE` | p.10 | VERIFIED |
| Reset | once after power-on; 2.8 ms reload | p.10 | VERIFIED |
| PROM | 8 × 16-bit: [0] factory, [1..6] C1..C6, [7] serial+CRC4 (bits 3..0) | p.8, 12–13 | VERIFIED |
| Conversion time max | OSR 256/512/1024/2048/4096 → 0.60/1.17/2.28/4.54/9.04 ms | p.3 | VERIFIED |
| ADC read before conversion done | returns 0 and corrupts conversion | p.11 | VERIFIED |
| Compensation | dT, TEMP, OFF, SENS, P formulas + 2nd order (T < 20 °C and T < −15 °C) | p.8–9 | VERIFIED |
| Output units | TEMP 0.01 °C, P 0.01 mbar (= 1 Pa) | p.8 | VERIFIED |
| Worked example | C1..C6 = 40127/36924/23317/23282/33464/28312, D1 = 9085466, D2 = 8569150 → 2007 / 100009 | p.8 | VERIFIED, **reproduced by `test_datasheet_example`** |
| Range | 10–1200 mbar linear; full accuracy 450–1100 mbar; −40…85 °C | p.3–4 | VERIFIED |
| Intermediate widths | dT·C6 41 bit, OFF/SENS 41 bit → int64 required | p.8 | VERIFIED |
| CRC-4 algorithm | datasheet only cites application note AN520; algorithm implemented from AN520 (not provided) | p.13 | **UNCONFIRMED** — test only checks self-consistency |

## 3. Bosch BMP581 (BST-BMP581-DS004-13)

| Parameter | Value | Where | Status |
|---|---|---|---|
| Interfaces | SPI (≤ 12 MHz at VDDIO ≥ 1.62 V, ≤ 7 MHz below; modes 0/3, 3- and 4-wire), I2C (≤ 1 MHz), I3C | p.1, timing table, 5.x | VERIFIED |
| I2C address | 0x46 (SDO = 0) / 0x47 (SDO = 1); SDO must not float | p.39 | VERIFIED |
| Default interface after power-up/soft reset | I2C/I3C; first SPI access must be a dummy read | 4.1, 4.3.9 | VERIFIED |
| SPI framing | bit7 of first byte: 1 = read, 0 = write; read data follows address immediately | Fig. 11/12 p.38 | VERIFIED (figure) |
| Burst reads | address auto-increments | 5.4 | VERIFIED |
| Supply | VDD 1.71–3.6 V, VDDIO 1.08–3.6 V | p.1 | VERIFIED |
| CHIP_ID (0x01) | reset value 0b01010000 = **0x50** | 7.1 | DERIVED from reset value |
| Soft reset | write 0xB6 to CMD (0x7E); 2 ms | 4.3.10, p.9 | VERIFIED |
| Post-power-up checks | CHIP_ID ≠ 0; STATUS.nvm_rdy = 1, nvm_err = 0; INT_STATUS.por = 1 | 4.3.9 | VERIFIED |
| Registers used | OSR_CONFIG 0x36, ODR_CONFIG 0x37, OSR_EFF 0x38 (bit7 odr_is_valid), INT_STATUS 0x27 (clear-on-read; bit0 drdy, bit3 oor_p, bit4 por), STATUS 0x28, DATA 0x1D–0x22 | ch. 7 | VERIFIED |
| Data format | T = raw/2¹⁶ °C (signed 24); p = raw/2⁶ Pa (signed 24) — **no calibration coefficients needed**, compensation on-chip | 4.5, 7.11–7.16 | VERIFIED |
| Data-register reset value | 0x7F in all six bytes → used as "no sample yet" marker | 7.11–7.16 | DERIVED |
| Consistent readout | single burst read | 4.5.1 | VERIFIED |
| Operating range | 30–125 kPa | p.1 | VERIFIED |
| Noise (typ. @ 100 kPa) | x4: 0.41 Pa; x16: 0.21 Pa; x32: 0.15 Pa | Table 9 | VERIFIED |

*Correction to the original task brief:* the master prompt asked for BMP581 "calibration coefficients and compensation
formulas". The BMP581 has none in the host path — that is a verified property of this part (the BMP390/BMP280 family differ).

## 4. STM32H755 (datasheet uploaded and checked for the facts below)
| Fact | Status |
|---|---|
| Dual core Cortex-M7 + Cortex-M4; LQFP144 package exists | VERIFIED |
| **Two** FDCAN controllers (FDCAN1, FDCAN2; one is TT-FDCAN) | VERIFIED (section 3.40, device summary table) |
| 6 SPI, 4 I2C, 2 x 32-bit timers, 4 watchdogs | VERIFIED |
| Pin alternate functions for LQFP144, FDCAN kernel clock, bit-timing values | **UNCONFIRMED** (needs board + CubeMX; the reference manual RM0399 was not provided) |

## 4b. DroneCAN (dronecan.github.io, fetched during development)
| Fact | Value | Status |
|---|---|---|
| Data type IDs | NodeStatus 341, TrueAirspeed 1020, **IndicatedAirspeed 1021**, RawAirData 1027, StaticPressure 1028, StaticTemperature 1029 | VERIFIED (spec "7. List of standard data types"); my initial memory had 1020/1021 swapped |
| Payload layouts | StaticPressure = float32 + float16; StaticTemperature = float16 x2; IAS = float16 x2; NodeStatus = u32 uptime, u2 health, u3 mode, u3 sub-mode, u16 vendor | VERIFIED |
| CAN ID | priority(5) / type ID(16) / service-not-message(1) / source node(7); tail byte SOT-EOT-toggle-TID(5), single frame 0xC0 or TID | VERIFIED (spec 4.1) |
| Priorities, publish rates, node ID, bitrate | not specified by the spec for this product | provisional |
| Multi-frame (RawAirData) | needs data-type signature + CRC (libcanard) | not implemented |

## 5. How the numbers in tests were produced
- Datasheet example vector (2007 / 100009): taken from MS5611 p.8.
- Other MS5611 vectors: independent Python model using C-style truncating division (not the driver code), see `Tests/test_ms5611.c` comments.
- BMP581 vectors: hand-computed Q-format conversions, verified with Python (`101325*64 = 0x62F340`, `-10*65536 = 0xF60000`).

## 6. Test evidence (executed in the authoring sandbox)
`./scripts/check.sh`: gcc `-fanalyzer` + strict warnings on all modules; 8 test suites under ASan/UBSan, 7425 assertions, 0 failed
(details in `docs/TEST_PLAN.md`); `./scripts/mutation_check.sh`: 19/19 injected bugs detected.
**No hardware run, no ARM cross-compile, no real STM32 HAL build has been done.**
