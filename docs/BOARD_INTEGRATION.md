# Board integration - sensor part (source: "Air Data Box - sensor wiring and board configuration", A. Jabbes, 13.11.2026)

Status: the **sensor** part of the board is integrated. CAN (bitrate, node ID, FDCAN instance/bit timing), clock tree and power are still to come
from the board owner; `ADB_BOARD_CONFIGURED` therefore stays undefined and the platform code does not build for the target yet (by design).

## Wiring received and where it lives in the firmware
| Sensor | Bus | Pins | Firmware |
|---|---|---|---|
| BMP581 (static) | SPI4 | CS PE11, SCK PE12, MISO PE13, MOSI PE14, INT PD15 (optional, unused) | `d_bmp` in `Platform/stm32h7/app_main.c`, SPI mode 0, handle `ADB_SPI_HANDLE` (= hspi4) |
| Honeywell RSC (differential) | SPI2 | CS_ADC PB12, CS_EE PD11, SCK PB13, MISO PB14, MOSI PB15, DRDY PD8 (optional, unused) | `d_rsc_adc` (mode 1) and `d_rsc_ee` (mode 0), handle `ADB_SPI_RSC_HANDLE` (= hspi2) |
| MS5611 (static) | **I2C4** | SCL PD12, SDA PD13, address 0x77 (CSB low), 10 kOhm pull-ups, 100 kHz | new I2C transport `adb_platform_i2c_bus()`, `d_ms` |
| Supply sense | ADC | PF8, Vin = Vpin x 9.33 (10 kOhm / 1.2 kOhm) | not used by the firmware (could be added as a board-voltage diagnostic) |
| Autopilot | - | ArduPilot, ArduCopter 4.6.2 | `RawAirData` + StaticPressure + StaticTemperature published; see below |
| FDCAN TX FIFO | - | 8 elements | `ADB_CAN_TX_FIFO_DEPTH = 8` (>= 3 needed for RawAirData) |

The company's `board_config.h` is used unchanged; firmware-only constants (I2C timeout, SCL/SDA pins for bus recovery) are appended in a clearly marked block
at its end, so the file can be replaced by a newer version from the board owner without losing them.

## Cross-checks against the datasheets (done when the document arrived)
| Statement in the board document | Check | Result |
|---|---|---|
| MS5611 in I2C mode because PS is tied to VDD | MS5611 datasheet p.6: PS high selects I2C | consistent |
| MS5611 address 0x77 with CSB low | datasheet p.12: address 111011Cx, C = complement of CSB -> CSB low gives 1110111 = 0x77 | consistent |
| 10 kOhm I2C pull-ups | datasheet Figure 17 shows 10 kOhm | consistent |
| RSC SPI lines with 1 kOhm pull-ups | Honeywell Table 5 / Figure 7: 1 kOhm on MISO, SCLK, CS_ADC, CS_EE, MOSI | consistent |
| RSC alone on SPI2, mode switch only between ADC and EEPROM | RSC p.23-24: EEPROM mode 0/3, ADC mode 1 | consistent; platform re-programs the mode only when it changes |
| SCLK up to 5 MHz | RSC min clock period 150 ns (6.66 MHz max); BMP581 12 MHz | consistent |
| Pin functions | STM32H755 datasheet alternate-function table: PE12 = SPI4_SCK, PB15 = SPI2_MOSI, PD12 = I2C4_SCL, PD13 = I2C4_SDA seen; PE13, PE14, PB13, PB14 could not be matched by text extraction | **partly verified**; CubeMX rejects a wrong assignment |
| Divider 10 k / 1.2 k -> x9.33 | (10 + 1.2) / 1.2 = 9.33 | consistent |
| ArduPilot Copter and airspeed | ArduPilot Copter parameter list contains `ARSPD_TYPE` with value 8 = DroneCAN (docs 4.3.6 / 4.5.5); older forum posts saying Copter has no airspeed support are outdated | consistent; which message the backend subscribes to is **UNCONFIRMED** here (public airspeed nodes publish RawAirData) |

## Pin check against the STM32H755 datasheet (done with the supplied datasheet text)
- All 13 GPIO pins (PB12-15, PD8, PD11-13, PD15, PE11-14) exist on the LQFP144 package, and every function name used (SPI2_SCK/MISO/MOSI, SPI4_SCK/MISO/MOSI, I2C4_SCL/SDA) appears in the datasheet's pin/alternate-function tables: `python3 scripts/validate_ioc.py --datasheet-text <datasheet.txt>` passes.
- **Not verified:** the alternate-function *numbers* (the PDF text layout drops empty cells, so column positions cannot be read reliably) and which function sits on exactly which pin beyond name presence. CubeMX assigns AF numbers itself and refuses invalid pin/function pairs.
- Defaults chosen in `CubeIDE/air_data_box.ioc` while clock/power are pending: SPI prescaler /64 (SCLK <= 5 MHz for any kernel clock up to 320 MHz), TIM2 prescaler 63 (1 MHz at 64 MHz only), I2C4 CubeMX defaults, no RCC/PWR/FDCAN settings. The firmware checks the 1 MHz time base at start-up.

## Firmware changes made for this wiring (v0.3.0)
- I2C transport for the MS5611 (`adb_i2c_dev_t`): 7-bit address shifted for the HAL, command write then **separate** read with a STOP in between (datasheet p.12-13),
  `ADB_I2C_TIMEOUT_MS`.
- **Stuck-bus recovery:** the datasheet (p.12) says a failed power-on reset can leave the MS5611 holding SDA low; remedy is clock pulses then reset. The transport
  now, on a failed transfer, de-initialises I2C, clocks SCL up to 9 times until SDA is released, sends a STOP, re-initialises, and retries once (at most every 100 ms).
- Two SPI handles (SPI4 for BMP581, SPI2 for RSC) instead of one shared bus; the MS5611 chip select is gone.
- 10th test suite (`test_platform`, 110 checks) using a recording fake HAL; 8 new mutants (34 total, all caught).

## Still open / for the bring-up
1. CAN: instance, bitrate, bit timing, node ID, termination (board owner). 2. Clock tree and power (board owner).
3. `RCC_FLAG_PORRST` and the HAL calls used by the platform layer are compiled only against a stub: first build with the real HAL will show any naming differences.
4. I2C timing: each blocking MS5611 transfer takes ~0.3-0.5 ms at 100 kHz; the main loop therefore has up to ~0.5 ms of blocking per MS5611 call. Not measured.
5. Zero persistence (flash/EEPROM) is still not implemented.
6. Confirm on the board that ArduPilot 4.6.2 consumes RawAirData for `ARSPD_TYPE = 8` and the baro from StaticPressure/StaticTemperature
   (parameters to check: `CAN_P1_DRIVER`, `ARSPD_TYPE`, `ARSPD_USE`, and the barometer's DroneCAN instance).
