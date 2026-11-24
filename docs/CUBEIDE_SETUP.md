# STM32CubeIDE project setup (Cortex-M7) - with the pins known so far

## 0. What this is, and what it is not
- **Not included: a generated CubeIDE project.** A real project needs the Cube code generator and the ST HAL/CMSIS sources, which are not
  available outside CubeIDE/CubeMX. What *is* included is the CubeMX configuration file that produces it: **`CubeIDE/air_data_box.ioc`**
  (section 1), plus the exact code to paste into the generated `main.c`, the include paths/defines, and the formulas for values that depend on the
  not-yet-received clock tree.
- **The `.ioc` has never been loaded in CubeMX.** It was written from the structure of a real CubeMX 6.11.1 dual-core STM32H755 file and is
  checked offline by `scripts/validate_ioc.py` (see `CubeIDE/README.md`). If CubeMX refuses it, section 2 onwards gives the same setup manually (about 20-30 minutes).
- **Source of every value:** the board document (Kepstronic, 13.11.2026), `board_config.h`, and the sensor datasheets. Items marked
  **[CLOCK]** or **[CAN]** cannot be filled until that information arrives.
- Menu names vary slightly between CubeIDE versions; the intent of each step is what matters.

## 1. Fast route: create the project from the supplied `.ioc`
1. Get the repository onto the computer (unzip it) - a path **without spaces**.
2. CubeIDE: **File -> New -> STM32 Project from an Existing STM32CubeMX Configuration File (.ioc)** -> browse to `CubeIDE/air_data_box.ioc` -> name `air_data_box`, language **C**, binary **Executable**, type **STM32Cube** -> Finish.
3. CubeIDE may offer to migrate to the installed firmware package version: accept. CubeMX may list warnings: read them; typical harmless ones are about the clock configuration, which is intentionally not set yet.
4. Open the `.ioc` and **compare it with sections 2-4 below** (pins, SPI/I2C/TIM2 parameters). If anything differs, correct it by hand - the section 2 tables are the authority.
5. **Set the three pending things before generating code for hardware** (they were left at CubeMX defaults on purpose):
   - **Power supply source** (Pinout & Configuration -> System Core -> PWR, "Power supply source"): LDO vs. SMPS depends on how the board is wired - **a wrong choice can stop the chip from starting**. Ask the board owner.
   - **Clock tree** (Clock Configuration tab) - oscillator and PLLs.
   - **FDCAN** - section 2.6.
6. **Then re-check the values that depend on the clock** (section 4): TIM2 prescaler (the `.ioc` has **63 = 1 MHz only for a 64 MHz timer clock**), I2C4 timing (re-enter 100 kHz so CubeMX recomputes it), and the SPI prescaler (the `.ioc` uses **/64, which keeps SCLK <= 5 MHz for any kernel clock up to 320 MHz**, so it is safe but can be raised for speed).
7. **Safety net:** `adb_app_init()` measures TIM2 against SysTick and returns `ADB_ERR_CONFIG` - starting nothing - if the time base is not 1 MHz (+/-10 %). A silent node after power-up with a good CAN link therefore points first at the TIM2 prescaler.
8. Continue with section 6 (link the firmware folders) and section 7 (`main.c`).

## 1b. Manual route: create the project by hand
1. CubeIDE: **File -> New -> STM32 Project**, **MCU/MPU Selector**, search `STM32H755ZITx` (LQFP144), select it, Next.
2. Project name: `air_data_box`. Targeted language **C**, targeted binary **Executable**, project type **STM32Cube**. Finish and open the generated `.ioc`.
3. A dual-core part creates one project per core (typically `air_data_box_CM7`, `air_data_box_CM4`, plus a common one). **All our code goes into the CM7 project.** Leave the CM4 project as generated (its default start-up synchronisation with the CM7 is kept; confirm with the board owner whether they want the CM4 boot disabled).

## 2. Pinout and peripherals (Pinout & Configuration tab)
Assign everything below to the **Cortex-M7** context.

### 2.1 SPI4 - BMP581
| Item | Value |
|---|---|
| Mode | Full-Duplex Master |
| Pins | PE12 = SPI4_SCK, PE13 = SPI4_MISO, PE14 = SPI4_MOSI |
| Frame format / data size / first bit | Motorola / 8 bits / MSB first |
| Clock polarity / phase | CPOL **Low**, CPHA **1 Edge** (mode 0) |
| NSS signal type | **Software** (chip select is a GPIO) |
| Prescaler | **[CLOCK]** smallest value with kernel clock / prescaler <= 5 MHz (table in section 4) |
| CRC, TI mode, NSSP | disabled |
| Handle name | `hspi4` (default) |
| Pin speed (GPIO settings) | SCK and MOSI **High** or Very High |

### 2.2 SPI2 - Honeywell RSC
Same settings as SPI4 but **PB13 = SPI2_SCK, PB14 = SPI2_MISO, PB15 = SPI2_MOSI**, handle `hspi2`. The firmware switches the clock phase
at run time (RSC EEPROM mode 0, ADC mode 1), so CubeMX only needs the mode-0 default.

### 2.3 I2C4 - MS5611
| Item | Value |
|---|---|
| Pins | PD12 = I2C4_SCL, PD13 = I2C4_SDA (external 10 kOhm pull-ups on the board; no internal pull-up) |
| Speed mode | Standard, **100 kHz** |
| Timing | let CubeMX compute it from the kernel clock **[CLOCK]**; do not edit it by hand |
| Addressing / own address | 7-bit / none |
| Analog filter | enabled, digital filter 0 |
| Handle name | `hi2c4` |

The device address 0x77 is set in firmware (`MS5611_I2C_ADDR`), not in CubeMX.

### 2.4 GPIO
| Pin | Mode | Initial level | User label (suggested) |
|---|---|---|---|
| PE11 | GPIO_Output, push-pull, no pull | **High** | `CS_BMP581` |
| PB12 | GPIO_Output, push-pull, no pull | **High** | `CS_RSC_ADC` |
| PD11 | GPIO_Output, push-pull, no pull | **High** | `CS_RSC_EE` |
| PD15 | GPIO_Input, no pull | - | `BMP581_INT` (optional, unused) |
| PD8 | GPIO_Input, no pull | - | `RSC_DRDY` (optional, unused) |
| PF8 | not configured now | - | `VIN_SENSE` (optional ADC input; not used by the firmware) |

**Chip selects must start High**, and CS_RSC_ADC and CS_RSC_EE must never be low at the same time (the firmware guarantees the second).
PB12 also has an SPI2_NSS alternate function; keep it as a plain GPIO.

### 2.5 Timer - 1 MHz time base
| Item | Value |
|---|---|
| Timer | **TIM2** (32-bit) - handle `htim2` |
| Clock source | Internal Clock |
| Prescaler | **[CLOCK]** = (timer kernel clock in Hz / 1 000 000) - 1 (the supplied `.ioc` has 63, valid only for 64 MHz) |
| Counter mode / period | Up / **4294967295** (0xFFFFFFFF) |
| Interrupts / DMA | none |

The firmware reads the counter directly (`adb_time_us()`), so the timer must really run at 1 MHz.

### 2.6 FDCAN **[CAN]** (pending from the board owner)
Instance, pins, kernel clock and bit timing are still to come. Settings already decided:
| Item | Value |
|---|---|
| Frame format | **Classic mode** (no FD, no bit-rate switching) |
| Mode | Normal |
| Handle name | `hfdcan1` (change `ADB_CAN_HANDLE` in `board_config.h` if another instance is used) |
| Tx FIFO/queue | **FIFO mode, 8 elements** (must match `ADB_CAN_TX_FIFO_DEPTH`; at least 3 are required for RawAirData) |
| Bit timing | per the agreed bitrate `<BITRATE>`, sample point about 87.5 % |
| Rx filters | none for now (GetNodeInfo and node-ID allocation will need a receive path later) |

### 2.7 Watchdog (optional, recommended)
IWDG: about 1 s timeout (e.g. LSI 32 kHz, prescaler 32, reload 1000). Refresh it in the main loop after `adb_app_step()`. The generated handle name is usually `hiwdg1` on STM32H7.

## 3. Clock configuration **[CLOCK]** (pending)
Nothing can be finalised until the board owner sends the oscillator and clock tree. Note from the Clock Configuration tab: the kernel clock of SPI2, SPI4,
I2C4 and the "APB1 timer clock" for TIM2, then use the formulas in section 4.

## 4. Values that depend on the clock
SPI: SCLK = kernel clock / prescaler. Required: **<= 5 MHz** (RSC limit 6.66 MHz, BMP581 12 MHz; 5 MHz keeps margin with the 1 kOhm pull-ups on the lines). Prescalers are powers of two (2 ... 256).

| SPI kernel clock | Prescaler | Resulting SCLK |
|---|---|---|
| 50 MHz | 16 | 3.13 MHz |
| 64 MHz | 16 | 4.00 MHz |
| 80 MHz | 16 | 5.00 MHz |
| 100 MHz | 32 | 3.13 MHz |
| 120 MHz | 32 | 3.75 MHz |
| 150 MHz | 32 | 4.69 MHz |
| 200 MHz | 64 | 3.13 MHz |
| 240 MHz | 64 | 3.75 MHz |

TIM2 prescaler = timer clock / 1 MHz - 1 (e.g. 200 MHz -> 199, 240 MHz -> 239, 120 MHz -> 119).

## 5. Project Manager tab
- Toolchain: STM32CubeIDE. **Keep user code when re-generating: enabled** (the default - the pasted lines in section 7 live in USER CODE blocks).
- Minimum stack size **0x2000** (8 KB), heap **0x400**. (Stack use is not measured yet; this is a safe starting point.)
- Code Generator: "Generate peripheral initialization as a pair of .c/.h files per peripheral" is optional.
- Click **Generate code** (Project -> Generate Code).

## 6. Add the firmware sources to the CM7 project
Our folder is called `Drivers`, and the generated project already has a `Drivers` folder (HAL and CMSIS). To avoid the clash, link our folders under other names.

1. Clone or unzip the repository to a path **without spaces**, outside the CubeIDE workspace, e.g. `C:\work\air-data-box_firmware`.
2. In Project Explorer, right-click the **`air_data_box_CM7`** project -> **New -> Folder -> Advanced >> -> Link to alternate location (Linked Folder)** -> Browse. Create these four links:

| Link name | Target folder in the repository |
|---|---|
| `adb_drivers` | `Drivers` (contains Common, MS5611, BMP581, RSC, CAN) |
| `adb_services` | `Services` (AirData, Filters, SensorManager) |
| `adb_application` | `Application` |
| `adb_platform` | `Platform/stm32h7` |

   **Do not link** `Tests/`, `Platform/stub/` or `scripts/` (host-only).
3. **Project -> Properties -> C/C++ General -> Paths and Symbols**, apply to all configurations and languages:
   - *Source Location* tab -> Add Folder: the four linked folders above (linked folders are not always compiled automatically).
   - *Includes* (GNU C) -> add:
     `${ProjDirPath}/adb_drivers/Common`, `${ProjDirPath}/adb_drivers/MS5611`, `${ProjDirPath}/adb_drivers/BMP581`,
     `${ProjDirPath}/adb_drivers/RSC`, `${ProjDirPath}/adb_drivers/CAN`, `${ProjDirPath}/adb_services/AirData`,
     `${ProjDirPath}/adb_services/Filters`, `${ProjDirPath}/adb_services/SensorManager`, `${ProjDirPath}/adb_application`,
     `${ProjDirPath}/adb_platform`
   - *Symbols* -> add `ADB_BOARD_CONFIGURED` (leave the value empty) **only when** the CAN, clock and power items in section 8 are confirmed. Until then the build stops with the intentional `#error` in `board_config.h`.
4. Compiler settings: CubeIDE's defaults for the Cortex-M7 are what the code needs (hard-float with the double-precision FPU `fpv5-d16`, link with the math library). The RSC compensation uses `double`.

*Alternative to linking:* copy the folders into the CM7 project (`Core/Src` and `Core/Inc` flattened) and keep them in sync by hand. Linking is cleaner because the repository stays the single source of truth.

## 7. Edit the generated `main.c` (CM7 project only)
Paste **only inside** the `USER CODE` blocks so a later re-generation does not delete it.

```c
/* USER CODE BEGIN Includes */
#include "app_main.h"
/* USER CODE END Includes */
```
```c
/* USER CODE BEGIN 2 */
HAL_TIM_Base_Start(&htim2);              /* 1 MHz time base - must run before the application starts */
/* [CAN] HAL_FDCAN_Start(&hfdcan1);        start the CAN controller BEFORE adb_app_init() once FDCAN is configured */
if (adb_app_init() != ADB_OK) { Error_Handler(); }   /* fails only if TIM2 is not a 1 MHz time base or a parameter is invalid; a missing sensor is NOT an error (retried every second) */
/* USER CODE END 2 */
```
```c
  /* Inside while (1): */
  /* USER CODE BEGIN 3 */
  adb_app_step();                        /* sensors, fusion, air data, DroneCAN publishing */
  /* HAL_IWDG_Refresh(&hiwdg1);             watchdog: refresh AFTER the step (name as generated) */
  /* USER CODE END 3 */
```
`ADB_BOOT_IS_COLD()` in `board_config.h` reads the RCC reset flags (power-on reset = cold boot). Its flag name is written from memory of the H7 HAL and must be checked on the first build.

## 8. Information still needed from the board owner
| Item | Used for | Where it goes |
|---|---|---|
| FDCAN instance, pins, kernel clock, **bitrate**, sample point | CAN output | CubeMX FDCAN settings; `ADB_CAN_HANDLE` |
| Oscillator and clock tree | SPI/I2C/TIM2 values | section 4 formulas |
| Power / supervisor details | brown-out and cold-boot detection | `ADB_BOOT_IS_COLD()` |
| Node ID (fixed or dynamic) | DroneCAN identity | `app_cfg_t.node_id` (now 42, provisional) |
| CM4 boot policy | dual-core start-up | CM4 project / option bytes |

## 9. First build and the first checks
1. **Project -> Build** the CM7 project. Expected: no errors from the `adb_*` folders. **Not yet tried with the real toolchain/HAL** - if a name in `adb_platform.c` (for example the reset-flag macro or an FDCAN call) differs from the real HAL, the compiler will say exactly which; send the message back and it will be fixed.
2. Debug session sanity checks (before connecting the CAN bus):
   - Watch `adb_time_us()`: two reads one second apart must differ by 1 000 000 +/- 0.1 %.
   - `hspi4.Init.CLKPhase` and `hspi2.Init.CLKPhase`: `SPI_PHASE_1EDGE`; the RSC ADC switches SPI2 to `SPI_PHASE_2EDGE` at run time.
   - Scope SCLK on SPI4 and SPI2: at most 5 MHz; scope I2C4 SCL: about 100 kHz.
3. Continue with `docs/BENCH_TEST_PROCEDURE.md`.
