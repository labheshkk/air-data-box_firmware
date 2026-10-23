# air-data-box-firmware

STM32H755 Air Data Box firmware: Honeywell RSCDJJI060MDSE3 (differential), Bosch BMP581 + TE MS5611-01BA03 (static) -> DroneCAN.

## Status (read this first)
Complete, host-tested firmware source: three sensor drivers, filters, sensor manager (health, dual-static fusion, zeroing), air-data maths,
DroneCAN encoder, application scheduler, and an STM32H7 platform layer. **Not yet run on hardware, not built with the real STM32 HAL.**
Open items that only hardware can close are listed in `docs/assumptions_open_questions.md` (section C) and `docs/TEST_PLAN.md`.

## Layout
```
Drivers/Common        adb_status.h, adb_bus.h        error codes, mockable SPI/I2C boundary
Drivers/MS5611        ms5611.[ch]
Drivers/BMP581        bmp581.[ch]
Drivers/RSC           rsc.[ch]
Drivers/CAN           dronecan.[ch]                  single-frame DroneCAN encoders (not libcanard)
Services/AirData      airdata.[ch]                   IAS / CAS / TAS / pressure altitude
Services/Filters      filters.[ch]
Services/SensorManager sensor_manager.[ch]
Application           app.[ch]                       scheduler, publishing, diagnostics events
Platform/stm32h7      board_config.h adb_platform.[ch] app_main.c   HAL glue (fill in board_config.h)
Platform/stub         host-only HAL stub for syntax checking
Tests                 8 host test suites + Makefile
scripts               check.sh (quality gate), mutation_check.sh
docs                  ARCHITECTURE
```

## Quality gate (host: gcc + make)
```
./scripts/check.sh            # gcc -fanalyzer + strict warnings (-Werror) on every module, then all tests under ASan+UBSan
./scripts/mutation_check.sh   # injects 19 bugs, requires the tests to catch each one
```

## Integrating on the STM32H755 (CM7)
1. Create a CubeMX project: SPI master, 4 CS GPIOs, 32-bit timer at 1 MHz, FDCAN, IWDG. Details: `docs/hardware_checklist.md`.
2. Add `Drivers/`, `Services/`, `Application/`, `Platform/stm32h7/` to the build; fill `board_config.h`, define `ADB_BOARD_CONFIGURED`.
3. In `main.c`: `HAL_TIM_Base_Start(&htim2); HAL_FDCAN_Start(&hfdcan1); adb_app_init(); while (1) { adb_app_step(); HAL_IWDG_Refresh(&hiwdg); }`
4. Follow `docs/TEST_PLAN.md` section B in order.

## First bring-up debugging
- `ADB_ERR_CALIB`: floating MISO / absent chip. `ADB_ERR_CRC`: PROM/EEPROM corrupted or CRC algorithm assumption (see open items U1, MS5611 note).
- `ADB_ERR_ID` on BMP581: check CSB/SDO wiring and the 0x50 assumption (A12).
- Events (`app_event_t`) via `hw.event` give a compact log: init results, zero outcome, fault-flag changes, TX drops.
