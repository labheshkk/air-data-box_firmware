# Changelog

## 0.3.2 - CubeMX project definition
- NEW: CubeIDE/air_data_box.ioc (+ make_ioc.py, README) - STM32H755ZITx dual core, pins from the board document; never loaded in CubeMX (see CubeIDE/README.md).
- NEW: scripts/validate_ioc.py - checks the .ioc against board_config.h and the datasheet text; self-test proves it rejects 16 broken variants; run by scripts/check.sh.
- NEW: adb_time_base_ok(): the firmware refuses to start if TIM2 is not a 1 MHz time base (guards the pending clock-dependent prescaler); adb_app_init() now returns ADB_ERR_CONFIG in that case.
- CHANGE: docs/CUBEIDE_SETUP.md leads with the .ioc import route; manual route kept. test_platform 110 -> 115 checks; mutation check 34 -> 36 mutants.

## 0.3.1 - bench procedure and CubeIDE setup (requested by the board owner)
- NEW: docs/BENCH_TEST_PROCEDURE.md - first power-up with the DroneCAN GUI Tool + USB-CAN adapter (17 steps with expected results, fault-flag decode table, troubleshooting).
- NEW: docs/CUBEIDE_SETUP.md - CubeIDE/CubeMX setup with all known pins, linking the firmware into the CM7 project, main.c snippets. No generated project is shipped (not possible without the Cube generator and HAL); CAN and clock items are marked as pending.
- NEW: Platform/stm32h7/app_main.h - entry-point prototypes for main.c.

## 0.3.0 - sensor wiring from the board owner (board document 13.11.2026)
- NEW: I2C transport for the MS5611 (I2C4, address 0x77) with stuck-SDA recovery and retry.
- CHANGE: BMP581 on SPI4, RSC on SPI2 (separate handles); MS5611 chip select removed; company `board_config.h` adopted with a marked firmware-only block.
- NEW: test_platform (fake-HAL tests of SPI framing/mode switching, I2C, CAN header, bus-off/stuck-TX restart); mutation check 26 -> 34 mutants.
- NEW: docs/BOARD_INTEGRATION.md. CAN, clock and power parts of the board are still pending.

## 0.2.0 - response to preliminary review
- FIX (critical): BMP581 data-ready never asserted - INT_SOURCE.drdy_data_reg_en is now enabled at init and read back.
- FIX: NodeStatus uptime no longer wraps with the 32-bit microsecond clock (64-bit accumulation).
- NEW: FDCAN link supervisor - bus-off recovery with back-off, stuck-TX flush.
- FIX: boot auto-zero only after a cold boot; optional persisted zero; acceptance limit 150 -> 100 Pa.
- NEW: RawAirData (multi-frame, 20 Hz) with TX-space check; IAS/RawAirData selectable.
- FIX: CI runs `bash scripts/check.sh` so it works even when the executable bit is lost (e.g. commits made from Windows).
- NEW: docs/REVIEW_RESPONSE.md; test suites 8 -> 9; mutation check 19 -> 26 mutants.

## 0.1.0 - first delivery
- Drivers (MS5611, BMP581, RSC), filters, sensor manager, air-data maths, DroneCAN single-frame encoder, application, STM32H7 platform layer, docs, host tests.
