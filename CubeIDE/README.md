# CubeIDE / CubeMX project definition

`air_data_box.ioc` - the CubeMX configuration for the air data box (STM32H755ZITx, LQFP144, dual core, everything on the Cortex-M7) with the
pins from the board document: SPI4 (BMP581), SPI2 (RSC), I2C4 (MS5611), the three chip-select outputs, TIM2 as a 1 MHz time base.
**Still missing on purpose:** FDCAN, the clock tree and the power supply setting (pending from the board owner).

**Status: never loaded in CubeMX.** It was written from the structure of a real CubeMX 6.11.1 dual-core STM32H755 file and is checked by
`scripts/validate_ioc.py` (well-formedness, consistency with `Platform/stm32h7/board_config.h`, datasheet pin/function names, SPI/timer limits;
the validator also proves it rejects 16 kinds of broken file). If CubeMX shows warnings on opening, read them and accept the dialogs; if it
refuses the file, use the manual steps in `docs/CUBEIDE_SETUP.md` (section 2 onwards) - the end result is the same.

Use: CubeIDE -> File -> New -> **STM32 Project from an Existing STM32CubeMX Configuration File (.ioc)** -> select `CubeIDE/air_data_box.ioc`.
Details and what to set next: `docs/CUBEIDE_SETUP.md`.

`make_ioc.py` regenerates the file (`python3 CubeIDE/make_ioc.py`); run `python3 scripts/validate_ioc.py` afterwards.
