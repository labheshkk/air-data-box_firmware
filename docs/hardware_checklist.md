# Hardware / CubeMX configuration checklist (must be confirmed - nothing is assumed silently)
Step-by-step CubeIDE setup with the known pins: `CUBEIDE_SETUP.md`.


Verified from the STM32H755 datasheet (uploaded): dual core (Cortex-M7 + M4), LQFP144 available, **2 FDCAN controllers (FDCAN1, FDCAN2)**,
6 SPI, 4 I2C, 2x 32-bit timers, 2x SysTick, 4 watchdogs. All pin alternate functions, kernel clocks and the core to run on: **UNCONFIRMED**.

| Item | Needed | Status |
|---|---|---|
| Board / schematic | pin map for SPI, 4 chip selects, CAN transceiver | unknown |
| Sensor supply | 3.3 V (RSC variant is 3.3 V; MS5611 1.8-3.6 V; BMP581 VDD 1.71-3.6 V, VDDIO 1.08-3.6 V) | confirm rail + logic levels |
| SPI | **provided:** BMP581 on SPI4 (PE11 CS, PE12/13/14), RSC on SPI2 (PB12 CS_ADC, PD11 CS_EE, PB13/14/15); 8-bit MSB first, soft NSS, SCLK <= 5 MHz; RSC needs modes 0 and 1 | CubeMX: set up SPI4/SPI2 as in the board document |
| I2C | **provided:** MS5611 on I2C4 (PD12 SCL, PD13 SDA), 0x77, 100 kHz, 10 kOhm pull-ups | CubeMX: I2C4 at 100 kHz |
| Pull-ups | RSC and BMP581 SPI lines: 1 kOhm (board document; matches RSC datasheet Table 5) | provided |
| RSC DRDY | not used (time-based wait); optional improvement | -- |
| BMP581 | SPI <= 12 MHz (7 MHz if VDDIO <= 1.62 V); if I2C is used SDO must be tied (0x46/0x47), never floating; CSB has an optional internal pull-up in I2C/I3C mode (datasheet 5.6) | confirm bus + CSB wiring |
| MS5611 | PS pin low selects SPI; CSB idle high | confirm |
| Timebase | TIM2 or TIM5 (32-bit) free-running at exactly 1 MHz | configure |
| FDCAN | classic CAN, bitrate **from the customer**, sample point ~87.5 %, TX FIFO mode, 120 ohm termination per bus design | confirm bitrate / node ID |
| Watchdog | IWDG, refresh after `adb_app_step()` | configure |
| Core | firmware on CM7; CM4 not started | confirm |
| TIM2 time base | prescaler = timer clock / 1 MHz - 1; the firmware self-checks it against SysTick (+/-10 %) and refuses to start if wrong | set after the clock tree is known |
| PWR supply source | LDO or SMPS - must match the board wiring or the chip may not start | **ask the board owner** |
| FDCAN TX FIFO | >= 3 elements (RawAirData is 3 frames); set `ADB_CAN_TX_FIFO_DEPTH` to match | configure |
| Reset-cause flags | confirm `RCC_FLAG_PORRST` / reset-flag names and that flags are cleared after reading (cold-boot detection) | confirm in HAL |
| Zero persistence | flash/EEPROM area + CRC for (praw0, traw0) via `zero_load/zero_store` | **not implemented** |
| Autopilot | type (ArduPilot/PX4), airspeed parameter (e.g. ARSPD_TYPE = DroneCAN), whether RawAirData and/or IAS is consumed | **ask the integrator** |
