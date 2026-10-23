# Hardware / CubeMX configuration checklist (must be confirmed - nothing is assumed silently)

Verified from the STM32H755 datasheet (uploaded): dual core (Cortex-M7 + M4), LQFP144 available, **2 FDCAN controllers (FDCAN1, FDCAN2)**,
6 SPI, 4 I2C, 2x 32-bit timers, 2x SysTick, 4 watchdogs. All pin alternate functions, kernel clocks and the core to run on: **UNCONFIRMED**.

| Item | Needed | Status |
|---|---|---|
| Board / schematic | pin map for SPI, 4 chip selects, CAN transceiver | unknown |
| Sensor supply | 3.3 V (RSC variant is 3.3 V; MS5611 1.8-3.6 V; BMP581 VDD 1.71-3.6 V, VDDIO 1.08-3.6 V) | confirm rail + logic levels |
| SPI | one instance, 8-bit MSB first, soft NSS, SCLK <= 5 MHz; RSC needs modes 0 and 1 (platform re-inits on switch) | confirm instance/pins |
| Pull-ups | RSC: 1 kohm on MISO, SCLK, CS_ADC, CS_EE, MOSI (datasheet Table 5) | confirm on PCB |
| RSC DRDY | not used (time-based wait); optional improvement | -- |
| BMP581 | SPI <= 12 MHz (7 MHz if VDDIO <= 1.62 V); if I2C is used SDO must be tied (0x46/0x47), never floating; CSB has an optional internal pull-up in I2C/I3C mode (datasheet 5.6) | confirm bus + CSB wiring |
| MS5611 | PS pin low selects SPI; CSB idle high | confirm |
| Timebase | TIM2 or TIM5 (32-bit) free-running at exactly 1 MHz | configure |
| FDCAN | classic CAN, bitrate **from the customer**, sample point ~87.5 %, TX FIFO mode, 120 ohm termination per bus design | confirm bitrate / node ID |
| Watchdog | IWDG, refresh after `adb_app_step()` | configure |
| Core | firmware on CM7; CM4 not started | confirm |
