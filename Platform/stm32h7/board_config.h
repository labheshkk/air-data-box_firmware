/* board_config.h - THE ONLY FILE THAT KNOWS THE BOARD. Fill in from the schematic / STM32CubeMX project.
 *
 * Nothing here is invented: every value below is a placeholder that must be confirmed. The build is stopped
 * (#error) until ADB_BOARD_CONFIGURED is defined, so a wrong pin map can never be shipped silently.
 *
 * What CubeMX must generate (see docs/hardware_checklist.md):
 *   - SPIx (master, 8-bit, MSB first, software NSS) with the SCK/MISO/MOSI pins of the sensor bus, prescaler
 *     chosen so SCLK <= 5 MHz (RSC limit 6.66 MHz abs. max, MS5611 20 MHz, BMP581 12 MHz)
 *   - GPIO outputs, initial level HIGH: CS_BMP581, CS_MS5611, CS_RSC_ADC, CS_RSC_EE
 *   - a 32-bit timer (TIM2 or TIM5) free-running at exactly 1 MHz  -> microsecond time base
 *   - FDCANx in classic mode, bit timing for the agreed bitrate, TX FIFO enabled, started with HAL_FDCAN_Start()
 *   - IWDG (optional but recommended), refreshed in the main loop
 *   - firmware runs on the Cortex-M7; the Cortex-M4 core stays unused */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "stm32h7xx_hal.h"

/* #define ADB_BOARD_CONFIGURED */           /* <- define once every item below has been confirmed */
#ifndef ADB_BOARD_CONFIGURED
#  ifndef ADB_HOST_SYNTAX_CHECK
#    error "board_config.h not configured: fill in handles/pins from the schematic, then define ADB_BOARD_CONFIGURED"
#  endif
#endif

extern SPI_HandleTypeDef   hspi4;            /* BMP581: PE12 SCK, PE13 MISO, PE14 MOSI */
extern SPI_HandleTypeDef   hspi2;            /* RSC: PB13 SCK, PB14 MISO, PB15 MOSI */
extern I2C_HandleTypeDef   hi2c4;            /* MS5611: PD12 SCL, PD13 SDA (I2C mode, PS tied to VDD) */
extern TIM_HandleTypeDef   htim2;            /* 1 MHz free-running 32-bit timer */
extern FDCAN_HandleTypeDef hfdcan1;          /* UNCONFIRMED: which FDCAN instance / transceiver       */
#define ADB_SPI_HANDLE     hspi4             /* BMP581 */
#define ADB_SPI_RSC_HANDLE hspi2             /* RSC */
#define ADB_I2C_HANDLE     hi2c4             /* MS5611 */
#define ADB_TIM_HANDLE     htim2
#define ADB_CAN_HANDLE     hfdcan1

/* Chip selects (GPIO port + pin), outputs, idle high */
#define CS_BMP581_PORT     GPIOE             /* PE11 */
#define CS_BMP581_PIN      GPIO_PIN_11
#define CS_RSC_ADC_PORT    GPIOB             /* PB12 */
#define CS_RSC_ADC_PIN     GPIO_PIN_12
#define CS_RSC_EE_PORT     GPIOD             /* PD11 */
#define CS_RSC_EE_PIN      GPIO_PIN_11
/* no CS for the MS5611: it is on I2C4, address 0x77 (CSB tied low) */
#define MS5611_I2C_ADDR    0x77u

/* Optional inputs */
#define BMP581_INT_PORT    GPIOD             /* PD15 */
#define BMP581_INT_PIN     GPIO_PIN_15
#define RSC_DRDY_PORT      GPIOD             /* PD8 */
#define RSC_DRDY_PIN       GPIO_PIN_8
#define VIN_SENSE_PORT     GPIOF             /* PF8, Vin = Vpin x 9.33 (10k / 1.2k) */
#define VIN_SENSE_PIN      GPIO_PIN_8

#define ADB_SPI_TIMEOUT_MS 5u

/* FDCAN TX FIFO depth: MUST equal the "Tx FIFO Queue Elements" value configured in CubeMX (>= 3 so a 3-frame
 * RawAirData transfer fits). Set to 8 in CubeMX. */
#define ADB_CAN_TX_FIFO_DEPTH 8u

/* Cold-boot detection for the auto-zero safeguard (see app.h, hw.cold_boot): true ONLY after a power-on / brown-out
 * reset, false after watchdog, software or pin reset (those can happen in flight). The RCC reset-flag names below are
 * from memory of the H7 HAL and are UNCONFIRMED against the real stm32h7xx_hal_rcc.h; the flags must be cleared after
 * reading (done in adb_app_init). If unsure, define it as 0: the box then never auto-zeroes (safe, but offset stays
 * uncorrected until a maintenance zero request). */
#define ADB_BOOT_IS_COLD()  (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != 0U)


/* ======================================================================================================
 * Firmware-side additions (NOT part of the Kepstronic file; kept in one block so their file can be dropped
 * in again without losing them). Derived from "Air Data Box - sensor wiring and board configuration"
 * (A. Jabbes, 13.11.2026): MS5611 on I2C4 (SCL PD12, SDA PD13), 10 kOhm pull-ups, 100 kHz.
 * ====================================================================================================== */
#define ADB_I2C_TIMEOUT_MS   5u              /* per HAL call; 100 kHz bus: a 4-byte transfer takes ~0.5 ms */
#define ADB_I2C_SCL_PORT     GPIOD           /* PD12 - used only by the stuck-bus recovery (9 clock pulses) */
#define ADB_I2C_SCL_PIN      GPIO_PIN_12
#define ADB_I2C_SDA_PORT     GPIOD           /* PD13 */
#define ADB_I2C_SDA_PIN      GPIO_PIN_13

#endif
