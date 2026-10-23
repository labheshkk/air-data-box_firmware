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

extern SPI_HandleTypeDef   hspi1;            /* UNCONFIRMED: which SPI instance carries the sensors */
extern TIM_HandleTypeDef   htim2;            /* UNCONFIRMED: 1 MHz free-running 32-bit timer         */
extern FDCAN_HandleTypeDef hfdcan1;          /* UNCONFIRMED: which FDCAN instance / transceiver       */
#define ADB_SPI_HANDLE     hspi1
#define ADB_TIM_HANDLE     htim2
#define ADB_CAN_HANDLE     hfdcan1

/* Chip selects (GPIO port + pin) - UNCONFIRMED placeholders */
#define CS_BMP581_PORT     GPIOA
#define CS_BMP581_PIN      GPIO_PIN_4
#define CS_MS5611_PORT     GPIOA
#define CS_MS5611_PIN      GPIO_PIN_3
#define CS_RSC_ADC_PORT    GPIOB
#define CS_RSC_ADC_PIN     GPIO_PIN_0
#define CS_RSC_EE_PORT     GPIOB
#define CS_RSC_EE_PIN      GPIO_PIN_1

#define ADB_SPI_TIMEOUT_MS 5u

#endif
