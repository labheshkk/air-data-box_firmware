/* adb_platform.h - STM32H7 HAL implementation of the adb_bus_t / time / CAN abstractions.
 * STATUS: written against the STM32H7 HAL API from the HAL documentation, syntax-checked against a hand-written
 * stub only (Platform/stub). NOT compiled with the real HAL and NOT run on hardware. */
#ifndef ADB_PLATFORM_H
#define ADB_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>
#include "adb_bus.h"
#include "dronecan.h"
#include "board_config.h"

typedef struct {                     /* one SPI device = bus handle + chip select + SPI mode */
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint32_t           cpol, cpha;   /* SPI_POLARITY_x / SPI_PHASE_x */
} adb_spi_dev_t;

typedef struct {                     /* one I2C device = bus handle + 7-bit address (MS5611: 0x77, CSB low) */
    I2C_HandleTypeDef *hi2c;
    uint16_t           addr7;
    bool               have_recovered;
    uint32_t           t_last_recover_us;
} adb_i2c_dev_t;

/* Fill an adb_bus_t that talks to `dev`. `dev` must outlive the bus. */
void     adb_platform_spi_bus(adb_bus_t *bus, adb_spi_dev_t *dev);
void     adb_platform_i2c_bus(adb_bus_t *bus, adb_i2c_dev_t *dev);

/* Free a hung I2C bus: MS5611 datasheet p.12 says a failed power-on reset can leave the sensor holding SDA low
 * (acknowledge state); the remedy is several SCLK pulses, then a reset. Pulses SCL up to 9 times until SDA is
 * released, sends a STOP, and re-initialises the I2C peripheral. Called automatically (at most every 100 ms)
 * when a transfer fails. */
void     adb_i2c_recover(adb_i2c_dev_t *dev);

/* 1 MHz time base from a free-running 32-bit timer (wraps every 71.6 min; all consumers use wrap-safe subtraction). */
uint32_t adb_time_us(void);
void     adb_delay_us(uint32_t us);

/* Self-check of the 1 MHz time base: measures the timer against SysTick (HAL_GetTick) over 10 ms and accepts +/-10 %.
 * Catches a wrong TIM2 prescaler (e.g. after the clock tree changed) - every sensor wait and timeout depends on this clock.
 * Bounded: a stopped SysTick returns false instead of hanging. Does not detect small drift. */
bool     adb_time_base_ok(void);

/* Classic-CAN, 29-bit ID transmit. Returns false if the TX FIFO is full or the HAL reports an error. */
bool     adb_can_tx(void *ctx, const dc_frame_t *f);

/* CAN link supervision: call every main-loop pass. Detects bus-off / stuck TX FIFO (policy in Drivers/CAN/can_supervisor)
 * and restarts the FDCAN controller after a back-off. */
void     adb_can_service(uint32_t now_us);
uint32_t adb_can_tx_free(void *ctx);           /* free TX FIFO elements, usable as app_hw_t.can_tx_free */
uint32_t adb_can_restart_count(void);          /* diagnostics */

#endif
