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

/* Fill an adb_bus_t that talks to `dev`. `dev` must outlive the bus. */
void     adb_platform_spi_bus(adb_bus_t *bus, adb_spi_dev_t *dev);

/* 1 MHz time base from a free-running 32-bit timer (wraps every 71.6 min; all consumers use wrap-safe subtraction). */
uint32_t adb_time_us(void);
void     adb_delay_us(uint32_t us);

/* Classic-CAN, 29-bit ID transmit. Returns false if the TX FIFO is full or the HAL reports an error. */
bool     adb_can_tx(void *ctx, const dc_frame_t *f);

#endif
