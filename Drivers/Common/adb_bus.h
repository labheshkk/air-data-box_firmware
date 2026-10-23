/* adb_bus.h - minimal, mockable transport abstraction (HAL boundary).
 *
 * Sensor drivers only see this struct; STM32 HAL code (SPI/I2C) lives in a
 * platform file that fills it in. Unit tests fill it with a mock.
 *
 *  write()      : one transaction that only sends `len` bytes.
 *  write_read() : send `txlen` bytes, then read `rxlen` bytes.
 *                 SPI: ONE chip-select frame (clock txlen+rxlen bytes, discard
 *                      the first txlen received bytes).
 *                 I2C: write phase followed by (repeated-start) read phase.
 *  read()       : (optional, may be NULL) pure read of `len` bytes in one transaction.
 *                 SPI: clocks `len` bytes while sending 0x00 filler. Needed by devices whose
 *                 data starts at the first clock (Honeywell RSC ADC).
 *  delay_us()   : blocking delay, used ONLY in init/reset paths.
 *
 * All callbacks must return ADB_OK on success, ADB_ERR_BUS / ADB_ERR_TIMEOUT
 * on HAL failure. They must never return ADB_OK after a HAL error. */
#ifndef ADB_BUS_H
#define ADB_BUS_H

#include <stdint.h>
#include "adb_status.h"

typedef enum { ADB_BUS_SPI = 0, ADB_BUS_I2C = 1 } adb_bus_kind_t;

typedef struct adb_bus {
    void          *ctx;
    adb_bus_kind_t kind;
    adb_status_t (*write)(void *ctx, const uint8_t *tx, uint16_t len);
    adb_status_t (*write_read)(void *ctx, const uint8_t *tx, uint16_t txlen,
                               uint8_t *rx, uint16_t rxlen);
    adb_status_t (*read)(void *ctx, uint8_t *rx, uint16_t len);
    void         (*delay_us)(void *ctx, uint32_t us);
} adb_bus_t;

#endif
