/* bmp581.h - Bosch BMP581 barometric pressure sensor driver.
 * Source of truth: Bosch BST-BMP581-DS004-13 (uploaded bst-bmp581-ds004.pdf).
 *
 * The BMP581 compensates internally: the data registers already hold
 * temperature (signed 24, Q16 degC) and pressure (signed 24, Q6 Pa) - there are
 * NO calibration coefficients to read (datasheet 4.5 / registers 7.11-7.16).
 * Driver therefore returns fixed-point values with no floating point:
 *   pressure_pa_q6 : Pa * 64      (1 LSB = 1/64 Pa)
 *   temp_c_q16     : degC * 65536 (1 LSB = 1/65536 degC) */
#ifndef BMP581_H
#define BMP581_H

#include <stdint.h>
#include <stdbool.h>
#include "adb_bus.h"

typedef enum {                       /* OSR_CONFIG osr_p / osr_t encoding, reg 0x36 */
    BMP581_OSR_X1 = 0, BMP581_OSR_X2, BMP581_OSR_X4, BMP581_OSR_X8,
    BMP581_OSR_X16, BMP581_OSR_X32, BMP581_OSR_X64, BMP581_OSR_X128
} bmp581_osr_t;

typedef struct {
    bmp581_osr_t osr_p;
    bmp581_osr_t osr_t;
    uint8_t      odr;               /* ODR_CONFIG.odr code 0x00..0x1F (reg 0x37) */
} bmp581_config_t;

typedef struct {
    int32_t pressure_pa_q6;
    int32_t temp_c_q16;
} bmp581_sample_t;

typedef struct {
    adb_bus_t       bus;
    bmp581_config_t cfg;
    bmp581_sample_t last;
    bool            have_sample;
    uint32_t        bus_error_count;
    uint32_t        invalid_count;
} bmp581_t;

/* Pure conversion helpers (unit-testable). */
int32_t bmp581_sign_extend24(uint32_t v);
adb_status_t bmp581_decode_burst(const uint8_t b[6], bmp581_sample_t *out);

/* Full start-up per datasheet 4.3.9/4.3.10, then configure NORMAL mode.
 * Blocks ~5 ms. Fails (and says why) instead of returning a half-working device. */
adb_status_t bmp581_init(bmp581_t *dev, const adb_bus_t *bus, const bmp581_config_t *cfg);

/* Burst-read T+P (6 bytes from 0x1D, single burst = consistent data, datasheet 4.5.1). */
adb_status_t bmp581_read(bmp581_t *dev, bmp581_sample_t *out);

/* Preferred runtime call: returns a sample only when a NEW one is ready
 * (*new_sample == true). No new data is NOT an error (returns ADB_OK). */
adb_status_t bmp581_poll(bmp581_t *dev, bmp581_sample_t *out, bool *new_sample);

#endif
