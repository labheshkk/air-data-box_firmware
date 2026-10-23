/* ms5611.h - TE MS5611-01BA03 barometric pressure sensor driver.
 * Source of truth: TE datasheet 06/2017 (uploaded 2917208.pdf). Section/page
 * references are given in ms5611.c and docs/datasheet_verification.md.
 *
 * Units: pressure in Pa (datasheet unit 0.01 mbar == 1 Pa), temperature in
 * 0.01 degC. Integer arithmetic only (datasheet formulas are integer; 64-bit
 * intermediates are REQUIRED, see ms5611.c). */
#ifndef MS5611_H
#define MS5611_H

#include <stdint.h>
#include <stdbool.h>
#include "adb_bus.h"

typedef enum {
    MS5611_OSR_256 = 0, MS5611_OSR_512, MS5611_OSR_1024,
    MS5611_OSR_2048, MS5611_OSR_4096
} ms5611_osr_t;

typedef struct {
    int32_t pressure_pa;     /* compensated, 1 LSB = 1 Pa            */
    int32_t temp_centi_c;    /* compensated, 1 LSB = 0.01 degC       */
} ms5611_sample_t;

typedef enum {
    MS5611_ST_START_TEMP = 0, MS5611_ST_WAIT_TEMP,
    MS5611_ST_START_PRESS,    MS5611_ST_WAIT_PRESS
} ms5611_state_t;

typedef struct {
    adb_bus_t       bus;
    ms5611_osr_t    osr;
    uint8_t         temp_every_n;      /* 1 temperature conversion per N pressure conversions (>=1) */
    uint16_t        prom[8];           /* raw PROM words 0..7 */
    /* runtime state */
    ms5611_state_t  state;
    uint32_t        t_start_us;
    uint8_t         press_count;
    uint32_t        d1, d2;            /* last raw ADC values (0 = none yet) */
    ms5611_sample_t last;
    bool            have_sample;
    uint32_t        bus_error_count;
    uint32_t        invalid_count;
} ms5611_t;

/* Pure functions (no hardware) - unit-testable against datasheet example. */
uint8_t      ms5611_crc4(const uint16_t prom[8]);
adb_status_t ms5611_compensate(const uint16_t prom[8], uint32_t d1, uint32_t d2,
                               ms5611_sample_t *out);
uint32_t     ms5611_conv_time_max_us(ms5611_osr_t osr);

/* Reset, read+validate PROM. Blocks ~3 ms (reset reload time). */
adb_status_t ms5611_init(ms5611_t *dev, const adb_bus_t *bus, ms5611_osr_t osr,
                         uint8_t temp_every_n);

/* Non-blocking state machine. Call at >= 1 kHz with a monotonic microsecond
 * clock. *new_sample is set true when dev->last holds a fresh pressure value.
 * Return != ADB_OK describes the failure of THIS call; the machine recovers
 * by restarting the conversion cycle on the next call. */
adb_status_t ms5611_poll(ms5611_t *dev, uint32_t now_us, bool *new_sample);

#endif
