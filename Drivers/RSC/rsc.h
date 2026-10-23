/* rsc.h - Honeywell TruStability RSC series driver (SPI, external-math transfer function).
 * Source: Honeywell 32321348 Issue G. Target part: RSCDJJI060MDSE3 (+/-60 mbar differential).
 *
 * The chip contains an EEPROM (coefficients) and a 24-bit ADC on the SAME SPI pins but with
 * DIFFERENT chip selects and SPI modes: EEPROM mode 0 (or 3), ADC mode 1 (datasheet p.23-24).
 * The platform therefore supplies TWO adb_bus_t objects (ee_bus, adc_bus). The platform must
 * guarantee CS_EE and CS_ADC are never low at the same time.
 *
 * Open items that the datasheet does not settle (UNCONFIRMED, see docs/datasheet_verification.md):
 *   U1  initial value / byte range of the EEPROM CRC-16-CCITT        -> candidates tried, outcome reported
 *   U2  whether "Traw" in the polynomial is the raw 14-bit code       -> assumed: raw code (signed)
 * Both are made visible at runtime instead of being hidden (rsc_t.crc_verified, zero-check in the manager). */
#ifndef RSC_H
#define RSC_H

#include <stdint.h>
#include <stdbool.h>
#include "adb_bus.h"

#define RSC_EEPROM_SIZE 452u

typedef enum {                    /* Table 3-1 / 3-2 */
    RSC_N20 = 0, RSC_N45, RSC_N90, RSC_N175, RSC_N330, RSC_N600, RSC_N1000,
    RSC_F40, RSC_F90, RSC_F180, RSC_F350, RSC_F660, RSC_F1200, RSC_F2000,
    RSC_RATE_COUNT
} rsc_rate_t;

typedef enum { RSC_CRC_ACCEPT_UNVERIFIED = 0, RSC_CRC_STRICT = 1 } rsc_crc_policy_t;

typedef struct {
    rsc_rate_t        rate;
    uint8_t           temp_every_n;       /* 1 temperature conversion per N pressure conversions */
    rsc_crc_policy_t  crc_policy;
    const char       *expected_catalog;   /* e.g. "RSCDJJI060MDSE3"; NULL = skip comparison (warning only) */
} rsc_config_t;

typedef struct {
    double   p_range, p_min;              /* in sensor engineering units                       */
    double   offset[4], span[4], shape[4];/* index = power of T / P                            */
    double   unit_to_pa;                  /* engineering unit -> Pa                            */
    uint8_t  adc_cfg[4];                  /* ADC config registers 0..3 from EEPROM 61,63,65,67 */
    char     catalog[17];
    bool     crc_ok;                      /* some candidate matched                            */
} rsc_calib_t;

typedef struct {
    double  pressure_pa;                  /* compensated, zero-corrected, in Pa                */
    double  temp_c;
    int32_t p_raw;                        /* signed 24-bit ADC code                            */
    int32_t t_code;                       /* signed 14-bit temperature code                    */
} rsc_sample_t;

typedef enum { RSC_ST_START_TEMP = 0, RSC_ST_WAIT_TEMP, RSC_ST_START_PRESS, RSC_ST_WAIT_PRESS } rsc_state_t;

typedef struct {
    adb_bus_t     ee_bus, adc_bus;
    rsc_config_t  cfg;
    rsc_calib_t   cal;
    bool          crc_verified;           /* true only if a CRC candidate matched (U1)         */
    bool          catalog_match;          /* EEPROM catalog string equals expected_catalog     */
    double        praw_azero;             /* datasheet section 5 zero correction (raw counts)  */
    rsc_state_t   state;
    uint32_t      t_start_us;
    uint8_t       press_count;
    int32_t       t_code_last;
    bool          have_temp;
    rsc_sample_t  last;
    bool          have_sample;
    uint32_t      bus_error_count, invalid_count;
} rsc_t;

/* ---- pure functions (unit-testable) ---- */
uint16_t     rsc_crc16_ccitt(const uint8_t *d, uint32_t n, uint16_t init);
adb_status_t rsc_parse_eeprom(const uint8_t ee[RSC_EEPROM_SIZE], rsc_crc_policy_t policy, rsc_calib_t *out);
int32_t      rsc_decode_temp_code(uint32_t raw24);        /* first 14 bits, two's complement */
int32_t      rsc_decode_pressure_raw(uint32_t raw24);     /* 24-bit two's complement          */
adb_status_t rsc_compensate(const rsc_calib_t *c, int32_t p_raw, int32_t t_code, double praw_azero, double *p_pa);
double       rsc_zero_correction(const rsc_calib_t *c, int32_t praw0, int32_t traw0); /* datasheet 5.0 step 3 */
uint32_t     rsc_conv_time_us(rsc_rate_t r);              /* typ. +5 % tolerance + margin */
uint8_t      rsc_config1_byte(rsc_rate_t r, bool temperature);

/* ---- hardware paths ---- */
adb_status_t rsc_init(rsc_t *d, const adb_bus_t *ee_bus, const adb_bus_t *adc_bus, const rsc_config_t *cfg);
adb_status_t rsc_poll(rsc_t *d, uint32_t now_us, bool *new_sample);
void         rsc_set_zero(rsc_t *d, int32_t praw0, int32_t traw0);

#endif
