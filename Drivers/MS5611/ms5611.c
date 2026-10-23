/* ms5611.c - see ms5611.h. Datasheet page numbers are those of the 06/2017 PDF. */
#include "ms5611.h"
#include <stddef.h>

/* Commands - datasheet p.10, Figure 4 "Command structure" */
#define CMD_RESET        0x1Eu
#define CMD_CONV_D1(o)   ((uint8_t)(0x40u | ((unsigned)(o) << 1)))  /* 0x40,42,44,46,48 */
#define CMD_CONV_D2(o)   ((uint8_t)(0x50u | ((unsigned)(o) << 1)))  /* 0x50,52,54,56,58 */
#define CMD_ADC_READ     0x00u
#define CMD_PROM_READ(a) ((uint8_t)(0xA0u | ((unsigned)(a) << 1)))  /* 0xA0..0xAE */

#define T_RESET_RELOAD_US  2800u   /* p.10 Fig.5: 2.8 ms reload after reset */

/* Max conversion time per OSR, datasheet p.3 ADC table, "Max" column (us). */
static const uint32_t k_conv_max_us[5] = { 600u, 1170u, 2280u, 4540u, 9040u };

uint32_t ms5611_conv_time_max_us(ms5611_osr_t osr)
{
    return (osr <= MS5611_OSR_4096) ? k_conv_max_us[osr] : k_conv_max_us[MS5611_OSR_4096];
}

/* CRC-4 per TE application note AN520 (referenced on datasheet p.13; the
 * algorithm itself is NOT printed in the datasheet -> verify on hardware).
 * CRC nibble = PROM word 7 bits 3..0; those 8 low bits are zeroed for the calc. */
uint8_t ms5611_crc4(const uint16_t prom_in[8])
{
    uint16_t p[8];
    for (int i = 0; i < 8; i++) p[i] = prom_in[i];
    p[7] &= 0xFF00u;
    uint16_t rem = 0;
    for (int cnt = 0; cnt < 16; cnt++) {
        if (cnt & 1) rem ^= (uint16_t)(p[cnt >> 1] & 0x00FFu);
        else         rem ^= (uint16_t)(p[cnt >> 1] >> 8);
        for (int b = 8; b > 0; b--) {
            if (rem & 0x8000u) rem = (uint16_t)(((unsigned)rem << 1) ^ 0x3000u);
            else               rem = (uint16_t)((unsigned)rem << 1);
        }
    }
    return (uint8_t)((rem >> 12) & 0x0Fu);
}

/* Compensation - datasheet p.8 (first order) and p.9 (second order).
 * All intermediates int64: dT*C6 reaches 2^41, OFF/SENS 2^33, D1*SENS 2^57 (p.8
 * lists these bit-widths). Divisions by powers of two use explicit division, not
 * shifts, so negative values follow the datasheet's C semantics (truncate to 0). */
adb_status_t ms5611_compensate(const uint16_t prom[8], uint32_t d1, uint32_t d2,
                               ms5611_sample_t *out)
{
    if (prom == NULL || out == NULL) return ADB_ERR_PARAM;
    if (d1 == 0u || d2 == 0u || d1 > 0xFFFFFFu || d2 > 0xFFFFFFu) return ADB_ERR_INVALID;

    const int64_t c1 = prom[1], c2 = prom[2], c3 = prom[3],
                  c4 = prom[4], c5 = prom[5], c6 = prom[6];

    int64_t dT   = (int64_t)d2 - c5 * 256;                    /* dT = D2 - C5*2^8        */
    int64_t temp = 2000 + (dT * c6) / 8388608;                /* 2000 + dT*C6/2^23       */
    int64_t off  = c2 * 65536 + (c4 * dT) / 128;              /* C2*2^16 + C4*dT/2^7     */
    int64_t sens = c1 * 32768 + (c3 * dT) / 256;              /* C1*2^15 + C3*dT/2^8     */

    if (temp < 2000) {                                        /* 2nd order, p.9          */
        int64_t t2    = (dT * dT) / 2147483648LL;             /* dT^2 / 2^31             */
        int64_t dt20  = temp - 2000;
        int64_t off2  = (5 * dt20 * dt20) / 2;
        int64_t sens2 = (5 * dt20 * dt20) / 4;
        if (temp < -1500) {                                   /* very low temperature    */
            int64_t dt15 = temp + 1500;
            off2  += 7 * dt15 * dt15;
            sens2 += (11 * dt15 * dt15) / 2;
        }
        temp -= t2; off -= off2; sens -= sens2;
    }
    int64_t p = (((int64_t)d1 * sens) / 2097152 - off) / 32768;  /* (D1*SENS/2^21 - OFF)/2^15 */

    /* Valid output ranges, datasheet p.8: TEMP -40..85 degC, P 10..1200 mbar. */
    out->temp_centi_c = (int32_t)temp;
    out->pressure_pa  = (int32_t)p;
    if (temp < -4000 || temp > 8500 || p < 1000 || p > 120000) return ADB_ERR_RANGE;
    return ADB_OK;
}

static adb_status_t read_prom_word(ms5611_t *d, uint8_t addr, uint16_t *w)
{
    uint8_t cmd = CMD_PROM_READ(addr), rx[2];
    adb_status_t s = d->bus.write_read(d->bus.ctx, &cmd, 1, rx, 2);
    if (s != ADB_OK) return s;
    *w = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);          /* MSB first, p.11 */
    return ADB_OK;
}

adb_status_t ms5611_init(ms5611_t *d, const adb_bus_t *bus, ms5611_osr_t osr,
                         uint8_t temp_every_n)
{
    if (!d || !bus || !bus->write || !bus->write_read || !bus->delay_us ||
        osr > MS5611_OSR_4096 || temp_every_n < 1u) return ADB_ERR_PARAM;
    *d = (ms5611_t){0};
    d->bus = *bus; d->osr = osr; d->temp_every_n = temp_every_n;

    uint8_t rst = CMD_RESET;                                  /* p.10: reset once after power-on */
    adb_status_t s = d->bus.write(d->bus.ctx, &rst, 1);
    if (s != ADB_OK) return s;
    d->bus.delay_us(d->bus.ctx, T_RESET_RELOAD_US + 200u);    /* 2.8 ms + margin */

    for (uint8_t a = 0; a < 8; a++) {
        s = read_prom_word(d, a, &d->prom[a]);
        if (s != ADB_OK) return s;
    }
    /* Plausibility: C1..C6 all 0x0000 or all 0xFFFF means a dead/absent device. */
    bool all0 = true, allf = true;
    for (int i = 1; i <= 6; i++) { if (d->prom[i]) all0 = false; if (d->prom[i] != 0xFFFFu) allf = false; }
    if (all0 || allf) return ADB_ERR_CALIB;
    if (ms5611_crc4(d->prom) != (d->prom[7] & 0x0Fu)) return ADB_ERR_CRC;

    d->state = MS5611_ST_START_TEMP;                          /* first cycle gets a temperature */
    return ADB_OK;
}

static adb_status_t start_conv(ms5611_t *d, bool temp, uint32_t now_us)
{
    uint8_t cmd = temp ? CMD_CONV_D2(d->osr) : CMD_CONV_D1(d->osr);
    adb_status_t s = d->bus.write(d->bus.ctx, &cmd, 1);
    if (s != ADB_OK) return s;
    d->t_start_us = now_us;
    return ADB_OK;
}

static adb_status_t read_adc(ms5611_t *d, uint32_t *raw)
{
    uint8_t cmd = CMD_ADC_READ, rx[3];
    adb_status_t s = d->bus.write_read(d->bus.ctx, &cmd, 1, rx, 3);
    if (s != ADB_OK) return s;
    *raw = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
    return ADB_OK;                                            /* 0 => conversion not done (p.11) */
}

adb_status_t ms5611_poll(ms5611_t *d, uint32_t now_us, bool *new_sample)
{
    if (!d || !new_sample) return ADB_ERR_PARAM;
    *new_sample = false;
    adb_status_t s;

    switch (d->state) {
    case MS5611_ST_START_TEMP:
    case MS5611_ST_START_PRESS: {
        bool temp = (d->state == MS5611_ST_START_TEMP);
        s = start_conv(d, temp, now_us);
        if (s != ADB_OK) { d->bus_error_count++; return s; }  /* stay in START: retry next call */
        d->state = temp ? MS5611_ST_WAIT_TEMP : MS5611_ST_WAIT_PRESS;
        return ADB_OK;
    }
    case MS5611_ST_WAIT_TEMP:
    case MS5611_ST_WAIT_PRESS: {
        bool temp = (d->state == MS5611_ST_WAIT_TEMP);
        if ((uint32_t)(now_us - d->t_start_us) < ms5611_conv_time_max_us(d->osr))
            return ADB_OK;                                    /* p.11: reading early gives 0 / corrupts */
        uint32_t raw = 0;
        s = read_adc(d, &raw);
        if (s != ADB_OK) {
            d->bus_error_count++;
            d->state = temp ? MS5611_ST_START_TEMP : MS5611_ST_START_PRESS;
            return s;
        }
        if (raw == 0u) {                                      /* conversion result invalid */
            d->invalid_count++;
            d->state = temp ? MS5611_ST_START_TEMP : MS5611_ST_START_PRESS;
            return ADB_ERR_INVALID;
        }
        if (temp) {
            d->d2 = raw;
            d->state = MS5611_ST_START_PRESS;
            return ADB_OK;
        }
        d->d1 = raw;
        d->press_count++;
        if (d->press_count >= d->temp_every_n) { d->press_count = 0; d->state = MS5611_ST_START_TEMP; }
        else                                    { d->state = MS5611_ST_START_PRESS; }
        if (d->d2 == 0u) return ADB_ERR_NOT_READY;            /* no temperature yet */
        ms5611_sample_t smp;
        s = ms5611_compensate(d->prom, d->d1, d->d2, &smp);
        d->last = smp;                                        /* keep for diagnostics even if RANGE */
        if (s != ADB_OK) { d->invalid_count++; return s; }
        d->have_sample = true;
        *new_sample = true;
        return ADB_OK;
    }
    }
    return ADB_ERR_STATE;
}
