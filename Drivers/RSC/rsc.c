/* rsc.c - see rsc.h. Page numbers refer to Honeywell 32321348 Issue G. */
#include "rsc.h"
#include <math.h>
#include <string.h>
#include <stddef.h>

/* EEPROM map, Tables 2-1..2-6 (p.21-23) */
#define EE_CATALOG   0u      /* 16 ASCII bytes                    */
#define EE_PRANGE    27u     /* float, LSB first                  */
#define EE_PMIN      31u     /* float                             */
#define EE_PUNIT     35u     /* 5 ASCII bytes                     */
#define EE_ADC_CFG0  61u     /* ADC_CONFIG_00..03 at 61,63,65,67  */
#define EE_OFFSET    130u    /* 4 floats: coefficient 0..3        */
#define EE_SPAN      210u
#define EE_SHAPE     290u
#define EE_CHECKSUM  450u    /* uint16, LSB first                 */

/* Commands (p.24-26) */
#define CMD_EE_READ_BASE 0x03u   /* [0000 X011], X = address bit 8 */
#define CMD_RESET        0x06u
#define CMD_START        0x08u
#define CMD_WREG_ALL     0x43u   /* [0100 RRNN] reg 0, 4 bytes     */
#define CMD_WREG_REG1    0x44u   /* reg 1, 1 byte                  */

#define T_STARTUP_US     2500u   /* <= 2 ms startup (Table 5)                                   */
#define T_RESET_US       1000u   /* UNCONFIRMED: datasheet gives no reset-to-command time       */

static const uint32_t k_conv_typ_us[RSC_RATE_COUNT] = {   /* Table 3-2 */
    49990u, 22250u, 11260u, 5780u, 3040u, 1680u, 1010u,   /* normal: 20,45,90,175,330,600,1000 SPS */
    25000u, 11120u, 5630u, 2890u, 1520u, 840u, 510u       /* fast:   40,90,180,350,660,1200,2000   */
};

/* ---------------------------------------------------------------- pure helpers */
uint16_t rsc_crc16_ccitt(const uint8_t *d, uint32_t n, uint16_t init)
{
    uint16_t crc = init;                                   /* polynomial 0x1021, p.29 */
    for (uint32_t i = 0; i < n; i++) {
        crc = (uint16_t)(crc ^ (uint16_t)((uint16_t)d[i] << 8));
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? (uint16_t)(((unsigned)crc << 1) ^ 0x1021u) : (uint16_t)((unsigned)crc << 1);
    }
    return crc;
}

static int read_float_le(const uint8_t *p, double *out)
{
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    float f; memcpy(&f, &u, sizeof f);                     /* IEEE-754 binary32, p.30 4.5 */
    if (!isfinite(f)) return -1;
    *out = (double)f;
    return 0;
}

static char lower_ascii(char ch) { return (ch >= 'A' && ch <= 'Z') ? (char)(ch + ('a' - 'A')) : ch; }

static adb_status_t unit_factor(const uint8_t *u5, double *f)
{
    char s[6];
    size_t n = 0;
    for (size_t i = 0; i < 5u; i++) {
        char ch = (char)u5[i];
        if (ch == '\0' || ch == ' ') break;
        s[n++] = ch;
    }
    s[n] = '\0';
    static const struct { const char *name; double f; } tab[] = {
        { "Pa", 1.0 }, { "kPa", 1000.0 }, { "MPa", 1.0e6 }, { "mbar", 100.0 }, { "bar", 1.0e5 },
        { "psi", 6894.757293 }, { "inH2O", 249.0889 } };  /* inH2O reference temperature UNCONFIRMED */
    for (size_t i = 0; i < sizeof tab / sizeof tab[0]; i++) {
        const char *a = s;
        const char *b = tab[i].name;
        while (*a != '\0' && *b != '\0' && lower_ascii(*a) == lower_ascii(*b)) { a++; b++; }
        if (*a == '\0' && *b == '\0') { *f = tab[i].f; return ADB_OK; }
    }
    return ADB_ERR_CONFIG;
}

adb_status_t rsc_parse_eeprom(const uint8_t ee[RSC_EEPROM_SIZE], rsc_crc_policy_t policy, rsc_calib_t *c)
{
    if (!ee || !c) return ADB_ERR_PARAM;
    memset(c, 0, sizeof *c);

    uint16_t stored = (uint16_t)(ee[EE_CHECKSUM] | ((unsigned)ee[EE_CHECKSUM + 1] << 8));
    /* U1: CRC init value / range not stated in the datasheet -> try the usual candidates. */
    static const uint16_t inits[2] = { 0x0000u, 0xFFFFu };
    for (int i = 0; i < 2 && !c->crc_ok; i++)
        if (rsc_crc16_ccitt(ee, EE_CHECKSUM, inits[i]) == stored) c->crc_ok = true;
    if (!c->crc_ok && policy == RSC_CRC_STRICT) return ADB_ERR_CRC;

    memcpy(c->catalog, ee + EE_CATALOG, 16); c->catalog[16] = '\0';
    for (int i = 0; i < 16; i++) if ((unsigned char)c->catalog[i] < 0x20u || (unsigned char)c->catalog[i] > 0x7Eu) c->catalog[i] = '\0';

    if (read_float_le(ee + EE_PRANGE, &c->p_range) || read_float_le(ee + EE_PMIN, &c->p_min)) return ADB_ERR_CALIB;
    if (!(c->p_range > 0.0)) return ADB_ERR_CALIB;
    adb_status_t s = unit_factor(ee + EE_PUNIT, &c->unit_to_pa);
    if (s != ADB_OK) return s;
    for (unsigned k = 0; k < 4u; k++) {
        if (read_float_le(ee + EE_OFFSET + 4u * k, &c->offset[k]) ||
            read_float_le(ee + EE_SPAN   + 4u * k, &c->span[k])   ||
            read_float_le(ee + EE_SHAPE  + 4u * k, &c->shape[k])) return ADB_ERR_CALIB;
        c->adc_cfg[k] = ee[EE_ADC_CFG0 + 2u * k];
    }
    return ADB_OK;
}

int32_t rsc_decode_temp_code(uint32_t raw24)
{
    int32_t v = (int32_t)((raw24 >> 10) & 0x3FFFu);        /* first 14 bits are valid, rest is noise (p.26) */
    return (v & 0x2000) ? v - 0x4000 : v;
}
int32_t rsc_decode_pressure_raw(uint32_t raw24)
{
    raw24 &= 0xFFFFFFu;
    return (int32_t)((raw24 & 0x800000u) ? (raw24 | 0xFF000000u) : raw24);
}

static double poly3(const double c[4], double x) { return ((c[3] * x + c[2]) * x + c[1]) * x + c[0]; }

/* Section 1.3 (p.20). Double precision is used instead of the sample code's float: the
 * Cortex-M7 FPU supports it and it only reduces round-off in the cubic terms. */
adb_status_t rsc_compensate(const rsc_calib_t *c, int32_t p_raw, int32_t t_code, double azero, double *p_pa)
{
    if (!c || !p_pa) return ADB_ERR_PARAM;
    double t = (double)t_code;
    double pint1 = ((double)p_raw + azero) - poly3(c->offset, t);
    double den   = poly3(c->span, t);
    if (!isfinite(den) || fabs(den) < 1e-30) return ADB_ERR_INVALID;
    double pint2 = pint1 / den;
    double pfs   = poly3(c->shape, pint2);
    double p_eng = pfs * c->p_range + c->p_min;
    if (!isfinite(p_eng)) return ADB_ERR_INVALID;
    *p_pa = p_eng * c->unit_to_pa;
    return ADB_OK;
}

/* Zero (offset) correction. Datasheet 5.0 step 3 gives  Praw_AZero = Offset(Traw0) - Praw0, which makes
 * zero pressure map to the BOTTOM of the range (Pint1 = 0). That is right for 0..Pmax (gage/absolute)
 * parts, but for a bidirectional differential range such as +/-60 mbar zero pressure is MID-scale
 * (Table 4-4 of the datasheet describes autozero as "50 % full scale"). We therefore generalise:
 * target Pcomp_FS = (0 - Pmin)/Prange, solve shape(x) = target for Pint2 (Newton, shape is near-identity),
 * then  Praw_AZero = Offset(T0) + x*Span(T0) - Praw0.  For Pmin = 0 this reduces to the datasheet formula.
 * UNCONFIRMED against Honeywell's sample code (open item U3). */
double rsc_zero_correction(const rsc_calib_t *c, int32_t praw0, int32_t traw0)
{
    double t0 = (double)traw0;
    double target = (0.0 - c->p_min) / c->p_range;
    double x = target;
    for (int i = 0; i < 8; i++) {
        double f  = poly3(c->shape, x) - target;
        double df = (3.0 * c->shape[3] * x + 2.0 * c->shape[2]) * x + c->shape[1];
        if (fabs(df) < 1e-12) break;
        x -= f / df;
    }
    return poly3(c->offset, t0) + x * poly3(c->span, t0) - (double)praw0;
}

uint32_t rsc_conv_time_us(rsc_rate_t r)
{
    uint32_t t = k_conv_typ_us[(r < RSC_RATE_COUNT) ? r : RSC_N20];
    return t + t / 20u + 100u;                             /* +5 % stated tolerance (p.25) + margin */
}

uint8_t rsc_config1_byte(rsc_rate_t r, bool temperature)   /* Table 3-1: DR[7:5] MODE[4:3] 1 TS 0 */
{
    unsigned fast = (r >= RSC_F40) ? 1u : 0u;
    unsigned dr   = fast ? (unsigned)(r - RSC_F40) : (unsigned)r;
    return (uint8_t)((dr << 5) | (fast ? 0x10u : 0x00u) | 0x04u | (temperature ? 0x02u : 0x00u));
}

/* ---------------------------------------------------------------- hardware paths */
static adb_status_t read_eeprom(rsc_t *d, uint8_t *ee)
{
    uint8_t tx[2] = { (uint8_t)CMD_EE_READ_BASE, 0x00u };  /* address 0, bit 8 = 0; address auto-increments (p.24) */
    return d->ee_bus.write_read(d->ee_bus.ctx, tx, 2, ee, RSC_EEPROM_SIZE);
}

adb_status_t rsc_init(rsc_t *d, const adb_bus_t *ee_bus, const adb_bus_t *adc_bus, const rsc_config_t *cfg)
{
    if (!d || !ee_bus || !adc_bus || !cfg || !ee_bus->write_read || !adc_bus->write || !adc_bus->read ||
        !adc_bus->delay_us || cfg->rate >= RSC_RATE_COUNT || cfg->temp_every_n < 1u) return ADB_ERR_PARAM;
    *d = (rsc_t){0};
    d->ee_bus = *ee_bus; d->adc_bus = *adc_bus; d->cfg = *cfg;

    d->adc_bus.delay_us(d->adc_bus.ctx, T_STARTUP_US);
    uint8_t ee[RSC_EEPROM_SIZE];
    adb_status_t s = read_eeprom(d, ee);
    if (s != ADB_OK) return s;

    int all0 = 1, allf = 1;                                /* dead bus / floating MISO */
    for (uint32_t i = 0; i < RSC_EEPROM_SIZE; i++) { if (ee[i] != 0x00u) all0 = 0; if (ee[i] != 0xFFu) allf = 0; }
    if (all0 || allf) return ADB_ERR_CALIB;

    s = rsc_parse_eeprom(ee, cfg->crc_policy, &d->cal);
    if (s != ADB_OK) return s;
    d->crc_verified = d->cal.crc_ok;
    d->catalog_match = (cfg->expected_catalog == NULL) ||
                       (strncmp(d->cal.catalog, cfg->expected_catalog, strlen(cfg->expected_catalog)) == 0);

    uint8_t rst = CMD_RESET;                               /* p.26: reset, then write all 4 config registers */
    s = d->adc_bus.write(d->adc_bus.ctx, &rst, 1);
    if (s != ADB_OK) return s;
    d->adc_bus.delay_us(d->adc_bus.ctx, T_RESET_US);
    uint8_t wreg[5] = { CMD_WREG_ALL, d->cal.adc_cfg[0], d->cal.adc_cfg[1], d->cal.adc_cfg[2], d->cal.adc_cfg[3] };
    s = d->adc_bus.write(d->adc_bus.ctx, wreg, 5);
    if (s != ADB_OK) return s;

    d->state = RSC_ST_START_TEMP;
    return ADB_OK;
}

static adb_status_t start_conv(rsc_t *d, bool temp, uint32_t now_us)
{
    /* WREG register 1 (rate + TS bit) followed by START, in one chip-select frame (p.26). */
    uint8_t tx[3] = { CMD_WREG_REG1, rsc_config1_byte(d->cfg.rate, temp), CMD_START };
    adb_status_t s = d->adc_bus.write(d->adc_bus.ctx, tx, 3);
    if (s == ADB_OK) d->t_start_us = now_us;
    return s;
}

adb_status_t rsc_poll(rsc_t *d, uint32_t now_us, bool *new_sample)
{
    if (!d || !new_sample) return ADB_ERR_PARAM;
    *new_sample = false;
    adb_status_t s;
    switch (d->state) {
    case RSC_ST_START_TEMP:
    case RSC_ST_START_PRESS: {
        bool temp = (d->state == RSC_ST_START_TEMP);
        s = start_conv(d, temp, now_us);
        if (s != ADB_OK) { d->bus_error_count++; return s; }
        d->state = temp ? RSC_ST_WAIT_TEMP : RSC_ST_WAIT_PRESS;
        return ADB_OK;
    }
    case RSC_ST_WAIT_TEMP:
    case RSC_ST_WAIT_PRESS: {
        bool temp = (d->state == RSC_ST_WAIT_TEMP);
        if ((uint32_t)(now_us - d->t_start_us) < rsc_conv_time_us(d->cfg.rate)) return ADB_OK;
        uint8_t b[3];
        s = d->adc_bus.read(d->adc_bus.ctx, b, 3);
        rsc_state_t restart = temp ? RSC_ST_START_TEMP : RSC_ST_START_PRESS;
        if (s != ADB_OK) { d->bus_error_count++; d->state = restart; return s; }
        uint32_t raw = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
        if (temp) {
            int32_t tc = rsc_decode_temp_code(raw);
            double tdeg = (double)tc * 0.03125;            /* p.26 */
            if (tdeg < -40.0 || tdeg > 85.0) { d->invalid_count++; d->state = restart; return ADB_ERR_RANGE; }
            d->t_code_last = tc; d->have_temp = true;
            d->state = RSC_ST_START_PRESS;
            return ADB_OK;
        }
        d->press_count++;
        if (d->press_count >= d->cfg.temp_every_n) { d->press_count = 0; d->state = RSC_ST_START_TEMP; }
        else d->state = RSC_ST_START_PRESS;
        if (!d->have_temp) return ADB_ERR_NOT_READY;
        int32_t pr = rsc_decode_pressure_raw(raw);
        if (pr == 0x7FFFFF || pr == -0x800000) { d->invalid_count++; return ADB_ERR_RANGE; }   /* ADC saturated, Table 3-4 */
        double ppa;
        s = rsc_compensate(&d->cal, pr, d->t_code_last, d->praw_azero, &ppa);
        if (s != ADB_OK) { d->invalid_count++; return s; }
        d->last.p_raw = pr; d->last.t_code = d->t_code_last;
        d->last.temp_c = (double)d->t_code_last * 0.03125;
        d->last.pressure_pa = ppa;
        double lo = (d->cal.p_min - 0.02 * d->cal.p_range) * d->cal.unit_to_pa;
        double hi = (d->cal.p_min + 1.02 * d->cal.p_range) * d->cal.unit_to_pa;
        if (ppa < lo || ppa > hi) { d->invalid_count++; return ADB_ERR_RANGE; }                 /* value kept in d->last */
        d->have_sample = true; *new_sample = true;
        return ADB_OK;
    }
    }
    return ADB_ERR_STATE;
}

void rsc_set_zero(rsc_t *d, int32_t praw0, int32_t traw0)
{
    if (d) d->praw_azero = rsc_zero_correction(&d->cal, praw0, traw0);
}
