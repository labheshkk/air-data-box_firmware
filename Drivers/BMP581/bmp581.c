/* bmp581.c - see bmp581.h. References are to BST-BMP581-DS004-13. */
#include "bmp581.h"
#include <stddef.h>

/* Register map - datasheet chapter 7 */
#define REG_CHIP_ID      0x01u   /* 7.1  reset value 0b01010000 = 0x50                  */
#define REG_TEMP_XLSB    0x1Du   /* 7.11 .. 7.16: T XLSB,LSB,MSB then P XLSB,LSB,MSB     */
#define REG_INT_STATUS   0x27u   /* 7.21 clear-on-read; bit0 drdy, bit3 oor_p, bit4 por */
#define REG_STATUS       0x28u   /* 7.22 bit1 nvm_rdy, bit2 nvm_err                     */
#define REG_DSP_CONFIG   0x30u   /* 7.27 reset 0x03 (compensation enabled)              */
#define REG_OSR_CONFIG   0x36u   /* 7.33 [6]press_en [5:3]osr_p [2:0]osr_t               */
#define REG_ODR_CONFIG   0x37u   /* 7.34 [7]deep_dis [6:2]odr [1:0]pwr_mode              */
#define REG_OSR_EFF      0x38u   /* 7.35 [7] odr_is_valid                                */
#define REG_CMD          0x7Eu   /* 7.36 0xB6 = soft reset (4.3.10)                      */

#define CHIP_ID_VALUE        0x50u
#define CMD_SOFT_RESET       0xB6u
#define PWR_MODE_STANDBY     0x0u
#define PWR_MODE_NORMAL      0x1u

#define INT_DRDY   (1u << 0)
#define INT_OOR_P  (1u << 3)
#define INT_POR    (1u << 4)
#define ST_NVM_RDY (1u << 1)
#define ST_NVM_ERR (1u << 2)

#define SPI_READ_FLAG  0x80u     /* Fig.11 p.38: MOSI first bit 'R' = 1 for read, 'W' = 0 for write */

/* Timing: t_powup 2 ms, tsoft_res 2 ms (p.9 table). We add margin. */
#define T_SOFT_RESET_US  3000u

/* Operating range, datasheet p.1: 30..125 kPa; -40..85 degC (operating range table). */
#define P_MIN_Q6   (30000 * 64)
#define P_MAX_Q6   (125000 * 64)
#define T_MIN_Q16  (-40 * 65536)
#define T_MAX_Q16  (85 * 65536)

/* Reset value of all six data registers is 0x7F (regs 7.11-7.16) => "no sample yet". */
#define RAW24_NO_DATA 0x7F7F7Fu

static adb_status_t reg_read(bmp581_t *d, uint8_t reg, uint8_t *buf, uint16_t n)
{
    uint8_t tx = (d->bus.kind == ADB_BUS_SPI) ? (uint8_t)(reg | SPI_READ_FLAG) : reg;
    /* Burst reads auto-increment the register address (datasheet 5.4). */
    return d->bus.write_read(d->bus.ctx, &tx, 1, buf, n);
}

static adb_status_t reg_write(bmp581_t *d, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), val };   /* bit7 = 0 => write */
    return d->bus.write(d->bus.ctx, tx, 2);
}

int32_t bmp581_sign_extend24(uint32_t v)
{
    v &= 0xFFFFFFu;
    return (int32_t)((v & 0x800000u) ? (v | 0xFF000000u) : v);
}

adb_status_t bmp581_decode_burst(const uint8_t b[6], bmp581_sample_t *out)
{
    if (!b || !out) return ADB_ERR_PARAM;
    uint32_t t_raw = ((uint32_t)b[2] << 16) | ((uint32_t)b[1] << 8) | b[0];  /* 0x1F,0x1E,0x1D */
    uint32_t p_raw = ((uint32_t)b[5] << 16) | ((uint32_t)b[4] << 8) | b[3];  /* 0x22,0x21,0x20 */
    if (t_raw == RAW24_NO_DATA && p_raw == RAW24_NO_DATA) return ADB_ERR_NOT_READY;
    out->temp_c_q16    = bmp581_sign_extend24(t_raw);   /* T[degC] = raw / 2^16 (4.5) */
    out->pressure_pa_q6 = bmp581_sign_extend24(p_raw);  /* p[Pa]   = raw / 2^6  (4.5) */
    if (out->pressure_pa_q6 < P_MIN_Q6 || out->pressure_pa_q6 > P_MAX_Q6 ||
        out->temp_c_q16 < T_MIN_Q16 || out->temp_c_q16 > T_MAX_Q16) return ADB_ERR_RANGE;
    return ADB_OK;
}

/* SPI needs a dummy read first to leave the default I2C/I3C mode (4.1, 4.3.9). */
static adb_status_t spi_dummy_read(bmp581_t *d)
{
    if (d->bus.kind != ADB_BUS_SPI) return ADB_OK;
    uint8_t junk;
    return reg_read(d, REG_CHIP_ID, &junk, 1);
}

adb_status_t bmp581_init(bmp581_t *d, const adb_bus_t *bus, const bmp581_config_t *cfg)
{
    if (!d || !bus || !cfg || !bus->write || !bus->write_read || !bus->delay_us ||
        cfg->osr_p > BMP581_OSR_X128 || cfg->osr_t > BMP581_OSR_X128 || cfg->odr > 0x1Fu)
        return ADB_ERR_PARAM;
    *d = (bmp581_t){0};
    d->bus = *bus; d->cfg = *cfg;

    adb_status_t s = spi_dummy_read(d);                       /* ignore value, only bus errors matter */
    if (s != ADB_OK) return s;

    s = reg_write(d, REG_CMD, CMD_SOFT_RESET);                /* 4.3.10 */
    if (s != ADB_OK) return s;
    d->bus.delay_us(d->bus.ctx, T_SOFT_RESET_US);
    s = spi_dummy_read(d);                                    /* soft reset returns HIF to I2C mode */
    if (s != ADB_OK) return s;

    uint8_t v;
    s = reg_read(d, REG_CHIP_ID, &v, 1);
    if (s != ADB_OK) return s;
    if (v != CHIP_ID_VALUE) return ADB_ERR_ID;

    s = reg_read(d, REG_INT_STATUS, &v, 1);                   /* 4.3.9: por must be set after reset */
    if (s != ADB_OK) return s;
    if (!(v & INT_POR)) return ADB_ERR_STATE;

    s = reg_read(d, REG_STATUS, &v, 1);
    if (s != ADB_OK) return s;
    if (!(v & ST_NVM_RDY) || (v & ST_NVM_ERR)) return ADB_ERR_STATE;

    /* Configure while in STANDBY (device is in standby after reset). */
    uint8_t osr = (uint8_t)((1u << 6) | ((unsigned)cfg->osr_p << 3) | (unsigned)cfg->osr_t);  /* press_en=1 */
    s = reg_write(d, REG_OSR_CONFIG, osr);
    if (s != ADB_OK) return s;
    uint8_t odr = (uint8_t)((1u << 7) | ((unsigned)cfg->odr << 2) | PWR_MODE_NORMAL);         /* deep_dis=1 */
    s = reg_write(d, REG_ODR_CONFIG, odr);
    if (s != ADB_OK) return s;

    /* Read back: device must report valid ODR/OSR combination (7.33/7.34 notes). */
    s = reg_read(d, REG_OSR_EFF, &v, 1);
    if (s != ADB_OK) return s;
    if (!(v & 0x80u)) return ADB_ERR_CONFIG;
    s = reg_read(d, REG_ODR_CONFIG, &v, 1);
    if (s != ADB_OK) return s;
    if ((v & 0x3u) != PWR_MODE_NORMAL) return ADB_ERR_CONFIG;
    return ADB_OK;
}

adb_status_t bmp581_read(bmp581_t *d, bmp581_sample_t *out)
{
    if (!d || !out) return ADB_ERR_PARAM;
    uint8_t b[6];
    adb_status_t s = reg_read(d, REG_TEMP_XLSB, b, 6);        /* ONE burst, datasheet 4.5.1 */
    if (s != ADB_OK) { d->bus_error_count++; return s; }
    s = bmp581_decode_burst(b, out);
    if (s != ADB_OK && s != ADB_ERR_RANGE) { d->invalid_count++; return s; }
    d->last = *out;
    if (s == ADB_OK) d->have_sample = true; else d->invalid_count++;
    return s;
}

/* Reads INT_STATUS (clear-on-read, 7.21): returns a sample only if drdy was set, so
 * the same measurement is never reported twice. oor_p flag => ADB_ERR_RANGE. */
adb_status_t bmp581_poll(bmp581_t *d, bmp581_sample_t *out, bool *new_sample)
{
    if (!d || !out || !new_sample) return ADB_ERR_PARAM;
    *new_sample = false;
    uint8_t is;
    adb_status_t s = reg_read(d, REG_INT_STATUS, &is, 1);
    if (s != ADB_OK) { d->bus_error_count++; return s; }
    if (!(is & INT_DRDY)) return ADB_OK;                       /* nothing new yet - not an error */
    s = bmp581_read(d, out);
    if (s == ADB_OK) {
        if (is & INT_OOR_P) return ADB_ERR_RANGE;              /* sensor's own out-of-range flag */
        *new_sample = true;
    }
    return s;
}
