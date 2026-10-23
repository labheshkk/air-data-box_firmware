/* test_bmp581.c - host tests for the BMP581 driver using a register-file mock.
 * The mock models: SPI needing a dummy read to leave I2C mode (datasheet 4.1),
 * soft reset (0xB6 -> reset values, back to I2C mode), clear-on-read INT_STATUS,
 * and the 0x7F7F7F "no data" reset pattern of the data registers. */
#include <string.h>
#include "bmp581.h"
#include "test_common.h"

typedef struct {
    uint8_t reg[128];
    int     spi_mode_active;     /* becomes 1 after a dummy read in SPI */
    int     is_spi;
    int     fail_next;           /* force bus error on next transaction */
    int     ignored_before_dummy;
    int     odr_invalid;         /* make odr_is_valid read 0 */
    uint8_t chip_id;
    uint8_t status_reset;
} mock_t;

static void mock_reset_regs(mock_t *m)
{
    memset(m->reg, 0, sizeof m->reg);
    m->reg[0x01] = m->chip_id;
    m->reg[0x27] = 0x10;                 /* por set */
    m->reg[0x28] = m->status_reset;      /* nvm_rdy */
    m->reg[0x30] = 0x03;
    m->reg[0x37] = 0x70;                 /* reset value 0111 0000 */
    for (int r = 0x1D; r <= 0x22; r++) m->reg[r] = 0x7F;
    m->spi_mode_active = 0;
}
static void mock_init(mock_t *m, int is_spi)
{
    memset(m, 0, sizeof *m);
    m->is_spi = is_spi; m->chip_id = 0x50; m->status_reset = 0x02;
    mock_reset_regs(m);
}
static adb_status_t m_write(void *c, const uint8_t *tx, uint16_t len)
{
    mock_t *m = c;
    if (m->fail_next) { m->fail_next = 0; return ADB_ERR_BUS; }
    if (len != 2) return ADB_ERR_PARAM;
    if (m->is_spi && !m->spi_mode_active) { m->ignored_before_dummy++; return ADB_OK; }  /* device ignores it */
    if (tx[0] & 0x80) return ADB_ERR_PARAM;                   /* write must have bit7 = 0 */
    if (tx[0] == 0x7E && tx[1] == 0xB6) { mock_reset_regs(m); return ADB_OK; }
    m->reg[tx[0]] = tx[1];
    if (tx[0] == 0x37 && (tx[1] & 3) == 1) {                  /* NORMAL mode: first sample appears later */
        /* data registers keep reset pattern until test injects a sample */
    }
    return ADB_OK;
}
static adb_status_t m_write_read(void *c, const uint8_t *tx, uint16_t txlen, uint8_t *rx, uint16_t rxlen)
{
    mock_t *m = c;
    if (m->fail_next) { m->fail_next = 0; return ADB_ERR_BUS; }
    if (txlen != 1) return ADB_ERR_PARAM;
    uint8_t reg = tx[0];
    if (m->is_spi) {
        if (!(reg & 0x80)) return ADB_ERR_PARAM;              /* SPI read must have bit7 = 1 */
        reg &= 0x7F;
        if (!m->spi_mode_active) { m->spi_mode_active = 1; memset(rx, 0xFF, rxlen); return ADB_OK; }  /* dummy read */
    } else if (reg & 0x80) return ADB_ERR_PARAM;
    for (uint16_t i = 0; i < rxlen; i++) {
        uint8_t r = (uint8_t)(reg + i);
        uint8_t v = m->reg[r];
        if (r == 0x38) v = m->odr_invalid ? 0x00 : 0x80;     /* odr_is_valid flag */
        rx[i] = v;
    }
    if (reg <= 0x27 && reg + rxlen > 0x27) m->reg[0x27] = 0;  /* INT_STATUS clear-on-read */
    return ADB_OK;
}
static void m_delay(void *c, uint32_t us) { (void)c; (void)us; }
static adb_bus_t bus_of(mock_t *m)
{
    return (adb_bus_t){ .ctx = m, .kind = m->is_spi ? ADB_BUS_SPI : ADB_BUS_I2C,
                        .write = m_write, .write_read = m_write_read, .delay_us = m_delay };
}
static const bmp581_config_t CFG = { .osr_p = BMP581_OSR_X4, .osr_t = BMP581_OSR_X1, .odr = 0x0F };

static void inject(mock_t *m, int32_t t_q16, int32_t p_q6, uint8_t int_status)
{
    uint32_t t = (uint32_t)t_q16 & 0xFFFFFFu, p = (uint32_t)p_q6 & 0xFFFFFFu;
    m->reg[0x1D] = (uint8_t)t; m->reg[0x1E] = (uint8_t)(t >> 8); m->reg[0x1F] = (uint8_t)(t >> 16);
    m->reg[0x20] = (uint8_t)p; m->reg[0x21] = (uint8_t)(p >> 8); m->reg[0x22] = (uint8_t)(p >> 16);
    m->reg[0x27] = int_status;
}

static void test_sign_extend(void)
{
    CHECK_EQ_I(bmp581_sign_extend24(0x000001), 1);
    CHECK_EQ_I(bmp581_sign_extend24(0x7FFFFF), 8388607);
    CHECK_EQ_I(bmp581_sign_extend24(0x800000), -8388608);
    CHECK_EQ_I(bmp581_sign_extend24(0xFFFFFF), -1);
    CHECK_EQ_I(bmp581_sign_extend24(0xF60000), -655360);      /* -10 degC in Q16 */
}
static void test_decode(void)
{
    bmp581_sample_t s;
    /* 25.0 degC = 0x190000, 101325 Pa = 6484800 = 0x62F340 (burst order: T x,l,m, P x,l,m) */
    const uint8_t b[6] = { 0x00, 0x00, 0x19, 0x40, 0xF3, 0x62 };
    CHECK_EQ_I(bmp581_decode_burst(b, &s), ADB_OK);
    CHECK_EQ_I(s.temp_c_q16, 25 * 65536);
    CHECK_EQ_I(s.pressure_pa_q6, 101325 * 64);
    const uint8_t cold[6] = { 0x00, 0x00, 0xF6, 0x40, 0xF3, 0x62 };            /* -10 degC */
    CHECK_EQ_I(bmp581_decode_burst(cold, &s), ADB_OK);
    CHECK_EQ_I(s.temp_c_q16, -10 * 65536);
    const uint8_t nodata[6] = { 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F };
    CHECK_EQ_I(bmp581_decode_burst(nodata, &s), ADB_ERR_NOT_READY);
    const uint8_t low[6] = { 0x00, 0x00, 0x19, 0x00, 0x88, 0x13 };             /* 20 kPa (below 30 kPa spec) */
    CHECK_EQ_I(bmp581_decode_burst(low, &s), ADB_ERR_RANGE);
    CHECK_EQ_I(s.pressure_pa_q6, 20000 * 64);
    CHECK_EQ_I(bmp581_decode_burst(NULL, &s), ADB_ERR_PARAM);
}
static void init_ok(int is_spi)
{
    mock_t m; bmp581_t d; adb_bus_t b;
    mock_init(&m, is_spi); b = bus_of(&m);
    CHECK_EQ_I(bmp581_init(&d, &b, &CFG), ADB_OK);
    CHECK_EQ_I(m.reg[0x36], 0x50);          /* press_en=1, osr_p=x4 (010), osr_t=x1 (000) */
    CHECK_EQ_I(m.reg[0x37], 0xBD);          /* deep_dis=1, odr=0x0F, NORMAL */
    CHECK_EQ_I(m.ignored_before_dummy, 0);  /* driver did the SPI dummy read first */
}
static void test_init_ok_spi(void) { init_ok(1); }
static void test_init_ok_i2c(void) { init_ok(0); }
static void test_init_failures(void)
{
    mock_t m; bmp581_t d; adb_bus_t b;
    mock_init(&m, 0); m.chip_id = 0x00; mock_reset_regs(&m); b = bus_of(&m);
    CHECK_EQ_I(bmp581_init(&d, &b, &CFG), ADB_ERR_ID);
    mock_init(&m, 0); m.status_reset = 0x06; mock_reset_regs(&m); b = bus_of(&m);       /* nvm_err set */
    CHECK_EQ_I(bmp581_init(&d, &b, &CFG), ADB_ERR_STATE);
    mock_init(&m, 0); m.status_reset = 0x00; mock_reset_regs(&m); b = bus_of(&m);       /* nvm not ready */
    CHECK_EQ_I(bmp581_init(&d, &b, &CFG), ADB_ERR_STATE);
    mock_init(&m, 0); m.odr_invalid = 1; b = bus_of(&m);
    CHECK_EQ_I(bmp581_init(&d, &b, &CFG), ADB_ERR_CONFIG);
    mock_init(&m, 0); m.fail_next = 1; b = bus_of(&m);
    CHECK_EQ_I(bmp581_init(&d, &b, &CFG), ADB_ERR_BUS);
    bmp581_config_t bad = CFG; bad.odr = 0x20;
    mock_init(&m, 0); b = bus_of(&m);
    CHECK_EQ_I(bmp581_init(&d, &b, &bad), ADB_ERR_PARAM);
    CHECK_EQ_I(bmp581_init(&d, NULL, &CFG), ADB_ERR_PARAM);
}
static void test_poll_flow(void)
{
    mock_t m; bmp581_t d; adb_bus_t b; bmp581_sample_t s; bool ns;
    mock_init(&m, 1); b = bus_of(&m);
    CHECK_EQ_I(bmp581_init(&d, &b, &CFG), ADB_OK);
    CHECK_EQ_I(bmp581_poll(&d, &s, &ns), ADB_OK);             /* no drdy yet: not an error, no sample */
    CHECK(!ns);
    inject(&m, 22 * 65536, 98000 * 64, 0x01);                 /* drdy */
    CHECK_EQ_I(bmp581_poll(&d, &s, &ns), ADB_OK);
    CHECK(ns); CHECK_EQ_I(s.pressure_pa_q6, 98000 * 64); CHECK_EQ_I(s.temp_c_q16, 22 * 65536);
    CHECK_EQ_I(bmp581_poll(&d, &s, &ns), ADB_OK);             /* INT_STATUS was cleared -> no duplicate */
    CHECK(!ns);
    inject(&m, 22 * 65536, 98000 * 64, 0x09);                 /* drdy + oor_p */
    CHECK_EQ_I(bmp581_poll(&d, &s, &ns), ADB_ERR_RANGE);
    CHECK(!ns);
    m.fail_next = 1;
    CHECK_EQ_I(bmp581_poll(&d, &s, &ns), ADB_ERR_BUS);
    CHECK_EQ_I(d.bus_error_count, 1);
    inject(&m, 22 * 65536, 97000 * 64, 0x01);                 /* recovers */
    CHECK_EQ_I(bmp581_poll(&d, &s, &ns), ADB_OK); CHECK(ns);
    inject(&m, 0x7F7F7F, 0x7F7F7F, 0x01);                     /* drdy but reset pattern */
    CHECK_EQ_I(bmp581_poll(&d, &s, &ns), ADB_ERR_NOT_READY); CHECK(!ns);
}

int main(void)
{
    RUN(test_sign_extend); RUN(test_decode); RUN(test_init_ok_spi); RUN(test_init_ok_i2c);
    RUN(test_init_failures); RUN(test_poll_flow);
    TEST_SUMMARY();
    return g_tests_failed ? 1 : 0;
}
