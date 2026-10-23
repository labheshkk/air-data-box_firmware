/* test_rsc.c - host tests for the Honeywell RSC driver.
 * Vectors: temperature examples from datasheet Table 3-3 / p.26-27; CRC check values of the standard
 * CRC-16 variants; compensation reference computed by an independent Python model (double precision).
 * The EEPROM image is SYNTHETIC (invented coefficients with the datasheet's layout), not a real chip dump. */
#include <math.h>
#include <string.h>
#include "rsc.h"
#include "test_common.h"

#define NEAR(a, b, tol) do { g_tests_run++; double _a = (a), _b = (b); \
    if (!(fabs(_a - _b) <= (tol))) { g_tests_failed++; \
    printf("  FAIL %s:%d: %s ~ %s (got %.9g, expected %.9g)\n", __FILE__, __LINE__, #a, #b, _a, _b); } } while (0)

static void put_f(uint8_t *ee, unsigned a, float f) { memcpy(ee + a, &f, 4); }   /* host is little-endian */

static void build_ee(uint8_t ee[RSC_EEPROM_SIZE])
{
    memset(ee, 0, RSC_EEPROM_SIZE);
    memcpy(ee, "RSCDJJI060MDSE3", 15);
    put_f(ee, 27, 120.0f); put_f(ee, 31, -60.0f);
    memcpy(ee + 35, "mbar ", 5);
    ee[61] = 0x11; ee[63] = 0x22; ee[65] = 0x33; ee[67] = 0x44;
    const float off[4] = { -4000000.0f, 0.125f, 0, 0 }, span[4] = { 8000000.0f, 128.0f, 0, 0 }, shape[4] = { 0, 1.0f, 0.015625f, 0 };
    for (unsigned k = 0; k < 4; k++) { put_f(ee, 130 + 4 * k, off[k]); put_f(ee, 210 + 4 * k, span[k]); put_f(ee, 290 + 4 * k, shape[k]); }
    uint16_t crc = rsc_crc16_ccitt(ee, 450, 0xFFFFu);            /* self-consistent (candidate init 0xFFFF) */
    ee[450] = (uint8_t)crc; ee[451] = (uint8_t)(crc >> 8);
}

/* ---- mocks ---- */
typedef struct {
    uint8_t  ee[RSC_EEPROM_SIZE];
    uint32_t now_us;
    int      ts, pending; uint32_t start_us, true_conv_us;
    int32_t  p_raw, t_code;
    int      n_reset, n_wreg_all, n_start, n_read, n_early_read, fail_next_adc_write, fail_next_read, fail_ee;
    uint8_t  wreg[4];
    int      saturate;
} mock_t;

static adb_status_t ee_wr(void *c, const uint8_t *tx, uint16_t txlen, uint8_t *rx, uint16_t rxlen)
{
    mock_t *m = c;
    if (m->fail_ee) return ADB_ERR_BUS;
    if (txlen != 2 || tx[0] != 0x03 || tx[1] != 0x00 || rxlen != RSC_EEPROM_SIZE) return ADB_ERR_PARAM;
    memcpy(rx, m->ee, rxlen); return ADB_OK;
}
static adb_status_t adc_w(void *c, const uint8_t *tx, uint16_t len)
{
    mock_t *m = c;
    if (m->fail_next_adc_write) { m->fail_next_adc_write = 0; return ADB_ERR_BUS; }
    if (len == 1 && tx[0] == 0x06) { m->n_reset++; return ADB_OK; }
    if (len == 5 && tx[0] == 0x43) { m->n_wreg_all++; memcpy(m->wreg, tx + 1, 4); return ADB_OK; }
    if (len == 3 && tx[0] == 0x44 && tx[2] == 0x08) {
        m->ts = (tx[1] >> 1) & 1; m->pending = 1; m->start_us = m->now_us; m->n_start++; return ADB_OK; }
    return ADB_ERR_PARAM;
}
static adb_status_t adc_r(void *c, uint8_t *rx, uint16_t len)
{
    mock_t *m = c;
    if (m->fail_next_read) { m->fail_next_read = 0; return ADB_ERR_BUS; }
    if (len != 3) return ADB_ERR_PARAM;
    m->n_read++;
    if (!m->pending || (uint32_t)(m->now_us - m->start_us) < m->true_conv_us) { m->n_early_read++; memset(rx, 0, 3); return ADB_OK; }
    m->pending = 0;
    uint32_t v = m->ts ? ((uint32_t)(m->t_code & 0x3FFF) << 10) : ((uint32_t)m->p_raw & 0xFFFFFFu);
    if (!m->ts && m->saturate) v = 0x7FFFFFu;
    rx[0] = (uint8_t)(v >> 16); rx[1] = (uint8_t)(v >> 8); rx[2] = (uint8_t)v; return ADB_OK;
}
static void dly(void *c, uint32_t us) { ((mock_t *)c)->now_us += us; }
static void setup(mock_t *m, adb_bus_t *ee, adb_bus_t *adc)
{
    memset(m, 0, sizeof *m); build_ee(m->ee); m->p_raw = 1000000; m->t_code = 800; m->true_conv_us = 11260;
    *ee  = (adb_bus_t){ .ctx = m, .kind = ADB_BUS_SPI, .write_read = ee_wr, .delay_us = dly };
    *adc = (adb_bus_t){ .ctx = m, .kind = ADB_BUS_SPI, .write = adc_w, .read = adc_r, .delay_us = dly };
}
static const rsc_config_t CFG = { RSC_N90, 4, RSC_CRC_ACCEPT_UNVERIFIED, "RSCDJJI060MDSE3" };

/* ---- tests ---- */
static void test_crc_vectors(void)
{
    const uint8_t s[9] = { '1','2','3','4','5','6','7','8','9' };
    CHECK_EQ_I(rsc_crc16_ccitt(s, 9, 0xFFFF), 0x29B1);   /* CRC-16/CCITT-FALSE check value (also in DroneCAN spec) */
    CHECK_EQ_I(rsc_crc16_ccitt(s, 9, 0x0000), 0x31C3);   /* CRC-16/XMODEM check value */
}
static void test_decoders(void)
{
    CHECK_EQ_I(rsc_decode_temp_code(0x0960u << 10), 2400);           /* 75 C   (Table 3-3, Example 1) */
    CHECK_EQ_I(rsc_decode_temp_code(0x3CE0u << 10), -800);           /* -25 C  (Example 2)            */
    CHECK_EQ_I(rsc_decode_temp_code(0x3B00u << 10), -1280);          /* -40 C                         */
    CHECK_EQ_I(rsc_decode_temp_code(0x3FF8u << 10), -8);             /* -0.25 C                       */
    CHECK_EQ_I(rsc_decode_temp_code((0x0320u << 10) | 0x3FFu), 800); /* low 10 bits are noise: ignored */
    CHECK_EQ_I(rsc_decode_pressure_raw(0x7FFFFF), 8388607);
    CHECK_EQ_I(rsc_decode_pressure_raw(0x800000), -8388608);
    CHECK_EQ_I(rsc_decode_pressure_raw(0xFFFFFF), -1);
    CHECK_EQ_I(rsc_decode_pressure_raw(0x000001), 1);
}
static void test_config_byte_and_times(void)
{
    CHECK_EQ_I(rsc_config1_byte(RSC_N20, true), 0x06);
    CHECK_EQ_I(rsc_config1_byte(RSC_N20, false), 0x04);
    CHECK_EQ_I(rsc_config1_byte(RSC_N330, false), 0x84);
    CHECK_EQ_I(rsc_config1_byte(RSC_N1000, false), 0xC4);
    CHECK_EQ_I(rsc_config1_byte(RSC_F2000, false), 0xD4);
    CHECK_EQ_I(rsc_config1_byte(RSC_F40, true), 0x16);
    CHECK_EQ_I(rsc_conv_time_us(RSC_N90), 11260 + 563 + 100);
}
static void test_parse_eeprom(void)
{
    uint8_t ee[RSC_EEPROM_SIZE]; rsc_calib_t c; build_ee(ee);
    CHECK_EQ_I(rsc_parse_eeprom(ee, RSC_CRC_STRICT, &c), ADB_OK);
    CHECK(c.crc_ok); NEAR(c.p_range, 120.0, 0); NEAR(c.p_min, -60.0, 0); NEAR(c.unit_to_pa, 100.0, 0);
    NEAR(c.offset[0], -4000000.0, 0); NEAR(c.span[1], 128.0, 0); NEAR(c.shape[2], 0.015625, 0);
    CHECK_EQ_I(c.adc_cfg[0], 0x11); CHECK_EQ_I(c.adc_cfg[3], 0x44);
    CHECK(strcmp(c.catalog, "RSCDJJI060MDSE3") == 0);
    ee[300] ^= 0x01;                                                  /* corrupt a coefficient byte */
    CHECK_EQ_I(rsc_parse_eeprom(ee, RSC_CRC_STRICT, &c), ADB_ERR_CRC);
    CHECK_EQ_I(rsc_parse_eeprom(ee, RSC_CRC_ACCEPT_UNVERIFIED, &c), ADB_OK);
    CHECK(!c.crc_ok);                                                 /* accepted but flagged */
    build_ee(ee); put_f(ee, 27, 0.0f);                                /* range 0 */
    CHECK_EQ_I(rsc_parse_eeprom(ee, RSC_CRC_ACCEPT_UNVERIFIED, &c), ADB_ERR_CALIB);
    build_ee(ee); put_f(ee, 130, NAN);
    CHECK_EQ_I(rsc_parse_eeprom(ee, RSC_CRC_ACCEPT_UNVERIFIED, &c), ADB_ERR_CALIB);
    build_ee(ee); memcpy(ee + 35, "furlg", 5);
    CHECK_EQ_I(rsc_parse_eeprom(ee, RSC_CRC_ACCEPT_UNVERIFIED, &c), ADB_ERR_CONFIG);
    CHECK_EQ_I(rsc_parse_eeprom(NULL, RSC_CRC_STRICT, &c), ADB_ERR_PARAM);
}
static void test_compensation_and_zero(void)
{
    uint8_t ee[RSC_EEPROM_SIZE]; rsc_calib_t c; double p; build_ee(ee);
    rsc_parse_eeprom(ee, RSC_CRC_STRICT, &c);
    CHECK_EQ_I(rsc_compensate(&c, 1000000, 800, 0.0, &p), ADB_OK);   NEAR(p, 1476.4648925836539, 1e-6);
    CHECK_EQ_I(rsc_compensate(&c, -3000000, -800, 0.0, &p), ADB_OK); NEAR(p, -4477.392354854047, 1e-6);
    CHECK_EQ_I(rsc_compensate(&c, 0, 0, 0.0, &p), ADB_OK);           NEAR(p, 46.875, 1e-9);
    CHECK_EQ_I(rsc_compensate(&c, 3900000, 1600, 0.0, &p), ADB_OK);  NEAR(p, 5727.738744592875, 1e-6);
    double az = rsc_zero_correction(&c, 12345, 800);
    NEAR(az, 7790.078889620956, 1e-6);                               /* mid-scale target (Pmin = -60) */
    CHECK_EQ_I(rsc_compensate(&c, 12345, 800, az, &p), ADB_OK);      NEAR(p, 0.0, 1e-6);   /* zero pressure -> 0 Pa */
    rsc_calib_t bad = c; bad.span[0] = 0; bad.span[1] = 0;           /* degenerate span: must not divide by zero */
    CHECK_EQ_I(rsc_compensate(&bad, 1, 0, 0.0, &p), ADB_ERR_INVALID);
}
static void test_init(void)
{
    mock_t m; adb_bus_t e, a; rsc_t d; setup(&m, &e, &a);
    CHECK_EQ_I(rsc_init(&d, &e, &a, &CFG), ADB_OK);
    CHECK(d.crc_verified); CHECK(d.catalog_match);
    CHECK_EQ_I(m.n_reset, 1); CHECK_EQ_I(m.n_wreg_all, 1);
    CHECK_EQ_I(m.wreg[0], 0x11); CHECK_EQ_I(m.wreg[3], 0x44);        /* config registers copied from EEPROM */
    setup(&m, &e, &a); m.fail_ee = 1;
    CHECK_EQ_I(rsc_init(&d, &e, &a, &CFG), ADB_ERR_BUS);
    setup(&m, &e, &a); memset(m.ee, 0xFF, sizeof m.ee);               /* floating MISO */
    CHECK_EQ_I(rsc_init(&d, &e, &a, &CFG), ADB_ERR_CALIB);
    setup(&m, &e, &a); m.ee[200] ^= 1;
    rsc_config_t strict = CFG; strict.crc_policy = RSC_CRC_STRICT;
    CHECK_EQ_I(rsc_init(&d, &e, &a, &strict), ADB_ERR_CRC);
    CHECK_EQ_I(rsc_init(&d, &e, &a, &CFG), ADB_OK); CHECK(!d.crc_verified);
    setup(&m, &e, &a); rsc_config_t other = CFG; other.expected_catalog = "RSCDRRM020NDSE3";
    CHECK_EQ_I(rsc_init(&d, &e, &a, &other), ADB_OK); CHECK(!d.catalog_match);   /* wrong part: flagged */
    setup(&m, &e, &a); rsc_config_t bad = CFG; bad.rate = RSC_RATE_COUNT;
    CHECK_EQ_I(rsc_init(&d, &e, &a, &bad), ADB_ERR_PARAM);
    setup(&m, &e, &a); m.fail_next_adc_write = 1;
    CHECK_EQ_I(rsc_init(&d, &e, &a, &CFG), ADB_ERR_BUS);
}
static int run(mock_t *m, rsc_t *d, int want, int max_ms, adb_status_t *last)
{
    int got = 0;
    for (int ms = 0; ms < max_ms && got < want; ms++) {
        m->now_us += 1000; bool ns; adb_status_t s = rsc_poll(d, m->now_us, &ns);
        if (s != ADB_OK && s != ADB_ERR_NOT_READY) *last = s;
        if (ns) got++;
    }
    return got;
}
static void test_poll(void)
{
    mock_t m; adb_bus_t e, a; rsc_t d; adb_status_t last = ADB_OK; setup(&m, &e, &a);
    rsc_init(&d, &e, &a, &CFG);
    CHECK_EQ_I(run(&m, &d, 1, 200, &last), 1); CHECK_EQ_I(last, ADB_OK);
    NEAR(d.last.pressure_pa, 1476.4648925836539, 1e-6); NEAR(d.last.temp_c, 25.0, 1e-9);
    CHECK_EQ_I(m.n_early_read, 0);                                    /* never read before conversion done */
    CHECK_EQ_I(run(&m, &d, 8, 1000, &last), 8);
    CHECK_EQ_I(m.n_early_read, 0);
}
static void test_poll_faults(void)
{
    mock_t m; adb_bus_t e, a; rsc_t d; adb_status_t last = ADB_OK; bool ns; setup(&m, &e, &a);
    rsc_init(&d, &e, &a, &CFG);
    m.fail_next_adc_write = 1; m.now_us += 1000;
    CHECK_EQ_I(rsc_poll(&d, m.now_us, &ns), ADB_ERR_BUS);
    CHECK_EQ_I(run(&m, &d, 1, 200, &last), 1);                        /* recovers */
    m.fail_next_read = 1; last = ADB_OK; run(&m, &d, 1000, 40, &last);
    CHECK(d.bus_error_count >= 2); CHECK(run(&m, &d, 1, 200, &last) >= 1);
    m.saturate = 1; last = ADB_OK;                                    /* ADC full-scale code */
    CHECK_EQ_I(run(&m, &d, 1, 100, &last), 0); CHECK_EQ_I(last, ADB_ERR_RANGE);
    m.saturate = 0; m.p_raw = 0x7F0000;                               /* ~+120 mbar on a +/-60 mbar part -> out of range */
    last = ADB_OK; run(&m, &d, 1, 200, &last); CHECK_EQ_I(last, ADB_ERR_RANGE);
}
int main(void)
{
    RUN(test_crc_vectors); RUN(test_decoders); RUN(test_config_byte_and_times); RUN(test_parse_eeprom);
    RUN(test_compensation_and_zero); RUN(test_init); RUN(test_poll); RUN(test_poll_faults);
    TEST_SUMMARY(); return g_tests_failed ? 1 : 0;
}
