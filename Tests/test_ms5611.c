/* test_ms5611.c - host tests for the MS5611 driver.
 * Reference vectors:
 *  - datasheet example (p.8): C1..C6, D1=9085466, D2=8569150 -> TEMP=2007, P=100009
 *  - other vectors computed by an independent Python model using C-style
 *    (truncating) integer division, see docs/datasheet_verification.md.
 * NOTE: the CRC test is only a self-consistency test (no official CRC vector is
 * printed in the datasheet); verify against a real PROM on hardware. */
#include <string.h>
#include "ms5611.h"
#include "test_common.h"

static const uint16_t DS_C[8] = { 0x0000, 40127, 36924, 23317, 23282, 33464, 28312, 0x0000 };

typedef struct {
    uint16_t prom[8];
    uint32_t now_us;
    uint32_t d1_val, d2_val;       /* what the ADC returns after a finished conversion */
    uint32_t true_conv_us;         /* actual conversion duration of the mock chip      */
    int      pending;              /* 0 none, 1 = D1, 2 = D2 */
    uint32_t conv_start_us;
    int      fail_next_write, fail_next_read;
    int      n_reset, n_conv_d1, n_conv_d2, n_adc_read, n_early_adc_read;
    int      force_zero_adc;
} mock_t;

static adb_status_t m_write(void *c, const uint8_t *tx, uint16_t len)
{
    mock_t *m = c;
    if (m->fail_next_write) { m->fail_next_write = 0; return ADB_ERR_BUS; }
    if (len != 1) return ADB_ERR_PARAM;
    if (tx[0] == 0x1E) { m->n_reset++; return ADB_OK; }
    if (tx[0] >= 0x40 && tx[0] <= 0x48 && !(tx[0] & 0x10)) { m->n_conv_d1++; m->pending = 1; m->conv_start_us = m->now_us; return ADB_OK; }
    if (tx[0] >= 0x50 && tx[0] <= 0x58) { m->n_conv_d2++; m->pending = 2; m->conv_start_us = m->now_us; return ADB_OK; }
    return ADB_ERR_PARAM;
}
static adb_status_t m_write_read(void *c, const uint8_t *tx, uint16_t txlen, uint8_t *rx, uint16_t rxlen)
{
    mock_t *m = c;
    if (m->fail_next_read) { m->fail_next_read = 0; return ADB_ERR_BUS; }
    if (txlen != 1) return ADB_ERR_PARAM;
    if (tx[0] >= 0xA0 && tx[0] <= 0xAE && rxlen == 2) {
        uint16_t w = m->prom[(tx[0] - 0xA0) >> 1];
        rx[0] = (uint8_t)(w >> 8); rx[1] = (uint8_t)w; return ADB_OK;
    }
    if (tx[0] == 0x00 && rxlen == 3) {
        m->n_adc_read++;
        uint32_t v = 0;                                       /* datasheet p.11: early/repeat read gives 0 */
        if (m->pending && (uint32_t)(m->now_us - m->conv_start_us) >= m->true_conv_us)
            v = (m->pending == 1) ? m->d1_val : m->d2_val;
        else m->n_early_adc_read++;
        if (m->force_zero_adc) v = 0;
        m->pending = 0;
        rx[0] = (uint8_t)(v >> 16); rx[1] = (uint8_t)(v >> 8); rx[2] = (uint8_t)v;
        return ADB_OK;
    }
    return ADB_ERR_PARAM;
}
static void m_delay(void *c, uint32_t us) { ((mock_t *)c)->now_us += us; }

static void mock_init(mock_t *m)
{
    memset(m, 0, sizeof *m);
    memcpy(m->prom, DS_C, sizeof DS_C);
    m->prom[0] = 0x1234;
    m->prom[7] = ms5611_crc4(m->prom) & 0x0Fu;                /* self-consistent CRC nibble */
    m->d1_val = 9085466; m->d2_val = 8569150; m->true_conv_us = 8220;
}
static adb_bus_t bus_of(mock_t *m)
{
    return (adb_bus_t){ .ctx = m, .kind = ADB_BUS_SPI, .write = m_write, .write_read = m_write_read, .delay_us = m_delay };
}

static void test_datasheet_example(void)
{
    ms5611_sample_t s;
    CHECK_EQ_I(ms5611_compensate(DS_C, 9085466, 8569150, &s), ADB_OK);
    CHECK_EQ_I(s.temp_centi_c, 2007);          /* datasheet p.8: 20.07 degC   */
    CHECK_EQ_I(s.pressure_pa, 100009);         /* datasheet p.8: 1000.09 mbar */
}
static void test_second_order_cold(void)
{
    ms5611_sample_t s;
    CHECK_EQ_I(ms5611_compensate(DS_C, 9000000, 8000000, &s), ADB_OK);   /* -0.61 degC, 2nd order active */
    CHECK_EQ_I(s.temp_centi_c, -61);
    CHECK_EQ_I(s.pressure_pa, 94423);
    CHECK_EQ_I(ms5611_compensate(DS_C, 8400000, 8000000, &s), ADB_OK);
    CHECK_EQ_I(s.pressure_pa, 83434);
}
static void test_out_of_range_flagged(void)
{
    ms5611_sample_t s;                         /* temp -44.30 degC is outside -40..85 */
    CHECK_EQ_I(ms5611_compensate(DS_C, 6000000, 7000000, &s), ADB_ERR_RANGE);
    CHECK_EQ_I(s.temp_centi_c, -4430);         /* still returned for diagnostics */
    CHECK_EQ_I(s.pressure_pa, 35423);
    CHECK_EQ_I(ms5611_compensate(DS_C, 6000000, 6100000, &s), ADB_ERR_RANGE);   /* very-low-temp branch */
    CHECK_EQ_I(s.temp_centi_c, -9158);
    CHECK_EQ_I(s.pressure_pa, 26549);
}
static void test_invalid_inputs(void)
{
    ms5611_sample_t s;
    CHECK_EQ_I(ms5611_compensate(DS_C, 0, 8569150, &s), ADB_ERR_INVALID);
    CHECK_EQ_I(ms5611_compensate(DS_C, 9085466, 0, &s), ADB_ERR_INVALID);
    CHECK_EQ_I(ms5611_compensate(DS_C, 0x1000000u, 8569150, &s), ADB_ERR_INVALID);  /* > 24 bit */
    CHECK_EQ_I(ms5611_compensate(NULL, 1, 1, &s), ADB_ERR_PARAM);
    CHECK_EQ_I(ms5611_compensate(DS_C, 1, 1, NULL), ADB_ERR_PARAM);
}
static void test_extreme_no_overflow(void)
{
    ms5611_sample_t s;                         /* worst-case raw/coeff combos must not be UB (run under UBSan) */
    uint16_t p[8] = { 0, 65535, 65535, 65535, 65535, 0, 65535, 0 };
    (void)ms5611_compensate(p, 0xFFFFFF, 0xFFFFFF, &s);
    uint16_t q[8] = { 0, 65535, 65535, 65535, 65535, 65535, 65535, 0 };
    (void)ms5611_compensate(q, 0xFFFFFF, 1, &s);
    CHECK(1);
}
static void test_init_ok_and_failures(void)
{
    mock_t m; ms5611_t d; adb_bus_t b;
    mock_init(&m); b = bus_of(&m);
    CHECK_EQ_I(ms5611_init(&d, &b, MS5611_OSR_4096, 4), ADB_OK);
    CHECK_EQ_I(m.n_reset, 1);
    CHECK(m.now_us >= 2800);                                  /* waited >= 2.8 ms reload */

    mock_init(&m); b = bus_of(&m); m.prom[3] ^= 0x0100;       /* corrupt a coefficient -> CRC must fail */
    CHECK_EQ_I(ms5611_init(&d, &b, MS5611_OSR_4096, 4), ADB_ERR_CRC);

    mock_init(&m); b = bus_of(&m); for (int i = 0; i < 8; i++) m.prom[i] = 0xFFFF;  /* floating MISO */
    CHECK_EQ_I(ms5611_init(&d, &b, MS5611_OSR_4096, 4), ADB_ERR_CALIB);

    mock_init(&m); b = bus_of(&m); memset(m.prom, 0, sizeof m.prom);                 /* dead chip */
    CHECK_EQ_I(ms5611_init(&d, &b, MS5611_OSR_4096, 4), ADB_ERR_CALIB);

    mock_init(&m); b = bus_of(&m); m.fail_next_write = 1;     /* reset command fails */
    CHECK_EQ_I(ms5611_init(&d, &b, MS5611_OSR_4096, 4), ADB_ERR_BUS);

    mock_init(&m); b = bus_of(&m);
    CHECK_EQ_I(ms5611_init(&d, &b, (ms5611_osr_t)9, 4), ADB_ERR_PARAM);
    CHECK_EQ_I(ms5611_init(&d, &b, MS5611_OSR_256, 0), ADB_ERR_PARAM);
    CHECK_EQ_I(ms5611_init(NULL, &b, MS5611_OSR_256, 1), ADB_ERR_PARAM);
}
/* Run the state machine at 1 kHz until n samples are produced or timeout. */
static int run_poll(mock_t *m, ms5611_t *d, int want, int max_ms, adb_status_t *last)
{
    int got = 0;
    for (int ms = 0; ms < max_ms && got < want; ms++) {
        m->now_us += 1000;
        bool ns; adb_status_t s = ms5611_poll(d, m->now_us, &ns);
        if (s != ADB_OK) *last = s;
        if (ns) got++;
    }
    return got;
}
static void test_poll_normal_cycle(void)
{
    mock_t m; ms5611_t d; adb_bus_t b; adb_status_t last = ADB_OK;
    mock_init(&m); b = bus_of(&m);
    CHECK_EQ_I(ms5611_init(&d, &b, MS5611_OSR_4096, 4), ADB_OK);
    CHECK_EQ_I(run_poll(&m, &d, 1, 100, &last), 1);
    CHECK_EQ_I(last, ADB_OK);
    CHECK_EQ_I(d.last.pressure_pa, 100009);
    CHECK_EQ_I(d.last.temp_centi_c, 2007);
    CHECK_EQ_I(m.n_early_adc_read, 0);                        /* never read before conversion finished */
    /* 8 more samples: with temp_every_n=4 expect 1 + 2 temperature conversions overall */
    CHECK_EQ_I(run_poll(&m, &d, 8, 1000, &last), 8);
    CHECK_EQ_I(m.n_conv_d2, 3);
    CHECK_EQ_I(m.n_early_adc_read, 0);
}
static void test_poll_waits_for_conversion(void)
{
    mock_t m; ms5611_t d; adb_bus_t b; bool ns;
    mock_init(&m); b = bus_of(&m);
    ms5611_init(&d, &b, MS5611_OSR_4096, 1);
    m.now_us += 1000; CHECK_EQ_I(ms5611_poll(&d, m.now_us, &ns), ADB_OK);   /* start temp conversion */
    m.now_us += 5000; CHECK_EQ_I(ms5611_poll(&d, m.now_us, &ns), ADB_OK);   /* 5 ms < 9.04 ms max: must not read */
    CHECK_EQ_I(m.n_adc_read, 0);
    m.now_us += 4100; CHECK_EQ_I(ms5611_poll(&d, m.now_us, &ns), ADB_OK);   /* now >= 9.04 ms: read */
    CHECK_EQ_I(m.n_adc_read, 1);
}
static void test_poll_bus_error_recovers(void)
{
    mock_t m; ms5611_t d; adb_bus_t b; adb_status_t last = ADB_OK; bool ns;
    mock_init(&m); b = bus_of(&m); m.true_conv_us = 1880;      /* typical OSR-1024 conversion time (datasheet p.3) */
    ms5611_init(&d, &b, MS5611_OSR_1024, 2);
    m.fail_next_write = 1; m.now_us += 1000;
    CHECK_EQ_I(ms5611_poll(&d, m.now_us, &ns), ADB_ERR_BUS);
    CHECK_EQ_I(d.bus_error_count, 1);
    CHECK_EQ_I(run_poll(&m, &d, 1, 100, &last), 1);           /* recovers by itself */
    m.fail_next_read = 1;
    last = ADB_OK; run_poll(&m, &d, 0, 20, &last);
    CHECK(d.bus_error_count >= 1);
    CHECK(run_poll(&m, &d, 1, 100, &last) >= 1);
}
static void test_poll_zero_adc_is_invalid(void)
{
    mock_t m; ms5611_t d; adb_bus_t b; adb_status_t last = ADB_OK;
    mock_init(&m); b = bus_of(&m); m.force_zero_adc = 1;
    ms5611_init(&d, &b, MS5611_OSR_256, 1);
    CHECK_EQ_I(run_poll(&m, &d, 1, 50, &last), 0);
    CHECK_EQ_I(last, ADB_ERR_INVALID);
    CHECK(d.invalid_count >= 1);
    CHECK(!d.have_sample);
}
static void test_conv_times(void)
{
    CHECK_EQ_I(ms5611_conv_time_max_us(MS5611_OSR_256), 600);
    CHECK_EQ_I(ms5611_conv_time_max_us(MS5611_OSR_4096), 9040);
}

int main(void)
{
    RUN(test_datasheet_example); RUN(test_second_order_cold); RUN(test_out_of_range_flagged);
    RUN(test_invalid_inputs); RUN(test_extreme_no_overflow); RUN(test_init_ok_and_failures);
    RUN(test_poll_normal_cycle); RUN(test_poll_waits_for_conversion);
    RUN(test_poll_bus_error_recovers); RUN(test_poll_zero_adc_is_invalid); RUN(test_conv_times);
    TEST_SUMMARY();
    return g_tests_failed ? 1 : 0;
}
