/* test_dronecan.c - vectors generated independently with Python (struct 'e'/'f' formats) plus the CAN-ID and
 * tail-byte rules of the DroneCAN spec (4.1). A small spec-based decoder round-trips every encoder. */
#include <math.h>
#include <string.h>
#include "dronecan.h"
#include "test_common.h"

static void test_f16_vectors(void)
{
    static const struct { float v; uint16_t h; } t[] = {
        { 0.0f, 0x0000 }, { -0.0f, 0x8000 }, { 1.0f, 0x3C00 }, { 0.5f, 0x3800 }, { -2.0f, 0xC000 }, { 0.25f, 0x3400 },
        { 65504.0f, 0x7BFF }, { 65520.0f, 0x7C00 }, { 70000.0f, 0x7C00 }, { 6.1035e-05f, 0x0400 }, { 5.96e-08f, 0x0001 },
        { 1e-9f, 0x0000 }, { 0.1f, 0x2E66 }, { 98.97433f, 0x5630 }, { 288.15f, 0x5C81 }, { 31.298432f, 0x4FD3 },
        { 1.0009765625f, 0x3C01 }, { 1.00048828125f, 0x3C00 /* tie -> even */ }, { INFINITY, 0x7C00 }, { -INFINITY, 0xFC00 } };
    for (unsigned i = 0; i < sizeof t / sizeof t[0]; i++) CHECK_EQ_I(dc_f32_to_f16(t[i].v), t[i].h);
    uint16_t nan16 = dc_f32_to_f16(NAN);
    CHECK((nan16 & 0x7C00) == 0x7C00 && (nan16 & 0x03FF) != 0);          /* NaN stays NaN */
    CHECK(dc_f16_to_f32(0x3C00) == 1.0f); CHECK(dc_f16_to_f32(0xC000) == -2.0f); CHECK(dc_f16_to_f32(0x0001) > 5.9e-8f);
    CHECK(isinf(dc_f16_to_f32(0x7C00))); CHECK(isnan(dc_f16_to_f32(0x7E00)));
    for (uint32_t h = 0; h < 0x7C00; h += 7) CHECK_EQ_I(dc_f32_to_f16(dc_f16_to_f32((uint16_t)h)), h);   /* exact round-trip */
}

/* spec-based header decoder */
typedef struct { unsigned prio, dtid, svc, node, tail_sot, tail_eot, tail_toggle, tid; } hdr_t;
static hdr_t dec(const dc_frame_t *f)
{
    hdr_t h; h.prio = (f->id >> 24) & 0x1F; h.dtid = (f->id >> 8) & 0xFFFF; h.svc = (f->id >> 7) & 1; h.node = f->id & 0x7F;
    uint8_t t = f->data[f->len - 1]; h.tail_sot = t >> 7; h.tail_eot = (t >> 6) & 1; h.tail_toggle = (t >> 5) & 1; h.tid = t & 0x1F;
    return h;
}

static void test_node_status(void)
{
    dc_node_t n; dc_frame_t f; CHECK_EQ_I(dc_init(&n, 42), ADB_OK);
    CHECK_EQ_I(dc_encode_node_status(&n, 1, 0, DC_MODE_OPERATIONAL, 0x1234, &f), ADB_OK);
    CHECK_EQ_I(f.id, 0x1801552A); CHECK_EQ_I(f.len, 8);
    static const uint8_t e[8] = { 1, 0, 0, 0, 0x00, 0x34, 0x12, 0xC0 };
    CHECK(memcmp(f.data, e, 8) == 0);
    hdr_t h = dec(&f); CHECK_EQ_I(h.dtid, 341); CHECK_EQ_I(h.node, 42); CHECK_EQ_I(h.svc, 0);
    CHECK_EQ_I(h.tail_sot, 1); CHECK_EQ_I(h.tail_eot, 1); CHECK_EQ_I(h.tail_toggle, 0);
    CHECK_EQ_I(dc_encode_node_status(&n, 0x01020304, 2, DC_MODE_INITIALIZATION, 0, &f), ADB_OK);
    CHECK_EQ_I(f.data[0], 4); CHECK_EQ_I(f.data[3], 1); CHECK_EQ_I(f.data[4], (2 << 6) | (1 << 3)); CHECK_EQ_I(f.data[7], 0xC1);
    CHECK_EQ_I(dc_encode_node_status(&n, 0, 4, DC_MODE_OPERATIONAL, 0, &f), ADB_ERR_PARAM);   /* health is 2 bits */
}
static void test_static_pressure(void)
{
    dc_node_t n; dc_frame_t f; dc_init(&n, 42);
    CHECK_EQ_I(dc_encode_static_pressure(&n, 101325.0f, 0.25f, &f), ADB_OK);
    CHECK_EQ_I(f.id, 0x1004042A); CHECK_EQ_I(f.len, 7);
    static const uint8_t e[7] = { 0x80, 0xE6, 0xC5, 0x47, 0x00, 0x34, 0xC0 };     /* float32 LE, float16 LE, tail */
    CHECK(memcmp(f.data, e, 7) == 0);
    CHECK_EQ_I(dc_encode_static_pressure(&n, 90000.5f, 0.1f, &f), ADB_OK);
    static const uint8_t e2[7] = { 0x40, 0xC8, 0xAF, 0x47, 0x66, 0x2E, 0xC1 };
    CHECK(memcmp(f.data, e2, 7) == 0);
    CHECK_EQ_I(dc_encode_static_pressure(&n, NAN, 0.1f, &f), ADB_ERR_PARAM);         /* never serialise NaN as a measurement */
    CHECK_EQ_I(dc_encode_static_pressure(&n, 1.0f, -1.0f, &f), ADB_ERR_PARAM);
    CHECK_EQ_I(dc_encode_static_pressure(&n, 1.0f, 0.1f, NULL), ADB_ERR_PARAM);
}
static void test_temperature_and_ias(void)
{
    dc_node_t n; dc_frame_t f; dc_init(&n, 42);
    CHECK_EQ_I(dc_encode_static_temperature(&n, 288.15f, 0.25f, &f), ADB_OK);
    CHECK_EQ_I(f.id, 0x1004052A); CHECK_EQ_I(f.len, 5);
    static const uint8_t t[5] = { 0x81, 0x5C, 0x00, 0x34, 0xC0 }; CHECK(memcmp(f.data, t, 5) == 0);
    CHECK_EQ_I(dc_encode_indicated_airspeed(&n, 31.298432f, 1.0f, &f), ADB_OK);
    CHECK_EQ_I(f.id, 0x1003FD2A); CHECK_EQ_I(f.len, 5);
    static const uint8_t a[5] = { 0xD3, 0x4F, 0x00, 0x3C, 0xC0 }; CHECK(memcmp(f.data, a, 5) == 0);
    CHECK_EQ_I(dec(&f).dtid, 1021);                                                 /* IAS = 1021, TAS = 1020 (spec) */
    CHECK_EQ_I(dc_encode_indicated_airspeed(&n, INFINITY, 1.0f, &f), ADB_ERR_PARAM);
}
static void test_transfer_id_per_type_and_wrap(void)
{
    dc_node_t n; dc_frame_t f; dc_init(&n, 7);
    for (int i = 0; i < 40; i++) {
        dc_encode_static_pressure(&n, 90000.0f, 1.0f, &f);
        CHECK_EQ_I(dec(&f).tid, i % 32);                                            /* 5-bit transfer ID wraps 31 -> 0 */
    }
    dc_encode_static_temperature(&n, 288.0f, 1.0f, &f); CHECK_EQ_I(dec(&f).tid, 0);  /* independent counter per data type */
    dc_encode_indicated_airspeed(&n, 10.0f, 1.0f, &f);  CHECK_EQ_I(dec(&f).tid, 0);
    dc_encode_static_temperature(&n, 288.0f, 1.0f, &f); CHECK_EQ_I(dec(&f).tid, 1);
}
static void test_node_id_validation(void)
{
    dc_node_t n;
    CHECK_EQ_I(dc_init(&n, 0), ADB_ERR_PARAM); CHECK_EQ_I(dc_init(&n, 128), ADB_ERR_PARAM);
    CHECK_EQ_I(dc_init(&n, 1), ADB_OK); CHECK_EQ_I(dc_init(&n, 127), ADB_OK); CHECK_EQ_I(dc_init(NULL, 5), ADB_ERR_PARAM);
}
int main(void)
{
    RUN(test_f16_vectors); RUN(test_node_status); RUN(test_static_pressure); RUN(test_temperature_and_ias);
    RUN(test_transfer_id_per_type_and_wrap); RUN(test_node_id_validation);
    TEST_SUMMARY(); return g_tests_failed ? 1 : 0;
}
