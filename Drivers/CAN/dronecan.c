/* dronecan.c - see dronecan.h */
#include "dronecan.h"
#include <math.h>
#include <string.h>
#include <stddef.h>

enum { TID_STATUS = 0, TID_STATIC_P, TID_STATIC_T, TID_IAS };

adb_status_t dc_init(dc_node_t *n, uint8_t node_id)
{
    if (!n || node_id < 1u || node_id > 127u) return ADB_ERR_PARAM;   /* 0 = anonymous/unknown, not allowed here */
    memset(n, 0, sizeof *n); n->node_id = node_id; return ADB_OK;
}

uint16_t dc_f32_to_f16(float v)
{
    uint32_t x; memcpy(&x, &v, 4);
    uint32_t sign = (x >> 16) & 0x8000u, exp = (x >> 23) & 0xFFu, man = x & 0x7FFFFFu;
    if (exp == 0xFFu) return (uint16_t)(sign | 0x7C00u | (man ? 0x200u : 0u));          /* Inf / NaN (quiet) */
    int e = (int)exp - 127 + 15;
    if (e >= 31) return (uint16_t)(sign | 0x7C00u);                                     /* overflow -> Inf */
    if (e <= 0) {                                                                       /* subnormal or zero */
        if (e < -10) return (uint16_t)sign;                                             /* underflow to +/-0 */
        man |= 0x800000u;                                                               /* implicit leading 1 */
        uint32_t shift = (uint32_t)(14 - e), half = 1u << (shift - 1u);
        uint32_t m = man >> shift, rem = man & ((1u << shift) - 1u);
        if (rem > half || (rem == half && (m & 1u))) m++;                               /* round to nearest even */
        return (uint16_t)(sign | m);
    }
    uint32_t m = man >> 13, rem = man & 0x1FFFu;
    uint32_t h = ((uint32_t)e << 10) | m;
    if (rem > 0x1000u || (rem == 0x1000u && (h & 1u))) h++;                             /* may carry into exponent: correct */
    return (uint16_t)(sign | h);
}

float dc_f16_to_f32(uint16_t h)
{
    uint32_t sign = ((uint32_t)h & 0x8000u) << 16, exp = (h >> 10) & 0x1Fu, man = h & 0x3FFu, x;
    if (exp == 0u) {
        if (man == 0u) x = sign;
        else { int e = -1; do { e++; man <<= 1; } while (!(man & 0x400u));              /* normalise subnormal */
               x = sign | ((uint32_t)(127 - 15 - e) << 23) | ((man & 0x3FFu) << 13); }
    } else if (exp == 31u) x = sign | 0x7F800000u | (man << 13);
    else x = sign | ((exp - 15u + 127u) << 23) | (man << 13);
    float f; memcpy(&f, &x, 4); return f;
}

static uint32_t make_id(uint8_t prio, uint16_t dtid, uint8_t node) { return ((uint32_t)prio << 24) | ((uint32_t)dtid << 8) | node; }
static uint8_t  tail(dc_node_t *n, int which) { uint8_t t = (uint8_t)(0xC0u | (n->tid[which] & 0x1Fu)); n->tid[which] = (uint8_t)((n->tid[which] + 1u) & 0x1Fu); return t; }
static void put_f32(uint8_t *p, float f) { uint32_t u; memcpy(&u, &f, 4); p[0] = (uint8_t)u; p[1] = (uint8_t)(u >> 8); p[2] = (uint8_t)(u >> 16); p[3] = (uint8_t)(u >> 24); }
static void put_f16(uint8_t *p, float f) { uint16_t h = dc_f32_to_f16(f); p[0] = (uint8_t)h; p[1] = (uint8_t)(h >> 8); }

adb_status_t dc_encode_node_status(dc_node_t *n, uint32_t uptime_s, uint8_t health, dc_mode_t mode,
                                   uint16_t vendor_code, dc_frame_t *out)
{
    if (!n || !out || n->node_id == 0u || health > 3u || (unsigned)mode > 7u) return ADB_ERR_PARAM;
    memset(out, 0, sizeof *out);
    out->id = make_id(DC_PRIO_STATUS, (uint16_t)DC_DTID_NODE_STATUS, n->node_id);
    out->data[0] = (uint8_t)uptime_s; out->data[1] = (uint8_t)(uptime_s >> 8);              /* uint32 uptime_sec, LE */
    out->data[2] = (uint8_t)(uptime_s >> 16); out->data[3] = (uint8_t)(uptime_s >> 24);
    out->data[4] = (uint8_t)(((unsigned)health << 6) | ((unsigned)mode << 3));                        /* health[7:6] mode[5:3] sub_mode[2:0]=0 */
    out->data[5] = (uint8_t)vendor_code; out->data[6] = (uint8_t)(vendor_code >> 8);        /* uint16 vendor code, LE */
    out->data[7] = tail(n, TID_STATUS); out->len = 8;
    return ADB_OK;
}

adb_status_t dc_encode_static_pressure(dc_node_t *n, float p, float var, dc_frame_t *out)
{
    if (!n || !out || n->node_id == 0u || !isfinite(p) || !(var >= 0.0f)) return ADB_ERR_PARAM;   /* never publish NaN as data */
    memset(out, 0, sizeof *out);
    out->id = make_id(DC_PRIO_AIRDATA, (uint16_t)DC_DTID_STATIC_PRESSURE, n->node_id);
    put_f32(out->data, p);                      /* float32 static_pressure [Pa]            */
    put_f16(out->data + 4, var);                /* float16 static_pressure_variance [Pa^2] */
    out->data[6] = tail(n, TID_STATIC_P); out->len = 7;
    return ADB_OK;
}

adb_status_t dc_encode_static_temperature(dc_node_t *n, float k, float var, dc_frame_t *out)
{
    if (!n || !out || n->node_id == 0u || !isfinite(k) || !(var >= 0.0f)) return ADB_ERR_PARAM;
    memset(out, 0, sizeof *out);
    out->id = make_id(DC_PRIO_AIRDATA, (uint16_t)DC_DTID_STATIC_TEMPERATURE, n->node_id);
    put_f16(out->data, k); put_f16(out->data + 2, var);          /* float16 [K], float16 [K^2] */
    out->data[4] = tail(n, TID_STATIC_T); out->len = 5;
    return ADB_OK;
}

adb_status_t dc_encode_indicated_airspeed(dc_node_t *n, float v, float var, dc_frame_t *out)
{
    if (!n || !out || n->node_id == 0u || !isfinite(v) || !(var >= 0.0f)) return ADB_ERR_PARAM;
    memset(out, 0, sizeof *out);
    out->id = make_id(DC_PRIO_AIRDATA, (uint16_t)DC_DTID_INDICATED_AIRSPEED, n->node_id);
    put_f16(out->data, v); put_f16(out->data + 2, var);          /* float16 [m/s], float16 [(m/s)^2] */
    out->data[4] = tail(n, TID_IAS); out->len = 5;
    return ADB_OK;
}
