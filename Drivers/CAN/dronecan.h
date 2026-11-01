/* dronecan.h - minimal DroneCAN (UAVCAN v0, CAN 2.0B) single-frame message encoder.
 *
 * SCOPE AND HONESTY: this module implements the wire format for the four SINGLE-FRAME broadcast messages the
 * air-data box needs. It is written from the DroneCAN specification (4.1 CAN bus transport layer, 7. list of
 * standard data types) and is host-tested against independently generated vectors. It is NOT libcanard:
 *   - no reception, no multi-frame transfers (so no CRC / data-type signatures), no services
 *     (uavcan.protocol.GetNodeInfo, parameter service, dynamic node-ID allocation are NOT implemented)
 *   - therefore the node is visible on the bus (NodeStatus) and streams data, but is not configurable.
 * For a full node, replace this layer with libcanard + dronecan_dsdlc; the call sites in Application/ stay the same.
 *
 * Verified against the spec (dronecan.github.io, fetched 2026):
 *   NodeStatus 341, StaticPressure 1028, StaticTemperature 1029, IndicatedAirspeed 1021 (TrueAirspeed is 1020).
 *   CAN ID = priority(5) | message type ID(16) | service-not-message(1)=0 | source node ID(7).
 *   Tail byte = SOT(1) EOT(1) toggle(1) transfer-ID(5); single frame: 0xC0 | tid. Transfer ID per data type, mod 32. */
#ifndef DRONECAN_H
#define DRONECAN_H

#include <stdint.h>
#include <stdbool.h>
#include "adb_status.h"

#define DC_DTID_NODE_STATUS          341u
#define DC_DTID_INDICATED_AIRSPEED   1021u
#define DC_DTID_RAW_AIR_DATA         1027u
#define DC_DTID_STATIC_PRESSURE      1028u
#define DC_DTID_STATIC_TEMPERATURE   1029u

#define DC_PRIO_AIRDATA   16u            /* PROVISIONAL (libcanard "medium") */
#define DC_PRIO_STATUS    24u            /* PROVISIONAL (libcanard "low")    */

typedef struct { uint32_t id; uint8_t len; uint8_t data[8]; } dc_frame_t;   /* extended 29-bit ID, classic CAN */

#define DC_SIG_RAW_AIR_DATA          0xC77DF38BA122F5DAull   /* data type signature (libcanard show_data_type_info.py output, quoted publicly) */
#define DC_RAW_AIR_DATA_FRAMES       3u                      /* 17-byte payload + 2-byte CRC = 19 bytes -> 7 + 7 + 5 */

typedef struct { uint8_t node_id; uint8_t tid[5]; } dc_node_t;

typedef enum { DC_MODE_OPERATIONAL = 0, DC_MODE_INITIALIZATION = 1, DC_MODE_MAINTENANCE = 2 } dc_mode_t;

adb_status_t dc_init(dc_node_t *n, uint8_t node_id);                 /* node_id 1..127 */

uint16_t dc_f32_to_f16(float v);                                     /* IEEE-754 binary16, round-to-nearest-even */
float    dc_f16_to_f32(uint16_t h);

adb_status_t dc_encode_node_status(dc_node_t *n, uint32_t uptime_s, uint8_t health, dc_mode_t mode,
                                  uint16_t vendor_code, dc_frame_t *out);
adb_status_t dc_encode_static_pressure(dc_node_t *n, float pressure_pa, float variance_pa2, dc_frame_t *out);
adb_status_t dc_encode_static_temperature(dc_node_t *n, float temperature_k, float variance_k2, dc_frame_t *out);
adb_status_t dc_encode_indicated_airspeed(dc_node_t *n, float ias_ms, float variance_ms2, dc_frame_t *out);

/* uavcan.equipment.air_data.RawAirData (1027) - MULTI-FRAME (3 frames, all share one CAN ID and transfer ID).
 * Fields (spec 7.): flags u8; static_pressure f32 [Pa]; differential_pressure f32 [Pa]; four float16 temperatures [K]
 * (static sensor, differential sensor, static air, pitot); covariance omitted (empty tail array = valid).
 * Unused temperatures must be NaN (spec). Pressures must be finite. The caller must queue all three frames
 * back-to-back and in order (spec 4.1: identical IDs must be transmitted in queue order). */
adb_status_t dc_encode_raw_air_data(dc_node_t *n, uint8_t flags, float static_pa, float diff_pa,
                                    float static_sensor_k, float diff_sensor_k, float static_air_k, float pitot_k,
                                    dc_frame_t out[DC_RAW_AIR_DATA_FRAMES]);

#endif
