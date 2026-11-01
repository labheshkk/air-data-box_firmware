# Changelog

## 0.2.0 - response to preliminary review
- FIX (critical): BMP581 data-ready never asserted - INT_SOURCE.drdy_data_reg_en is now enabled at init and read back.
- FIX: NodeStatus uptime no longer wraps with the 32-bit microsecond clock (64-bit accumulation).
- NEW: FDCAN link supervisor - bus-off recovery with back-off, stuck-TX flush.
- FIX: boot auto-zero only after a cold boot; optional persisted zero; acceptance limit 150 -> 100 Pa.
- NEW: RawAirData (multi-frame, 20 Hz) with TX-space check; IAS/RawAirData selectable.
- FIX: CI runs `bash scripts/check.sh` so it works even when the executable bit is lost (e.g. commits made from Windows).
- NEW: docs/REVIEW_RESPONSE.md; test suites 8 -> 9; mutation check 19 -> 26 mutants.

## 0.1.0 - first delivery
- Drivers (MS5611, BMP581, RSC), filters, sensor manager, air-data maths, DroneCAN single-frame encoder, application, STM32H7 platform layer, docs, host tests.
