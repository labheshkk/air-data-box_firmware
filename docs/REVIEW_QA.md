# Review Q&A - questions an experienced embedded engineer is likely to ask

Answer from your own understanding; the pointers show where the evidence is.

1. **Why a cooperative loop and no RTOS?** Three sensors, ~100 frames/s, each driver is a non-blocking state machine; nothing needs preemption.
   Blocking happens only in init/re-init (<= ~8 ms). Fewer moving parts, no priority-inversion class of bugs. (`app.h` header)
2. **Why does the MS5611 driver wait the *maximum* conversion time?** Datasheet p.11: reading early returns 0 and corrupts the conversion; typical
   times differ by temperature/voltage. (`ms5611.c`, test `test_poll_waits_for_conversion`)
3. **Why int64 in the MS5611 maths?** dT*C6 reaches 2^41; D1*SENS 2^57 (datasheet p.8 lists 41-bit intermediates). 32-bit would overflow. UBSan test covers extremes.
4. **Why does the BMP581 driver have no calibration code?** The chip compensates internally; registers hold Q16 degC / Q6 Pa (datasheet 4.5).
5. **Why not average the two static sensors?** A frozen or biased sensor would silently shift altitude. We blend only when they agree, otherwise arbitrate by continuity
   and flag it. (`sensor_manager.h`, tests `test_disagreement_*`)
6. **What if both disagree and there is no history?** Output invalid; guessing is worse than saying nothing.
7. **Why is V = sqrt(2q/rho) only "indicated"?** Fixed sea-level density, incompressible flow. Error vs compressible CAS is 1 % at 99 m/s. TAS needs outside-air temperature.
8. **What about the RSC zero offset?** TEB is +/-90 Pa raw (about 12 m/s at standstill), +/-30 Pa after zero. Boot-time zero with sanity limits; wind at boot is detected and flagged.
9. **Why double precision for the RSC polynomial?** Cubic terms in float lose bits; Cortex-M7 has a double FPU; cost is negligible at < 100 Hz.
10. **Why is invalid data not published as zero?** A zero airspeed is a plausible value and would hide a fault. Silence + NodeStatus health lets the consumer detect it.
11. **Is this DroneCAN-compliant?** It emits spec-conformant single-frame messages (IDs/layouts verified from the spec) but is not a full node. No services, so tools may not
    show name/version. libcanard is the intended replacement. (`dronecan.h`)
12. **How do you know the tests are meaningful?** Mutation check: 19 injected bugs, all detected; two real bugs were found by the scenario tests while developing.
13. **What is NOT verified?** Everything that needs a real board: bus timing, RSC open items, CRC init, real noise, CAN electrical layer. (`TEST_PLAN.md` B)
14. **What would you do first with the hardware?** Test plan B steps 1-8, logic analyser first, before trusting any number.
