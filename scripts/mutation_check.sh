#!/bin/sh
# Mutation sanity check: apply one deliberate bug at a time and require the test-suite to FAIL.
# A surviving mutant means a test gap. Usage: ./scripts/mutation_check.sh
cd "$(dirname "$0")/.."
KILLED=0; SURVIVED=0; INVALID=0
mutate() {   # file, sed-expression, description
    cp "$1" "$1.orig"
    sed -i "$2" "$1"
    if cmp -s "$1" "$1.orig"; then echo "  [skip ] pattern not found: $3"; mv "$1.orig" "$1"; return; fi
    if ! make -C Tests clean all >/dev/null 2>&1; then echo "  [invalid] mutant does not compile (not counted): $3"; INVALID=$((INVALID+1));
    elif make -C Tests test >/dev/null 2>&1; then echo "  [SURVIVED] $3"; SURVIVED=$((SURVIVED+1));
    else echo "  [killed ] $3"; KILLED=$((KILLED+1)); fi
    mv "$1.orig" "$1"
}
mutate Drivers/MS5611/ms5611.c 's/(5 \* dt20 \* dt20) \/ 2;/(5 * dt20 * dt20) \/ 4;/'          "MS5611 second-order OFF2 coefficient wrong"
mutate Drivers/MS5611/ms5611.c 's/< ms5611_conv_time_max_us(d->osr))/< 100u)/'                  "MS5611 reads ADC before conversion finishes"
mutate Drivers/MS5611/ms5611.c 's/int64_t p = (((int64_t)d1 \* sens) \/ 2097152 - off) \/ 32768;/int64_t p = (((int64_t)d1 * sens) \/ 2097152 + off) \/ 32768;/' "MS5611 sign error in pressure"
mutate Drivers/BMP581/bmp581.c 's/if (!(v \& 0x80u)) return ADB_ERR_CONFIG;/if (0) return ADB_ERR_CONFIG;/' "BMP581 ignores invalid ODR/OSR flag"
mutate Drivers/BMP581/bmp581.c 's/#define SPI_READ_FLAG  0x80u/#define SPI_READ_FLAG  0x00u/' "BMP581 SPI read flag missing"
mutate Drivers/BMP581/bmp581.c 's/(v \& 0x800000u) ? (v | 0xFF000000u) : v/v/'                  "BMP581 no sign extension"
mutate Drivers/RSC/rsc.c 's/(temperature ? 0x02u : 0x00u)/(temperature ? 0x01u : 0x00u)/'     "RSC temperature-select bit wrong"
mutate Drivers/RSC/rsc.c 's/return (v \& 0x2000) ? v - 0x4000 : v;/return v;/'                "RSC temperature sign ignored"
mutate Drivers/RSC/rsc.c 's/double pint2 = pint1 \/ den;/double pint2 = pint1 * den;/'         "RSC divides span incorrectly"
mutate Services/AirData/airdata.c 's/if (dp < -c->dp_neg_tol_pa) return ADB_ERR_RANGE;/if (0) return ADB_ERR_RANGE;/' "airdata accepts reversed tubes"
mutate Services/AirData/airdata.c 's/ISA_EXP        0.190263f/ISA_EXP        0.19f/'          "airdata altitude exponent wrong"
mutate Services/SensorManager/sensor_manager.c 's/sm->arb = (fabsf(A->value - sm->last_fused) <= fabsf(B->value - sm->last_fused)) ? ARB_A : ARB_B;/sm->arb = (fabsf(A->value - sm->last_fused) >= fabsf(B->value - sm->last_fused)) ? ARB_A : ARB_B;/' "fusion picks the WRONG sensor on disagreement"
mutate Services/SensorManager/sensor_manager.c 's/if (c->consec_err >= sm->cfg.fail_after_errors)/if (c->consec_err >= sm->cfg.fail_after_errors + 60000)/' "sensor never declared failed"
mutate Services/SensorManager/sensor_manager.c 's/if (fabs(mean) > (double)sm->cfg.zero_max_abs_pa) sm->zero_state = SM_ZERO_FAIL_OFFSET;/if (0) sm->zero_state = SM_ZERO_FAIL_OFFSET;/' "zeroing accepts huge offset"
mutate Services/Filters/filters.c 's/float a = dt_s \/ (f->tau_s + dt_s);/float a = dt_s \/ (f->tau_s * dt_s);/' "low-pass alpha wrong"
mutate Drivers/CAN/dronecan.c 's/0xC0u | (n->tid\[which\] \& 0x1Fu)/0x80u | (n->tid[which] \& 0x1Fu)/' "CAN tail byte EOT bit missing"
mutate Drivers/CAN/dronecan.c 's/DC_DTID_INDICATED_AIRSPEED/DC_DTID_STATIC_PRESSURE/'        "IAS published under wrong data type ID (no-op if macro only in header)"
mutate Application/app.c 's/if (o->dp_valid) {/if (0) {/'                                       "airspeed never published"
mutate Application/app.c 's/a->zero_started_us = (now != 0u) ? now : 1u;/a->zero_started_us = a->zero_started_us ? a->zero_started_us : now;/' "zero timeout not reset per attempt (regression of the bug found earlier)"
echo "mutants killed by a failing test: $KILLED, survived: $SURVIVED, invalid (did not compile): $INVALID"
[ "$SURVIVED" -eq 0 ]
