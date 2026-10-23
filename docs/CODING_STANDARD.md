# Coding rules used in this project

These are the rules the code is actually written and checked against. This project does **not** claim MISRA-C or
functional-safety compliance; it follows MISRA-style habits where practical.

1. **Error handling:** every fallible function returns `adb_status_t`; only `ADB_OK` means success. Outputs are always
   initialised, even on error. Callers must check the return value.
2. **No dynamic memory, no recursion, no unbounded loops.** All state lives in caller-owned structs.
3. **Hardware isolation:** driver and service code never includes STM32 headers; it talks to hardware only through `adb_bus_t`.
4. **Fixed-width integers** for registers and raw data; `int64_t` where the datasheet's intermediate width requires it.
5. **Blocking only in init.** Runtime paths are non-blocking; time is passed in (`now_us`) so they are testable.
6. **Every constant that comes from a datasheet carries a page/section reference** in a comment, and appears in `docs/datasheet_verification.md`.
7. **Provisional values** (thresholds, densities, rates) are named constants/config structs, never magic numbers.
8. **Warnings are errors:** `-Wall -Wextra -Wpedantic -Wconversion -Wshadow ... -Werror`, plus `gcc -fanalyzer`.
9. **Tests are part of the change:** each module ships with a host test covering nominal, boundary, invalid and failure/recovery cases,
   run under AddressSanitizer + UBSan.
10. **Run `scripts/check.sh` before every commit.**
