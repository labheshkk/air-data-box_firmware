/* test_common.h - minimal host-side test macros (no external dependencies). */
#ifndef TEST_COMMON_H
#define TEST_COMMON_H
#include <stdio.h>
#include <stdint.h>

static int g_tests_run = 0, g_tests_failed = 0;

#define CHECK(cond) do { g_tests_run++; if (!(cond)) { g_tests_failed++; \
    printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_EQ_I(a, b) do { g_tests_run++; long long _a = (long long)(a), _b = (long long)(b); \
    if (_a != _b) { g_tests_failed++; \
    printf("  FAIL %s:%d: %s == %s  (got %lld, expected %lld)\n", __FILE__, __LINE__, #a, #b, _a, _b); } } while (0)
#define RUN(fn) do { printf("[ RUN ] %s\n", #fn); fn(); } while (0)
#define TEST_SUMMARY() do { printf("\n%d checks, %d failed\n", g_tests_run, g_tests_failed); } while (0)

#endif
