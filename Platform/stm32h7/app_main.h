/* app_main.h - entry points called from the CubeMX-generated main.c of the Cortex-M7 project (see docs/CUBEIDE_SETUP.md). */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "adb_status.h"

/* Call once, after MX_*_Init(), HAL_TIM_Base_Start(&htim2) and HAL_FDCAN_Start(&hfdcan1).
 * Returns ADB_ERR_CONFIG (and starts nothing) if TIM2 is not running at 1 MHz; check the result. A missing sensor is NOT an error here. */
adb_status_t adb_app_init(void);

/* Call from the main loop as often as possible (>= 1 kHz), then refresh the watchdog. */
void adb_app_step(void);

#endif
