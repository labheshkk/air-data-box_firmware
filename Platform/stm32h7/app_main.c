/* app_main.c - the two entry points to call from the CubeMX-generated main.c (CM7 project):
 *
 *     // after MX_..._Init() for SPI, GPIO, TIM2, FDCAN:
 *     HAL_TIM_Base_Start(&htim2);  HAL_FDCAN_Start(&hfdcan1);
 *     adb_app_init();
 *     while (1) { adb_app_step(); HAL_IWDG_Refresh(&hiwdg);   // watchdog fed AFTER the step
 *     }
 *
 * Chip-select GPIOs must be configured as outputs and driven HIGH before adb_app_init(). */
#include <stddef.h>
#include <string.h>
#include "adb_platform.h"
#include "app.h"

static adb_spi_dev_t d_bmp, d_ms, d_rsc_adc, d_rsc_ee;
static app_t g_app;

adb_status_t adb_app_init(void)
{
    d_bmp     = (adb_spi_dev_t){ &ADB_SPI_HANDLE, CS_BMP581_PORT,  CS_BMP581_PIN,  SPI_POLARITY_LOW, SPI_PHASE_1EDGE };  /* mode 0 (datasheet: 0 or 3) */
    d_ms      = (adb_spi_dev_t){ &ADB_SPI_HANDLE, CS_MS5611_PORT,  CS_MS5611_PIN,  SPI_POLARITY_LOW, SPI_PHASE_1EDGE };  /* mode 0 */
    d_rsc_adc = (adb_spi_dev_t){ &ADB_SPI_HANDLE, CS_RSC_ADC_PORT, CS_RSC_ADC_PIN, SPI_POLARITY_LOW, SPI_PHASE_2EDGE };  /* mode 1 (ADC) */
    d_rsc_ee  = (adb_spi_dev_t){ &ADB_SPI_HANDLE, CS_RSC_EE_PORT,  CS_RSC_EE_PIN,  SPI_POLARITY_LOW, SPI_PHASE_1EDGE };  /* mode 0 (EEPROM) */

    app_hw_t hw;
    memset(&hw, 0, sizeof hw);
    adb_platform_spi_bus(&hw.bmp_bus, &d_bmp);
    adb_platform_spi_bus(&hw.ms_bus, &d_ms);
    adb_platform_spi_bus(&hw.rsc_adc_bus, &d_rsc_adc);
    adb_platform_spi_bus(&hw.rsc_ee_bus, &d_rsc_ee);
    hw.can_tx = adb_can_tx;
    hw.event = NULL;                                     /* hook a UART/ITM logger here: void (*)(void*, uint16_t code, int32_t arg) */
    app_cfg_t cfg = app_default_cfg();                   /* PROVISIONAL defaults: node 42, 50 Hz output, see docs */
    return app_init(&g_app, &cfg, &hw, adb_time_us());
}
void adb_app_step(void) { app_step(&g_app, adb_time_us()); }
