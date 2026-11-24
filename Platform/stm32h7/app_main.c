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
#include "app_main.h"
#include "adb_platform.h"
#include "app.h"

static adb_spi_dev_t d_bmp, d_rsc_adc, d_rsc_ee;          /* BMP581 on SPI4; RSC on SPI2 (ADC and EEPROM, different chip selects) */
static adb_i2c_dev_t d_ms;                                 /* MS5611 on I2C4, address 0x77 */
static app_t g_app;
static bool  g_started;

adb_status_t adb_app_init(void)
{
    g_started = false;
    if (!adb_time_base_ok()) return ADB_ERR_CONFIG;      /* TIM2 is not running at 1 MHz: every sensor wait would be wrong - do not start */
    /* Wiring from the Kepstronic board document (13.11.2026). SPI modes from the datasheets: BMP581 mode 0 (0 or 3),
     * RSC EEPROM mode 0, RSC ADC mode 1. The RSC is alone on SPI2, so the mode is re-programmed only between the two. */
    d_bmp     = (adb_spi_dev_t){ &ADB_SPI_HANDLE,     CS_BMP581_PORT,  CS_BMP581_PIN,  SPI_POLARITY_LOW, SPI_PHASE_1EDGE };
    d_rsc_adc = (adb_spi_dev_t){ &ADB_SPI_RSC_HANDLE, CS_RSC_ADC_PORT, CS_RSC_ADC_PIN, SPI_POLARITY_LOW, SPI_PHASE_2EDGE };
    d_rsc_ee  = (adb_spi_dev_t){ &ADB_SPI_RSC_HANDLE, CS_RSC_EE_PORT,  CS_RSC_EE_PIN,  SPI_POLARITY_LOW, SPI_PHASE_1EDGE };
    d_ms      = (adb_i2c_dev_t){ &ADB_I2C_HANDLE, MS5611_I2C_ADDR, false, 0u };

    app_hw_t hw;
    memset(&hw, 0, sizeof hw);
    adb_platform_spi_bus(&hw.bmp_bus, &d_bmp);
    adb_platform_i2c_bus(&hw.ms_bus, &d_ms);
    adb_platform_spi_bus(&hw.rsc_adc_bus, &d_rsc_adc);
    adb_platform_spi_bus(&hw.rsc_ee_bus, &d_rsc_ee);
    hw.can_tx = adb_can_tx;
    hw.can_tx_free = adb_can_tx_free;                     /* RawAirData is skipped, never split, when the FIFO is short */
    hw.cold_boot = ADB_BOOT_IS_COLD();                  /* auto-zero only after power-on/brown-out, never after an in-flight reset */
    __HAL_RCC_CLEAR_RESET_FLAGS();
    hw.zero_load = NULL; hw.zero_store = NULL;          /* TODO(hardware engineer): persist the zero in flash/EEPROM with a CRC so a warm reboot reuses it */
    hw.event = NULL;                                     /* hook a UART/ITM logger here: void (*)(void*, uint16_t code, int32_t arg) */
    app_cfg_t cfg = app_default_cfg();                   /* PROVISIONAL defaults: node 42, 50 Hz output, see docs */
    adb_status_t s = app_init(&g_app, &cfg, &hw, adb_time_us());
    g_started = (s == ADB_OK);
    return s;
}
void adb_app_step(void)
{
    if (!g_started) return;                              /* init refused or failed: stay silent */
    uint32_t now = adb_time_us();
    adb_can_service(now);                               /* bus-off recovery runs even if the application is busy or degraded */
    app_step(&g_app, now);
}
