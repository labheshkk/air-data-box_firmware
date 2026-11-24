/* test_platform.c - host tests for Platform/stm32h7/adb_platform.c using a RECORDING FAKE of the STM32 HAL.
 *
 * WHAT THIS PROVES: our glue logic - chip-select framing, SPI mode switching, I2C address shifting and
 * write/stop/read structure, stuck-bus recovery, CAN header construction, bus-off/stuck-TX restart wiring - does
 * what is intended, given HAL functions that behave as documented.
 * WHAT IT CANNOT PROVE: that the real HAL behaves as the fake does. The fake is written from the HAL documentation
 * (by the same author as the code), so it must be confirmed on the board. */
#include <string.h>
#include "adb_platform.h"
#include "test_common.h"

/* ------------------------------------------------------------------ fake HAL state */
static GPIO_TypeDef g_a, g_b, g_d, g_e, g_f;
GPIO_TypeDef *GPIOA = &g_a, *GPIOB = &g_b, *GPIOD = &g_d, *GPIOE = &g_e, *GPIOF = &g_f;
SPI_HandleTypeDef hspi2, hspi4;
I2C_HandleTypeDef hi2c4;
TIM_HandleTypeDef htim2;
FDCAN_HandleTypeDef hfdcan1;

enum { EV_CS = 1, EV_SPI_INIT, EV_XFER, EV_I2C_TX, EV_I2C_RX, EV_I2C_DEINIT, EV_I2C_INIT, EV_GPIO_INIT, EV_SCL_RISE, EV_CAN_ADD, EV_CAN_STOP, EV_CAN_START };
typedef struct { int type; uint32_t a, b; uint8_t d[24]; } ev_t;
static ev_t LOG[512]; static int NL;
static void logev(int t, uint32_t a, uint32_t b, const uint8_t *d, uint32_t n)
{ if (NL < 512) { LOG[NL].type = t; LOG[NL].a = a; LOG[NL].b = b; memset(LOG[NL].d, 0, 24); if (d) memcpy(LOG[NL].d, d, n > 24 ? 24 : n); NL++; } }
static int count(int t) { int n = 0; for (int i = 0; i < NL; i++) if (LOG[i].type == t) n++; return n; }
static const ev_t *nth(int t, int k) { for (int i = 0; i < NL; i++) if (LOG[i].type == t && k-- == 0) return &LOG[i]; return NULL; }

static HAL_StatusTypeDef spi_rc, i2c_rc;
static int i2c_fail_next, sda_stuck;
static GPIO_PinState last_scl = GPIO_PIN_SET;
static uint32_t TIME_US; static uint32_t tim_scale_pct = 100u; static int systick_stalled;
static int can_bus_off; static uint32_t can_free = 8; static HAL_StatusTypeDef can_rc;
static FDCAN_TxHeaderTypeDef can_hdr; static uint8_t can_data[8];

static int hid(const SPI_HandleTypeDef *h) { return h == &hspi2 ? 2 : h == &hspi4 ? 4 : 0; }

uint32_t stub_tim_counter(TIM_HandleTypeDef *h) { (void)h; TIME_US += 1u; return (uint32_t)(((uint64_t)TIME_US * tim_scale_pct) / 100u); }   /* each read advances 1 us */
uint32_t HAL_GetTick(void) { if (!systick_stalled) TIME_US += 100u; return TIME_US / 1000u; }          /* SysTick: 1 ms ticks of the 'real' time */
uint32_t stub_rcc_flag(uint32_t f) { (void)f; return 1u; }
void stub_rcc_clear(void) { }

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h) { logev(EV_SPI_INIT, (uint32_t)hid(h), 0, NULL, 0); return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx, uint16_t n, uint32_t to)
{
    (void)to; logev(EV_XFER, (uint32_t)hid(h), n, tx, n);
    for (uint16_t i = 0; i < n; i++) rx[i] = (uint8_t)(0xA0u + i);                       /* recognisable response pattern */
    return spi_rc;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s)
{
    if (p == GPIOD && pin == GPIO_PIN_12) {                                               /* SCL line (recovery) */
        if (last_scl == GPIO_PIN_RESET && s == GPIO_PIN_SET) { logev(EV_SCL_RISE, 0, 0, NULL, 0); if (sda_stuck > 0) sda_stuck--; }
        last_scl = s; return;
    }
    logev(EV_CS, pin, (uint32_t)s, NULL, 0);
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin) { (void)p; (void)pin; return sda_stuck > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET; }
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g) { (void)p; logev(EV_GPIO_INIT, g->Pin, g->Mode, NULL, 0); }
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *h) { (void)h; logev(EV_I2C_INIT, 0, 0, NULL, 0); return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *h) { (void)h; logev(EV_I2C_DEINIT, 0, 0, NULL, 0); return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *h, uint16_t a, uint8_t *d, uint16_t n, uint32_t to)
{
    (void)h; (void)to; logev(EV_I2C_TX, a, n, d, n);
    if (i2c_fail_next > 0) { i2c_fail_next--; return i2c_rc; }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *h, uint16_t a, uint8_t *d, uint16_t n, uint32_t to)
{
    (void)h; (void)to; logev(EV_I2C_RX, a, n, NULL, 0);
    for (uint16_t i = 0; i < n; i++) d[i] = (uint8_t)(0xB0u + i);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h, FDCAN_TxHeaderTypeDef *hd, uint8_t *d)
{ (void)h; can_hdr = *hd; memcpy(can_data, d, 8); logev(EV_CAN_ADD, hd->Identifier, 0, NULL, 0); return can_rc; }
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *h, FDCAN_ProtocolStatusTypeDef *ps)
{ (void)h; memset(ps, 0, sizeof *ps); ps->BusOff = (uint32_t)can_bus_off; return HAL_OK; }
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *h) { (void)h; return can_free; }
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *h) { (void)h; logev(EV_CAN_STOP, 0, 0, NULL, 0); return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h) { (void)h; logev(EV_CAN_START, 0, 0, NULL, 0); can_bus_off = 0; return HAL_OK; }

static void reset_fake(void)
{
    NL = 0; spi_rc = HAL_OK; i2c_rc = HAL_ERROR; i2c_fail_next = 0; sda_stuck = 0; last_scl = GPIO_PIN_SET;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW; hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi4.Init.CLKPolarity = SPI_POLARITY_LOW; hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
}

/* ------------------------------------------------------------------ SPI */
static adb_spi_dev_t bmp = { &hspi4, NULL, GPIO_PIN_11, SPI_POLARITY_LOW, SPI_PHASE_1EDGE };
static adb_spi_dev_t rsc_adc = { &hspi2, NULL, GPIO_PIN_12, SPI_POLARITY_LOW, SPI_PHASE_2EDGE };
static adb_spi_dev_t rsc_ee = { &hspi2, NULL, GPIO_PIN_11, SPI_POLARITY_LOW, SPI_PHASE_1EDGE };

static void test_spi_write_framing(void)
{
    reset_fake(); bmp.cs_port = GPIOE; adb_bus_t b; adb_platform_spi_bus(&b, &bmp);
    const uint8_t tx[2] = { 0x7E, 0xB6 };
    CHECK_EQ_I(b.write(b.ctx, tx, 2), ADB_OK);
    CHECK_EQ_I(NL, 3);                                                                    /* CS low, transfer, CS high - nothing else */
    CHECK_EQ_I(LOG[0].type, EV_CS); CHECK_EQ_I(LOG[0].b, 0);
    CHECK_EQ_I(LOG[1].type, EV_XFER); CHECK_EQ_I(LOG[1].a, 4); CHECK_EQ_I(LOG[1].b, 2); CHECK_EQ_I(LOG[1].d[0], 0x7E); CHECK_EQ_I(LOG[1].d[1], 0xB6);
    CHECK_EQ_I(LOG[2].type, EV_CS); CHECK_EQ_I(LOG[2].b, 1);
    CHECK_EQ_I(count(EV_SPI_INIT), 0);                                                    /* same mode: no re-init */
}
static void test_spi_write_read_is_one_cs_frame(void)
{
    reset_fake(); bmp.cs_port = GPIOE; adb_bus_t b; adb_platform_spi_bus(&b, &bmp);
    const uint8_t tx[1] = { 0x81 }; uint8_t rx[3] = { 0 };
    CHECK_EQ_I(b.write_read(b.ctx, tx, 1, rx, 3), ADB_OK);
    CHECK_EQ_I(count(EV_CS), 2); CHECK_EQ_I(count(EV_XFER), 1);                           /* ONE chip-select frame for command + data */
    const ev_t *x = nth(EV_XFER, 0);
    CHECK_EQ_I(x->b, 4); CHECK_EQ_I(x->d[0], 0x81); CHECK_EQ_I(x->d[1], 0); CHECK_EQ_I(x->d[3], 0);   /* filler is 0x00 */
    CHECK_EQ_I(rx[0], 0xA1); CHECK_EQ_I(rx[1], 0xA2); CHECK_EQ_I(rx[2], 0xA3);            /* first txlen received bytes are discarded */
}
static void test_spi_read_starts_at_first_clock(void)
{
    reset_fake(); rsc_adc.cs_port = GPIOB; adb_bus_t b; adb_platform_spi_bus(&b, &rsc_adc); uint8_t rx[3];
    CHECK_EQ_I(b.read(b.ctx, rx, 3), ADB_OK);
    CHECK_EQ_I(rx[0], 0xA0); CHECK_EQ_I(rx[1], 0xA1); CHECK_EQ_I(rx[2], 0xA2);            /* RSC data begins at the FIRST clock */
    CHECK_EQ_I(nth(EV_XFER, 0)->d[0], 0x00);
}
static void test_spi_cs_released_on_error(void)
{
    reset_fake(); bmp.cs_port = GPIOE; adb_bus_t b; adb_platform_spi_bus(&b, &bmp); const uint8_t tx[2] = { 1, 2 };
    spi_rc = HAL_TIMEOUT; CHECK_EQ_I(b.write(b.ctx, tx, 2), ADB_ERR_TIMEOUT);
    CHECK_EQ_I(LOG[NL - 1].type, EV_CS); CHECK_EQ_I(LOG[NL - 1].b, 1);                    /* CS high again, bus not left selected */
    spi_rc = HAL_ERROR; CHECK_EQ_I(b.write(b.ctx, tx, 2), ADB_ERR_BUS);
    CHECK_EQ_I(LOG[NL - 1].b, 1);
}
static void test_spi_parameter_limits(void)
{
    reset_fake(); bmp.cs_port = GPIOE; adb_bus_t b; adb_platform_spi_bus(&b, &bmp); uint8_t big[600] = { 0 }; uint8_t rx[600];
    CHECK_EQ_I(b.write(b.ctx, big, 17), ADB_ERR_PARAM);
    CHECK_EQ_I(b.write(b.ctx, NULL, 2), ADB_ERR_PARAM);
    CHECK_EQ_I(b.write_read(b.ctx, big, 2, rx, 480), ADB_ERR_PARAM);                      /* 482 > frame buffer */
    CHECK_EQ_I(b.write_read(b.ctx, big, 2, rx, 452), ADB_OK);                             /* RSC EEPROM image (2 + 452) fits */
    CHECK_EQ_I(b.read(b.ctx, rx, 0), ADB_ERR_PARAM);
    CHECK_EQ_I(count(EV_XFER), 1);                                                        /* rejected calls never touched the bus */
}
static void test_rsc_spi_mode_switching(void)
{
    reset_fake(); rsc_adc.cs_port = GPIOB; rsc_ee.cs_port = GPIOD; adb_bus_t ee, adc; adb_platform_spi_bus(&ee, &rsc_ee); adb_platform_spi_bus(&adc, &rsc_adc);
    uint8_t rx[4]; const uint8_t t1[1] = { 0x03 };
    CHECK_EQ_I(ee.write_read(ee.ctx, t1, 1, rx, 2), ADB_OK); CHECK_EQ_I(count(EV_SPI_INIT), 0);        /* EEPROM = mode 0 = current */
    CHECK_EQ_I(adc.read(adc.ctx, rx, 3), ADB_OK);   CHECK_EQ_I(count(EV_SPI_INIT), 1);                  /* ADC needs mode 1: one re-init */
    CHECK_EQ_I(hspi2.Init.CLKPhase, SPI_PHASE_2EDGE);
    CHECK_EQ_I(LOG[1 + 0].type, EV_XFER);                                                  /* (sanity: EEPROM frame logged first) */
    int init_idx = -1, cs_idx = -1; for (int i = 0; i < NL; i++) { if (LOG[i].type == EV_SPI_INIT) init_idx = i; if (LOG[i].type == EV_CS && LOG[i].a == GPIO_PIN_12 && LOG[i].b == 0) cs_idx = i; }
    CHECK(init_idx >= 0 && cs_idx > init_idx);                                             /* mode is programmed BEFORE the chip select drops */
    for (int i = 0; i < 20; i++) adc.read(adc.ctx, rx, 3);
    CHECK_EQ_I(count(EV_SPI_INIT), 1);                                                     /* steady state: no re-init while polling the ADC */
    CHECK_EQ_I(ee.write_read(ee.ctx, t1, 1, rx, 2), ADB_OK); CHECK_EQ_I(count(EV_SPI_INIT), 2);        /* back to mode 0 */
}
static void test_independent_spi_buses(void)
{
    reset_fake(); bmp.cs_port = GPIOE; rsc_adc.cs_port = GPIOB; adb_bus_t b, r; adb_platform_spi_bus(&b, &bmp); adb_platform_spi_bus(&r, &rsc_adc);
    uint8_t rx[3]; const uint8_t t[1] = { 0x81 };
    r.read(r.ctx, rx, 3);                                                                  /* switches SPI2 to mode 1 */
    CHECK_EQ_I(count(EV_SPI_INIT), 1); CHECK_EQ_I(nth(EV_SPI_INIT, 0)->a, 2);
    b.write_read(b.ctx, t, 1, rx, 3);                                                      /* BMP581 on SPI4 is unaffected */
    CHECK_EQ_I(count(EV_SPI_INIT), 1);
    CHECK_EQ_I(nth(EV_XFER, 1)->a, 4);                                                     /* used the SPI4 handle */
}

/* ------------------------------------------------------------------ I2C */
static adb_i2c_dev_t ms;
static void ms_fresh(void) { ms = (adb_i2c_dev_t){ &hi2c4, 0x77u, false, 0u }; }

static void test_i2c_address_and_stop_structure(void)
{
    reset_fake(); ms_fresh(); adb_bus_t b; adb_platform_i2c_bus(&b, &ms);
    const uint8_t rst = 0x1E;
    CHECK_EQ_I(b.write(b.ctx, &rst, 1), ADB_OK);
    CHECK_EQ_I(nth(EV_I2C_TX, 0)->a, 0xEE);                                                /* 0x77 << 1 = 0xEE (HAL uses the 8-bit address) */
    CHECK_EQ_I(b.kind, ADB_BUS_I2C);
    NL = 0; const uint8_t cmd = 0xA2; uint8_t rx[2];
    CHECK_EQ_I(b.write_read(b.ctx, &cmd, 1, rx, 2), ADB_OK);
    CHECK_EQ_I(NL, 2); CHECK_EQ_I(LOG[0].type, EV_I2C_TX); CHECK_EQ_I(LOG[1].type, EV_I2C_RX);   /* command first, then a separate read */
    CHECK_EQ_I(LOG[0].d[0], 0xA2); CHECK_EQ_I(LOG[1].a, 0xEE); CHECK_EQ_I(LOG[1].b, 2);
    CHECK_EQ_I(rx[0], 0xB0); CHECK_EQ_I(rx[1], 0xB1);
}
static void test_i2c_recovery_and_retry(void)
{
    reset_fake(); ms_fresh(); adb_bus_t b; adb_platform_i2c_bus(&b, &ms); const uint8_t cmd = 0x48;
    i2c_fail_next = 1;                                                                     /* first attempt fails (e.g. SDA held low) */
    CHECK_EQ_I(b.write(b.ctx, &cmd, 1), ADB_OK);                                           /* recovered + retried transparently */
    CHECK_EQ_I(count(EV_I2C_TX), 2); CHECK_EQ_I(count(EV_I2C_DEINIT), 1); CHECK_EQ_I(count(EV_I2C_INIT), 1);
    int tx0 = -1, deinit = -1, init = -1, tx1 = -1; for (int i = 0; i < NL; i++) { if (LOG[i].type == EV_I2C_TX) { if (tx0 < 0) tx0 = i; else tx1 = i; } if (LOG[i].type == EV_I2C_DEINIT) deinit = i; if (LOG[i].type == EV_I2C_INIT) init = i; }
    CHECK(tx0 < deinit && deinit < init && init < tx1);                                    /* fail -> deinit -> init -> retry */
    /* rate limit: an absent sensor must not trigger a recovery on every call */
    NL = 0; i2c_fail_next = 2; i2c_rc = HAL_TIMEOUT;
    CHECK_EQ_I(b.write(b.ctx, &cmd, 1), ADB_ERR_TIMEOUT);
    CHECK_EQ_I(count(EV_I2C_DEINIT), 0); CHECK_EQ_I(count(EV_I2C_TX), 1);                  /* < 100 ms since the last recovery: no retry storm */
    TIME_US += 200000u; NL = 0; i2c_fail_next = 2; i2c_rc = HAL_ERROR;
    CHECK_EQ_I(b.write(b.ctx, &cmd, 1), ADB_ERR_BUS);                                      /* retry also fails -> error reported honestly */
    CHECK_EQ_I(count(EV_I2C_DEINIT), 1); CHECK_EQ_I(count(EV_I2C_TX), 2);
}
static void test_i2c_stuck_sda_pulses(void)
{
    reset_fake(); ms_fresh(); adb_bus_t b; adb_platform_i2c_bus(&b, &ms); const uint8_t cmd = 0x1E;
    sda_stuck = 3; i2c_fail_next = 1; i2c_rc = HAL_ERROR;
    CHECK_EQ_I(b.write(b.ctx, &cmd, 1), ADB_OK);
    CHECK_EQ_I(count(EV_SCL_RISE), 3);                                                     /* stops clocking as soon as SDA is released */
    reset_fake(); ms_fresh(); adb_platform_i2c_bus(&b, &ms);
    sda_stuck = 1000; i2c_fail_next = 1; i2c_rc = HAL_ERROR;
    (void)b.write(b.ctx, &cmd, 1);
    CHECK_EQ_I(count(EV_SCL_RISE), 9);                                                     /* never more than 9 pulses */
    int saw_od_sda = 0; for (int i = 0; i < NL; i++) if (LOG[i].type == EV_GPIO_INIT && LOG[i].a == GPIO_PIN_13 && LOG[i].b == GPIO_MODE_OUTPUT_OD) saw_od_sda = 1;
    CHECK(saw_od_sda);                                                                     /* STOP condition generated */
}
static void test_i2c_parameter_checks(void)
{
    reset_fake(); ms_fresh(); adb_bus_t b; adb_platform_i2c_bus(&b, &ms); uint8_t rx[4]; const uint8_t t = 1;
    CHECK_EQ_I(b.write(b.ctx, NULL, 1), ADB_ERR_PARAM);
    CHECK_EQ_I(b.write(b.ctx, &t, 0), ADB_ERR_PARAM);
    CHECK_EQ_I(b.write_read(b.ctx, &t, 1, rx, 0), ADB_ERR_PARAM);
    CHECK_EQ_I(b.write_read(NULL, &t, 1, rx, 2), ADB_ERR_PARAM);
    CHECK(b.read == NULL);                                                                 /* MS5611 never needs a pure read */
    CHECK_EQ_I(NL, 0);
}

/* ------------------------------------------------------------------ CAN */
static void test_can_tx_header(void)
{
    can_rc = HAL_OK; dc_frame_t f; memset(&f, 0, sizeof f); f.id = 0x1004032Au; f.len = 8; for (int i = 0; i < 8; i++) f.data[i] = (uint8_t)(0x10 + i);
    CHECK(adb_can_tx(NULL, &f));
    CHECK_EQ_I(can_hdr.Identifier, 0x1004032A); CHECK_EQ_I(can_hdr.IdType, FDCAN_EXTENDED_ID);
    CHECK_EQ_I(can_hdr.TxFrameType, FDCAN_DATA_FRAME); CHECK_EQ_I(can_hdr.DataLength, FDCAN_DLC_BYTES_8);
    CHECK_EQ_I(can_hdr.FDFormat, FDCAN_CLASSIC_CAN); CHECK_EQ_I(can_hdr.BitRateSwitch, FDCAN_BRS_OFF);
    CHECK_EQ_I(can_data[0], 0x10); CHECK_EQ_I(can_data[7], 0x17);
    f.len = 5; CHECK(adb_can_tx(NULL, &f)); CHECK_EQ_I(can_hdr.DataLength, FDCAN_DLC_BYTES_5);
    f.len = 9; CHECK(!adb_can_tx(NULL, &f));                                               /* classic CAN: max 8 */
    can_rc = HAL_ERROR; f.len = 8; CHECK(!adb_can_tx(NULL, &f));                           /* HAL refused (FIFO full) is reported */
    CHECK(!adb_can_tx(NULL, NULL));
    can_free = 5; CHECK_EQ_I(adb_can_tx_free(NULL), 5);
}
static void test_can_bus_off_and_stuck_tx_recovery(void)
{
    reset_fake(); can_bus_off = 0; can_free = 8; uint32_t t = 1000000u;
    adb_can_service(t); CHECK_EQ_I(count(EV_CAN_STOP), 0);                                 /* healthy: nothing happens */
    can_bus_off = 1; t += 1000000u; adb_can_service(t);                                    /* bus-off detected: recovery scheduled, not instant */
    t += 50000u; adb_can_service(t); CHECK_EQ_I(count(EV_CAN_STOP), 0);                    /* still inside the 100 ms back-off */
    t += 60000u; adb_can_service(t);
    CHECK_EQ_I(count(EV_CAN_STOP), 1); CHECK_EQ_I(count(EV_CAN_START), 1);                 /* Stop then Start after the back-off */
    CHECK(nth(EV_CAN_STOP, 0) < nth(EV_CAN_START, 0));
    CHECK_EQ_I(adb_can_restart_count(), 1);
    t += 20000000u; can_free = 0;                                                          /* TX FIFO stuck full (no ACK on the bus) */
    adb_can_service(t); t += 500000u; adb_can_service(t); CHECK_EQ_I(count(EV_CAN_STOP), 1);   /* 0.5 s: not yet */
    t += 600000u; adb_can_service(t);
    CHECK_EQ_I(count(EV_CAN_STOP), 2); CHECK_EQ_I(adb_can_restart_count(), 2);              /* flushed after 1 s (spec: abort after 1 s) */
}
static void test_time_base_selfcheck(void)
{
    reset_fake(); TIME_US = 5000000u; systick_stalled = 0;
    tim_scale_pct = 100u; CHECK(adb_time_base_ok());                       /* timer really at 1 MHz */
    tim_scale_pct = 625u; CHECK(!adb_time_base_ok());                      /* prescaler set for 64 MHz but the timer clock is 400 MHz: 6.25x too fast */
    tim_scale_pct = 25u;  CHECK(!adb_time_base_ok());                      /* 4x too slow */
    tim_scale_pct = 105u; CHECK(adb_time_base_ok());                       /* 5 % off: inside the +/-10 % window (documents the tolerance) */
    tim_scale_pct = 100u; systick_stalled = 1; CHECK(!adb_time_base_ok()); /* SysTick not running: bounded wait, returns instead of hanging */
    systick_stalled = 0; tim_scale_pct = 100u;
}
static void test_time_base(void) { TIME_US = 1000; uint32_t a = adb_time_us(), b = adb_time_us(); CHECK(b > a); }

int main(void)
{
    RUN(test_spi_write_framing); RUN(test_spi_write_read_is_one_cs_frame); RUN(test_spi_read_starts_at_first_clock);
    RUN(test_spi_cs_released_on_error); RUN(test_spi_parameter_limits); RUN(test_rsc_spi_mode_switching); RUN(test_independent_spi_buses);
    RUN(test_i2c_address_and_stop_structure); RUN(test_i2c_recovery_and_retry); RUN(test_i2c_stuck_sda_pulses); RUN(test_i2c_parameter_checks);
    RUN(test_can_tx_header); RUN(test_can_bus_off_and_stuck_tx_recovery); RUN(test_time_base); RUN(test_time_base_selfcheck);
    TEST_SUMMARY(); return g_tests_failed ? 1 : 0;
}
