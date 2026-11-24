/* adb_platform.c - see adb_platform.h */
#include "adb_platform.h"
#include "can_supervisor.h"
#include <string.h>

/* ---- time ---- */
uint32_t adb_time_us(void) { return __HAL_TIM_GET_COUNTER(&ADB_TIM_HANDLE); }
void adb_delay_us(uint32_t us)
{
    uint32_t t0 = adb_time_us();
    while ((uint32_t)(adb_time_us() - t0) < us) { }          /* init/retry paths only */
}

#ifndef ADB_TIMEBASE_SPIN_LIMIT
#define ADB_TIMEBASE_SPIN_LIMIT 50000000u    /* iterations, far more than 10 ms even at 480 MHz */
#endif
bool adb_time_base_ok(void)
{
    uint32_t guard = 0u;
    uint32_t tick0 = HAL_GetTick();
    while (HAL_GetTick() == tick0) { if (++guard > ADB_TIMEBASE_SPIN_LIMIT) return false; }      /* align to a SysTick edge */
    tick0 = HAL_GetTick();
    uint32_t us0 = adb_time_us();
    guard = 0u;
    while ((uint32_t)(HAL_GetTick() - tick0) < 10u) { if (++guard > ADB_TIMEBASE_SPIN_LIMIT) return false; }
    uint32_t dus = (uint32_t)(adb_time_us() - us0);
    return dus >= 9000u && dus <= 11000u;                                                          /* 10 ms +/- 10 % */
}

/* ---- SPI ---- */
#define SPI_MAX_FRAME 480u           /* >= 2 + 452 (RSC EEPROM image) */

static adb_status_t from_hal(HAL_StatusTypeDef s)
{
    switch (s) { case HAL_OK: return ADB_OK; case HAL_TIMEOUT: return ADB_ERR_TIMEOUT; default: return ADB_ERR_BUS; }
}

/* The RSC EEPROM (mode 0) and ADC (mode 1) share SCK/MISO/MOSI but need different clock phase: re-apply the
 * mode only when it changes. HAL_SPI_Init() re-programs CFG2 with the peripheral disabled. */
static adb_status_t apply_mode(adb_spi_dev_t *d)
{
    if (d->hspi->Init.CLKPolarity == d->cpol && d->hspi->Init.CLKPhase == d->cpha) return ADB_OK;
    d->hspi->Init.CLKPolarity = d->cpol;
    d->hspi->Init.CLKPhase = d->cpha;
    return from_hal(HAL_SPI_Init(d->hspi));
}
static void cs_low(adb_spi_dev_t *d)  { HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_RESET); }
static void cs_high(adb_spi_dev_t *d) { HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET); }

static adb_status_t spi_xfer(adb_spi_dev_t *d, const uint8_t *tx, uint8_t *rx, uint16_t n)
{
    adb_status_t s = apply_mode(d);
    if (s != ADB_OK) return s;
    cs_low(d);
    HAL_StatusTypeDef h = HAL_SPI_TransmitReceive(d->hspi, (uint8_t *)(uintptr_t)tx, rx, n, ADB_SPI_TIMEOUT_MS);
    cs_high(d);                                              /* always release CS, even on error */
    return from_hal(h);
}

static adb_status_t bus_write(void *ctx, const uint8_t *tx, uint16_t len)
{
    adb_spi_dev_t *d = ctx; uint8_t junk[16];
    if (!d || !tx || len == 0u || len > sizeof junk) return ADB_ERR_PARAM;
    return spi_xfer(d, tx, junk, len);
}
static adb_status_t bus_write_read(void *ctx, const uint8_t *tx, uint16_t txlen, uint8_t *rx, uint16_t rxlen)
{
    adb_spi_dev_t *d = ctx; static uint8_t txb[SPI_MAX_FRAME], rxb[SPI_MAX_FRAME];   /* static: keeps the stack small; single-threaded */
    if (!d || !tx || !rx || txlen == 0u || (uint32_t)txlen + rxlen > SPI_MAX_FRAME) return ADB_ERR_PARAM;
    memcpy(txb, tx, txlen); memset(txb + txlen, 0x00, rxlen);          /* 0x00 filler while the device answers */
    adb_status_t s = spi_xfer(d, txb, rxb, (uint16_t)(txlen + rxlen));  /* ONE chip-select frame */
    if (s == ADB_OK) memcpy(rx, rxb + txlen, rxlen);
    return s;
}
static adb_status_t bus_read(void *ctx, uint8_t *rx, uint16_t len)
{
    adb_spi_dev_t *d = ctx; static uint8_t txb[SPI_MAX_FRAME];
    if (!d || !rx || len == 0u || len > SPI_MAX_FRAME) return ADB_ERR_PARAM;
    memset(txb, 0x00, len);
    return spi_xfer(d, txb, rx, len);
}
static void bus_delay(void *ctx, uint32_t us) { (void)ctx; adb_delay_us(us); }

void adb_platform_spi_bus(adb_bus_t *bus, adb_spi_dev_t *dev)
{
    bus->ctx = dev; bus->kind = ADB_BUS_SPI;
    bus->write = bus_write; bus->write_read = bus_write_read; bus->read = bus_read; bus->delay_us = bus_delay;
}

/* ---- I2C (MS5611) ---- */
#define I2C_RECOVER_MIN_INTERVAL_US 100000u

void adb_i2c_recover(adb_i2c_dev_t *d)
{
    if (!d) return;
    (void)HAL_I2C_DeInit(d->hi2c);                           /* releases the pins (MspDeInit) */
    GPIO_InitTypeDef g;
    memset(&g, 0, sizeof g);
    g.Pin = ADB_I2C_SCL_PIN; g.Mode = GPIO_MODE_OUTPUT_OD; g.Pull = GPIO_PULLUP; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ADB_I2C_SCL_PORT, &g);
    g.Pin = ADB_I2C_SDA_PIN; g.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(ADB_I2C_SDA_PORT, &g);
    HAL_GPIO_WritePin(ADB_I2C_SCL_PORT, ADB_I2C_SCL_PIN, GPIO_PIN_SET);
    for (int i = 0; i < 9 && HAL_GPIO_ReadPin(ADB_I2C_SDA_PORT, ADB_I2C_SDA_PIN) == GPIO_PIN_RESET; i++) {
        HAL_GPIO_WritePin(ADB_I2C_SCL_PORT, ADB_I2C_SCL_PIN, GPIO_PIN_RESET); adb_delay_us(5u);
        HAL_GPIO_WritePin(ADB_I2C_SCL_PORT, ADB_I2C_SCL_PIN, GPIO_PIN_SET);   adb_delay_us(5u);
    }
    g.Pin = ADB_I2C_SDA_PIN; g.Mode = GPIO_MODE_OUTPUT_OD;   /* STOP condition: SDA rises while SCL is high */
    HAL_GPIO_Init(ADB_I2C_SDA_PORT, &g);
    HAL_GPIO_WritePin(ADB_I2C_SDA_PORT, ADB_I2C_SDA_PIN, GPIO_PIN_RESET); adb_delay_us(5u);
    HAL_GPIO_WritePin(ADB_I2C_SDA_PORT, ADB_I2C_SDA_PIN, GPIO_PIN_SET);   adb_delay_us(5u);
    (void)HAL_I2C_Init(d->hi2c);                             /* MspInit restores the alternate-function pins */
    d->have_recovered = true;
    d->t_last_recover_us = adb_time_us();
}

static adb_status_t i2c_do(adb_i2c_dev_t *d, const uint8_t *tx, uint16_t txlen, uint8_t *rx, uint16_t rxlen)
{
    uint16_t dev = (uint16_t)(d->addr7 << 1);                /* HAL wants the 8-bit (shifted) address */
    if (txlen > 0u) {                                        /* each HAL call ends with STOP, as the datasheet figures show */
        adb_status_t s = from_hal(HAL_I2C_Master_Transmit(d->hi2c, dev, (uint8_t *)(uintptr_t)tx, txlen, ADB_I2C_TIMEOUT_MS));
        if (s != ADB_OK) return s;
    }
    if (rxlen > 0u) return from_hal(HAL_I2C_Master_Receive(d->hi2c, dev, rx, rxlen, ADB_I2C_TIMEOUT_MS));
    return ADB_OK;
}

/* On failure: recover the bus (rate-limited, so an absent sensor does not cause constant re-initialisation)
 * and retry the transfer once. */
static adb_status_t i2c_with_recovery(adb_i2c_dev_t *d, const uint8_t *tx, uint16_t txlen, uint8_t *rx, uint16_t rxlen)
{
    adb_status_t s = i2c_do(d, tx, txlen, rx, rxlen);
    if (s != ADB_OK && (!d->have_recovered || (uint32_t)(adb_time_us() - d->t_last_recover_us) >= I2C_RECOVER_MIN_INTERVAL_US)) {
        adb_i2c_recover(d);
        s = i2c_do(d, tx, txlen, rx, rxlen);
    }
    return s;
}

static adb_status_t i2c_write(void *ctx, const uint8_t *tx, uint16_t len)
{
    adb_i2c_dev_t *d = ctx;
    if (!d || !tx || len == 0u || len > 16u) return ADB_ERR_PARAM;
    return i2c_with_recovery(d, tx, len, NULL, 0u);
}
static adb_status_t i2c_write_read(void *ctx, const uint8_t *tx, uint16_t txlen, uint8_t *rx, uint16_t rxlen)
{
    adb_i2c_dev_t *d = ctx;
    if (!d || !tx || !rx || txlen == 0u || rxlen == 0u || txlen > 16u || rxlen > 16u) return ADB_ERR_PARAM;
    return i2c_with_recovery(d, tx, txlen, rx, rxlen);
}

void adb_platform_i2c_bus(adb_bus_t *bus, adb_i2c_dev_t *dev)
{
    bus->ctx = dev; bus->kind = ADB_BUS_I2C;
    bus->write = i2c_write; bus->write_read = i2c_write_read; bus->read = NULL; bus->delay_us = bus_delay;
}

/* ---- CAN ---- */
static uint32_t dlc_code(uint8_t n)
{
    static const uint32_t t[9] = { FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4,
                                   FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8 };
    return t[(n <= 8u) ? n : 8u];
}
bool adb_can_tx(void *ctx, const dc_frame_t *f)
{
    (void)ctx;
    if (!f || f->len > 8u) return false;
    FDCAN_TxHeaderTypeDef h;
    memset(&h, 0, sizeof h);
    h.Identifier = f->id;                                    /* 29-bit DroneCAN ID */
    h.IdType = FDCAN_EXTENDED_ID;
    h.TxFrameType = FDCAN_DATA_FRAME;
    h.DataLength = dlc_code(f->len);
    h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    h.BitRateSwitch = FDCAN_BRS_OFF;
    h.FDFormat = FDCAN_CLASSIC_CAN;
    h.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    h.MessageMarker = 0u;
    /* FIFO mode keeps frames of the same ID in order (spec 4.1: identical IDs must be sent in queue order). */
    return HAL_FDCAN_AddMessageToTxFifoQ(&ADB_CAN_HANDLE, &h, (uint8_t *)(uintptr_t)f->data) == HAL_OK;
}

/* ---- CAN supervision (bus-off recovery, stuck TX flush) ---- */
static cansup_t g_cansup;
static bool     g_cansup_ready;
static uint32_t g_can_restarts;

uint32_t adb_can_tx_free(void *ctx)
{
    (void)ctx;
    return HAL_FDCAN_GetTxFifoFreeLevel(&ADB_CAN_HANDLE);
}
uint32_t adb_can_restart_count(void) { return g_can_restarts; }

void adb_can_service(uint32_t now_us)
{
    if (!g_cansup_ready) { cansup_init(&g_cansup, NULL); g_cansup_ready = true; }
    FDCAN_ProtocolStatusTypeDef ps;
    bool bus_off = false;
    if (HAL_FDCAN_GetProtocolStatus(&ADB_CAN_HANDLE, &ps) == HAL_OK) bus_off = (ps.BusOff != 0u);
    if (cansup_step(&g_cansup, now_us, bus_off, HAL_FDCAN_GetTxFifoFreeLevel(&ADB_CAN_HANDLE)) == CANSUP_RESTART) {
        /* Stop() puts the controller in INIT mode (also flushes what is queued), Start() re-integrates into the bus
         * (waits for 11 consecutive recessive bits, i.e. the standard bus-off recovery sequence). */
        (void)HAL_FDCAN_Stop(&ADB_CAN_HANDLE);
        (void)HAL_FDCAN_Start(&ADB_CAN_HANDLE);
        g_can_restarts++;
    }
}
